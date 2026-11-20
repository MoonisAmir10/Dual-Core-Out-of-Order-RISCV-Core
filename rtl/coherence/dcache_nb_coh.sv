`timescale 1ns / 1ps

// ============================================================================
// MOESI Coherent 2-Way Set-Associative Non-Blocking L1 Data Cache (Phase 3)
//
// Upgrades dcache_nb.sv with full 5-State MOESI Cache Coherence:
//   - Stable States: I, S, E, O, M (parameter ENABLE_MOESI_O=1; set 0 for MESI)
//   - Dedicated Duplicate Snoop Tag/State Array -> 0-stall parallel snoops on
//     non-colliding sets and zero cross-core combinational loops
//   - 2 Non-Blocking MSHRs with explicit transient race resolution:
//       * SM_AD / OM_AD Upgrade lost to peer GetM/Upgrade -> auto-promotes
//         unsent MSHR from COH_REQ_UPGRADE to COH_REQ_GETM seamlessly!
//       * Exclusive detection on GetS (peer MISS -> E; silent E -> M on store)
//       * Dirty C2C sharing on snoop GetS (M -> O with 0 L2 WB in MOESI mode!)
//   - 2-Entry Writeback Buffer (WBB) is fully snooped on GetS / GetM / Upgrade:
//       * Supplies C2C dirty data on GetS / GetM
//       * Cancels stale WBB entry on GetM / Upgrade to guarantee zero stale WB
//   - Full RV32A (LR.W / SC.W + 9x AMO*.W) with snoop-triggered reservation clear
//   - FENCE.I Dirty Clean-All Sweep (M -> E, O -> S)
//   - Emits snoop_inv_valid / snoop_inv_laddr pulse to lsq2w for RVWMO
//     Speculative Load Snoop-Invalidation Replay!
// ============================================================================
module dcache_nb_coh #(
    parameter integer AWIDTH         = 20,  // 20 -> 1 MB unified physical window
    parameter integer NUM_SETS       = 128, // 128 -> 4 KB (2-way x 16B); 2 -> formal BMC
    parameter integer TAG_W          = 5,   // ROB tag width (32-entry ROB)
    parameter integer ID_W           = 7,   // LSQ/Core transaction ID width
    parameter integer ENABLE_MOESI_O = 1    // 1: MOESI (M->O C2C), 0: MESI (M->S + L2 WB)
) (
    input  wire                clk,
    input  wire                reset_n,

    // Split-Transaction CPU Request Channel (LSQ -> Coherent L1D)
    input  wire                req_valid,
    output reg                 req_ready,
    input  wire                req_is_write,
    input  wire [3:0]          req_amo_op,
    input  wire [AWIDTH-1:0]   req_addr,
    input  wire [31:0]         req_wdata,
    input  wire [3:0]          req_wstrb,
    input  wire [ID_W-1:0]     req_id,
    input  wire [TAG_W-1:0]    req_rob_tag,

    // Port A: 1-Cycle Load / Atomic Hit Response (L1D -> LSQ)
    output reg                 hit_resp_valid,
    output reg  [ID_W-1:0]     hit_resp_id,
    output reg  [31:0]         hit_resp_rdata,

    // Port B: MSHR Miss/Upgrade Fill Response (L1D -> LSQ)
    output reg                 fill_resp_valid,
    output reg  [ID_W-1:0]     fill_resp_id,
    output reg  [31:0]         fill_resp_rdata,

    // Selective Branch-Squash Kill (kills younger MSHR load waiters cleanly)
    input  wire                flush,
    input  wire                squash_valid,
    input  wire [TAG_W-1:0]    squash_tag,
    input  wire [TAG_W-1:0]    rob_head,

    // FENCE.I Dirty Writeback Clean-All Control
    input  wire                clean_all_req,
    output reg                 clean_all_done,

    // Speculative Load Snoop-Invalidation Replay Pulse (L1D -> LSQ)
    output reg                 snoop_inv_valid,
    output reg  [AWIDTH-5:0]   snoop_inv_laddr,

    // Channel A: Outgoing Coherence Request (L1D -> coh_interconnect)
    output reg                 coh_req_valid,
    input  wire                coh_req_ready,
    output reg  [1:0]          coh_req_op,    // GETS / GETM / UPGRADE
    output reg  [AWIDTH-5:0]   coh_req_laddr,
    output reg                 coh_req_id,    // 0: MSHR0, 1: MSHR1

    // Channel B: Incoming Snoop Probe (coh_interconnect -> L1D)
    input  wire                snoop_valid,
    input  wire [1:0]          snoop_op,      // GETS / GETM / UPGRADE
    input  wire [AWIDTH-5:0]   snoop_laddr,

    // Channel C (Part 1): Combinational Snoop Response (L1D -> coh_interconnect)
    output reg  [1:0]          snoop_resp_op,   // MISS / SHARED / INVACK / DATA
    output reg  [127:0]        snoop_resp_data, // 128-bit dirty C2C line when DATA

    // Channel C (Part 2): Dirty WBB Eviction / Clean-All Writeback (L1D -> Interconnect -> L2)
    output wire                wb_valid,
    input  wire                wb_ready,
    output wire [AWIDTH-5:0]   wb_laddr,
    output wire [127:0]        wb_data,

    // Channel D: Incoming Coherence Data / Upgrade ACK (coh_interconnect -> L1D)
    input  wire                coh_resp_valid,
    input  wire                coh_resp_id,
    input  wire [127:0]        coh_resp_data,
    input  wire                coh_resp_shared,      // 1->S, 0->E on GETS
    input  wire                coh_resp_from_c2c,    // 1 if data came from peer C2C
    input  wire                coh_resp_upgrade_ack, // 1->Upgrade S/O->M (no data transfer)

    // Verification Observability: Exact Write Linearization Hook (for coh_checker / formal)
    output wire                dbg_wr_commit_valid,
    output wire [AWIDTH-5:0]   dbg_wr_commit_laddr,
    output wire [127:0]        dbg_wr_commit_data,

    // Status & Hardware PMU Observability Pulses
    output wire                dcache_idle,
    output wire                dcache_writes_idle,
    output wire                dcache_any_miss,
    output wire                cache_hit,
    output wire                cache_miss,
    output wire                cache_wb,
    output wire                cache_hum,
    output wire                cache_mum_cyc,
    output wire                cache_mshr_coalesce,
    output wire                pmu_silent_e2m,
    output wire                pmu_upgrade_miss
`ifdef FORMAL
    ,output wire [NUM_SETS*3-1:0]             f_way0_state,
    output wire [NUM_SETS*(AWIDTH-4-$clog2(NUM_SETS))-1:0] f_way0_tag,
    output wire [NUM_SETS*8-1:0]              f_way0_b0,
    output wire [NUM_SETS*3-1:0]              f_way1_state,
    output wire [NUM_SETS*(AWIDTH-4-$clog2(NUM_SETS))-1:0] f_way1_tag,
    output wire [NUM_SETS*8-1:0]              f_way1_b0,
    output wire [NUM_SETS*3-1:0]              f_snp_way0_state,
    output wire [NUM_SETS*(AWIDTH-4-$clog2(NUM_SETS))-1:0] f_snp_way0_tag,
    output wire [NUM_SETS*3-1:0]              f_snp_way1_state,
    output wire [NUM_SETS*(AWIDTH-4-$clog2(NUM_SETS))-1:0] f_snp_way1_tag,
    output wire                               f_wbb0_v,
    output wire [AWIDTH-5:0]                  f_wbb0_laddr,
    output wire [7:0]                         f_wbb0_b0,
    output wire                               f_wbb1_v,
    output wire [AWIDTH-5:0]                  f_wbb1_laddr,
    output wire [7:0]                         f_wbb1_b0,
    output wire [AWIDTH-5:0]                  f_rf_laddr
`endif
);

`include "rv32ima_defs.vh"
`include "coh_defs.vh"

localparam integer IDX_W   = $clog2(NUM_SETS);
localparam integer LADDR_W = AWIDTH - 4;
localparam integer CTAG_W  = LADDR_W - IDX_W;

// ----------------------------------------------------------------------------
// 1. Helper Functions (128-bit Line Word Extract, Byte-Masked Merge, AMO ALU)
// ----------------------------------------------------------------------------
function automatic [31:0] get_word(input [127:0] line, input [1:0] woff);
    begin
        case (woff)
            2'd0: get_word = line[31:0];
            2'd1: get_word = line[63:32];
            2'd2: get_word = line[95:64];
            2'd3: get_word = line[127:96];
        endcase
    end
endfunction

function automatic [127:0] merge_word(
    input [127:0] line,
    input [1:0]   woff,
    input [31:0]  wdata,
    input [3:0]   wstrb
);
    reg [127:0] res;
    reg [31:0]  cur, nw;
    begin
        res = line;
        cur = get_word(line, woff);
        nw  = {
            wstrb[3] ? wdata[31:24] : cur[31:24],
            wstrb[2] ? wdata[23:16] : cur[23:16],
            wstrb[1] ? wdata[15:8]  : cur[15:8],
            wstrb[0] ? wdata[7:0]   : cur[7:0]
        };
        case (woff)
            2'd0: res[31:0]   = nw;
            2'd1: res[63:32]  = nw;
            2'd2: res[95:64]  = nw;
            2'd3: res[127:96] = nw;
        endcase
        merge_word = res;
    end
endfunction

function automatic [15:0] expand_line_mask(input [1:0] woff, input [3:0] wstrb);
    begin
        case (woff)
            2'd0: expand_line_mask = {12'd0, wstrb};
            2'd1: expand_line_mask = {8'd0,  wstrb, 4'd0};
            2'd2: expand_line_mask = {4'd0,  wstrb, 8'd0};
            2'd3: expand_line_mask = {wstrb, 12'd0};
        endcase
    end
endfunction

function automatic [127:0] merge_line_bytes(
    input [127:0] base_line,
    input [127:0] over_line,
    input [15:0]  mask
);
    integer b;
    begin
        for (b = 0; b < 16; b = b + 1)
            merge_line_bytes[b*8 +: 8] = mask[b] ? over_line[b*8 +: 8] : base_line[b*8 +: 8];
    end
endfunction

function automatic [31:0] amo_alu(
    input [3:0]  op,
    input [31:0] old_val,
    input [31:0] operand
);
    begin
        case (op)
            AMO_SWAP: amo_alu = operand;
            AMO_ADD:  amo_alu = old_val + operand;
            AMO_XOR:  amo_alu = old_val ^ operand;
            AMO_AND:  amo_alu = old_val & operand;
            AMO_OR:   amo_alu = old_val | operand;
            AMO_MIN:  amo_alu = ($signed(old_val) < $signed(operand)) ? old_val : operand;
            AMO_MAX:  amo_alu = ($signed(old_val) > $signed(operand)) ? old_val : operand;
            AMO_MINU: amo_alu = (old_val < operand) ? old_val : operand;
            AMO_MAXU: amo_alu = (old_val > operand) ? old_val : operand;
            default:  amo_alu = operand;
        endcase
    end
endfunction

function automatic is_dirty_state(input [2:0] st);
    begin
        is_dirty_state = (st == COH_M) || (st == COH_O);
    end
endfunction

function automatic is_write_perm(input [2:0] st);
    begin
        is_write_perm = (st == COH_M) || (st == COH_E);
    end
endfunction

// ----------------------------------------------------------------------------
// 2. Main Cache Arrays + Duplicate Snoop Tag/State Arrays + Reservation + Clean
// ----------------------------------------------------------------------------
reg [2:0]        way0_state [0:NUM_SETS-1];
reg [CTAG_W-1:0] way0_tag   [0:NUM_SETS-1];
reg [127:0]      way0_data  [0:NUM_SETS-1];

reg [2:0]        way1_state [0:NUM_SETS-1];
reg [CTAG_W-1:0] way1_tag   [0:NUM_SETS-1];
reg [127:0]      way1_data  [0:NUM_SETS-1];

// Duplicate Snoop Tag/State Arrays (strictly mirrored on every posedge!)
reg [2:0]        snp_way0_state [0:NUM_SETS-1];
reg [CTAG_W-1:0] snp_way0_tag   [0:NUM_SETS-1];
reg [2:0]        snp_way1_state [0:NUM_SETS-1];
reg [CTAG_W-1:0] snp_way1_tag   [0:NUM_SETS-1];

reg              lru_arr [0:NUM_SETS-1];

// RV32A LR/SC Reservation Register (16B cache-line granule)
reg               resv_valid;
reg [LADDR_W-1:0] resv_laddr;

// FENCE.I Dirty Clean-All Sweep FSM
reg               clean_active;
reg [IDX_W:0]     clean_idx;

// ----------------------------------------------------------------------------
// 3. 2 MSHRs + 8 Waiters + 2-Entry Snoopable WBB
// ----------------------------------------------------------------------------
reg               m0_valid, m0_refilled, m0_sent;
reg [1:0]         m0_coh_op;     // COH_REQ_GETS / COH_REQ_GETM / COH_REQ_UPGRADE
reg [LADDR_W-1:0] m0_laddr;
reg [15:0]        m0_st_mask;
reg [127:0]       m0_st_data;
reg [127:0]       m0_fill_line;
reg               m0_snoop_reinv; // Set if invalidated while refilled waiters still draining
reg [3:0]         m0_amo_op;
reg [1:0]         m0_amo_woff;
reg [31:0]        m0_amo_wdata;
reg [ID_W-1:0]    m0_amo_id;

reg               m1_valid, m1_refilled, m1_sent;
reg [1:0]         m1_coh_op;
reg [LADDR_W-1:0] m1_laddr;
reg [15:0]        m1_st_mask;
reg [127:0]       m1_st_data;
reg [127:0]       m1_fill_line;
reg               m1_snoop_reinv;
reg [3:0]         m1_amo_op;
reg [1:0]         m1_amo_woff;
reg [31:0]        m1_amo_wdata;
reg [ID_W-1:0]    m1_amo_id;

// Waiters 0..3 -> MSHR0; 4..7 -> MSHR1
reg               w_v    [0:7];
reg [ID_W-1:0]    w_id   [0:7];
reg [TAG_W-1:0]   w_tag  [0:7];
reg [1:0]         w_woff [0:7];

// 2-Entry Snoopable Writeback Buffer (MI_A / OI_A)
reg               wbb0_v, wbb1_v;
reg [LADDR_W-1:0] wbb0_laddr, wbb1_laddr;
reg [127:0]       wbb0_data, wbb1_data;

// Delayed reinvalidation pulse for a waiter drained after a mid-drain snoop
reg               reinv_pend_v;
reg [LADDR_W-1:0] reinv_pend_laddr;

// ----------------------------------------------------------------------------
// 4. Combinational Snoop Lookup (Duplicate Snoop Tag Array + WBB)
// ----------------------------------------------------------------------------
wire [IDX_W-1:0]  snp_set  = snoop_laddr[IDX_W-1:0];
wire [CTAG_W-1:0] snp_ctag = snoop_laddr[LADDR_W-1:IDX_W];

wire snp_hit_w0  = (snp_way0_state[snp_set] != COH_I) && (snp_way0_tag[snp_set] == snp_ctag);
wire snp_hit_w1  = (snp_way1_state[snp_set] != COH_I) && (snp_way1_tag[snp_set] == snp_ctag);
wire snp_hit_c   = snp_hit_w0 || snp_hit_w1;
wire [2:0]   snp_c_state = snp_hit_w0 ? snp_way0_state[snp_set] : snp_way1_state[snp_set];
wire [127:0] snp_c_data  = snp_hit_w0 ? way0_data[snp_set]      : way1_data[snp_set];

wire snp_hit_wbb0 = wbb0_v && (wbb0_laddr == snoop_laddr);
wire snp_hit_wbb1 = wbb1_v && (wbb1_laddr == snoop_laddr);
wire snp_hit_wbb  = snp_hit_wbb0 || snp_hit_wbb1;
wire [127:0] snp_wbb_data = snp_hit_wbb0 ? wbb0_data : wbb1_data;

wire snp_is_gets = (snoop_op == COH_REQ_GETS);
wire snp_is_getm = (snoop_op == COH_REQ_GETM);
wire snp_is_upg  = (snoop_op == COH_REQ_UPGRADE);
wire snp_is_inv  = snp_is_getm || snp_is_upg;

// Compute next MOESI state for snooped cache way & Channel C snoop response
reg [2:0] snp_next_state;

always @(*) begin
    snoop_resp_op   = SNP_RESP_MISS;
    snoop_resp_data = 128'd0;
    snp_next_state  = snp_c_state;

    if (snoop_valid) begin
        if (snp_hit_c) begin
            case (snp_c_state)
                COH_S: begin
                    if (snp_is_gets) begin
                        snoop_resp_op  = SNP_RESP_SHARED;
                        snp_next_state = COH_S;
                    end else begin
                        snoop_resp_op  = SNP_RESP_INVACK;
                        snp_next_state = COH_I;
                    end
                end
                COH_E: begin
                    if (snp_is_gets) begin
                        snoop_resp_op  = SNP_RESP_SHARED;
                        snp_next_state = COH_S;
                    end else begin
                        snoop_resp_op  = SNP_RESP_INVACK;
                        snp_next_state = COH_I;
                    end
                end
                COH_O: begin
                    if (snp_is_gets) begin
                        snoop_resp_op   = SNP_RESP_DATA;
                        snoop_resp_data = snp_c_data;
                        snp_next_state  = COH_O;
                    end else if (snp_is_getm) begin
                        snoop_resp_op   = SNP_RESP_DATA;
                        snoop_resp_data = snp_c_data;
                        snp_next_state  = COH_I;
                    end else begin
                        // Peer Upgrade from S -> M: peer already holds identical data!
                        // Relinquish O ownership -> I (0 data transfer!)
                        snoop_resp_op  = SNP_RESP_INVACK;
                        snp_next_state = COH_I;
                    end
                end
                COH_M: begin
                    if (snp_is_gets) begin
                        snoop_resp_op   = SNP_RESP_DATA;
                        snoop_resp_data = snp_c_data;
                        // MOESI: M -> O (0 L2 WB!). MESI ablation: M -> S (interconnect writes L2!)
                        snp_next_state  = (ENABLE_MOESI_O != 0) ? COH_O : COH_S;
                    end else begin
                        // GetM (or Upgrade impossible under SWMR): supply dirty data & invalidate
                        snoop_resp_op   = SNP_RESP_DATA;
                        snoop_resp_data = snp_c_data;
                        snp_next_state  = COH_I;
                    end
                end
                default: begin
                    snoop_resp_op  = SNP_RESP_MISS;
                    snp_next_state = COH_I;
                end
            endcase
        end else if (snp_hit_wbb) begin
            // Dirty line sitting in our WBB (MI_A / OI_A): supply C2C data on GetS/GetM!
            // On Upgrade, peer already has identical data in S -> cancel stale WBB!
            if (snp_is_gets || snp_is_getm) begin
                snoop_resp_op   = SNP_RESP_DATA;
                snoop_resp_data = snp_wbb_data;
            end else begin
                snoop_resp_op   = SNP_RESP_INVACK;
            end
        end
    end
end

// ----------------------------------------------------------------------------
// 5. Combinational CPU Lookup, MOESI Permission Check & MSHR Arbitration
// ----------------------------------------------------------------------------
wire [LADDR_W-1:0] req_laddr = req_addr[AWIDTH-1:4];
wire [IDX_W-1:0]   req_set   = req_addr[4 +: IDX_W];
wire [CTAG_W-1:0]  req_ctag  = req_addr[AWIDTH-1 : 4 + IDX_W];
wire [1:0]         req_woff  = req_addr[3:2];

wire req_is_amo = (req_amo_op != AMO_NONE);
wire req_is_lr  = (req_amo_op == AMO_LR);
wire req_is_sc  = (req_amo_op == AMO_SC);
wire req_is_rmw = req_is_amo && !req_is_lr && !req_is_sc;
wire req_needs_wr_perm = req_is_write || req_is_sc || req_is_rmw;

wire sc_resv_match = resv_valid && (resv_laddr == req_laddr);

wire hit_w0  = (way0_state[req_set] != COH_I) && (way0_tag[req_set] == req_ctag);
wire hit_w1  = (way1_state[req_set] != COH_I) && (way1_tag[req_set] == req_ctag);
wire tag_hit = hit_w0 || hit_w1;
wire [2:0] hit_state = hit_w0 ? way0_state[req_set] : way1_state[req_set];

// Read hits in S, E, O, M; Write/AMO/SC hits ONLY in E or M!
wire perm_hit  = tag_hit && (!req_needs_wr_perm || is_write_perm(hit_state));
wire upg_miss  = tag_hit && req_needs_wr_perm && !is_write_perm(hit_state); // S or O!

wire wbb_hit    = (wbb0_v && (wbb0_laddr == req_laddr)) || (wbb1_v && (wbb1_laddr == req_laddr));
wire m0_match   = m0_valid && (m0_laddr == req_laddr);
wire m1_match   = m1_valid && (m1_laddr == req_laddr);
wire mshr_match = m0_match || m1_match;

wire m0_has_waiter = w_v[0] || w_v[1] || w_v[2] || w_v[3];
wire m1_has_waiter = w_v[4] || w_v[5] || w_v[6] || w_v[7];
wire m0_has_wfree  = !w_v[0] || !w_v[1] || !w_v[2] || !w_v[3];
wire m1_has_wfree  = !w_v[4] || !w_v[5] || !w_v[6] || !w_v[7];

wire [1:0] m0_free_widx = !w_v[0] ? 2'd0 : (!w_v[1] ? 2'd1 : (!w_v[2] ? 2'd2 : 2'd3));
wire [1:0] m1_free_widx = !w_v[4] ? 2'd0 : (!w_v[5] ? 2'd1 : (!w_v[6] ? 2'd2 : 2'd3));

wire [1:0] wbb_free_cnt      = {1'b0, !wbb0_v} + {1'b0, !wbb1_v};
wire [1:0] unfilled_mshr_cnt = {1'b0, (m0_valid && !m0_refilled)} + {1'b0, (m1_valid && !m1_refilled)};
wire       same_set_mshr     = (m0_valid && (m0_laddr[IDX_W-1:0] == req_set)) ||
                               (m1_valid && (m1_laddr[IDX_W-1:0] == req_set));
wire       can_alloc_mshr    = (!m0_valid || !m1_valid) && (wbb_free_cnt > unfilled_mshr_cnt) && !same_set_mshr;

wire m0_is_amo = (m0_amo_op != AMO_NONE);
wire m1_is_amo = (m1_amo_op != AMO_NONE);

// Snoop stalls CPU ONLY if snoop collides with the SAME set (duplicate tag gives 0-stall on other sets!)
// OR if snoop hits a dirty cache line needing the shared data array read port!
wire snoop_cpu_hazard = snoop_valid && ((snp_set == req_set) || (snp_hit_c && is_dirty_state(snp_c_state)));

reg req_is_hit_act;
reg req_is_coalesce_act;
reg req_is_new_miss_act; // Full miss (GetS/GetM) OR permission miss (Upgrade S/O -> M)
reg req_is_upg_act;
reg req_is_sc_fail_act;

always @(*) begin
    req_ready           = 1'b0;
    req_is_hit_act      = 1'b0;
    req_is_coalesce_act = 1'b0;
    req_is_new_miss_act = 1'b0;
    req_is_upg_act      = 1'b0;
    req_is_sc_fail_act  = 1'b0;

    if (req_valid && !clean_active && !clean_all_req && !coh_resp_valid && !snoop_cpu_hazard) begin
        if (req_is_sc && !sc_resv_match) begin
            // Broken reservation: SC.W fails locally in 1 cycle with ZERO bus traffic!
            req_ready          = 1'b1;
            req_is_sc_fail_act = 1'b1;
        end else if (!wbb_hit) begin
            if (req_is_amo) begin
                if (!mshr_match) begin
                    if (perm_hit) begin
                        req_ready      = 1'b1;
                        req_is_hit_act = 1'b1;
                    end else if (can_alloc_mshr) begin
                        req_ready           = 1'b1;
                        req_is_new_miss_act = 1'b1;
                        req_is_upg_act      = upg_miss;
                    end
                end
            end else if (m0_match) begin
                if (!m0_refilled && !m0_is_amo) begin
                    if (req_is_write) begin
                        // Only coalesce stores onto write-permitted MSHRs (GetM or Upgrade)
                        if (!m0_has_waiter && (m0_coh_op != COH_REQ_GETS)) begin
                            req_ready           = 1'b1;
                            req_is_coalesce_act = 1'b1;
                        end
                    end else if (m0_has_wfree && (m0_coh_op == COH_REQ_GETS)) begin
                        // Coalesce loads onto GetS MSHRs cleanly
                        req_ready           = 1'b1;
                        req_is_coalesce_act = 1'b1;
                    end
                end
            end else if (m1_match) begin
                if (!m1_refilled && !m1_is_amo) begin
                    if (req_is_write) begin
                        if (!m1_has_waiter && (m1_coh_op != COH_REQ_GETS)) begin
                            req_ready           = 1'b1;
                            req_is_coalesce_act = 1'b1;
                        end
                    end else if (m1_has_wfree && (m1_coh_op == COH_REQ_GETS)) begin
                        req_ready           = 1'b1;
                        req_is_coalesce_act = 1'b1;
                    end
                end
            end else if (perm_hit) begin
                req_ready      = 1'b1;
                req_is_hit_act = 1'b1;
            end else if (can_alloc_mshr) begin
                req_ready           = 1'b1;
                req_is_new_miss_act = 1'b1;
                req_is_upg_act      = upg_miss;
            end
        end
    end
end

// ----------------------------------------------------------------------------
// 6. Channel D Refill / Upgrade-ACK Decode & Victim Selection
// ----------------------------------------------------------------------------
wire m0_rf_now = coh_resp_valid && !coh_resp_id && m0_valid;
wire m1_rf_now = coh_resp_valid &&  coh_resp_id && m1_valid;
wire do_rf_now = m0_rf_now || m1_rf_now;

wire [1:0]         rf_coh_op   = coh_resp_id ? m1_coh_op    : m0_coh_op;
wire [LADDR_W-1:0] rf_laddr    = coh_resp_id ? m1_laddr     : m0_laddr;
wire [IDX_W-1:0]   rf_set      = rf_laddr[IDX_W-1:0];
wire [CTAG_W-1:0]  rf_ctag     = rf_laddr[LADDR_W-1:IDX_W];
wire [15:0]        rf_stmask   = coh_resp_id ? m1_st_mask   : m0_st_mask;
wire [127:0]       rf_stdata   = coh_resp_id ? m1_st_data   : m0_st_data;
wire [3:0]         rf_amo_op   = coh_resp_id ? m1_amo_op    : m0_amo_op;
wire [1:0]         rf_amo_woff = coh_resp_id ? m1_amo_woff  : m0_amo_woff;
wire [31:0]        rf_amo_wdat = coh_resp_id ? m1_amo_wdata : m0_amo_wdata;
wire [ID_W-1:0]    rf_amo_id   = coh_resp_id ? m1_amo_id    : m0_amo_id;

wire rf_is_upg   = coh_resp_upgrade_ack;
wire rf_is_getm  = (rf_coh_op == COH_REQ_GETM);
wire rf_is_amo   = (rf_amo_op != AMO_NONE);
wire rf_is_lr    = (rf_amo_op == AMO_LR);
wire rf_is_sc    = (rf_amo_op == AMO_SC);
wire rf_is_rmw   = rf_is_amo && !rf_is_lr && !rf_is_sc;
wire rf_sc_ok    = rf_is_sc && resv_valid && (resv_laddr == rf_laddr);

// For Upgrade ACK, line is ALREADY resident in way0 or way1!
wire rf_upg_hit_w0  = (way0_state[rf_set] != COH_I) && (way0_tag[rf_set] == rf_ctag);
wire [127:0] rf_base_line = rf_is_upg ? (rf_upg_hit_w0 ? way0_data[rf_set] : way1_data[rf_set])
                                      : coh_resp_data;

wire [31:0]  rf_old_word = get_word(rf_base_line, rf_amo_woff);
wire [31:0]  rf_amo_new  = rf_is_sc ? rf_amo_wdat : amo_alu(rf_amo_op, rf_old_word, rf_amo_wdat);
wire         rf_amo_wr   = rf_is_rmw || rf_sc_ok;
wire [127:0] rf_merged   = rf_amo_wr ? merge_word(rf_base_line, rf_amo_woff, rf_amo_new, 4'hF)
                                     : merge_line_bytes(rf_base_line, rf_stdata, rf_stmask);
wire [31:0]  rf_amo_resp = rf_is_sc ? (rf_sc_ok ? 32'd0 : 32'd1) : rf_old_word;

// Next MOESI state installed on refill / upgrade
// NOTE: GetM ALWAYS installs COH_M (even if SC failed!) to preserve dirty C2C ownership!
wire [2:0] rf_new_state = (rf_is_upg || rf_is_getm) ? COH_M :
                          coh_resp_shared           ? COH_S : COH_E;

// Victim selection for full data refill (never evicts on Upgrade ACK!)
wire rf_vic_way   = (way0_state[rf_set] == COH_I) ? 1'b0 :
                    (way1_state[rf_set] == COH_I) ? 1'b1 :
                    lru_arr[rf_set];
wire [2:0]         rf_vic_state = rf_vic_way ? way1_state[rf_set] : way0_state[rf_set];
wire [LADDR_W-1:0] rf_vic_laddr = {(rf_vic_way ? way1_tag[rf_set] : way0_tag[rf_set]), rf_set};
wire [127:0]       rf_vic_data  = rf_vic_way ? way1_data[rf_set]  : way0_data[rf_set];

// Handle simultaneous snoop + refill on the exact same victim way cleanly!
wire vic_snoop_inv     = snoop_valid && snp_hit_c && (snoop_laddr == rf_vic_laddr) && snp_is_inv;
wire vic_snoop_mesi_wb = snoop_valid && snp_hit_c && (snoop_laddr == rf_vic_laddr) &&
                         snp_is_gets && (ENABLE_MOESI_O == 0) && (rf_vic_state == COH_M);
wire rf_need_wb        = do_rf_now && !rf_is_upg && is_dirty_state(rf_vic_state) &&
                         !vic_snoop_inv && !vic_snoop_mesi_wb;

// Target way to write on refill/upgrade
wire rf_dst_way = rf_is_upg ? !rf_upg_hit_w0 : rf_vic_way;

// ----------------------------------------------------------------------------
// 7. Surviving Load Waiters & Channel A / Channel C Output Arbitration
// ----------------------------------------------------------------------------
wire [TAG_W-1:0] sq_br_age = squash_tag - rob_head;
wire [7:0] w_alive;
genvar gi;
generate
    for (gi = 0; gi < 8; gi = gi + 1) begin : gen_walive
        assign w_alive[gi] = w_v[gi] && !flush &&
                             !(squash_valid && ((w_tag[gi] - rob_head) > sq_br_age));
    end
endgenerate

wire m0_has_alive = |w_alive[3:0];
wire m1_has_alive = |w_alive[7:4];
wire [2:0] m0_first_alive = w_alive[0] ? 3'd0 : (w_alive[1] ? 3'd1 : (w_alive[2] ? 3'd2 : 3'd3));
wire [2:0] m1_first_alive = w_alive[4] ? 3'd4 : (w_alive[5] ? 3'd5 : (w_alive[6] ? 3'd6 : 3'd7));

reg         drain_v;
reg         drain_is_m1;
reg [2:0]   drain_wslot;
reg [127:0] drain_line;
reg         drain_reinv;
reg [LADDR_W-1:0] drain_laddr;

always @(*) begin
    drain_v     = 1'b0;
    drain_is_m1 = 1'b0;
    drain_wslot = 3'd0;
    drain_line  = 128'd0;
    drain_reinv = 1'b0;
    drain_laddr = {LADDR_W{1'b0}};
    if (do_rf_now && rf_is_amo) begin
        drain_v = 1'b0; // Atomic response emitted directly on fill_resp
    end else if (m0_rf_now && m0_has_alive) begin
        drain_v     = 1'b1;
        drain_is_m1 = 1'b0;
        drain_wslot = m0_first_alive;
        drain_line  = rf_merged;
        drain_reinv = 1'b0;
        drain_laddr = m0_laddr;
    end else if (m1_rf_now && m1_has_alive) begin
        drain_v     = 1'b1;
        drain_is_m1 = 1'b1;
        drain_wslot = m1_first_alive;
        drain_line  = rf_merged;
        drain_reinv = 1'b0;
        drain_laddr = m1_laddr;
    end else if (m0_valid && m0_refilled && m0_has_alive) begin
        drain_v     = 1'b1;
        drain_is_m1 = 1'b0;
        drain_wslot = m0_first_alive;
        drain_line  = m0_fill_line;
        drain_reinv = m0_snoop_reinv;
        drain_laddr = m0_laddr;
    end else if (m1_valid && m1_refilled && m1_has_alive) begin
        drain_v     = 1'b1;
        drain_is_m1 = 1'b1;
        drain_wslot = m1_first_alive;
        drain_line  = m1_fill_line;
        drain_reinv = m1_snoop_reinv;
        drain_laddr = m1_laddr;
    end
end

// Channel A Coherence Request Port (MSHR0 -> MSHR1)
// Note: Do not present an unsent MSHR on the exact cycle a snoop hits its laddr
// (so any UPGRADE -> GETM promotion registers cleanly before Channel A issue!)
wire m0_snp_block = snoop_valid && (snoop_laddr == m0_laddr);
wire m1_snp_block = snoop_valid && (snoop_laddr == m1_laddr);

always @(*) begin
    coh_req_valid = 1'b0;
    coh_req_op    = COH_REQ_GETS;
    coh_req_laddr = {LADDR_W{1'b0}};
    coh_req_id    = 1'b0;
    if (m0_valid && !m0_sent && !m0_refilled && !m0_snp_block) begin
        coh_req_valid = 1'b1;
        coh_req_op    = m0_coh_op;
        coh_req_laddr = m0_laddr;
        coh_req_id    = 1'b0;
    end else if (m1_valid && !m1_sent && !m1_refilled && !m1_snp_block) begin
        coh_req_valid = 1'b1;
        coh_req_op    = m1_coh_op;
        coh_req_laddr = m1_laddr;
        coh_req_id    = 1'b1;
    end
end

// Channel C Dirty WBB Writeback Port (WBB0 -> WBB1)
// Gate off WB valid on the exact cycle a peer GetM/Upgrade snoop cancels that WBB entry!
wire wbb0_snp_cancel = snoop_valid && snp_hit_wbb0 && snp_is_inv;
wire wbb1_snp_cancel = snoop_valid && snp_hit_wbb1 && snp_is_inv;
wire wbb0_eff_v      = wbb0_v && !wbb0_snp_cancel;
wire wbb1_eff_v      = wbb1_v && !wbb1_snp_cancel;

assign wb_valid = wbb0_eff_v || wbb1_eff_v;
assign wb_laddr = wbb0_eff_v ? wbb0_laddr : wbb1_laddr;
assign wb_data  = wbb0_eff_v ? wbb0_data  : wbb1_data;

// ----------------------------------------------------------------------------
// 8. Hit Data Computations, Debug Linearization Hook & PMU Telemetry
// ----------------------------------------------------------------------------
wire [127:0] hit_cur_line = hit_w0 ? way0_data[req_set] : way1_data[req_set];
wire [31:0]  hit_old_word = get_word(hit_cur_line, req_woff);
wire [31:0]  hit_amo_new  = req_is_sc ? req_wdata : amo_alu(req_amo_op, hit_old_word, req_wdata);
wire [127:0] hit_new_line = (req_is_amo && !req_is_lr)
                            ? merge_word(hit_cur_line, req_woff, hit_amo_new, 4'hF)
                            : merge_word(hit_cur_line, req_woff, req_wdata, req_wstrb);

wire cpu_hit_writes = req_valid && req_ready && req_is_hit_act &&
                      ((req_is_amo && !req_is_lr) || (req_is_write && (|req_wstrb)));
wire rf_writes      = do_rf_now && (rf_amo_wr || (|rf_stmask));

assign dbg_wr_commit_valid = cpu_hit_writes || rf_writes;
assign dbg_wr_commit_laddr = rf_writes ? rf_laddr  : req_laddr;
assign dbg_wr_commit_data  = rf_writes ? rf_merged : hit_new_line;

assign dcache_any_miss     = (m0_valid && !m0_refilled) || (m1_valid && !m1_refilled);
assign dcache_writes_idle  = !(m0_valid && ((|m0_st_mask) || m0_is_amo)) &&
                             !(m1_valid && ((|m1_st_mask) || m1_is_amo)) &&
                             !wbb0_v && !wbb1_v && !clean_active;
assign dcache_idle         = !m0_valid && !m1_valid && !wbb0_v && !wbb1_v &&
                             !hit_resp_valid && !fill_resp_valid && !clean_active;

assign cache_hit           = req_is_hit_act;
assign cache_miss          = req_is_new_miss_act;
assign cache_wb            = rf_need_wb;
assign cache_hum           = req_is_hit_act && dcache_any_miss;
assign cache_mum_cyc       = (m0_valid && !m0_refilled) && (m1_valid && !m1_refilled);
assign cache_mshr_coalesce = req_is_coalesce_act && !req_is_write;
assign pmu_silent_e2m      = cpu_hit_writes && (hit_state == COH_E);
assign pmu_upgrade_miss    = req_is_new_miss_act && req_is_upg_act;

wire [3:0] m0_alive_after = w_alive[3:0] & ~((drain_v && !drain_is_m1) ? (4'b0001 << drain_wslot[1:0]) : 4'b0000);
wire [3:0] m1_alive_after = w_alive[7:4] & ~((drain_v &&  drain_is_m1) ? (4'b0001 << drain_wslot[1:0]) : 4'b0000);

wire [IDX_W-1:0] cl_set   = clean_idx[IDX_W-1:0];
wire             cl_d0    = (clean_idx < NUM_SETS[IDX_W:0]) && is_dirty_state(way0_state[cl_set]);
wire             cl_d1    = (clean_idx < NUM_SETS[IDX_W:0]) && is_dirty_state(way1_state[cl_set]);
wire             wbb_has_space = (!wbb0_v || !wbb1_v || (wb_valid && wb_ready));

// ----------------------------------------------------------------------------
// 9. Sequential State Updates (Main + Duplicate Snoop Arrays Kept Bit-Identical)
// ----------------------------------------------------------------------------
integer k;

`ifdef FORMAL
integer fi;
initial begin
    for (fi = 0; fi < NUM_SETS; fi = fi + 1) begin
        way0_state[fi]     = COH_I;
        way0_tag[fi]       = {CTAG_W{1'b0}};
        way0_data[fi]      = 128'd0;
        way1_state[fi]     = COH_I;
        way1_tag[fi]       = {CTAG_W{1'b0}};
        way1_data[fi]      = 128'd0;
        snp_way0_state[fi] = COH_I;
        snp_way0_tag[fi]   = {CTAG_W{1'b0}};
        snp_way1_state[fi] = COH_I;
        snp_way1_tag[fi]   = {CTAG_W{1'b0}};
        lru_arr[fi]        = 1'b0;
    end
    resv_valid       = 1'b0;
    resv_laddr       = {LADDR_W{1'b0}};
    clean_active     = 1'b0;
    clean_idx        = {(IDX_W+1){1'b0}};
    clean_all_done   = 1'b0;
    snoop_inv_valid  = 1'b0;
    snoop_inv_laddr  = {LADDR_W{1'b0}};
    reinv_pend_v     = 1'b0;
    reinv_pend_laddr = {LADDR_W{1'b0}};
    m0_valid = 1'b0; m0_refilled = 1'b0; m0_sent = 1'b0; m0_coh_op = COH_REQ_GETS;
    m0_laddr = {LADDR_W{1'b0}}; m0_st_mask = 16'd0; m0_st_data = 128'd0;
    m0_fill_line = 128'd0; m0_snoop_reinv = 1'b0;
    m0_amo_op = AMO_NONE; m0_amo_woff = 2'd0; m0_amo_wdata = 32'd0; m0_amo_id = {ID_W{1'b0}};
    m1_valid = 1'b0; m1_refilled = 1'b0; m1_sent = 1'b0; m1_coh_op = COH_REQ_GETS;
    m1_laddr = {LADDR_W{1'b0}}; m1_st_mask = 16'd0; m1_st_data = 128'd0;
    m1_fill_line = 128'd0; m1_snoop_reinv = 1'b0;
    m1_amo_op = AMO_NONE; m1_amo_woff = 2'd0; m1_amo_wdata = 32'd0; m1_amo_id = {ID_W{1'b0}};
    for (fi = 0; fi < 8; fi = fi + 1) begin
        w_v[fi]    = 1'b0;
        w_id[fi]   = {ID_W{1'b0}};
        w_tag[fi]  = {TAG_W{1'b0}};
        w_woff[fi] = 2'd0;
    end
    wbb0_v = 1'b0; wbb0_laddr = {LADDR_W{1'b0}}; wbb0_data = 128'd0;
    wbb1_v = 1'b0; wbb1_laddr = {LADDR_W{1'b0}}; wbb1_data = 128'd0;
    hit_resp_valid  = 1'b0; hit_resp_id  = {ID_W{1'b0}}; hit_resp_rdata  = 32'd0;
    fill_resp_valid = 1'b0; fill_resp_id = {ID_W{1'b0}}; fill_resp_rdata = 32'd0;
end

always @(posedge clk) begin
`else
always @(posedge clk or negedge reset_n) begin
`endif
    if (!reset_n) begin
        for (k = 0; k < NUM_SETS; k = k + 1) begin
            way0_state[k]     <= COH_I;
            way0_tag[k]       <= {CTAG_W{1'b0}};
            way1_state[k]     <= COH_I;
            way1_tag[k]       <= {CTAG_W{1'b0}};
            snp_way0_state[k] <= COH_I;
            snp_way0_tag[k]   <= {CTAG_W{1'b0}};
            snp_way1_state[k] <= COH_I;
            snp_way1_tag[k]   <= {CTAG_W{1'b0}};
            lru_arr[k]        <= 1'b0;
`ifdef FORMAL
            way0_data[k]      <= 128'd0;
            way1_data[k]      <= 128'd0;
`endif
        end
        resv_valid       <= 1'b0;
        resv_laddr       <= {LADDR_W{1'b0}};
        clean_active     <= 1'b0;
        clean_idx        <= {(IDX_W+1){1'b0}};
        clean_all_done   <= 1'b0;
        snoop_inv_valid  <= 1'b0;
        snoop_inv_laddr  <= {LADDR_W{1'b0}};
        reinv_pend_v     <= 1'b0;
        reinv_pend_laddr <= {LADDR_W{1'b0}};

        m0_valid <= 1'b0; m0_refilled <= 1'b0; m0_sent <= 1'b0; m0_coh_op <= COH_REQ_GETS;
        m0_laddr <= {LADDR_W{1'b0}}; m0_st_mask <= 16'd0; m0_st_data <= 128'd0;
        m0_fill_line <= 128'd0; m0_snoop_reinv <= 1'b0;
        m0_amo_op <= AMO_NONE; m0_amo_woff <= 2'd0; m0_amo_wdata <= 32'd0; m0_amo_id <= {ID_W{1'b0}};

        m1_valid <= 1'b0; m1_refilled <= 1'b0; m1_sent <= 1'b0; m1_coh_op <= COH_REQ_GETS;
        m1_laddr <= {LADDR_W{1'b0}}; m1_st_mask <= 16'd0; m1_st_data <= 128'd0;
        m1_fill_line <= 128'd0; m1_snoop_reinv <= 1'b0;
        m1_amo_op <= AMO_NONE; m1_amo_woff <= 2'd0; m1_amo_wdata <= 32'd0; m1_amo_id <= {ID_W{1'b0}};

        for (k = 0; k < 8; k = k + 1) begin
            w_v[k]    <= 1'b0;
            w_id[k]   <= {ID_W{1'b0}};
            w_tag[k]  <= {TAG_W{1'b0}};
            w_woff[k] <= 2'd0;
        end
        wbb0_v <= 1'b0; wbb0_laddr <= {LADDR_W{1'b0}}; wbb0_data <= 128'd0;
        wbb1_v <= 1'b0; wbb1_laddr <= {LADDR_W{1'b0}}; wbb1_data <= 128'd0;
        hit_resp_valid  <= 1'b0; hit_resp_id  <= {ID_W{1'b0}}; hit_resp_rdata  <= 32'd0;
        fill_resp_valid <= 1'b0; fill_resp_id <= {ID_W{1'b0}}; fill_resp_rdata <= 32'd0;
    end else begin
        hit_resp_valid  <= 1'b0;
        fill_resp_valid <= 1'b0;
        clean_all_done  <= 1'b0;
        snoop_inv_valid <= 1'b0;
        reinv_pend_v    <= 1'b0;

        // 0. Emit delayed reinvalidation pulse if a snooped-while-draining waiter just returned
        if (reinv_pend_v) begin
            snoop_inv_valid <= 1'b1;
            snoop_inv_laddr <= reinv_pend_laddr;
        end

        // 1. Kill squashed MSHR load waiters immediately
        for (k = 0; k < 8; k = k + 1) begin
            if (w_v[k] && !w_alive[k])
                w_v[k] <= 1'b0;
        end

        // 2. WBB Drain to L2 (via Channel C)
        if (wb_valid && wb_ready) begin
            if (wbb0_eff_v)
                wbb0_v <= 1'b0;
            else if (wbb1_eff_v)
                wbb1_v <= 1'b0;
        end

        // 3. Mark Channel A Coherence Request as Sent
        if (coh_req_valid && coh_req_ready) begin
            if (!coh_req_id)
                m0_sent <= 1'b1;
            else
                m1_sent <= 1'b1;
        end

        // 4. Emit Fill Load Response for 1 Waiter & Release Completed MSHR
        if (drain_v) begin
            fill_resp_valid  <= 1'b1;
            fill_resp_id     <= w_id[drain_wslot];
            fill_resp_rdata  <= get_word(drain_line, w_woff[drain_wslot]);
            w_v[drain_wslot] <= 1'b0;
            if (drain_reinv) begin
                reinv_pend_v     <= 1'b1;
                reinv_pend_laddr <= drain_laddr;
            end
        end

        if (m0_valid && m0_refilled && (m0_alive_after == 4'b0000)) begin
            m0_valid       <= 1'b0;
            m0_refilled    <= 1'b0;
            m0_sent        <= 1'b0;
            m0_st_mask     <= 16'd0;
            m0_amo_op      <= AMO_NONE;
            m0_snoop_reinv <= 1'b0;
        end
        if (m1_valid && m1_refilled && (m1_alive_after == 4'b0000)) begin
            m1_valid       <= 1'b0;
            m1_refilled    <= 1'b0;
            m1_sent        <= 1'b0;
            m1_st_mask     <= 16'd0;
            m1_amo_op      <= AMO_NONE;
            m1_snoop_reinv <= 1'b0;
        end

        // 5. Handle Incoming Snoop Probe (Channel B — Always Non-Blocking!)
        if (snoop_valid) begin
            // 5a. Update BOTH Main and Duplicate Snoop Tag/State Arrays on cache hit
            if (snp_hit_w0) begin
                way0_state[snp_set]     <= snp_next_state;
                snp_way0_state[snp_set] <= snp_next_state;
            end else if (snp_hit_w1) begin
                way1_state[snp_set]     <= snp_next_state;
                snp_way1_state[snp_set] <= snp_next_state;
            end

            // 5b. Cancel WBB entry if peer takes M ownership via GetM or Upgrade!
            if (wbb0_snp_cancel)
                wbb0_v <= 1'b0;
            if (wbb1_snp_cancel)
                wbb1_v <= 1'b0;

            // 5c. On GetM / Upgrade invalidation:
            //     - Pulse snoop_inv_valid to lsq2w (RVWMO Speculative Load Replay!)
            //     - Clear LR/SC reservation if matching snoop_laddr
            //     - Promote unsent SM_AD/OM_AD Upgrade MSHR -> IM_AD (COH_REQ_GETM)!
            //     - Flag any refilled-but-draining MSHR matching snoop_laddr for reinv
            if (snp_is_inv) begin
                if (snp_hit_c || snp_hit_wbb ||
                    (m0_valid && m0_refilled && m0_laddr == snoop_laddr) ||
                    (m1_valid && m1_refilled && m1_laddr == snoop_laddr)) begin
                    snoop_inv_valid <= 1'b1;
                    snoop_inv_laddr <= snoop_laddr;
                end
                if (resv_valid && (resv_laddr == snoop_laddr))
                    resv_valid <= 1'b0;
                if (m0_valid && !m0_sent && !m0_refilled && (m0_laddr == snoop_laddr) && (m0_coh_op == COH_REQ_UPGRADE))
                    m0_coh_op <= COH_REQ_GETM;
                if (m1_valid && !m1_sent && !m1_refilled && (m1_laddr == snoop_laddr) && (m1_coh_op == COH_REQ_UPGRADE))
                    m1_coh_op <= COH_REQ_GETM;
                if (m0_valid && m0_refilled && (m0_laddr == snoop_laddr))
                    m0_snoop_reinv <= 1'b1;
                if (m1_valid && m1_refilled && (m1_laddr == snoop_laddr))
                    m1_snoop_reinv <= 1'b1;
            end
        end

        // 6. FENCE.I Dirty Clean-All Sweep FSM (paused 1 cyc on snoop for 0 race!)
        if (clean_all_req && !clean_active) begin
            clean_active <= 1'b1;
            clean_idx    <= {(IDX_W+1){1'b0}};
        end else if (clean_active && !m0_valid && !m1_valid && !snoop_valid) begin
            if (clean_idx < NUM_SETS[IDX_W:0]) begin
                if (cl_d0) begin
                    if (wbb_has_space) begin
                        // M -> E (clean sole copy), O -> S (clean shared copy)
                        way0_state[cl_set]     <= (way0_state[cl_set] == COH_M) ? COH_E : COH_S;
                        snp_way0_state[cl_set] <= (way0_state[cl_set] == COH_M) ? COH_E : COH_S;
                        if (!wbb0_v || (wb_valid && wb_ready && wbb0_eff_v)) begin
                            wbb0_v     <= 1'b1;
                            wbb0_laddr <= {way0_tag[cl_set], cl_set};
                            wbb0_data  <= way0_data[cl_set];
                        end else begin
                            wbb1_v     <= 1'b1;
                            wbb1_laddr <= {way0_tag[cl_set], cl_set};
                            wbb1_data  <= way0_data[cl_set];
                        end
                    end
                end else if (cl_d1) begin
                    if (wbb_has_space) begin
                        way1_state[cl_set]     <= (way1_state[cl_set] == COH_M) ? COH_E : COH_S;
                        snp_way1_state[cl_set] <= (way1_state[cl_set] == COH_M) ? COH_E : COH_S;
                        if (!wbb0_v || (wb_valid && wb_ready && wbb0_eff_v)) begin
                            wbb0_v     <= 1'b1;
                            wbb0_laddr <= {way1_tag[cl_set], cl_set};
                            wbb0_data  <= way1_data[cl_set];
                        end else begin
                            wbb1_v     <= 1'b1;
                            wbb1_laddr <= {way1_tag[cl_set], cl_set};
                            wbb1_data  <= way1_data[cl_set];
                        end
                    end
                end else begin
                    clean_idx <= clean_idx + {{IDX_W{1'b0}}, 1'b1};
                end
            end else if (!wbb0_v && !wbb1_v) begin
                clean_active   <= 1'b0;
                clean_all_done <= 1'b1;
            end
        end

        // 7. Handle Incoming Channel D Refill or Upgrade ACK
        if (do_rf_now) begin
            if (!rf_dst_way) begin
                way0_state[rf_set]     <= rf_new_state;
                way0_tag[rf_set]       <= rf_ctag;
                way0_data[rf_set]      <= rf_merged;
                snp_way0_state[rf_set] <= rf_new_state;
                snp_way0_tag[rf_set]   <= rf_ctag;
                lru_arr[rf_set]        <= 1'b1;
            end else begin
                way1_state[rf_set]     <= rf_new_state;
                way1_tag[rf_set]       <= rf_ctag;
                way1_data[rf_set]      <= rf_merged;
                snp_way1_state[rf_set] <= rf_new_state;
                snp_way1_tag[rf_set]   <= rf_ctag;
                lru_arr[rf_set]        <= 1'b0;
            end

            // Evicting a reserved line invalidates the local LR/SC reservation
            if (!rf_is_upg && (rf_vic_state != COH_I) && resv_valid && (rf_vic_laddr == resv_laddr))
                resv_valid <= 1'b0;

            // Push dirty victim (M or O) into WBB unless snoop already took ownership
            if (rf_need_wb) begin
                if (!wbb0_v || (wb_valid && wb_ready && wbb0_eff_v) || wbb0_snp_cancel) begin
                    wbb0_v     <= 1'b1;
                    wbb0_laddr <= rf_vic_laddr;
                    wbb0_data  <= rf_vic_data;
                end else begin
                    wbb1_v     <= 1'b1;
                    wbb1_laddr <= rf_vic_laddr;
                    wbb1_data  <= rf_vic_data;
                end
            end

            // Atomic refill response & reservation update
            if (rf_is_amo) begin
                fill_resp_valid <= 1'b1;
                fill_resp_id    <= rf_amo_id;
                fill_resp_rdata <= rf_amo_resp;
                if (rf_is_lr) begin
                    resv_valid <= 1'b1;
                    resv_laddr <= rf_laddr;
                end else if (rf_is_sc || (resv_valid && (resv_laddr == rf_laddr))) begin
                    resv_valid <= 1'b0;
                end
            end

            if (m0_rf_now) begin
                m0_fill_line <= rf_merged;
                m0_st_mask   <= 16'd0;
                m0_amo_op    <= AMO_NONE;
                if (rf_is_amo || (m0_alive_after == 4'b0000)) begin
                    m0_valid       <= 1'b0;
                    m0_refilled    <= 1'b0;
                    m0_sent        <= 1'b0;
                    m0_snoop_reinv <= 1'b0;
                end else begin
                    m0_refilled    <= 1'b1;
                    m0_snoop_reinv <= 1'b0;
                end
            end else begin
                m1_fill_line <= rf_merged;
                m1_st_mask   <= 16'd0;
                m1_amo_op    <= AMO_NONE;
                if (rf_is_amo || (m1_alive_after == 4'b0000)) begin
                    m1_valid       <= 1'b0;
                    m1_refilled    <= 1'b0;
                    m1_sent        <= 1'b0;
                    m1_snoop_reinv <= 1'b0;
                end else begin
                    m1_refilled    <= 1'b1;
                    m1_snoop_reinv <= 1'b0;
                end
            end
        end else if (req_valid && req_ready) begin
            // 8. Serve Accepted CPU Request (1-Cycle Hit / Coalesce / MSHR Alloc)
            if (req_is_sc_fail_act) begin
                resv_valid     <= 1'b0;
                hit_resp_valid <= 1'b1;
                hit_resp_id    <= req_id;
                hit_resp_rdata <= 32'd1; // SC.W failed locally
            end else if (req_is_hit_act) begin
                lru_arr[req_set] <= hit_w0 ? 1'b1 : 1'b0;
                if (req_is_amo) begin
                    if (req_is_lr) begin
                        resv_valid     <= 1'b1;
                        resv_laddr     <= req_laddr;
                        hit_resp_valid <= 1'b1;
                        hit_resp_id    <= req_id;
                        hit_resp_rdata <= hit_old_word;
                    end else begin
                        // SC.W (resv match) or AMO*.W RMW hit in E or M -> silent E->M!
                        if (hit_w0) begin
                            way0_data[req_set]      <= hit_new_line;
                            way0_state[req_set]     <= COH_M;
                            snp_way0_state[req_set] <= COH_M;
                        end else begin
                            way1_data[req_set]      <= hit_new_line;
                            way1_state[req_set]     <= COH_M;
                            snp_way1_state[req_set] <= COH_M;
                        end
                        if (req_is_sc || (resv_valid && (resv_laddr == req_laddr)))
                            resv_valid <= 1'b0;
                        hit_resp_valid <= 1'b1;
                        hit_resp_id    <= req_id;
                        hit_resp_rdata <= req_is_sc ? 32'd0 : hit_old_word;
                    end
                end else if (req_is_write) begin
                    // Store hit in E or M -> silent E->M upgrade!
                    if (hit_w0) begin
                        way0_data[req_set] <= hit_new_line;
                        if (|req_wstrb) begin
                            way0_state[req_set]     <= COH_M;
                            snp_way0_state[req_set] <= COH_M;
                        end
                    end else begin
                        way1_data[req_set] <= hit_new_line;
                        if (|req_wstrb) begin
                            way1_state[req_set]     <= COH_M;
                            snp_way1_state[req_set] <= COH_M;
                        end
                    end
                    if (resv_valid && (resv_laddr == req_laddr))
                        resv_valid <= 1'b0;
                end else begin
                    // Load hit in S, E, O, or M
                    hit_resp_valid <= 1'b1;
                    hit_resp_id    <= req_id;
                    hit_resp_rdata <= hit_old_word;
                end
            end else if (req_is_coalesce_act) begin
                if (m0_match) begin
                    if (req_is_write) begin
                        m0_st_mask <= m0_st_mask | expand_line_mask(req_woff, req_wstrb);
                        m0_st_data <= merge_word(m0_st_data, req_woff, req_wdata, req_wstrb);
                        if (resv_valid && (resv_laddr == req_laddr))
                            resv_valid <= 1'b0;
                    end else begin
                        w_v[{1'b0, m0_free_widx}]    <= 1'b1;
                        w_id[{1'b0, m0_free_widx}]   <= req_id;
                        w_tag[{1'b0, m0_free_widx}]  <= req_rob_tag;
                        w_woff[{1'b0, m0_free_widx}] <= req_woff;
                    end
                end else begin
                    if (req_is_write) begin
                        m1_st_mask <= m1_st_mask | expand_line_mask(req_woff, req_wstrb);
                        m1_st_data <= merge_word(m1_st_data, req_woff, req_wdata, req_wstrb);
                        if (resv_valid && (resv_laddr == req_laddr))
                            resv_valid <= 1'b0;
                    end else begin
                        w_v[{1'b1, m1_free_widx}]    <= 1'b1;
                        w_id[{1'b1, m1_free_widx}]   <= req_id;
                        w_tag[{1'b1, m1_free_widx}]  <= req_rob_tag;
                        w_woff[{1'b1, m1_free_widx}] <= req_woff;
                    end
                end
            end else if (req_is_new_miss_act) begin
                if (!m0_valid) begin
                    m0_valid       <= 1'b1;
                    m0_refilled    <= 1'b0;
                    m0_sent        <= 1'b0;
                    m0_snoop_reinv <= 1'b0;
                    m0_laddr       <= req_laddr;
                    m0_coh_op      <= req_is_upg_act    ? COH_REQ_UPGRADE :
                                      req_needs_wr_perm ? COH_REQ_GETM    : COH_REQ_GETS;
                    if (req_is_amo) begin
                        m0_st_mask   <= 16'd0;
                        m0_st_data   <= 128'd0;
                        m0_amo_op    <= req_amo_op;
                        m0_amo_woff  <= req_woff;
                        m0_amo_wdata <= req_wdata;
                        m0_amo_id    <= req_id;
                    end else if (req_is_write) begin
                        m0_st_mask <= expand_line_mask(req_woff, req_wstrb);
                        m0_st_data <= merge_word(128'd0, req_woff, req_wdata, req_wstrb);
                        m0_amo_op  <= AMO_NONE;
                        if (resv_valid && (resv_laddr == req_laddr))
                            resv_valid <= 1'b0;
                    end else begin
                        m0_st_mask <= 16'd0;
                        m0_st_data <= 128'd0;
                        m0_amo_op  <= AMO_NONE;
                        w_v[0]     <= 1'b1;
                        w_id[0]    <= req_id;
                        w_tag[0]   <= req_rob_tag;
                        w_woff[0]  <= req_woff;
                    end
                end else begin
                    m1_valid       <= 1'b1;
                    m1_refilled    <= 1'b0;
                    m1_sent        <= 1'b0;
                    m1_snoop_reinv <= 1'b0;
                    m1_laddr       <= req_laddr;
                    m1_coh_op      <= req_is_upg_act    ? COH_REQ_UPGRADE :
                                      req_needs_wr_perm ? COH_REQ_GETM    : COH_REQ_GETS;
                    if (req_is_amo) begin
                        m1_st_mask   <= 16'd0;
                        m1_st_data   <= 128'd0;
                        m1_amo_op    <= req_amo_op;
                        m1_amo_woff  <= req_woff;
                        m1_amo_wdata <= req_wdata;
                        m1_amo_id    <= req_id;
                    end else if (req_is_write) begin
                        m1_st_mask <= expand_line_mask(req_woff, req_wstrb);
                        m1_st_data <= merge_word(128'd0, req_woff, req_wdata, req_wstrb);
                        m1_amo_op  <= AMO_NONE;
                        if (resv_valid && (resv_laddr == req_laddr))
                            resv_valid <= 1'b0;
                    end else begin
                        m1_st_mask <= 16'd0;
                        m1_st_data <= 128'd0;
                        m1_amo_op  <= AMO_NONE;
                        w_v[4]     <= 1'b1;
                        w_id[4]    <= req_id;
                        w_tag[4]   <= req_rob_tag;
                        w_woff[4]  <= req_woff;
                    end
                end
            end
        end
    end
end

`ifdef FORMAL
genvar fs;
generate
    for (fs = 0; fs < NUM_SETS; fs = fs + 1) begin : gen_fobs
        assign f_way0_state[fs*3 +: 3]           = way0_state[fs];
        assign f_way0_tag[fs*CTAG_W +: CTAG_W]   = way0_tag[fs];
        assign f_way0_b0[fs*8 +: 8]              = way0_data[fs][7:0];
        assign f_way1_state[fs*3 +: 3]           = way1_state[fs];
        assign f_way1_tag[fs*CTAG_W +: CTAG_W]   = way1_tag[fs];
        assign f_way1_b0[fs*8 +: 8]              = way1_data[fs][7:0];
        assign f_snp_way0_state[fs*3 +: 3]       = snp_way0_state[fs];
        assign f_snp_way0_tag[fs*CTAG_W +: CTAG_W] = snp_way0_tag[fs];
        assign f_snp_way1_state[fs*3 +: 3]       = snp_way1_state[fs];
        assign f_snp_way1_tag[fs*CTAG_W +: CTAG_W] = snp_way1_tag[fs];
    end
endgenerate
assign f_wbb0_v     = wbb0_v;
assign f_wbb0_laddr = wbb0_laddr;
assign f_wbb0_b0    = wbb0_data[7:0];
assign f_wbb1_v     = wbb1_v;
assign f_wbb1_laddr = wbb1_laddr;
assign f_wbb1_b0    = wbb1_data[7:0];
assign f_rf_laddr   = rf_laddr;
`endif

endmodule
