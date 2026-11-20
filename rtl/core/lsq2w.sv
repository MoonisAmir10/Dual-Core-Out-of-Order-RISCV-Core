`timescale 1ns / 1ps

// ============================================================================
// 8-Entry Non-Blocking LQ + 8-Entry 3-Pointer SQ Speculative Load/Store Queue
// (Phase 1 Upgrade: Unified AWIDTH=20, Ordering Barrier Gate, ROB-Head RV32A
//  Atomic Execution Path, and Speculative Load Snoop-Invalidation Replay!)
// ============================================================================
module lsq2w #(
    parameter integer LQ_DEPTH = 8,
    parameter integer SQ_DEPTH = 8,
    parameter integer TAG_W    = 5,
    parameter integer ID_W     = 7,  // bit[6]=1: ROB-head Atomic; bit[6]=0: {lq_gen[2:0], lq_idx[2:0]}
    parameter integer AWIDTH   = 20  // 1 MB physical window
) (
    input  wire                 clk,
    input  wire                 reset_n,
    input  wire                 flush,
    input  wire                 squash_valid,
    input  wire [TAG_W-1:0]     squash_tag,
    input  wire [TAG_W-1:0]     rob_head,

    // Ordering Barrier Gate from ROB (blocks younger speculative loads past FENCE/AMO)
    input  wire                 barrier_valid,
    input  wire [TAG_W-1:0]     barrier_tag,

    // Single memory dispatch port
    input  wire                 disp_valid,
    input  wire                 disp_is_store,
    output wire                 disp_ready_load,
    output wire                 disp_ready_store,
    input  wire [TAG_W-1:0]     disp_rob_tag,
    input  wire [31:0]          disp_seq_id,
    input  wire [1:0]           disp_mem_size,
    input  wire                 disp_mem_unsigned,
    input  wire [31:0]          disp_imm,
    input  wire [31:0]          disp_vj,
    input  wire [TAG_W-1:0]     disp_qj,
    input  wire                 disp_qj_valid,
    input  wire [31:0]          disp_vk,
    input  wire [TAG_W-1:0]     disp_qk,
    input  wire                 disp_qk_valid,

    // ROB-Head Non-Speculative RV32A Atomic Request (LR.W / SC.W / AMO*.W)
    input  wire                 amo_req_valid,
    input  wire [3:0]           amo_req_op,
    input  wire [TAG_W-1:0]     amo_req_tag,
    input  wire [31:0]          amo_req_seq,
    input  wire [31:0]          amo_req_addr,
    input  wire [31:0]          amo_req_rs2,
    input  wire                 dc_writes_idle,

    // Dual-CDB wakeup snoop
    input  wire                 cdb0_valid,
    input  wire [TAG_W-1:0]     cdb0_tag,
    input  wire [31:0]          cdb0_val,
    input  wire                 cdb1_valid,
    input  wire [TAG_W-1:0]     cdb1_tag,
    input  wire [31:0]          cdb1_val,

    // 1-Cycle Speculative Issue-Stage Tag Wakeup + Registered EX->EX Bypass Data
    input  wire                 wk0_valid,
    input  wire [TAG_W-1:0]     wk0_tag,
    input  wire                 wk1_valid,
    input  wire [TAG_W-1:0]     wk1_tag,
    input  wire [31:0]          byp_alu0_val,
    input  wire [31:0]          byp_alu1_val,

    // ROB Commit Notifications (Store drain + Load retirement for Snoop-Replay LQ lifetime!)
    input  wire                 rob_commit_store,
    input  wire                 rob_commit0_load,
    input  wire [TAG_W-1:0]     rob_commit0_tag,
    input  wire                 rob_commit1_load,
    input  wire [TAG_W-1:0]     rob_commit1_tag,

    // External Coherence Invalidation Snoop -> Speculative Load Replay Detection!
    input  wire                 coh_snoop_inv_valid,
    input  wire [AWIDTH-5:0]    coh_snoop_inv_line,
    output reg                  lq_head0_replay,
    output reg                  lq_head1_replay,

    // LSU completion output -> CDB0 Arbiter Port 1
    output reg                  lsu_cdb_valid,
    input  wire                 lsu_cdb_grant,
    output reg  [TAG_W-1:0]     lsu_cdb_tag,
    output reg  [31:0]          lsu_cdb_seq,
    output reg  [31:0]          lsu_cdb_val,
    output reg                  lsu_cdb_mem_we,
    output reg  [31:0]          lsu_cdb_mem_addr,
    output reg  [3:0]           lsu_cdb_mem_wstrb,
    output reg  [31:0]          lsu_cdb_mem_wdata,

    // Split-Transaction Non-Blocking D-Cache Request Channel
    output wire                 dc_req_valid,
    input  wire                 dc_req_ready,
    output wire                 dc_req_is_write,
    output wire                 dc_req_is_mmio,
    output wire [3:0]           dc_req_amo_op,
    output wire [AWIDTH-1:0]    dc_req_addr,
    output wire [31:0]          dc_req_wdata,
    output wire [3:0]           dc_req_wstrb,
    output wire [ID_W-1:0]      dc_req_id,
    output wire [TAG_W-1:0]     dc_req_rob_tag,

    // Non-Blocking D-Cache Dual Response Channels (1-cyc Hit + MSHR Fill)
    input  wire                 dc_hit_valid,
    input  wire [ID_W-1:0]      dc_hit_id,
    input  wire [31:0]          dc_hit_rdata,

    input  wire                 dc_fill_valid,
    input  wire [ID_W-1:0]      dc_fill_id,
    input  wire [31:0]          dc_fill_rdata,

    // Status & telemetry
    output wire                 sq_empty,
    output wire                 trace_issue_valid,
    output wire [31:0]          trace_issue_seq,
    output wire                 stat_stlf_hit,
    output wire [1:0]           stat_ex_byp_cnt
);

`include "rv32ima_defs.vh"

localparam integer LQ_IDX_W = 3; // 8 entries
localparam integer SQ_IDX_W = 3; // 8 entries
localparam [1:0] BYP_NONE = 2'b00;
localparam [1:0] BYP_ALU0 = 2'b01;
localparam [1:0] BYP_ALU1 = 2'b10;

function automatic [3:0] calc_byte_mask(input [1:0] sz, input [1:0] lane);
    begin
        case (sz)
            MEM_BYTE: calc_byte_mask = 4'b0001 << lane;
            MEM_HALF: calc_byte_mask = lane[1] ? 4'b1100 : 4'b0011;
            default:  calc_byte_mask = 4'b1111;
        endcase
    end
endfunction

function automatic [31:0] align_store_data(input [1:0] sz, input [1:0] lane, input [31:0] raw);
    begin
        case (sz)
            MEM_BYTE: align_store_data = {24'd0, raw[7:0]} << (lane * 8);
            MEM_HALF: align_store_data = lane[1] ? {raw[15:0], 16'd0} : {16'd0, raw[15:0]};
            default:  align_store_data = raw;
        endcase
    end
endfunction

function automatic [31:0] format_load_data(
    input [1:0]  sz,
    input        uns,
    input [1:0]  lane,
    input [31:0] word_data
);
    reg [7:0]  bval;
    reg [15:0] hval;
    begin
        case (lane)
            2'b00: bval = word_data[7:0];
            2'b01: bval = word_data[15:8];
            2'b10: bval = word_data[23:16];
            2'b11: bval = word_data[31:24];
        endcase
        hval = lane[1] ? word_data[31:16] : word_data[15:0];
        case (sz)
            MEM_BYTE: format_load_data = uns ? {24'd0, bval} : {{24{bval[7]}}, bval};
            MEM_HALF: format_load_data = uns ? {16'd0, hval} : {{16{hval[15]}}, hval};
            default:  format_load_data = word_data;
        endcase
    end
endfunction

function automatic [31:0] amo_alu_rvfi(
    input [3:0]  op,
    input [31:0] old_val,
    input [31:0] operand
);
    begin
        case (op)
            AMO_SWAP: amo_alu_rvfi = operand;
            AMO_ADD:  amo_alu_rvfi = old_val + operand;
            AMO_XOR:  amo_alu_rvfi = old_val ^ operand;
            AMO_AND:  amo_alu_rvfi = old_val & operand;
            AMO_OR:   amo_alu_rvfi = old_val | operand;
            AMO_MIN:  amo_alu_rvfi = ($signed(old_val) < $signed(operand)) ? old_val : operand;
            AMO_MAX:  amo_alu_rvfi = ($signed(old_val) > $signed(operand)) ? old_val : operand;
            AMO_MINU: amo_alu_rvfi = (old_val < operand) ? old_val : operand;
            AMO_MAXU: amo_alu_rvfi = (old_val > operand) ? old_val : operand;
            default:  amo_alu_rvfi = operand;
        endcase
    end
endfunction

// ----------------------------------------------------------------------------
// 1. Store Queue (8-entry 3-pointer circular buffer)
// ----------------------------------------------------------------------------
reg [3:0] sq_head;
reg [3:0] sq_commit_ptr;
reg [3:0] sq_tail;

wire [3:0] sq_occ          = sq_tail - sq_head;
wire       sq_full         = (sq_occ == 4'd8);
wire       sq_has_comm_drn = (sq_commit_ptr != sq_head);
// True when ALL committed stores (older than ROB head) have drained into D$!
// Younger speculative stores in [sq_commit_ptr..sq_tail) must NOT block ROB-head FENCE/AMO/EBREAK.
assign     sq_empty        = !sq_has_comm_drn;

reg [TAG_W-1:0] sq_rob_tag    [0:SQ_DEPTH-1];
reg [31:0]      sq_seq_id     [0:SQ_DEPTH-1];
reg [1:0]       sq_size       [0:SQ_DEPTH-1];
reg [31:0]      sq_imm        [0:SQ_DEPTH-1];
reg [TAG_W-1:0] sq_qj         [0:SQ_DEPTH-1];
reg             sq_qj_valid   [0:SQ_DEPTH-1];
reg [1:0]       sq_qj_byp     [0:SQ_DEPTH-1];
reg [31:0]      sq_addr       [0:SQ_DEPTH-1];
reg             sq_addr_valid [0:SQ_DEPTH-1];
reg [31:0]      sq_vk         [0:SQ_DEPTH-1];
reg [TAG_W-1:0] sq_qk         [0:SQ_DEPTH-1];
reg             sq_qk_valid   [0:SQ_DEPTH-1];
reg [1:0]       sq_qk_byp     [0:SQ_DEPTH-1];
reg             sq_cdb_sent   [0:SQ_DEPTH-1];

// ----------------------------------------------------------------------------
// 2. Load Queue (8-entry non-blocking pool retained until ROB commit!)
// ----------------------------------------------------------------------------
reg             lq_valid      [0:LQ_DEPTH-1];
reg [2:0]       lq_gen        [0:LQ_DEPTH-1];
reg [TAG_W-1:0] lq_rob_tag    [0:LQ_DEPTH-1];
reg [31:0]      lq_seq_id     [0:LQ_DEPTH-1];
reg [1:0]       lq_size       [0:LQ_DEPTH-1];
reg             lq_uns        [0:LQ_DEPTH-1];
reg [31:0]      lq_imm        [0:LQ_DEPTH-1];
reg [TAG_W-1:0] lq_qj         [0:LQ_DEPTH-1];
reg             lq_qj_valid   [0:LQ_DEPTH-1];
reg [1:0]       lq_qj_byp     [0:LQ_DEPTH-1];
reg [31:0]      lq_addr       [0:LQ_DEPTH-1];
reg             lq_addr_valid [0:LQ_DEPTH-1];
reg [3:0]       lq_sq_snap    [0:LQ_DEPTH-1];
reg             lq_issued     [0:LQ_DEPTH-1];
reg             lq_done       [0:LQ_DEPTH-1];
reg             lq_cdb_sent   [0:LQ_DEPTH-1]; // Retained until ROB commit for snoop-replay!
reg             lq_from_stlf  [0:LQ_DEPTH-1]; // STLF reads local store -> immune to ext snoop!
reg             lq_replay     [0:LQ_DEPTH-1]; // Poisoned by external snoop invalidation!
reg [31:0]      lq_data       [0:LQ_DEPTH-1];

// ROB-Head Non-Speculative Atomic Execution Tracker
reg             amo_in_flight;
reg             amo_done;
reg [3:0]       amo_op_r;
reg [TAG_W-1:0] amo_tag_r;
reg [31:0]      amo_seq_r;
reg [31:0]      amo_addr_r;
reg [31:0]      amo_rs2_r;
reg [31:0]      amo_rdata_r;

// Live EX->EX bypassed addresses & store data
reg [31:0] lq_addr_eff [0:LQ_DEPTH-1];
reg [31:0] sq_addr_eff [0:SQ_DEPTH-1];
reg [31:0] sq_vk_eff   [0:SQ_DEPTH-1];
integer bi;

always @(*) begin
    bi = 0;
    for (bi = 0; bi < LQ_DEPTH; bi = bi + 1) begin
        if (lq_qj_byp[bi] == BYP_ALU0)
            lq_addr_eff[bi] = byp_alu0_val + lq_imm[bi];
        else if (lq_qj_byp[bi] == BYP_ALU1)
            lq_addr_eff[bi] = byp_alu1_val + lq_imm[bi];
        else
            lq_addr_eff[bi] = lq_addr[bi];
    end
    for (bi = 0; bi < SQ_DEPTH; bi = bi + 1) begin
        if (sq_qj_byp[bi] == BYP_ALU0)
            sq_addr_eff[bi] = byp_alu0_val + sq_imm[bi];
        else if (sq_qj_byp[bi] == BYP_ALU1)
            sq_addr_eff[bi] = byp_alu1_val + sq_imm[bi];
        else
            sq_addr_eff[bi] = sq_addr[bi];

        if (sq_qk_byp[bi] == BYP_ALU0)
            sq_vk_eff[bi] = byp_alu0_val;
        else if (sq_qk_byp[bi] == BYP_ALU1)
            sq_vk_eff[bi] = byp_alu1_val;
        else
            sq_vk_eff[bi] = sq_vk[bi];
    end
end

reg                lq_has_free;
reg [LQ_IDX_W-1:0] lq_free_idx;
integer fi;
always @(*) begin
    fi          = 0;
    lq_has_free = 1'b0;
    lq_free_idx = 3'd0;
    for (fi = LQ_DEPTH - 1; fi >= 0; fi = fi - 1) begin
        if (!lq_valid[fi]) begin
            lq_has_free = 1'b1;
            lq_free_idx = fi[LQ_IDX_W-1:0];
        end
    end
end

wire [TAG_W-1:0] sq_br_age   = squash_tag - rob_head;
wire [TAG_W-1:0] bar_age     = barrier_tag - rob_head;
wire [TAG_W-1:0] rob_head1   = rob_head + {{(TAG_W-1){1'b0}}, 1'b1};

assign disp_ready_load  = !flush && !squash_valid && lq_has_free;
assign disp_ready_store = !flush && !squash_valid && !sq_full;

// Same-cycle Dual-CDB & Early Issue-Wakeup check on incoming dispatch operands
wire        dj_c0      = disp_qj_valid && cdb0_valid && (disp_qj == cdb0_tag);
wire        dj_c1      = disp_qj_valid && !dj_c0 && cdb1_valid && (disp_qj == cdb1_tag);
wire        dj_w0      = disp_qj_valid && !dj_c0 && !dj_c1 && wk0_valid && (disp_qj == wk0_tag);
wire        dj_w1      = disp_qj_valid && !dj_c0 && !dj_c1 && !dj_w0 && wk1_valid && (disp_qj == wk1_tag);
wire [31:0] disp_vj_eff   = dj_c0 ? cdb0_val : (dj_c1 ? cdb1_val : disp_vj);
wire        disp_qj_v_eff = (dj_c0 || dj_c1 || dj_w0 || dj_w1) ? 1'b0 : disp_qj_valid;
wire [1:0]  disp_qjbyp_e  = dj_w0 ? BYP_ALU0 : (dj_w1 ? BYP_ALU1 : BYP_NONE);

wire        dk_c0      = disp_qk_valid && cdb0_valid && (disp_qk == cdb0_tag);
wire        dk_c1      = disp_qk_valid && !dk_c0 && cdb1_valid && (disp_qk == cdb1_tag);
wire        dk_w0      = disp_qk_valid && !dk_c0 && !dk_c1 && wk0_valid && (disp_qk == wk0_tag);
wire        dk_w1      = disp_qk_valid && !dk_c0 && !dk_c1 && !dk_w0 && wk1_valid && (disp_qk == wk1_tag);
wire [31:0] disp_vk_eff   = dk_c0 ? cdb0_val : (dk_c1 ? cdb1_val : disp_vk);
wire        disp_qk_v_eff = (dk_c0 || dk_c1 || dk_w0 || dk_w1) ? 1'b0 : disp_qk_valid;
wire [1:0]  disp_qkbyp_e  = dk_w0 ? BYP_ALU0 : (dk_w1 ? BYP_ALU1 : BYP_NONE);

wire [31:0] disp_addr_eff = disp_vj_eff + disp_imm;

wire do_disp_load  = disp_valid && !disp_is_store && disp_ready_load;
wire do_disp_store = disp_valid &&  disp_is_store && disp_ready_store;

// ----------------------------------------------------------------------------
// 2b. Combinational Speculative Load Snoop-Invalidation Replay Detection!
// ----------------------------------------------------------------------------
wire        dc_hit_is_amo  = dc_hit_valid  && dc_hit_id[ID_W-1];
wire        dc_fill_is_amo = dc_fill_valid && dc_fill_id[ID_W-1];
wire [2:0]  hit_idx        = dc_hit_id[2:0];
wire [2:0]  hit_gen        = dc_hit_id[5:3];
wire [2:0]  fill_idx       = dc_fill_id[2:0];
wire [2:0]  fill_gen       = dc_fill_id[5:3];

reg [LQ_DEPTH-1:0] lq_snoop_hit_now;
integer sni;
always @(*) begin
    sni              = 0;
    lq_snoop_hit_now = {LQ_DEPTH{1'b0}};
    lq_head0_replay  = 1'b0;
    lq_head1_replay  = 1'b0;
    for (sni = 0; sni < LQ_DEPTH; sni = sni + 1) begin
        if (lq_valid[sni] && !lq_from_stlf[sni]) begin
            // Detect snoop match on any SPECULATIVE load (lq_rob_tag != rob_head) that has
            // received data (or is receiving right now). Exempting an unpoisoned load that is
            // already at rob_head guarantees forward progress (zero false-sharing livelock under
            // tight peer AMOSWAP loops) while preserving 100% RVWMO/CoRR replay on all speculative loads!
            if (coh_snoop_inv_valid &&
                (lq_rob_tag[sni] != rob_head) &&
                (lq_addr_eff[sni][AWIDTH-1:4] == coh_snoop_inv_line) &&
                (lq_done[sni] || lq_cdb_sent[sni] ||
                 (lq_issued[sni] && dc_hit_valid  && !dc_hit_is_amo  && (hit_idx  == sni[2:0]) && (hit_gen  == lq_gen[sni])) ||
                 (lq_issued[sni] && dc_fill_valid && !dc_fill_is_amo && (fill_idx == sni[2:0]) && (fill_gen == lq_gen[sni])))) begin
                lq_snoop_hit_now[sni] = 1'b1;
            end
            if (lq_replay[sni] || lq_snoop_hit_now[sni]) begin
                if (lq_rob_tag[sni] == rob_head)
                    lq_head0_replay = 1'b1;
                if (lq_rob_tag[sni] == rob_head1)
                    lq_head1_replay = 1'b1;
            end
        end
    end
end

// ----------------------------------------------------------------------------
// 3. Conservative Disambiguation, Barrier Gate & 1-Cycle STLF per LQ slot
// ----------------------------------------------------------------------------
reg        lq_can_stlf  [0:LQ_DEPTH-1];
reg [31:0] lq_stlf_data [0:LQ_DEPTH-1];
reg        lq_can_cache [0:LQ_DEPTH-1];

integer    li1, si1;
reg [3:0]  older_cnt, s_ptr;
reg [2:0]  s_idx;
reg [3:0]  l_rmask, s_wstrb;
reg        l_blocked, l_stlf_hit;
reg [31:0] l_stlf_word;

always @(*) begin
    li1         = 0;
    si1         = 0;
    older_cnt   = 4'd0;
    s_ptr       = 4'd0;
    s_idx       = 3'd0;
    l_rmask     = 4'd0;
    s_wstrb     = 4'd0;
    l_blocked   = 1'b0;
    l_stlf_hit  = 1'b0;
    l_stlf_word = 32'd0;

    for (li1 = 0; li1 < LQ_DEPTH; li1 = li1 + 1) begin
        lq_can_stlf[li1]  = 1'b0;
        lq_stlf_data[li1] = 32'd0;
        lq_can_cache[li1] = 1'b0;

        if (lq_valid[li1] && lq_addr_valid[li1] && !lq_issued[li1] && !lq_done[li1] && !lq_cdb_sent[li1] &&
            !(squash_valid && ((lq_rob_tag[li1] - rob_head) > sq_br_age)) &&
            !(barrier_valid && ((lq_rob_tag[li1] - rob_head) > bar_age))) begin
            older_cnt   = lq_sq_snap[li1] - sq_head;
            l_rmask     = calc_byte_mask(lq_size[li1], lq_addr_eff[li1][1:0]);
            l_blocked   = 1'b0;
            l_stlf_hit  = 1'b0;
            l_stlf_word = 32'd0;

            for (si1 = 0; si1 < SQ_DEPTH; si1 = si1 + 1) begin
                if (si1[3:0] < older_cnt) begin
                    s_ptr = sq_head + si1[3:0];
                    s_idx = s_ptr[2:0];
                    if (!sq_addr_valid[s_idx]) begin
                        l_blocked = 1'b1;
                    end else if (sq_addr_eff[s_idx][31:2] == lq_addr_eff[li1][31:2]) begin
                        s_wstrb = calc_byte_mask(sq_size[s_idx], sq_addr_eff[s_idx][1:0]);
                        if ((s_wstrb & l_rmask) != 4'b0000) begin
                            if ((s_wstrb & l_rmask) == l_rmask) begin
                                if (!sq_qk_valid[s_idx]) begin
                                    l_stlf_hit  = 1'b1;
                                    l_stlf_word = align_store_data(sq_size[s_idx], sq_addr_eff[s_idx][1:0], sq_vk_eff[s_idx]);
                                    l_blocked   = 1'b0;
                                end else begin
                                    l_stlf_hit  = 1'b0;
                                    l_blocked   = 1'b1;
                                end
                            end else begin
                                l_stlf_hit = 1'b0;
                                l_blocked  = 1'b1;
                            end
                        end
                    end
                end
            end

            if (!l_blocked) begin
                if (lq_addr_eff[li1][31:24] == 8'h02) begin
                    // Uncached CLINT MMIO load: strictly non-speculative at ROB head after SQ drain!
                    if ((lq_rob_tag[li1] == rob_head) && !sq_has_comm_drn && dc_writes_idle)
                        lq_can_cache[li1] = 1'b1;
                end else if (l_stlf_hit) begin
                    lq_can_stlf[li1]  = 1'b1;
                    lq_stlf_data[li1] = format_load_data(lq_size[li1], lq_uns[li1], lq_addr_eff[li1][1:0], l_stlf_word);
                end else begin
                    lq_can_cache[li1] = 1'b1;
                end
            end
        end
    end
end

// Pick oldest STLF-ready load and oldest Cache-ready load
reg                stlf_pick_v, cache_ld_pick_v;
reg [LQ_IDX_W-1:0] stlf_pick_idx, cache_ld_pick_idx;
reg [TAG_W-1:0]    stlf_best_age, cache_ld_best_age, tmp_age;
integer            li2;

always @(*) begin
    li2               = 0;
    stlf_pick_v       = 1'b0;
    stlf_pick_idx     = 3'd0;
    stlf_best_age     = {TAG_W{1'b1}};
    cache_ld_pick_v   = 1'b0;
    cache_ld_pick_idx = 3'd0;
    cache_ld_best_age = {TAG_W{1'b1}};
    tmp_age           = {TAG_W{1'b0}};

    for (li2 = 0; li2 < LQ_DEPTH; li2 = li2 + 1) begin
        tmp_age = lq_rob_tag[li2] - rob_head;
        if (lq_can_stlf[li2]) begin
            if (!stlf_pick_v || (tmp_age < stlf_best_age)) begin
                stlf_pick_v   = 1'b1;
                stlf_pick_idx = li2[LQ_IDX_W-1:0];
                stlf_best_age = tmp_age;
            end
        end
        if (lq_can_cache[li2]) begin
            if (!cache_ld_pick_v || (tmp_age < cache_ld_best_age)) begin
                cache_ld_pick_v   = 1'b1;
                cache_ld_pick_idx = li2[LQ_IDX_W-1:0];
                cache_ld_best_age = tmp_age;
            end
        end
    end
end

// ----------------------------------------------------------------------------
// 4. Split-Transaction D-Cache Request Port (Store Drain > ROB-Head AMO > Load)
// ----------------------------------------------------------------------------
wire [2:0] sq_head_idx = sq_head[2:0];
wire [2:0] sq_tail_idx = sq_tail[2:0];

wire dc_start_store = sq_has_comm_drn;
wire dc_start_amo   = !sq_has_comm_drn && sq_empty && dc_writes_idle &&
                      amo_req_valid && !amo_in_flight && !amo_done && !flush;
wire dc_start_load  = !sq_has_comm_drn && !dc_start_amo && !amo_in_flight &&
                      cache_ld_pick_v && !flush && !squash_valid;

assign dc_req_valid    = dc_start_store || dc_start_amo || dc_start_load;
assign dc_req_is_write = dc_start_store || (dc_start_amo && (amo_req_op != AMO_LR));
assign dc_req_is_mmio  = dc_start_store ? (sq_addr_eff[sq_head_idx][31:24] == 8'h02) :
                         dc_start_amo   ? (amo_req_addr[31:24] == 8'h02)             :
                                          (lq_addr_eff[cache_ld_pick_idx][31:24] == 8'h02);
assign dc_req_amo_op   = dc_start_amo ? amo_req_op : AMO_NONE;
assign dc_req_addr     = dc_start_store ? sq_addr_eff[sq_head_idx][AWIDTH-1:0] :
                         dc_start_amo   ? amo_req_addr[AWIDTH-1:0]             :
                                          lq_addr_eff[cache_ld_pick_idx][AWIDTH-1:0];
assign dc_req_wdata    = dc_start_store ? align_store_data(sq_size[sq_head_idx], sq_addr_eff[sq_head_idx][1:0], sq_vk_eff[sq_head_idx]) :
                                          amo_req_rs2;
assign dc_req_wstrb    = dc_start_store ? calc_byte_mask(sq_size[sq_head_idx], sq_addr_eff[sq_head_idx][1:0]) :
                         dc_start_amo   ? 4'hF : 4'd0;
assign dc_req_id       = dc_start_store ? {ID_W{1'b0}} :
                         dc_start_amo   ? {1'b1, {(ID_W-1){1'b0}}} :
                                          {1'b0, lq_gen[cache_ld_pick_idx], cache_ld_pick_idx};
assign dc_req_rob_tag  = dc_start_store ? sq_rob_tag[sq_head_idx] :
                         dc_start_amo   ? amo_req_tag             :
                                          lq_rob_tag[cache_ld_pick_idx];

wire dc_st_fire  = dc_start_store && dc_req_ready;
wire dc_amo_fire = dc_start_amo   && dc_req_ready;
wire dc_ld_fire  = dc_start_load  && dc_req_ready;

reg [3:0] sq_survive_cnt, sq_sq_ptr;
reg [2:0] sq_sq_idx;
integer   si4;

always @(*) begin
    si4            = 0;
    sq_survive_cnt = 4'd0;
    sq_sq_ptr      = 4'd0;
    sq_sq_idx      = 3'd0;
    for (si4 = 0; si4 < SQ_DEPTH; si4 = si4 + 1) begin
        if (si4[3:0] < (sq_tail - sq_commit_ptr)) begin
            sq_sq_ptr = sq_commit_ptr + si4[3:0];
            sq_sq_idx = sq_sq_ptr[2:0];
            if ((sq_rob_tag[sq_sq_idx] - rob_head) <= sq_br_age)
                sq_survive_cnt = sq_survive_cnt + 4'd1;
        end
    end
end

// ----------------------------------------------------------------------------
// 5. LSU CDB0 Completion Selector (AMO > Oldest Ready Load > Ready Store)
// ----------------------------------------------------------------------------
reg [1:0]          cdb_sel_kind; // 0: none, 1: AMO, 2: LOAD, 3: STORE
reg [LQ_IDX_W-1:0] cdb_sel_ld_idx;
reg [SQ_IDX_W-1:0] cdb_sel_st_idx;
reg [TAG_W-1:0]    cdb_best_age;
reg [3:0]          sq_scan_ptr;
reg [2:0]          sq_scan_idx;
integer            li3, si3;

wire        amo_sc_succ   = (amo_op_r == AMO_SC) && (amo_rdata_r == 32'd0);
wire        amo_is_rmw    = (amo_op_r != AMO_LR) && (amo_op_r != AMO_SC);
wire        amo_rvfi_we   = amo_is_rmw || amo_sc_succ;
wire [31:0] amo_rvfi_wdat = (amo_op_r == AMO_SC) ? (amo_sc_succ ? amo_rs2_r : 32'd0)
                                                 : amo_alu_rvfi(amo_op_r, amo_rdata_r, amo_rs2_r);

always @(*) begin
    li3               = 0;
    si3               = 0;
    lsu_cdb_valid     = 1'b0;
    lsu_cdb_tag       = {TAG_W{1'b0}};
    lsu_cdb_seq       = 32'd0;
    lsu_cdb_val       = 32'd0;
    lsu_cdb_mem_we    = 1'b0;
    lsu_cdb_mem_addr  = 32'd0;
    lsu_cdb_mem_wstrb = 4'd0;
    lsu_cdb_mem_wdata = 32'd0;

    cdb_sel_kind      = 2'd0;
    cdb_sel_ld_idx    = 3'd0;
    cdb_sel_st_idx    = 3'd0;
    cdb_best_age      = {TAG_W{1'b1}};
    sq_scan_ptr       = 4'd0;
    sq_scan_idx       = 3'd0;

    if (amo_done && !flush) begin
        lsu_cdb_valid     = 1'b1;
        cdb_sel_kind      = 2'd1;
        lsu_cdb_tag       = amo_tag_r;
        lsu_cdb_seq       = amo_seq_r;
        lsu_cdb_val       = amo_rdata_r;
        lsu_cdb_mem_we    = amo_rvfi_we;
        lsu_cdb_mem_addr  = amo_addr_r;
        lsu_cdb_mem_wstrb = amo_rvfi_we ? 4'hF : 4'd0;
        lsu_cdb_mem_wdata = amo_rvfi_we ? amo_rvfi_wdat : 32'd0;
    end else begin
        for (li3 = 0; li3 < LQ_DEPTH; li3 = li3 + 1) begin
            if (lq_valid[li3] && lq_done[li3] && !lq_cdb_sent[li3] && !flush) begin
                if (!lsu_cdb_valid || ((lq_rob_tag[li3] - rob_head) < cdb_best_age)) begin
                    lsu_cdb_valid     = 1'b1;
                    cdb_sel_kind      = 2'd2;
                    cdb_sel_ld_idx    = li3[LQ_IDX_W-1:0];
                    cdb_best_age      = lq_rob_tag[li3] - rob_head;
                    lsu_cdb_tag       = lq_rob_tag[li3];
                    lsu_cdb_seq       = lq_seq_id[li3];
                    lsu_cdb_val       = lq_data[li3];
                    lsu_cdb_mem_we    = 1'b0;
                    lsu_cdb_mem_addr  = lq_addr_eff[li3];
                    lsu_cdb_mem_wstrb = 4'd0;
                    lsu_cdb_mem_wdata = 32'd0;
                end
            end
        end

        if (!lsu_cdb_valid && !flush) begin
            for (si3 = 0; si3 < SQ_DEPTH; si3 = si3 + 1) begin
                if (si3[3:0] < (sq_tail - sq_commit_ptr)) begin
                    sq_scan_ptr = sq_commit_ptr + si3[3:0];
                    sq_scan_idx = sq_scan_ptr[2:0];
                    if (!lsu_cdb_valid && sq_addr_valid[sq_scan_idx] &&
                        !sq_qk_valid[sq_scan_idx] && !sq_cdb_sent[sq_scan_idx]) begin
                        lsu_cdb_valid     = 1'b1;
                        cdb_sel_kind      = 2'd3;
                        cdb_sel_st_idx    = sq_scan_idx;
                        lsu_cdb_tag       = sq_rob_tag[sq_scan_idx];
                        lsu_cdb_seq       = sq_seq_id[sq_scan_idx];
                        lsu_cdb_val       = 32'd0;
                        lsu_cdb_mem_we    = 1'b1;
                        lsu_cdb_mem_addr  = sq_addr_eff[sq_scan_idx];
                        lsu_cdb_mem_wstrb = calc_byte_mask(sq_size[sq_scan_idx], sq_addr_eff[sq_scan_idx][1:0]);
                        lsu_cdb_mem_wdata = align_store_data(sq_size[sq_scan_idx], sq_addr_eff[sq_scan_idx][1:0], sq_vk_eff[sq_scan_idx]);
                    end
                end
            end
        end
    end
end

assign stat_stlf_hit     = stlf_pick_v && !flush;
assign trace_issue_valid = !flush && (dc_ld_fire || dc_amo_fire || stlf_pick_v ||
                                      (lsu_cdb_valid && lsu_cdb_grant && (cdb_sel_kind == 2'd3)));
assign trace_issue_seq   = dc_amo_fire ? amo_req_seq                  :
                           dc_ld_fire  ? lq_seq_id[cache_ld_pick_idx] :
                           stlf_pick_v ? lq_seq_id[stlf_pick_idx]     : lsu_cdb_seq;

wire lsq_byp_ld_issue = (dc_ld_fire  && (lq_qj_byp[cache_ld_pick_idx] != BYP_NONE)) ||
                        (stlf_pick_v && (lq_qj_byp[stlf_pick_idx]     != BYP_NONE));
wire lsq_byp_st_issue = (lsu_cdb_valid && lsu_cdb_grant && (cdb_sel_kind == 2'd3)) &&
                        ((sq_qj_byp[cdb_sel_st_idx] != BYP_NONE) || (sq_qk_byp[cdb_sel_st_idx] != BYP_NONE));
assign stat_ex_byp_cnt  = {1'b0, lsq_byp_ld_issue} + {1'b0, lsq_byp_st_issue};

// ----------------------------------------------------------------------------
// 6. Sequential State Updates
// ----------------------------------------------------------------------------
integer k;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        sq_head       <= 4'd0;
        sq_commit_ptr <= 4'd0;
        sq_tail       <= 4'd0;
        amo_in_flight <= 1'b0;
        amo_done      <= 1'b0;
        amo_op_r      <= AMO_NONE;
        amo_tag_r     <= {TAG_W{1'b0}};
        amo_seq_r     <= 32'd0;
        amo_addr_r    <= 32'd0;
        amo_rs2_r     <= 32'd0;
        amo_rdata_r   <= 32'd0;
        for (k = 0; k < SQ_DEPTH; k = k + 1) begin
            sq_rob_tag[k]    <= {TAG_W{1'b0}};
            sq_seq_id[k]     <= 32'd0;
            sq_size[k]       <= MEM_WORD;
            sq_imm[k]        <= 32'd0;
            sq_qj[k]         <= {TAG_W{1'b0}};
            sq_qj_valid[k]   <= 1'b0;
            sq_qj_byp[k]     <= BYP_NONE;
            sq_addr[k]       <= 32'd0;
            sq_addr_valid[k] <= 1'b0;
            sq_vk[k]         <= 32'd0;
            sq_qk[k]         <= {TAG_W{1'b0}};
            sq_qk_valid[k]   <= 1'b0;
            sq_qk_byp[k]     <= BYP_NONE;
            sq_cdb_sent[k]   <= 1'b0;
        end
        for (k = 0; k < LQ_DEPTH; k = k + 1) begin
            lq_valid[k]      <= 1'b0;
            lq_gen[k]        <= 3'd0;
            lq_rob_tag[k]    <= {TAG_W{1'b0}};
            lq_seq_id[k]     <= 32'd0;
            lq_size[k]       <= MEM_WORD;
            lq_uns[k]        <= 1'b0;
            lq_imm[k]        <= 32'd0;
            lq_qj[k]         <= {TAG_W{1'b0}};
            lq_qj_valid[k]   <= 1'b0;
            lq_qj_byp[k]     <= BYP_NONE;
            lq_addr[k]       <= 32'd0;
            lq_addr_valid[k] <= 1'b0;
            lq_sq_snap[k]    <= 4'd0;
            lq_issued[k]     <= 1'b0;
            lq_done[k]       <= 1'b0;
            lq_cdb_sent[k]   <= 1'b0;
            lq_from_stlf[k]  <= 1'b0;
            lq_replay[k]     <= 1'b0;
            lq_data[k]       <= 32'd0;
        end
    end else begin
        // A. Split-Transaction D-Cache Handshake & Dual Response Capture
        if (dc_st_fire) begin
            sq_head <= sq_head + 4'd1;
        end else if (dc_amo_fire) begin
            amo_in_flight <= 1'b1;
            amo_op_r      <= amo_req_op;
            amo_tag_r     <= amo_req_tag;
            amo_seq_r     <= amo_req_seq;
            amo_addr_r    <= amo_req_addr;
            amo_rs2_r     <= amo_req_rs2;
        end else if (dc_ld_fire) begin
            lq_issued[cache_ld_pick_idx]    <= 1'b1;
            lq_from_stlf[cache_ld_pick_idx] <= 1'b0;
        end

        if (!flush) begin
            if (dc_hit_is_amo && amo_in_flight) begin
                amo_in_flight <= 1'b0;
                amo_done      <= 1'b1;
                amo_rdata_r   <= dc_hit_rdata;
            end else if (dc_hit_valid && !dc_hit_is_amo && lq_valid[hit_idx] &&
                         lq_issued[hit_idx] && (hit_gen == lq_gen[hit_idx])) begin
                lq_done[hit_idx] <= 1'b1;
                lq_data[hit_idx] <= format_load_data(lq_size[hit_idx], lq_uns[hit_idx],
                                                     lq_addr_eff[hit_idx][1:0], dc_hit_rdata);
            end

            if (dc_fill_is_amo && amo_in_flight) begin
                amo_in_flight <= 1'b0;
                amo_done      <= 1'b1;
                amo_rdata_r   <= dc_fill_rdata;
            end else if (dc_fill_valid && !dc_fill_is_amo && lq_valid[fill_idx] &&
                         lq_issued[fill_idx] && (fill_gen == lq_gen[fill_idx])) begin
                lq_done[fill_idx] <= 1'b1;
                lq_data[fill_idx] <= format_load_data(lq_size[fill_idx], lq_uns[fill_idx],
                                                      lq_addr_eff[fill_idx][1:0], dc_fill_rdata);
            end
        end

        // B. Queue updates
        if (flush) begin
            sq_tail       <= sq_commit_ptr;
            amo_in_flight <= 1'b0;
            amo_done      <= 1'b0;
            for (k = 0; k < LQ_DEPTH; k = k + 1) begin
                if (lq_valid[k]) lq_gen[k] <= lq_gen[k] + 3'd1;
                lq_valid[k]    <= 1'b0;
                lq_issued[k]   <= 1'b0;
                lq_done[k]     <= 1'b0;
                lq_cdb_sent[k] <= 1'b0;
                lq_replay[k]   <= 1'b0;
                lq_qj_byp[k]   <= BYP_NONE;
            end
            for (k = 0; k < SQ_DEPTH; k = k + 1) begin
                if (sq_qj_byp[k] != BYP_NONE) begin
                    sq_addr[k]   <= sq_addr_eff[k];
                    sq_qj_byp[k] <= BYP_NONE;
                end
                if (sq_qk_byp[k] != BYP_NONE) begin
                    sq_vk[k]     <= sq_vk_eff[k];
                    sq_qk_byp[k] <= BYP_NONE;
                end
            end
        end else begin
            // 0. Latch external coherence invalidation snoop hits on uncommitted loads!
            for (k = 0; k < LQ_DEPTH; k = k + 1) begin
                if (lq_snoop_hit_now[k])
                    lq_replay[k] <= 1'b1;
            end

            // 1. Capture pending EX->EX bypasses & snoop Dual-CDB + wk0/wk1
            for (k = 0; k < LQ_DEPTH; k = k + 1) begin
                if (lq_valid[k]) begin
                    if (lq_qj_byp[k] != BYP_NONE) begin
                        lq_addr[k]   <= lq_addr_eff[k];
                        lq_qj_byp[k] <= BYP_NONE;
                    end else if (lq_qj_valid[k]) begin
                        if (cdb0_valid && (lq_qj[k] == cdb0_tag)) begin
                            lq_addr[k]       <= cdb0_val + lq_imm[k];
                            lq_addr_valid[k] <= 1'b1;
                            lq_qj_valid[k]   <= 1'b0;
                            lq_qj_byp[k]     <= BYP_NONE;
                        end else if (cdb1_valid && (lq_qj[k] == cdb1_tag)) begin
                            lq_addr[k]       <= cdb1_val + lq_imm[k];
                            lq_addr_valid[k] <= 1'b1;
                            lq_qj_valid[k]   <= 1'b0;
                            lq_qj_byp[k]     <= BYP_NONE;
                        end else if (wk0_valid && (lq_qj[k] == wk0_tag)) begin
                            lq_addr_valid[k] <= 1'b1;
                            lq_qj_valid[k]   <= 1'b0;
                            lq_qj_byp[k]     <= BYP_ALU0;
                        end else if (wk1_valid && (lq_qj[k] == wk1_tag)) begin
                            lq_addr_valid[k] <= 1'b1;
                            lq_qj_valid[k]   <= 1'b0;
                            lq_qj_byp[k]     <= BYP_ALU1;
                        end
                    end
                end
            end
            for (k = 0; k < SQ_DEPTH; k = k + 1) begin
                if (sq_qj_byp[k] != BYP_NONE) begin
                    sq_addr[k]   <= sq_addr_eff[k];
                    sq_qj_byp[k] <= BYP_NONE;
                end else if (sq_qj_valid[k]) begin
                    if (cdb0_valid && (sq_qj[k] == cdb0_tag)) begin
                        sq_addr[k]       <= cdb0_val + sq_imm[k];
                        sq_addr_valid[k] <= 1'b1;
                        sq_qj_valid[k]   <= 1'b0;
                        sq_qj_byp[k]     <= BYP_NONE;
                    end else if (cdb1_valid && (sq_qj[k] == cdb1_tag)) begin
                        sq_addr[k]       <= cdb1_val + sq_imm[k];
                        sq_addr_valid[k] <= 1'b1;
                        sq_qj_valid[k]   <= 1'b0;
                        sq_qj_byp[k]     <= BYP_NONE;
                    end else if (wk0_valid && (sq_qj[k] == wk0_tag)) begin
                        sq_addr_valid[k] <= 1'b1;
                        sq_qj_valid[k]   <= 1'b0;
                        sq_qj_byp[k]     <= BYP_ALU0;
                    end else if (wk1_valid && (sq_qj[k] == wk1_tag)) begin
                        sq_addr_valid[k] <= 1'b1;
                        sq_qj_valid[k]   <= 1'b0;
                        sq_qj_byp[k]     <= BYP_ALU1;
                    end
                end

                if (sq_qk_byp[k] != BYP_NONE) begin
                    sq_vk[k]     <= sq_vk_eff[k];
                    sq_qk_byp[k] <= BYP_NONE;
                end else if (sq_qk_valid[k]) begin
                    if (cdb0_valid && (sq_qk[k] == cdb0_tag)) begin
                        sq_vk[k]       <= cdb0_val;
                        sq_qk_valid[k] <= 1'b0;
                        sq_qk_byp[k]   <= BYP_NONE;
                    end else if (cdb1_valid && (sq_qk[k] == cdb1_tag)) begin
                        sq_vk[k]       <= cdb1_val;
                        sq_qk_valid[k] <= 1'b0;
                        sq_qk_byp[k]   <= BYP_NONE;
                    end else if (wk0_valid && (sq_qk[k] == wk0_tag)) begin
                        sq_qk_valid[k] <= 1'b0;
                        sq_qk_byp[k]   <= BYP_ALU0;
                    end else if (wk1_valid && (sq_qk[k] == wk1_tag)) begin
                        sq_qk_valid[k] <= 1'b0;
                        sq_qk_byp[k]   <= BYP_ALU1;
                    end
                end
            end

            // 2. Fast 1-cycle STLF
            if (stlf_pick_v) begin
                lq_issued[stlf_pick_idx]    <= 1'b1;
                lq_done[stlf_pick_idx]      <= 1'b1;
                lq_from_stlf[stlf_pick_idx] <= 1'b1;
                lq_data[stlf_pick_idx]      <= lq_stlf_data[stlf_pick_idx];
            end

            // 3. CDB0 completion broadcast (loads stay valid in LQ until ROB commit!)
            if (lsu_cdb_valid && lsu_cdb_grant) begin
                if (cdb_sel_kind == 2'd1) begin
                    amo_done <= 1'b0;
                end else if (cdb_sel_kind == 2'd2) begin
                    lq_cdb_sent[cdb_sel_ld_idx] <= 1'b1;
                end else if (cdb_sel_kind == 2'd3) begin
                    sq_cdb_sent[cdb_sel_st_idx] <= 1'b1;
                end
            end

            // 4. ROB Commit: free committed Load entries & advance committed Store ptr
            if (rob_commit_store)
                sq_commit_ptr <= sq_commit_ptr + 4'd1;

            for (k = 0; k < LQ_DEPTH; k = k + 1) begin
                if (lq_valid[k] &&
                    ((rob_commit0_load && (lq_rob_tag[k] == rob_commit0_tag)) ||
                     (rob_commit1_load && (lq_rob_tag[k] == rob_commit1_tag)))) begin
                    lq_valid[k]    <= 1'b0;
                    lq_issued[k]   <= 1'b0;
                    lq_done[k]     <= 1'b0;
                    lq_cdb_sent[k] <= 1'b0;
                    lq_replay[k]   <= 1'b0;
                    lq_qj_byp[k]   <= BYP_NONE;
                    lq_gen[k]      <= lq_gen[k] + 3'd1;
                end
            end

            // 5. Selective branch squash OR single memory dispatch
            if (squash_valid) begin
                sq_tail <= sq_commit_ptr + sq_survive_cnt;
                for (k = 0; k < LQ_DEPTH; k = k + 1) begin
                    if (lq_valid[k] && ((lq_rob_tag[k] - rob_head) > sq_br_age)) begin
                        lq_valid[k]    <= 1'b0;
                        lq_issued[k]   <= 1'b0;
                        lq_done[k]     <= 1'b0;
                        lq_cdb_sent[k] <= 1'b0;
                        lq_replay[k]   <= 1'b0;
                        lq_qj_byp[k]   <= BYP_NONE;
                        lq_gen[k]      <= lq_gen[k] + 3'd1;
                    end
                end
            end else if (do_disp_load) begin
                lq_valid[lq_free_idx]      <= 1'b1;
                lq_rob_tag[lq_free_idx]    <= disp_rob_tag;
                lq_seq_id[lq_free_idx]     <= disp_seq_id;
                lq_size[lq_free_idx]       <= disp_mem_size;
                lq_uns[lq_free_idx]        <= disp_mem_unsigned;
                lq_imm[lq_free_idx]        <= disp_imm;
                lq_qj[lq_free_idx]         <= disp_qj;
                lq_qj_valid[lq_free_idx]   <= disp_qj_v_eff;
                lq_qj_byp[lq_free_idx]     <= disp_qjbyp_e;
                lq_addr[lq_free_idx]       <= disp_addr_eff;
                lq_addr_valid[lq_free_idx] <= !disp_qj_v_eff;
                lq_sq_snap[lq_free_idx]    <= sq_tail;
                lq_issued[lq_free_idx]     <= 1'b0;
                lq_done[lq_free_idx]       <= 1'b0;
                lq_cdb_sent[lq_free_idx]   <= 1'b0;
                lq_from_stlf[lq_free_idx]  <= 1'b0;
                lq_replay[lq_free_idx]     <= 1'b0;
                lq_data[lq_free_idx]       <= 32'd0;
            end else if (do_disp_store) begin
                sq_rob_tag[sq_tail_idx]    <= disp_rob_tag;
                sq_seq_id[sq_tail_idx]     <= disp_seq_id;
                sq_size[sq_tail_idx]       <= disp_mem_size;
                sq_imm[sq_tail_idx]        <= disp_imm;
                sq_qj[sq_tail_idx]         <= disp_qj;
                sq_qj_valid[sq_tail_idx]   <= disp_qj_v_eff;
                sq_qj_byp[sq_tail_idx]     <= disp_qjbyp_e;
                sq_addr[sq_tail_idx]       <= disp_addr_eff;
                sq_addr_valid[sq_tail_idx] <= !disp_qj_v_eff;
                sq_vk[sq_tail_idx]         <= disp_vk_eff;
                sq_qk[sq_tail_idx]         <= disp_qk;
                sq_qk_valid[sq_tail_idx]   <= disp_qk_v_eff;
                sq_qk_byp[sq_tail_idx]     <= disp_qkbyp_e;
                sq_cdb_sent[sq_tail_idx]   <= 1'b0;
                sq_tail                    <= sq_tail + 4'd1;
            end
        end
    end
end

endmodule
