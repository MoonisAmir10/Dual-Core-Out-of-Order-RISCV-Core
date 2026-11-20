`timescale 1ns / 1ps

// ============================================================================
// Single Bank of 4-Way Set-Associative Non-Inclusive (NINE) Unified L2 Cache
// (Phase 2: Default 16 KB/bank -> 256 sets x 4 ways x 16 B; 2 banks = 32 KB!)
//
// Features:
//   - 4-Way Set-Associative with 3-bit Tree Pseudo-LRU (Tree-PLRU)
//   - Write-Back, Full-Line Write-Allocate (128-bit L1 writebacks allocate on
//     miss in 1 cycle with zero DRAM read!)
//   - Configurable Hit Pipeline (default HIT_LAT=3 cycles) with lossless
//     resp_ready backpressure for 2-bank -> 2-master crossbar response muxing
//   - 2 MSHRs per bank (4 total across 2 banks!) with 2-waiter secondary read
//     coalescing & 2-entry Writeback Buffer (WBB) with exact hazard stalling
//   - 1D per-way arrays -> 100% Verilator, Icarus Verilog, and Yosys clean
// ============================================================================
module l2_bank #(
    parameter integer BADDR_W  = 15,  // Bank line address width (16-bit full line - 1 bank bit)
    parameter integer NUM_SETS = 256, // 256 x 4 x 16 B = 16 KB/bank (2 banks = 32 KB)
    parameter integer ID_W     = 4,   // Upstream transaction ID width
    parameter integer HIT_LAT  = 3    // >= 1 cycles hit response latency
) (
    input  wire                clk,
    input  wire                reset_n,

    // Upstream 128-bit Request Port (from 2x2 L2 Crossbar)
    input  wire                req_valid,
    output reg                 req_ready,
    input  wire                req_is_write, // 0: 128-bit read, 1: 128-bit dirty line writeback
    input  wire [BADDR_W-1:0]  req_baddr,
    input  wire [127:0]        req_wdata,
    input  wire [ID_W-1:0]     req_id,

    // Upstream 128-bit Read Response Port (lossless valid/ready)
    output wire                resp_valid,
    input  wire                resp_ready,
    output wire [ID_W-1:0]     resp_id,
    output wire [127:0]        resp_data,

    // Downstream Pipelined 128-bit DRAM Read Channel (up to 2 outstanding/bank)
    output reg                 dram_rd_valid,
    input  wire                dram_rd_ready,
    output reg  [BADDR_W-1:0]  dram_rd_baddr,
    output reg                 dram_rd_id,

    input  wire                dram_rresp_valid,
    input  wire                dram_rresp_id,
    input  wire [127:0]        dram_rresp_data,

    // Downstream 128-bit DRAM Writeback Channel
    output wire                dram_wr_valid,
    input  wire                dram_wr_ready,
    output wire [BADDR_W-1:0]  dram_wr_baddr,
    output wire [127:0]        dram_wr_data,

    // Status & PMU Pulses
    output wire                bank_idle,
    output wire                pmu_l2_hit,
    output wire                pmu_l2_miss,
    output wire                pmu_l2_wb
);

localparam integer IDX_W  = $clog2(NUM_SETS);
localparam integer CTAG_W = BADDR_W - IDX_W;

// ----------------------------------------------------------------------------
// 1. 3-Bit Tree-PLRU Helpers
//    p[2]: 0=left{0,1} MRU (evict right{2,3}), 1=right{2,3} MRU (evict left{0,1})
//    p[1]: 0=way0 MRU (evict 1), 1=way1 MRU (evict 0)
//    p[0]: 0=way2 MRU (evict 3), 1=way3 MRU (evict 2)
// ----------------------------------------------------------------------------
function automatic [1:0] pick_victim(
    input       v0, v1, v2, v3,
    input [2:0] p
);
    begin
        if (!v0)      pick_victim = 2'd0;
        else if (!v1) pick_victim = 2'd1;
        else if (!v2) pick_victim = 2'd2;
        else if (!v3) pick_victim = 2'd3;
        else if (!p[2])
            pick_victim = p[0] ? 2'd2 : 2'd3;
        else
            pick_victim = p[1] ? 2'd0 : 2'd1;
    end
endfunction

function automatic [2:0] upd_plru(input [2:0] p, input [1:0] way);
    begin
        case (way)
            2'd0: upd_plru = {1'b0, 1'b0, p[0]};
            2'd1: upd_plru = {1'b0, 1'b1, p[0]};
            2'd2: upd_plru = {1'b1, p[1], 1'b0};
            2'd3: upd_plru = {1'b1, p[1], 1'b1};
        endcase
    end
endfunction

// ----------------------------------------------------------------------------
// 2. 4-Way Cache Arrays (1D per Way -> 100% Verilator/Icarus/Yosys Clean!)
// ----------------------------------------------------------------------------
reg              way0_v   [0:NUM_SETS-1];
reg              way0_d   [0:NUM_SETS-1];
reg [CTAG_W-1:0] way0_tag [0:NUM_SETS-1];
reg [127:0]      way0_dat [0:NUM_SETS-1];

reg              way1_v   [0:NUM_SETS-1];
reg              way1_d   [0:NUM_SETS-1];
reg [CTAG_W-1:0] way1_tag [0:NUM_SETS-1];
reg [127:0]      way1_dat [0:NUM_SETS-1];

reg              way2_v   [0:NUM_SETS-1];
reg              way2_d   [0:NUM_SETS-1];
reg [CTAG_W-1:0] way2_tag [0:NUM_SETS-1];
reg [127:0]      way2_dat [0:NUM_SETS-1];

reg              way3_v   [0:NUM_SETS-1];
reg              way3_d   [0:NUM_SETS-1];
reg [CTAG_W-1:0] way3_tag [0:NUM_SETS-1];
reg [127:0]      way3_dat [0:NUM_SETS-1];

reg [2:0]        plru_arr [0:NUM_SETS-1];

// ----------------------------------------------------------------------------
// 3. Hit Latency Shift Pipeline + 2 MSHRs (2 Waiters/MSHR) + 2-Entry WBB
// ----------------------------------------------------------------------------
reg              hp_v   [0:HIT_LAT-1];
reg [ID_W-1:0]   hp_id  [0:HIT_LAT-1];
reg [127:0]      hp_dat [0:HIT_LAT-1];

reg               m0_v, m0_sent, m0_refilled;
reg [BADDR_W-1:0] m0_baddr;
reg [127:0]       m0_line;

reg               m1_v, m1_sent, m1_refilled;
reg [BADDR_W-1:0] m1_baddr;
reg [127:0]       m1_line;

// Waiters 0..1 -> MSHR0; Waiters 2..3 -> MSHR1
reg               w_v  [0:3];
reg [ID_W-1:0]    w_id [0:3];

// 2-Entry Writeback Buffer
reg               wbb0_v, wbb1_v;
reg [BADDR_W-1:0] wbb0_baddr, wbb1_baddr;
reg [127:0]       wbb0_data, wbb1_data;

// ----------------------------------------------------------------------------
// 4. Combinational Lookup, Hazard & Arbitration Logic
// ----------------------------------------------------------------------------
wire [IDX_W-1:0]  req_set  = req_baddr[IDX_W-1:0];
wire [CTAG_W-1:0] req_ctag = req_baddr[BADDR_W-1:IDX_W];

wire hit_w0  = way0_v[req_set] && (way0_tag[req_set] == req_ctag);
wire hit_w1  = way1_v[req_set] && (way1_tag[req_set] == req_ctag);
wire hit_w2  = way2_v[req_set] && (way2_tag[req_set] == req_ctag);
wire hit_w3  = way3_v[req_set] && (way3_tag[req_set] == req_ctag);
wire tag_hit = hit_w0 || hit_w1 || hit_w2 || hit_w3;

wire [1:0]   hit_way  = hit_w0 ? 2'd0 : (hit_w1 ? 2'd1 : (hit_w2 ? 2'd2 : 2'd3));
wire [127:0] hit_line = hit_w0 ? way0_dat[req_set] :
                        hit_w1 ? way1_dat[req_set] :
                        hit_w2 ? way2_dat[req_set] : way3_dat[req_set];

wire wbb_hit = (wbb0_v && (wbb0_baddr == req_baddr)) || (wbb1_v && (wbb1_baddr == req_baddr));
wire m0_hit  = m0_v && (m0_baddr == req_baddr);
wire m1_hit  = m1_v && (m1_baddr == req_baddr);

wire m0_has_wfree = !w_v[0] || !w_v[1];
wire m1_has_wfree = !w_v[2] || !w_v[3];
wire m0_free_wbit = !w_v[0] ? 1'b0 : 1'b1;
wire m1_free_wbit = !w_v[2] ? 1'b0 : 1'b1;

wire [1:0] wbb_free_cnt      = {1'b0, !wbb0_v} + {1'b0, !wbb1_v};
wire [1:0] unfilled_mshr_cnt = {1'b0, (m0_v && !m0_refilled)} + {1'b0, (m1_v && !m1_refilled)};
wire       same_set_mshr     = (m0_v && (m0_baddr[IDX_W-1:0] == req_set)) ||
                               (m1_v && (m1_baddr[IDX_W-1:0] == req_set));
wire       can_alloc_mshr    = (!m0_v || !m1_v) && (wbb_free_cnt > unfilled_mshr_cnt) && !same_set_mshr;

wire       hp_tail_v     = hp_v[HIT_LAT-1];
wire       hp_stall      = hp_tail_v && !resp_ready;
wire       any_refilled  = (m0_v && m0_refilled) || (m1_v && m1_refilled);

// Classify incoming request cleanly
reg act_rd_hit, act_wr_hit, act_wr_alloc, act_rd_coal, act_rd_miss;

always @(*) begin
    req_ready    = 1'b0;
    act_rd_hit   = 1'b0;
    act_wr_hit   = 1'b0;
    act_wr_alloc = 1'b0;
    act_rd_coal  = 1'b0;
    act_rd_miss  = 1'b0;

    // Prioritize DRAM refill & draining refilled MSHRs (guarantees 0 starvation!)
    if (req_valid && !dram_rresp_valid && !wbb_hit && !hp_stall && !any_refilled) begin
        if (req_is_write) begin
            // Full 128-bit dirty L1 writeback: must not collide with active read MSHR
            if (!m0_hit && !m1_hit) begin
                if (tag_hit) begin
                    req_ready  = 1'b1;
                    act_wr_hit = 1'b1;
                end else if (!same_set_mshr && (wbb_free_cnt > unfilled_mshr_cnt)) begin
                    // Direct full-line write-allocate without DRAM read!
                    req_ready    = 1'b1;
                    act_wr_alloc = 1'b1;
                end
            end
        end else begin
            // 128-bit read request
            if (m0_hit) begin
                if (!m0_refilled && m0_has_wfree) begin
                    req_ready   = 1'b1;
                    act_rd_coal = 1'b1;
                end
            end else if (m1_hit) begin
                if (!m1_refilled && m1_has_wfree) begin
                    req_ready   = 1'b1;
                    act_rd_coal = 1'b1;
                end
            end else if (tag_hit) begin
                req_ready  = 1'b1;
                act_rd_hit = 1'b1;
            end else if (can_alloc_mshr) begin
                req_ready   = 1'b1;
                act_rd_miss = 1'b1;
            end
        end
    end
end

// Victim selection for DRAM refill OR full-line writeback miss allocation
wire m0_rf_now = dram_rresp_valid && !dram_rresp_id && m0_v;
wire m1_rf_now = dram_rresp_valid &&  dram_rresp_id && m1_v;
wire do_refill = m0_rf_now || m1_rf_now;

wire [BADDR_W-1:0] ins_baddr = do_refill ? (m1_rf_now ? m1_baddr : m0_baddr) : req_baddr;
wire [IDX_W-1:0]   ins_set   = ins_baddr[IDX_W-1:0];
wire [CTAG_W-1:0]  ins_ctag  = ins_baddr[BADDR_W-1:IDX_W];
wire [1:0]         ins_vway  = pick_victim(way0_v[ins_set], way1_v[ins_set],
                                           way2_v[ins_set], way3_v[ins_set], plru_arr[ins_set]);

reg                vic_v, vic_d;
reg [CTAG_W-1:0]   vic_tag;
reg [127:0]        vic_dat;
always @(*) begin
    case (ins_vway)
        2'd0: begin vic_v = way0_v[ins_set]; vic_d = way0_d[ins_set]; vic_tag = way0_tag[ins_set]; vic_dat = way0_dat[ins_set]; end
        2'd1: begin vic_v = way1_v[ins_set]; vic_d = way1_d[ins_set]; vic_tag = way1_tag[ins_set]; vic_dat = way1_dat[ins_set]; end
        2'd2: begin vic_v = way2_v[ins_set]; vic_d = way2_d[ins_set]; vic_tag = way2_tag[ins_set]; vic_dat = way2_dat[ins_set]; end
        2'd3: begin vic_v = way3_v[ins_set]; vic_d = way3_d[ins_set]; vic_tag = way3_tag[ins_set]; vic_dat = way3_dat[ins_set]; end
    endcase
end

wire ins_need_wb = (do_refill || act_wr_alloc) && vic_v && vic_d;

// MSHR Waiter Drain Selector (fires whenever hit pipeline tail is empty)
reg         drain_v;
reg         drain_is_m1;
reg [1:0]   drain_wslot;
reg [127:0] drain_line;

always @(*) begin
    drain_v     = 1'b0;
    drain_is_m1 = 1'b0;
    drain_wslot = 2'd0;
    drain_line  = 128'd0;
    if (!hp_tail_v) begin
        if (m0_v && m0_refilled && (w_v[0] || w_v[1])) begin
            drain_v     = 1'b1;
            drain_is_m1 = 1'b0;
            drain_wslot = w_v[0] ? 2'd0 : 2'd1;
            drain_line  = m0_line;
        end else if (m1_v && m1_refilled && (w_v[2] || w_v[3])) begin
            drain_v     = 1'b1;
            drain_is_m1 = 1'b1;
            drain_wslot = w_v[2] ? 2'd2 : 2'd3;
            drain_line  = m1_line;
        end
    end
end

assign resp_valid = hp_tail_v || drain_v;
assign resp_id    = hp_tail_v ? hp_id[HIT_LAT-1]  : w_id[drain_wslot];
assign resp_data  = hp_tail_v ? hp_dat[HIT_LAT-1] : drain_line;

wire drain_fire = drain_v && resp_ready;

// Downstream DRAM Read Request (MSHR0 -> MSHR1)
always @(*) begin
    dram_rd_valid = 1'b0;
    dram_rd_baddr = {BADDR_W{1'b0}};
    dram_rd_id    = 1'b0;
    if (m0_v && !m0_sent && !m0_refilled) begin
        dram_rd_valid = 1'b1;
        dram_rd_baddr = m0_baddr;
        dram_rd_id    = 1'b0;
    end else if (m1_v && !m1_sent && !m1_refilled) begin
        dram_rd_valid = 1'b1;
        dram_rd_baddr = m1_baddr;
        dram_rd_id    = 1'b1;
    end
end

// Downstream DRAM Writeback Port (WBB0 -> WBB1)
assign dram_wr_valid = wbb0_v || wbb1_v;
assign dram_wr_baddr = wbb0_v ? wbb0_baddr : wbb1_baddr;
assign dram_wr_data  = wbb0_v ? wbb0_data  : wbb1_data;

// Status & PMU
reg hp_any_v;
integer hi;
always @(*) begin
    hi       = 0;
    hp_any_v = 1'b0;
    for (hi = 0; hi < HIT_LAT; hi = hi + 1)
        if (hp_v[hi]) hp_any_v = 1'b1;
end

assign bank_idle  = !m0_v && !m1_v && !wbb0_v && !wbb1_v && !hp_any_v;
assign pmu_l2_hit = act_rd_hit || act_wr_hit;
assign pmu_l2_miss = act_rd_miss;
assign pmu_l2_wb  = ins_need_wb;

// Remaining waiters check after drain_fire
wire [1:0] m0_w_after = {w_v[1], w_v[0]} & ~((drain_fire && !drain_is_m1) ? (2'b01 << drain_wslot[0]) : 2'b00);
wire [1:0] m1_w_after = {w_v[3], w_v[2]} & ~((drain_fire &&  drain_is_m1) ? (2'b01 << drain_wslot[0]) : 2'b00);

// ----------------------------------------------------------------------------
// 5. Sequential State Updates
// ----------------------------------------------------------------------------
integer k;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (k = 0; k < NUM_SETS; k = k + 1) begin
            way0_v[k]   <= 1'b0; way0_d[k] <= 1'b0;
            way1_v[k]   <= 1'b0; way1_d[k] <= 1'b0;
            way2_v[k]   <= 1'b0; way2_d[k] <= 1'b0;
            way3_v[k]   <= 1'b0; way3_d[k] <= 1'b0;
            plru_arr[k] <= 3'd0;
        end
        for (k = 0; k < HIT_LAT; k = k + 1) begin
            hp_v[k]   <= 1'b0;
            hp_id[k]  <= {ID_W{1'b0}};
            hp_dat[k] <= 128'd0;
        end
        m0_v <= 1'b0; m0_sent <= 1'b0; m0_refilled <= 1'b0;
        m0_baddr <= {BADDR_W{1'b0}}; m0_line <= 128'd0;
        m1_v <= 1'b0; m1_sent <= 1'b0; m1_refilled <= 1'b0;
        m1_baddr <= {BADDR_W{1'b0}}; m1_line <= 128'd0;
        for (k = 0; k < 4; k = k + 1) begin
            w_v[k]  <= 1'b0;
            w_id[k] <= {ID_W{1'b0}};
        end
        wbb0_v <= 1'b0; wbb0_baddr <= {BADDR_W{1'b0}}; wbb0_data <= 128'd0;
        wbb1_v <= 1'b0; wbb1_baddr <= {BADDR_W{1'b0}}; wbb1_data <= 128'd0;
    end else begin
        // 1. Advance Hit Latency Pipeline (unless tail stalled by !resp_ready)
        if (!hp_stall) begin
            for (k = HIT_LAT - 1; k >= 1; k = k - 1) begin
                hp_v[k]   <= hp_v[k-1];
                hp_id[k]  <= hp_id[k-1];
                hp_dat[k] <= hp_dat[k-1];
            end
            hp_v[0]   <= act_rd_hit;
            hp_id[0]  <= req_id;
            hp_dat[0] <= hit_line;
        end

        // 2. WBB Drain to Backing DRAM
        if (dram_wr_valid && dram_wr_ready) begin
            if (wbb0_v)
                wbb0_v <= 1'b0;
            else if (wbb1_v)
                wbb1_v <= 1'b0;
        end

        // 3. Mark MSHR DRAM Read as Sent
        if (dram_rd_valid && dram_rd_ready) begin
            if (!dram_rd_id)
                m0_sent <= 1'b1;
            else
                m1_sent <= 1'b1;
        end

        // 4. Drain MSHR Waiter & Free Completed MSHR
        if (drain_fire)
            w_v[drain_wslot] <= 1'b0;

        if (m0_v && m0_refilled && (m0_w_after == 2'b00)) begin
            m0_v        <= 1'b0;
            m0_sent     <= 1'b0;
            m0_refilled <= 1'b0;
        end
        if (m1_v && m1_refilled && (m1_w_after == 2'b00)) begin
            m1_v        <= 1'b0;
            m1_sent     <= 1'b0;
            m1_refilled <= 1'b0;
        end

        // 5. Handle Incoming DRAM Refill (owns cache array this cycle!)
        if (do_refill) begin
            case (ins_vway)
                2'd0: begin way0_v[ins_set] <= 1'b1; way0_d[ins_set] <= 1'b0; way0_tag[ins_set] <= ins_ctag; way0_dat[ins_set] <= dram_rresp_data; end
                2'd1: begin way1_v[ins_set] <= 1'b1; way1_d[ins_set] <= 1'b0; way1_tag[ins_set] <= ins_ctag; way1_dat[ins_set] <= dram_rresp_data; end
                2'd2: begin way2_v[ins_set] <= 1'b1; way2_d[ins_set] <= 1'b0; way2_tag[ins_set] <= ins_ctag; way2_dat[ins_set] <= dram_rresp_data; end
                2'd3: begin way3_v[ins_set] <= 1'b1; way3_d[ins_set] <= 1'b0; way3_tag[ins_set] <= ins_ctag; way3_dat[ins_set] <= dram_rresp_data; end
            endcase
            plru_arr[ins_set] <= upd_plru(plru_arr[ins_set], ins_vway);

            if (ins_need_wb) begin
                if (!wbb0_v || (dram_wr_valid && dram_wr_ready && wbb0_v)) begin
                    wbb0_v     <= 1'b1;
                    wbb0_baddr <= {vic_tag, ins_set};
                    wbb0_data  <= vic_dat;
                end else begin
                    wbb1_v     <= 1'b1;
                    wbb1_baddr <= {vic_tag, ins_set};
                    wbb1_data  <= vic_dat;
                end
            end

            if (m0_rf_now) begin
                m0_line     <= dram_rresp_data;
                m0_refilled <= 1'b1;
            end else begin
                m1_line     <= dram_rresp_data;
                m1_refilled <= 1'b1;
            end
        end else if (req_valid && req_ready) begin
            // 6. Serve Accepted Upstream Request
            if (act_rd_hit) begin
                plru_arr[req_set] <= upd_plru(plru_arr[req_set], hit_way);
            end else if (act_wr_hit) begin
                case (hit_way)
                    2'd0: begin way0_dat[req_set] <= req_wdata; way0_d[req_set] <= 1'b1; end
                    2'd1: begin way1_dat[req_set] <= req_wdata; way1_d[req_set] <= 1'b1; end
                    2'd2: begin way2_dat[req_set] <= req_wdata; way2_d[req_set] <= 1'b1; end
                    2'd3: begin way3_dat[req_set] <= req_wdata; way3_d[req_set] <= 1'b1; end
                endcase
                plru_arr[req_set] <= upd_plru(plru_arr[req_set], hit_way);
            end else if (act_wr_alloc) begin
                // Direct 128-bit dirty line install on L1 writeback miss!
                case (ins_vway)
                    2'd0: begin way0_v[ins_set] <= 1'b1; way0_d[ins_set] <= 1'b1; way0_tag[ins_set] <= ins_ctag; way0_dat[ins_set] <= req_wdata; end
                    2'd1: begin way1_v[ins_set] <= 1'b1; way1_d[ins_set] <= 1'b1; way1_tag[ins_set] <= ins_ctag; way1_dat[ins_set] <= req_wdata; end
                    2'd2: begin way2_v[ins_set] <= 1'b1; way2_d[ins_set] <= 1'b1; way2_tag[ins_set] <= ins_ctag; way2_dat[ins_set] <= req_wdata; end
                    2'd3: begin way3_v[ins_set] <= 1'b1; way3_d[ins_set] <= 1'b1; way3_tag[ins_set] <= ins_ctag; way3_dat[ins_set] <= req_wdata; end
                endcase
                plru_arr[ins_set] <= upd_plru(plru_arr[ins_set], ins_vway);

                if (ins_need_wb) begin
                    if (!wbb0_v || (dram_wr_valid && dram_wr_ready && wbb0_v)) begin
                        wbb0_v     <= 1'b1;
                        wbb0_baddr <= {vic_tag, ins_set};
                        wbb0_data  <= vic_dat;
                    end else begin
                        wbb1_v     <= 1'b1;
                        wbb1_baddr <= {vic_tag, ins_set};
                        wbb1_data  <= vic_dat;
                    end
                end
            end else if (act_rd_coal) begin
                if (m0_hit) begin
                    w_v[{1'b0, m0_free_wbit}]  <= 1'b1;
                    w_id[{1'b0, m0_free_wbit}] <= req_id;
                end else begin
                    w_v[{1'b1, m1_free_wbit}]  <= 1'b1;
                    w_id[{1'b1, m1_free_wbit}] <= req_id;
                end
            end else if (act_rd_miss) begin
                if (!m0_v) begin
                    m0_v        <= 1'b1;
                    m0_sent     <= 1'b0;
                    m0_refilled <= 1'b0;
                    m0_baddr    <= req_baddr;
                    w_v[0]      <= 1'b1;
                    w_id[0]     <= req_id;
                end else begin
                    m1_v        <= 1'b1;
                    m1_sent     <= 1'b0;
                    m1_refilled <= 1'b0;
                    m1_baddr    <= req_baddr;
                    w_v[2]      <= 1'b1;
                    w_id[2]     <= req_id;
                end
            end
        end
    end
end

endmodule
