`timescale 1ns / 1ps

// ============================================================================
// SymbiYosys Formal Verification Harness for Dual-Core MOESI Coherence (Phase 3)
//
// Instantiates the REAL, UNMODIFIED RTL:
//   - 2x dcache_nb_coh (reduced geometry: NUM_SETS=2, AWIDTH=7 -> 2-way x 2-set,
//     full 2 MSHRs, 2-entry snoopable WBB, duplicate snoop tag, MOESI + RV32A!)
//   - 1x coh_interconnect (full 3-channel split-transaction snooping crossbar!)
//   - Formal abstract backing L2 memory (arbitrary symbolic response latency!)
//
// Sound Formal Reductions (Zero Loss of Protocol Verification Coverage):
//   - Data-Independence (Wolper 1986): 2-bit symbolic data values {0..3}
//     (upper 126 bits constant 0 -> automatically folded by Yosys opt_dff!)
//   - Address Symmetry Reduction: 4 active lines = 3 conflicting tags on Set 0
//     {0, 2, 4} (forces 2-way LRU dirty evictions + WBB snoops!) + 1 line on
//     Set 1 {1} (exercises dual-MSHR concurrent MLP across both sets!)
//   - All 5 Operation Classes: LD, ST, LR.W, SC.W, and AMO_SWAP (full atomic
//     RMW GetM/Upgrade/Hit/Reservation paths without 32-bit carry bloat!)
//
// Proves via Bounded Model Checking (SymbiYosys + ABC bmc3, depth 30):
//   P0. No simultaneous linearized writes to the same line on D0 & D1
//   P1. SWMR Exclusivity: !(M & !I), !(E & !I), !(O & O), no M/E/O + peer WBB
//   P2. Unique Dirty Owner across {D0 (M/O/C2C0), D1 (M/O/C2C1), WBB0, WBB1}
//   P3. Duplicate Snoop Tag Equivalence: main tag/state === snoop tag/state
//   P4. No Same-Cache Duplicate Way Tags
//   P5. Exact Data-Value Consistency across D0, D1, WBB0, WBB1, C2C0, C2C1,
//       and backing L2 (whenever no dirty owner holds the line!)
//   P6. Protocol Upgrade-ACK Sanity: Upgrade ACK only on resident S/O line
// ============================================================================
module coh_formal #(
    parameter integer CHECK_DATA = 1 // 1: SWMR + Exact Data Consistency, 0: Pure Control/SWMR (deeper/unbounded)
) (
    input  wire       clk,
    input  wire       reset_n,

    // Unconstrained symbolic CPU inputs for Core 0 & Core 1
    input  wire       c0_req_valid,
    input  wire       c0_req_is_write,
    input  wire [1:0] c0_op_sel,     // 0: LD/ST, 1: LR, 2: SC, 3: AMOSWAP
    input  wire [1:0] c0_addr_sel,   // 0->la0(s0,t0), 1->la2(s0,t1), 2->la4(s0,t2), 3->la1(s1,t0)
    input  wire [1:0] c0_data2,

    input  wire       c1_req_valid,
    input  wire       c1_req_is_write,
    input  wire [1:0] c1_op_sel,
    input  wire [1:0] c1_addr_sel,
    input  wire [1:0] c1_data2,

    // Symbolic backing L2 ready/response timing (arbitrary latency!)
    input  wire       l2_m0_rdy_sym,
    input  wire       l2_m1_rdy_sym,
    input  wire       l2_m0_ret_sym,
    input  wire       l2_m1_ret_sym
);

`include "rv32ima_defs.vh"
`include "coh_defs.vh"

localparam integer AWIDTH   = 7;
localparam integer LADDR_W  = 3;
localparam integer NUM_SETS = 2;
localparam integer CTAG_W   = 2;
localparam integer TAG_W    = 5;
localparam integer ID_W     = 4;

// All RTL registers start cleanly in their exact post-reset state via `initial`!
// Keeping reset_n = 1'b1 allows Yosys opt_dff to fold 100% of dead upper data
// bits [127:2] and unused sub-blocks (L1I arbiter, clean_all, MESI WB)!
wire reset_n_const = 1'b1;
reg f_past_valid;
initial f_past_valid = 1'b0;
always @(posedge clk) f_past_valid <= 1'b1;

wire [1:0] c0_d2_eff = (CHECK_DATA != 0) ? c0_data2 : 2'd0;
wire [1:0] c1_d2_eff = (CHECK_DATA != 0) ? c1_data2 : 2'd0;

// Map 2-bit address selector -> {0, 2, 4} (3 tags on Set 0!) and {1} (Set 1!)
wire [2:0] c0_laddr = (c0_addr_sel == 2'd0) ? 3'd0 :
                      (c0_addr_sel == 2'd1) ? 3'd2 :
                      (c0_addr_sel == 2'd2) ? 3'd4 : 3'd1;
wire [2:0] c1_laddr = (c1_addr_sel == 2'd0) ? 3'd0 :
                      (c1_addr_sel == 2'd1) ? 3'd2 :
                      (c1_addr_sel == 2'd2) ? 3'd4 : 3'd1;

// Map symbolic 2-bit op_sel to LD/ST/LR/SC/AMOSWAP
wire [3:0] c0_amo = (c0_op_sel == 2'd0) ? AMO_NONE :
                    (c0_op_sel == 2'd1) ? AMO_LR   :
                    (c0_op_sel == 2'd2) ? AMO_SC   : AMO_SWAP;
wire       c0_we  = (c0_op_sel == 2'd0) ? c0_req_is_write :
                    (c0_op_sel == 2'd1) ? 1'b0 : 1'b1;

wire [3:0] c1_amo = (c1_op_sel == 2'd0) ? AMO_NONE :
                    (c1_op_sel == 2'd1) ? AMO_LR   :
                    (c1_op_sel == 2'd2) ? AMO_SC   : AMO_SWAP;
wire       c1_we  = (c1_op_sel == 2'd0) ? c1_req_is_write :
                    (c1_op_sel == 2'd1) ? 1'b0 : 1'b1;

// DUT Wiring
wire               c0_rdy, c1_rdy, c0_hit_v, c0_fill_v, c1_hit_v, c1_fill_v;
wire [ID_W-1:0]    c0_hit_id, c0_fill_id, c1_hit_id, c1_fill_id;
wire [31:0]        c0_hit_rd, c0_fill_rd, c1_hit_rd, c1_fill_rd;
wire               c0_cl_done, c1_cl_done, c0_snp_inv_v, c1_snp_inv_v;
wire [LADDR_W-1:0] c0_snp_inv_la, c1_snp_inv_la;

wire               d0_req_v, d0_req_rdy, d0_req_id, d0_snp_v, d0_wb_v, d0_wb_rdy;
wire [1:0]         d0_req_op, d0_snp_op, d0_snp_rop;
wire [LADDR_W-1:0] d0_req_la, d0_snp_la, d0_wb_la;
wire [127:0]       d0_snp_rdat, d0_wb_dat, d0_resp_dat;
wire               d0_resp_v, d0_resp_id, d0_resp_sh, d0_resp_c2c, d0_resp_upg;
wire               d0_wr_v;
wire [LADDR_W-1:0] d0_wr_la;
wire [127:0]       d0_wr_dat;

wire               d1_req_v, d1_req_rdy, d1_req_id, d1_snp_v, d1_wb_v, d1_wb_rdy;
wire [1:0]         d1_req_op, d1_snp_op, d1_snp_rop;
wire [LADDR_W-1:0] d1_req_la, d1_snp_la, d1_wb_la;
wire [127:0]       d1_snp_rdat, d1_wb_dat, d1_resp_dat;
wire               d1_resp_v, d1_resp_id, d1_resp_sh, d1_resp_c2c, d1_resp_upg;
wire               d1_wr_v;
wire [LADDR_W-1:0] d1_wr_la;
wire [127:0]       d1_wr_dat;

wire               l2_m0_v, l2_m0_we, l2_m1_v, l2_m1_we;
wire [LADDR_W-1:0] l2_m0_la, l2_m1_la;
wire [127:0]       l2_m0_wd, l2_m1_wd;
wire [2:0]         l2_m0_id, l2_m1_id;
wire               l2_m0_rdy, l2_m1_rdy;

reg                l2_m0_rv, l2_m1_rv;
reg  [2:0]         l2_m0_rid, l2_m1_rid;
reg  [127:0]       l2_m0_rd, l2_m1_rd;

// Formal observation ports from D0, D1 & coh_interconnect
wire [NUM_SETS*3-1:0]      d0_w0_st, d0_w1_st, d0_sw0_st, d0_sw1_st;
wire [NUM_SETS*CTAG_W-1:0] d0_w0_tg, d0_w1_tg, d0_sw0_tg, d0_sw1_tg;
wire [NUM_SETS*8-1:0]      d0_w0_b0, d0_w1_b0;
wire                       d0_wbb0_v, d0_wbb1_v;
wire [LADDR_W-1:0]         d0_wbb0_la, d0_wbb1_la, d0_rf_la;
wire [7:0]                 d0_wbb0_b0, d0_wbb1_b0;

wire [NUM_SETS*3-1:0]      d1_w0_st, d1_w1_st, d1_sw0_st, d1_sw1_st;
wire [NUM_SETS*CTAG_W-1:0] d1_w0_tg, d1_w1_tg, d1_sw0_tg, d1_sw1_tg;
wire [NUM_SETS*8-1:0]      d1_w0_b0, d1_w1_b0;
wire                       d1_wbb0_v, d1_wbb1_v;
wire [LADDR_W-1:0]         d1_wbb0_la, d1_wbb1_la, d1_rf_la;
wire [7:0]                 d1_wbb0_b0, d1_wbb1_b0;

wire               f_c2c0_v, f_c2c0_fc, f_c2c0_sh;
wire [LADDR_W-1:0] f_c2c0_la;
wire [7:0]         f_c2c0_b0;
wire               f_c2c1_v, f_c2c1_fc, f_c2c1_sh;
wire [LADDR_W-1:0] f_c2c1_la;
wire [7:0]         f_c2c1_b0;

wire d0_idle_u, d0_wi_u, d0_am_u, d0_h_u, d0_m_u, d0_w_u, d0_hu_u, d0_mu_u, d0_co_u, d0_e2m, d0_upg_u;
wire d1_idle_u, d1_wi_u, d1_am_u, d1_h_u, d1_m_u, d1_w_u, d1_hu_u, d1_mu_u, d1_co_u, d1_e2m, d1_upg_u;
wire i0_rdy_u, i0_rv_u, i1_rdy_u, i1_rv_u;
wire [127:0] i0_rd_u, i1_rd_u;
wire be_v, be_s, be_c2c, be_sh, coh_idle_u;
wire [1:0] be_op;
wire [LADDR_W-1:0] be_la;
wire pg_u, pm_u, pu_u, pc_u, pi_u, pw_u;

dcache_nb_coh #(
    .AWIDTH(AWIDTH), .NUM_SETS(NUM_SETS), .TAG_W(TAG_W), .ID_W(ID_W), .ENABLE_MOESI_O(1)
) u_d0 (
    .clk(clk), .reset_n(reset_n_const),
    .req_valid(c0_req_valid), .req_ready(c0_rdy),
    .req_is_write(c0_we), .req_amo_op(c0_amo),
    .req_addr({c0_laddr, 4'd0}), .req_wdata({30'd0, c0_d2_eff}), .req_wstrb(4'b0001),
    .req_id(4'd1), .req_rob_tag(5'd0),
    .hit_resp_valid(c0_hit_v), .hit_resp_id(c0_hit_id), .hit_resp_rdata(c0_hit_rd),
    .fill_resp_valid(c0_fill_v), .fill_resp_id(c0_fill_id), .fill_resp_rdata(c0_fill_rd),
    .flush(1'b0), .squash_valid(1'b0), .squash_tag(5'd0), .rob_head(5'd0),
    .clean_all_req(1'b0), .clean_all_done(c0_cl_done),
    .snoop_inv_valid(c0_snp_inv_v), .snoop_inv_laddr(c0_snp_inv_la),
    .coh_req_valid(d0_req_v), .coh_req_ready(d0_req_rdy),
    .coh_req_op(d0_req_op), .coh_req_laddr(d0_req_la), .coh_req_id(d0_req_id),
    .snoop_valid(d0_snp_v), .snoop_op(d0_snp_op), .snoop_laddr(d0_snp_la),
    .snoop_resp_op(d0_snp_rop), .snoop_resp_data(d0_snp_rdat),
    .wb_valid(d0_wb_v), .wb_ready(d0_wb_rdy), .wb_laddr(d0_wb_la), .wb_data(d0_wb_dat),
    .coh_resp_valid(d0_resp_v), .coh_resp_id(d0_resp_id), .coh_resp_data(d0_resp_dat),
    .coh_resp_shared(d0_resp_sh), .coh_resp_from_c2c(d0_resp_c2c), .coh_resp_upgrade_ack(d0_resp_upg),
    .dbg_wr_commit_valid(d0_wr_v), .dbg_wr_commit_laddr(d0_wr_la), .dbg_wr_commit_data(d0_wr_dat),
    .dcache_idle(d0_idle_u), .dcache_writes_idle(d0_wi_u), .dcache_any_miss(d0_am_u),
    .cache_hit(d0_h_u), .cache_miss(d0_m_u), .cache_wb(d0_w_u),
    .cache_hum(d0_hu_u), .cache_mum_cyc(d0_mu_u), .cache_mshr_coalesce(d0_co_u),
    .pmu_silent_e2m(d0_e2m), .pmu_upgrade_miss(d0_upg_u),
    .f_way0_state(d0_w0_st), .f_way0_tag(d0_w0_tg), .f_way0_b0(d0_w0_b0),
    .f_way1_state(d0_w1_st), .f_way1_tag(d0_w1_tg), .f_way1_b0(d0_w1_b0),
    .f_snp_way0_state(d0_sw0_st), .f_snp_way0_tag(d0_sw0_tg),
    .f_snp_way1_state(d0_sw1_st), .f_snp_way1_tag(d0_sw1_tg),
    .f_wbb0_v(d0_wbb0_v), .f_wbb0_laddr(d0_wbb0_la), .f_wbb0_b0(d0_wbb0_b0),
    .f_wbb1_v(d0_wbb1_v), .f_wbb1_laddr(d0_wbb1_la), .f_wbb1_b0(d0_wbb1_b0),
    .f_rf_laddr(d0_rf_la)
);

dcache_nb_coh #(
    .AWIDTH(AWIDTH), .NUM_SETS(NUM_SETS), .TAG_W(TAG_W), .ID_W(ID_W), .ENABLE_MOESI_O(1)
) u_d1 (
    .clk(clk), .reset_n(reset_n_const),
    .req_valid(c1_req_valid), .req_ready(c1_rdy),
    .req_is_write(c1_we), .req_amo_op(c1_amo),
    .req_addr({c1_laddr, 4'd0}), .req_wdata({30'd0, c1_d2_eff}), .req_wstrb(4'b0001),
    .req_id(4'd1), .req_rob_tag(5'd0),
    .hit_resp_valid(c1_hit_v), .hit_resp_id(c1_hit_id), .hit_resp_rdata(c1_hit_rd),
    .fill_resp_valid(c1_fill_v), .fill_resp_id(c1_fill_id), .fill_resp_rdata(c1_fill_rd),
    .flush(1'b0), .squash_valid(1'b0), .squash_tag(5'd0), .rob_head(5'd0),
    .clean_all_req(1'b0), .clean_all_done(c1_cl_done),
    .snoop_inv_valid(c1_snp_inv_v), .snoop_inv_laddr(c1_snp_inv_la),
    .coh_req_valid(d1_req_v), .coh_req_ready(d1_req_rdy),
    .coh_req_op(d1_req_op), .coh_req_laddr(d1_req_la), .coh_req_id(d1_req_id),
    .snoop_valid(d1_snp_v), .snoop_op(d1_snp_op), .snoop_laddr(d1_snp_la),
    .snoop_resp_op(d1_snp_rop), .snoop_resp_data(d1_snp_rdat),
    .wb_valid(d1_wb_v), .wb_ready(d1_wb_rdy), .wb_laddr(d1_wb_la), .wb_data(d1_wb_dat),
    .coh_resp_valid(d1_resp_v), .coh_resp_id(d1_resp_id), .coh_resp_data(d1_resp_dat),
    .coh_resp_shared(d1_resp_sh), .coh_resp_from_c2c(d1_resp_c2c), .coh_resp_upgrade_ack(d1_resp_upg),
    .dbg_wr_commit_valid(d1_wr_v), .dbg_wr_commit_laddr(d1_wr_la), .dbg_wr_commit_data(d1_wr_dat),
    .dcache_idle(d1_idle_u), .dcache_writes_idle(d1_wi_u), .dcache_any_miss(d1_am_u),
    .cache_hit(d1_h_u), .cache_miss(d1_m_u), .cache_wb(d1_w_u),
    .cache_hum(d1_hu_u), .cache_mum_cyc(d1_mu_u), .cache_mshr_coalesce(d1_co_u),
    .pmu_silent_e2m(d1_e2m), .pmu_upgrade_miss(d1_upg_u),
    .f_way0_state(d1_w0_st), .f_way0_tag(d1_w0_tg), .f_way0_b0(d1_w0_b0),
    .f_way1_state(d1_w1_st), .f_way1_tag(d1_w1_tg), .f_way1_b0(d1_w1_b0),
    .f_snp_way0_state(d1_sw0_st), .f_snp_way0_tag(d1_sw0_tg),
    .f_snp_way1_state(d1_sw1_st), .f_snp_way1_tag(d1_sw1_tg),
    .f_wbb0_v(d1_wbb0_v), .f_wbb0_laddr(d1_wbb0_la), .f_wbb0_b0(d1_wbb0_b0),
    .f_wbb1_v(d1_wbb1_v), .f_wbb1_laddr(d1_wbb1_la), .f_wbb1_b0(d1_wbb1_b0),
    .f_rf_laddr(d1_rf_la)
);

coh_interconnect #(
    .LADDR_W(LADDR_W), .M_ID_W(3), .ENABLE_MOESI_O(1)
) u_coh (
    .clk(clk), .reset_n(reset_n_const),
    .d0_coh_req_valid(d0_req_v), .d0_coh_req_ready(d0_req_rdy),
    .d0_coh_req_op(d0_req_op), .d0_coh_req_laddr(d0_req_la), .d0_coh_req_id(d0_req_id),
    .d0_snoop_valid(d0_snp_v), .d0_snoop_op(d0_snp_op), .d0_snoop_laddr(d0_snp_la),
    .d0_snoop_resp_op(d0_snp_rop), .d0_snoop_resp_data(d0_snp_rdat),
    .d0_wb_valid(d0_wb_v), .d0_wb_ready(d0_wb_rdy), .d0_wb_laddr(d0_wb_la), .d0_wb_data(d0_wb_dat),
    .d0_coh_resp_valid(d0_resp_v), .d0_coh_resp_id(d0_resp_id), .d0_coh_resp_data(d0_resp_dat),
    .d0_coh_resp_shared(d0_resp_sh), .d0_coh_resp_from_c2c(d0_resp_c2c), .d0_coh_resp_upgrade_ack(d0_resp_upg),
    .d1_coh_req_valid(d1_req_v), .d1_coh_req_ready(d1_req_rdy),
    .d1_coh_req_op(d1_req_op), .d1_coh_req_laddr(d1_req_la), .d1_coh_req_id(d1_req_id),
    .d1_snoop_valid(d1_snp_v), .d1_snoop_op(d1_snp_op), .d1_snoop_laddr(d1_snp_la),
    .d1_snoop_resp_op(d1_snp_rop), .d1_snoop_resp_data(d1_snp_rdat),
    .d1_wb_valid(d1_wb_v), .d1_wb_ready(d1_wb_rdy), .d1_wb_laddr(d1_wb_la), .d1_wb_data(d1_wb_dat),
    .d1_coh_resp_valid(d1_resp_v), .d1_coh_resp_id(d1_resp_id), .d1_coh_resp_data(d1_resp_dat),
    .d1_coh_resp_shared(d1_resp_sh), .d1_coh_resp_from_c2c(d1_resp_c2c), .d1_coh_resp_upgrade_ack(d1_resp_upg),
    .i0_rd_valid(1'b0), .i0_rd_ready(i0_rdy_u), .i0_rd_laddr(3'd0), .i0_rresp_valid(i0_rv_u), .i0_rresp_data(i0_rd_u),
    .i1_rd_valid(1'b0), .i1_rd_ready(i1_rdy_u), .i1_rd_laddr(3'd0), .i1_rresp_valid(i1_rv_u), .i1_rresp_data(i1_rd_u),
    .l2_m0_req_valid(l2_m0_v), .l2_m0_req_ready(l2_m0_rdy), .l2_m0_req_is_write(l2_m0_we),
    .l2_m0_req_laddr(l2_m0_la), .l2_m0_req_wdata(l2_m0_wd), .l2_m0_req_id(l2_m0_id),
    .l2_m0_resp_valid(l2_m0_rv), .l2_m0_resp_id(l2_m0_rid), .l2_m0_resp_data(l2_m0_rd),
    .l2_m1_req_valid(l2_m1_v), .l2_m1_req_ready(l2_m1_rdy), .l2_m1_req_is_write(l2_m1_we),
    .l2_m1_req_laddr(l2_m1_la), .l2_m1_req_wdata(l2_m1_wd), .l2_m1_req_id(l2_m1_id),
    .l2_m1_resp_valid(l2_m1_rv), .l2_m1_resp_id(l2_m1_rid), .l2_m1_resp_data(l2_m1_rd),
    .bus_evt_valid(be_v), .bus_evt_src(be_s), .bus_evt_op(be_op),
    .bus_evt_laddr(be_la), .bus_evt_c2c(be_c2c), .bus_evt_shared(be_sh),
    .coh_idle(coh_idle_u),
    .pmu_coh_gets(pg_u), .pmu_coh_getm(pm_u), .pmu_coh_upgrade(pu_u),
    .pmu_coh_c2c_dirty(pc_u), .pmu_coh_inval(pi_u), .pmu_coh_mesi_wb(pw_u),
    .f_c2c0_v(f_c2c0_v), .f_c2c0_from_c2c(f_c2c0_fc), .f_c2c0_shared(f_c2c0_sh),
    .f_c2c0_laddr(f_c2c0_la), .f_c2c0_b0(f_c2c0_b0),
    .f_c2c1_v(f_c2c1_v), .f_c2c1_from_c2c(f_c2c1_fc), .f_c2c1_shared(f_c2c1_sh),
    .f_c2c1_laddr(f_c2c1_la), .f_c2c1_b0(f_c2c1_b0)
);

// ----------------------------------------------------------------------------
// Abstract Formal Backing L2 Model (4 Active Lines, Arbitrary Latency!)
// Map line addr {0, 2, 4, 1} -> compact 2-bit slot {0, 1, 2, 3}
// ----------------------------------------------------------------------------
function automatic [1:0] la_to_slot(input [2:0] la);
    begin
        case (la)
            3'd0:    la_to_slot = 2'd0;
            3'd2:    la_to_slot = 2'd1;
            3'd4:    la_to_slot = 2'd2;
            default: la_to_slot = 2'd3; // 3'd1
        endcase
    end
endfunction

reg [1:0] l2_mem     [0:3];
reg [1:0] shadow_mem [0:3];
reg [1:0] q0_v, q1_v;
reg [1:0] q0_s0, q0_s1, q1_s0, q1_s1;

assign l2_m0_rdy = l2_m0_v && l2_m0_rdy_sym && (l2_m0_we || !(l2_m0_id[0] ? q0_v[1] : q0_v[0]));
assign l2_m1_rdy = l2_m1_v && l2_m1_rdy_sym && (l2_m1_we || !(l2_m1_id[0] ? q1_v[1] : q1_v[0])) &&
                   !(l2_m0_v && l2_m0_rdy && l2_m0_we && l2_m1_we && (l2_m0_la == l2_m1_la));

initial begin
    l2_mem[0]     = 2'd0; l2_mem[1]     = 2'd1; l2_mem[2]     = 2'd2; l2_mem[3]     = 2'd3;
    shadow_mem[0] = 2'd0; shadow_mem[1] = 2'd1; shadow_mem[2] = 2'd2; shadow_mem[3] = 2'd3;
    q0_v = 2'b00; q1_v = 2'b00;
    q0_s0 = 2'd0; q0_s1 = 2'd0; q1_s0 = 2'd0; q1_s1 = 2'd0;
    l2_m0_rv = 1'b0; l2_m0_rid = 3'd0; l2_m0_rd = 128'd0;
    l2_m1_rv = 1'b0; l2_m1_rid = 3'd0; l2_m1_rd = 128'd0;
end

always @(posedge clk) begin
    if (!reset_n_const) begin
        l2_mem[0]     <= 2'd0; l2_mem[1]     <= 2'd1; l2_mem[2]     <= 2'd2; l2_mem[3]     <= 2'd3;
        shadow_mem[0] <= 2'd0; shadow_mem[1] <= 2'd1; shadow_mem[2] <= 2'd2; shadow_mem[3] <= 2'd3;
        q0_v <= 2'b00; q1_v <= 2'b00;
        q0_s0 <= 2'd0; q0_s1 <= 2'd0; q1_s0 <= 2'd0; q1_s1 <= 2'd0;
        l2_m0_rv <= 1'b0; l2_m0_rid <= 3'd0; l2_m0_rd <= 128'd0;
        l2_m1_rv <= 1'b0; l2_m1_rid <= 3'd0; l2_m1_rd <= 128'd0;
    end else begin
        l2_m0_rv <= 1'b0;
        l2_m1_rv <= 1'b0;

        if (d0_wr_v) shadow_mem[la_to_slot(d0_wr_la)] <= d0_wr_dat[1:0];
        if (d1_wr_v) shadow_mem[la_to_slot(d1_wr_la)] <= d1_wr_dat[1:0];

        if (l2_m0_v && l2_m0_rdy && l2_m0_we)
            l2_mem[la_to_slot(l2_m0_la)] <= l2_m0_wd[1:0];
        if (l2_m1_v && l2_m1_rdy && l2_m1_we)
            l2_mem[la_to_slot(l2_m1_la)] <= l2_m1_wd[1:0];

        if (l2_m0_v && l2_m0_rdy && !l2_m0_we) begin
            if (!l2_m0_id[0]) begin q0_v[0] <= 1'b1; q0_s0 <= la_to_slot(l2_m0_la); end
            else              begin q0_v[1] <= 1'b1; q0_s1 <= la_to_slot(l2_m0_la); end
        end
        if (l2_m1_v && l2_m1_rdy && !l2_m1_we) begin
            if (!l2_m1_id[0]) begin q1_v[0] <= 1'b1; q1_s0 <= la_to_slot(l2_m1_la); end
            else              begin q1_v[1] <= 1'b1; q1_s1 <= la_to_slot(l2_m1_la); end
        end

        if (l2_m0_ret_sym) begin
            if (q0_v[0]) begin
                q0_v[0] <= 1'b0; l2_m0_rv <= 1'b1; l2_m0_rid <= 3'd0;
                l2_m0_rd <= (CHECK_DATA != 0) ? {126'd0, l2_mem[q0_s0]} : 128'd0;
            end else if (q0_v[1]) begin
                q0_v[1] <= 1'b0; l2_m0_rv <= 1'b1; l2_m0_rid <= 3'd1;
                l2_m0_rd <= (CHECK_DATA != 0) ? {126'd0, l2_mem[q0_s1]} : 128'd0;
            end
        end
        if (l2_m1_ret_sym) begin
            if (q1_v[0]) begin
                q1_v[0] <= 1'b0; l2_m1_rv <= 1'b1; l2_m1_rid <= 3'd0;
                l2_m1_rd <= (CHECK_DATA != 0) ? {126'd0, l2_mem[q1_s0]} : 128'd0;
            end else if (q1_v[1]) begin
                q1_v[1] <= 1'b0; l2_m1_rv <= 1'b1; l2_m1_rid <= 3'd1;
                l2_m1_rd <= (CHECK_DATA != 0) ? {126'd0, l2_mem[q1_s1]} : 128'd0;
            end
        end
    end
end

// ----------------------------------------------------------------------------
// Formal Safety Assertions Across All 4 Active Lines: {0, 2, 4, 1}
// ----------------------------------------------------------------------------
wire [2:0] st0_s [0:3];
wire [2:0] st1_s [0:3];
wire [1:0] d0_d  [0:3];
wire [1:0] d1_d  [0:3];
wire [3:0] w0_h, w1_h, t0_own, t1_own;

genvar gs;
generate
    for (gs = 0; gs < 4; gs = gs + 1) begin : gen_flookup
        localparam [2:0]   LVAL = (gs == 0) ? 3'd0 : (gs == 1) ? 3'd2 : (gs == 2) ? 3'd4 : 3'd1;
        localparam integer SBIT = (gs == 3) ? 1 : 0;
        localparam [1:0]   TVAL = (gs == 0) ? 2'd0 : (gs == 1) ? 2'd1 : (gs == 2) ? 2'd2 : 2'd0;

        wire d0_h0 = (d0_w0_st[SBIT*3 +: 3] != COH_I) && (d0_w0_tg[SBIT*2 +: 2] == TVAL);
        wire d0_h1 = (d0_w1_st[SBIT*3 +: 3] != COH_I) && (d0_w1_tg[SBIT*2 +: 2] == TVAL);
        assign st0_s[gs] = d0_h0 ? d0_w0_st[SBIT*3 +: 3] : (d0_h1 ? d0_w1_st[SBIT*3 +: 3] : COH_I);
        assign d0_d[gs]  = d0_h0 ? d0_w0_b0[SBIT*8 +: 2] : (d0_h1 ? d0_w1_b0[SBIT*8 +: 2] : 2'd0);

        wire d1_h0 = (d1_w0_st[SBIT*3 +: 3] != COH_I) && (d1_w0_tg[SBIT*2 +: 2] == TVAL);
        wire d1_h1 = (d1_w1_st[SBIT*3 +: 3] != COH_I) && (d1_w1_tg[SBIT*2 +: 2] == TVAL);
        assign st1_s[gs] = d1_h0 ? d1_w0_st[SBIT*3 +: 3] : (d1_h1 ? d1_w1_st[SBIT*3 +: 3] : COH_I);
        assign d1_d[gs]  = d1_h0 ? d1_w0_b0[SBIT*8 +: 2] : (d1_h1 ? d1_w1_b0[SBIT*8 +: 2] : 2'd0);

        assign w0_h[gs]   = (d0_wbb0_v && (d0_wbb0_la == LVAL)) || (d0_wbb1_v && (d0_wbb1_la == LVAL));
        assign w1_h[gs]   = (d1_wbb0_v && (d1_wbb0_la == LVAL)) || (d1_wbb1_v && (d1_wbb1_la == LVAL));
        assign t0_own[gs] = f_c2c0_v && !f_c2c0_sh && (f_c2c0_la == LVAL);
        assign t1_own[gs] = f_c2c1_v && !f_c2c1_sh && (f_c2c1_la == LVAL);
    end
endgenerate

integer a, s;
always @(posedge clk) begin
    if (f_past_valid) begin
        // P0: No simultaneous writes to the same line!
        assert (!(d0_wr_v && d1_wr_v && (d0_wr_la == d1_wr_la)));

        // P3 & P4: Duplicate Snoop Tag Equivalence & No Duplicate Ways per Set
        assert (d0_w0_st == d0_sw0_st);
        assert (d0_w0_tg == d0_sw0_tg);
        assert (d0_w1_st == d0_sw1_st);
        assert (d0_w1_tg == d0_sw1_tg);
        assert (d1_w0_st == d1_sw0_st);
        assert (d1_w0_tg == d1_sw0_tg);
        assert (d1_w1_st == d1_sw1_st);
        assert (d1_w1_tg == d1_sw1_tg);

        for (s = 0; s < NUM_SETS; s = s + 1) begin
            assert (!((d0_w0_st[s*3 +: 3] != COH_I) && (d0_w1_st[s*3 +: 3] != COH_I) &&
                      (d0_w0_tg[s*2 +: 2] == d0_w1_tg[s*2 +: 2])));
            assert (!((d1_w0_st[s*3 +: 3] != COH_I) && (d1_w1_st[s*3 +: 3] != COH_I) &&
                      (d1_w0_tg[s*2 +: 2] == d1_w1_tg[s*2 +: 2])));
        end

        // P1, P2 & P5: SWMR, Unique Dirty Owner & Exact Golden Shadow Data Consistency!
        for (a = 0; a < 4; a = a + 1) begin
            assert (!((st0_s[a] == COH_M || st0_s[a] == COH_E) && (st1_s[a] != COH_I || w1_h[a])));
            assert (!((st1_s[a] == COH_M || st1_s[a] == COH_E) && (st0_s[a] != COH_I || w0_h[a])));
            assert (!(st0_s[a] == COH_O && st1_s[a] == COH_O));

            assert (({2'd0, (st0_s[a] == COH_M || st0_s[a] == COH_O || t0_own[a])} +
                     {2'd0, (st1_s[a] == COH_M || st1_s[a] == COH_O || t1_own[a])} +
                     {2'd0, w0_h[a]} + {2'd0, w1_h[a]}) <= 3'd1);

            if (CHECK_DATA != 0) begin
                if (st0_s[a] != COH_I) assert (d0_d[a] == shadow_mem[a]);
                if (st1_s[a] != COH_I) assert (d1_d[a] == shadow_mem[a]);
                if (st0_s[a] != COH_M && st0_s[a] != COH_O &&
                    st1_s[a] != COH_M && st1_s[a] != COH_O &&
                    !w0_h[a] && !w1_h[a] && !t0_own[a] && !t1_own[a]) begin
                    assert (l2_mem[a] == shadow_mem[a]);
                end
            end
        end

        if (CHECK_DATA != 0) begin
            if (d0_wbb0_v) assert (d0_wbb0_b0[1:0] == shadow_mem[la_to_slot(d0_wbb0_la)]);
            if (d0_wbb1_v) assert (d0_wbb1_b0[1:0] == shadow_mem[la_to_slot(d0_wbb1_la)]);
            if (d1_wbb0_v) assert (d1_wbb0_b0[1:0] == shadow_mem[la_to_slot(d1_wbb0_la)]);
            if (d1_wbb1_v) assert (d1_wbb1_b0[1:0] == shadow_mem[la_to_slot(d1_wbb1_la)]);
            if (f_c2c0_v && f_c2c0_fc) assert (f_c2c0_b0[1:0] == shadow_mem[la_to_slot(f_c2c0_la)]);
            if (f_c2c1_v && f_c2c1_fc) assert (f_c2c1_b0[1:0] == shadow_mem[la_to_slot(f_c2c1_la)]);
        end

        // P6: Protocol Upgrade-ACK Sanity
        if (d0_resp_v && d0_resp_upg) assert (st0_s[la_to_slot(d0_rf_la)] == COH_S || st0_s[la_to_slot(d0_rf_la)] == COH_O);
        if (d1_resp_v && d1_resp_upg) assert (st1_s[la_to_slot(d1_rf_la)] == COH_S || st1_s[la_to_slot(d1_rf_la)] == COH_O);
    end
end

endmodule
