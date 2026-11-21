`timescale 1ns / 1ps

// ============================================================================
// Adversarial Dual-Cache MOESI Coherence Subsystem Stress Testbench (Phase 3)
//
// Full Subsystem Under Test:
//   D0 (4KB MOESI L1D) + D1 (4KB MOESI L1D)
//     <-> coh_interconnect (3-Channel Split-Transaction Snooping Crossbar)
//     <-> l2_cache_shared  (32KB 4-Way 2-Bank NINE Unified L2)
//     <-> main_memory_dram (256KB 4-Outstanding Pipelined DRAM + LFSR Jitter)
//     + coh_checker        (Runtime SWMR, Golden Shadow & Watchdog Monitor)
//
// Directed Tests (1-8) + 50,000-Op Randomized Dual-Core Stress (Test 9):
//   1. Exclusive E Detection + Silent E -> M Store Upgrade (0 bus traffic!)
//   2. MOESI Dirty C2C Sharing (M -> O on peer GetS, 0 L2 WB in MOESI!)
//   3. Simultaneous Dual-Upgrade Race (SM_AD + SM_AD -> loser promotes to GetM!)
//   4. Owner Upgrade Race (OM_AD + SM_AD -> both winner directions tested!)
//   5. WBB Snoop Intervention (MI_A/OI_A C2C supply + stale-WBB cancel!)
//   6. Cross-Core LR.W / SC.W Reservation Invalidation & Uncontended Success
//   7. Contended AMOADD.W Hammer (200 ops x 2 cores = exact +400, 0 lost!)
//   8. FENCE.I Dirty Clean-All Sweep under live peer snooping
//   9. 50,000-Op Randomized Mixed LD/ST/LR/SC/AMO/Clean-All w/ DRAM Jitter
//      + Every-Cycle SWMR, Duplicate-Tag, Unique-Owner & 128-bit Shadow Checks!
// ============================================================================
module tb_coh_subsystem;

parameter integer ENABLE_MOESI_O = 1;
parameter integer NUM_RAND_OPS   = 50000;

`include "rv32ima_defs.vh"
`include "coh_defs.vh"

localparam integer AWIDTH   = 20;
localparam integer LADDR_W  = 16;
localparam integer NUM_SETS = 128; // 4 KB per L1D
localparam integer TAG_W    = 5;
localparam integer ID_W     = 7;

reg clk, reset_n, jitter_en;

// Core 0 CPU <-> D0 signals
reg                c0_req_v, c0_req_we, c0_clean_req;
wire               c0_req_rdy, c0_clean_done;
reg  [3:0]         c0_req_amo;
reg  [AWIDTH-1:0]  c0_req_addr;
reg  [31:0]        c0_req_wdata;
reg  [3:0]         c0_req_wstrb;
reg  [ID_W-1:0]    c0_req_id;
wire               c0_hit_v, c0_fill_v, c0_snp_inv_v;
wire [ID_W-1:0]    c0_hit_id, c0_fill_id;
wire [31:0]        c0_hit_rdata, c0_fill_rdata;
wire [LADDR_W-1:0] c0_snp_inv_la;

// Core 1 CPU <-> D1 signals
reg                c1_req_v, c1_req_we, c1_clean_req;
wire               c1_req_rdy, c1_clean_done;
reg  [3:0]         c1_req_amo;
reg  [AWIDTH-1:0]  c1_req_addr;
reg  [31:0]        c1_req_wdata;
reg  [3:0]         c1_req_wstrb;
reg  [ID_W-1:0]    c1_req_id;
wire               c1_hit_v, c1_fill_v, c1_snp_inv_v;
wire [ID_W-1:0]    c1_hit_id, c1_fill_id;
wire [31:0]        c1_hit_rdata, c1_fill_rdata;
wire [LADDR_W-1:0] c1_snp_inv_la;

// D0 <-> Interconnect
wire               d0_coh_req_v, d0_coh_req_rdy, d0_coh_req_id;
wire [1:0]         d0_coh_req_op;
wire [LADDR_W-1:0] d0_coh_req_la;
wire               d0_snp_v;
wire [1:0]         d0_snp_op, d0_snp_resp_op;
wire [LADDR_W-1:0] d0_snp_la;
wire [127:0]       d0_snp_resp_dat;
wire               d0_wb_v, d0_wb_rdy;
wire [LADDR_W-1:0] d0_wb_la;
wire [127:0]       d0_wb_dat;
wire               d0_resp_v, d0_resp_id, d0_resp_sh, d0_resp_c2c, d0_resp_upg;
wire [127:0]       d0_resp_dat;
wire               d0_wr_v;
wire [LADDR_W-1:0] d0_wr_la;
wire [127:0]       d0_wr_dat;
wire               d0_idle, d0_wr_idle, d0_any_m, d0_hit_p, d0_miss_p, d0_wb_p;
wire               d0_hum_p, d0_mum_p, d0_coal_p, d0_e2m_p, d0_upg_p;

// D1 <-> Interconnect
wire               d1_coh_req_v, d1_coh_req_rdy, d1_coh_req_id;
wire [1:0]         d1_coh_req_op;
wire [LADDR_W-1:0] d1_coh_req_la;
wire               d1_snp_v;
wire [1:0]         d1_snp_op, d1_snp_resp_op;
wire [LADDR_W-1:0] d1_snp_la;
wire [127:0]       d1_snp_resp_dat;
wire               d1_wb_v, d1_wb_rdy;
wire [LADDR_W-1:0] d1_wb_la;
wire [127:0]       d1_wb_dat;
wire               d1_resp_v, d1_resp_id, d1_resp_sh, d1_resp_c2c, d1_resp_upg;
wire [127:0]       d1_resp_dat;
wire               d1_wr_v;
wire [LADDR_W-1:0] d1_wr_la;
wire [127:0]       d1_wr_dat;
wire               d1_idle, d1_wr_idle, d1_any_m, d1_hit_p, d1_miss_p, d1_wb_p;
wire               d1_hum_p, d1_mum_p, d1_coal_p, d1_e2m_p, d1_upg_p;

// Interconnect <-> L2 <-> DRAM
wire               l2_m0_v, l2_m0_rdy, l2_m0_we, l2_m0_rv;
wire [LADDR_W-1:0] l2_m0_la;
wire [127:0]       l2_m0_wd, l2_m0_rd;
wire [2:0]         l2_m0_id, l2_m0_rid;

wire               l2_m1_v, l2_m1_rdy, l2_m1_we, l2_m1_rv;
wire [LADDR_W-1:0] l2_m1_la;
wire [127:0]       l2_m1_wd, l2_m1_rd;
wire [2:0]         l2_m1_id, l2_m1_rid;

wire               dram_rd_v, dram_rd_rdy, dram_rr_v, dram_wr_v, dram_wr_rdy;
wire [LADDR_W-1:0] dram_rd_la, dram_wr_la;
wire [1:0]         dram_rd_id, dram_rr_id;
wire [127:0]       dram_rr_dat, dram_wr_dat;

wire               bus_evt_v, bus_evt_c2c, bus_evt_sh;
wire [1:0]         bus_evt_src, bus_evt_op;
wire [LADDR_W-1:0] bus_evt_la;
wire               coh_idle, l2_idle, l2_bc, dram_rd_p, dram_wr_p;
wire [1:0]         l2_hits_p, l2_misses_p, l2_wbs_p;
wire               pmu_gets, pmu_getm, pmu_upg, pmu_c2c, pmu_inval, pmu_mesi_wb;
wire               pmu_dma_rd_u, pmu_dma_wr_u;
wire               dma_rdy_u, dma_rv_u, dma_wv_u;
wire [LADDR_W-1:0] dma_wla_u;
wire [127:0]       dma_rd_u, dma_wdat_u;
wire               i0_rdy_u, i0_rv_u, i1_rdy_u, i1_rv_u;
wire [127:0]       i0_rd_u, i1_rd_u;

// ----------------------------------------------------------------------------
// DUT Instantiations
// ----------------------------------------------------------------------------
dcache_nb_coh #(
    .AWIDTH(AWIDTH), .NUM_SETS(NUM_SETS), .TAG_W(TAG_W), .ID_W(ID_W),
    .ENABLE_MOESI_O(ENABLE_MOESI_O)
) u_d0 (
    .clk(clk), .reset_n(reset_n),
    .req_valid(c0_req_v), .req_ready(c0_req_rdy),
    .req_is_write(c0_req_we), .req_amo_op(c0_req_amo),
    .req_addr(c0_req_addr), .req_wdata(c0_req_wdata), .req_wstrb(c0_req_wstrb),
    .req_id(c0_req_id), .req_rob_tag(5'd0),
    .hit_resp_valid(c0_hit_v), .hit_resp_id(c0_hit_id), .hit_resp_rdata(c0_hit_rdata),
    .fill_resp_valid(c0_fill_v), .fill_resp_id(c0_fill_id), .fill_resp_rdata(c0_fill_rdata),
    .flush(1'b0), .squash_valid(1'b0), .squash_tag(5'd0), .rob_head(5'd0),
    .clean_all_req(c0_clean_req), .clean_all_done(c0_clean_done),
    .snoop_inv_valid(c0_snp_inv_v), .snoop_inv_laddr(c0_snp_inv_la),
    .coh_req_valid(d0_coh_req_v), .coh_req_ready(d0_coh_req_rdy),
    .coh_req_op(d0_coh_req_op), .coh_req_laddr(d0_coh_req_la), .coh_req_id(d0_coh_req_id),
    .snoop_valid(d0_snp_v), .snoop_op(d0_snp_op), .snoop_laddr(d0_snp_la),
    .snoop_resp_op(d0_snp_resp_op), .snoop_resp_data(d0_snp_resp_dat),
    .wb_valid(d0_wb_v), .wb_ready(d0_wb_rdy), .wb_laddr(d0_wb_la), .wb_data(d0_wb_dat),
    .coh_resp_valid(d0_resp_v), .coh_resp_id(d0_resp_id), .coh_resp_data(d0_resp_dat),
    .coh_resp_shared(d0_resp_sh), .coh_resp_from_c2c(d0_resp_c2c), .coh_resp_upgrade_ack(d0_resp_upg),
    .dbg_wr_commit_valid(d0_wr_v), .dbg_wr_commit_laddr(d0_wr_la), .dbg_wr_commit_data(d0_wr_dat),
    .dcache_idle(d0_idle), .dcache_writes_idle(d0_wr_idle), .dcache_any_miss(d0_any_m),
    .cache_hit(d0_hit_p), .cache_miss(d0_miss_p), .cache_wb(d0_wb_p),
    .cache_hum(d0_hum_p), .cache_mum_cyc(d0_mum_p), .cache_mshr_coalesce(d0_coal_p),
    .pmu_silent_e2m(d0_e2m_p), .pmu_upgrade_miss(d0_upg_p)
);

dcache_nb_coh #(
    .AWIDTH(AWIDTH), .NUM_SETS(NUM_SETS), .TAG_W(TAG_W), .ID_W(ID_W),
    .ENABLE_MOESI_O(ENABLE_MOESI_O)
) u_d1 (
    .clk(clk), .reset_n(reset_n),
    .req_valid(c1_req_v), .req_ready(c1_req_rdy),
    .req_is_write(c1_req_we), .req_amo_op(c1_req_amo),
    .req_addr(c1_req_addr), .req_wdata(c1_req_wdata), .req_wstrb(c1_req_wstrb),
    .req_id(c1_req_id), .req_rob_tag(5'd0),
    .hit_resp_valid(c1_hit_v), .hit_resp_id(c1_hit_id), .hit_resp_rdata(c1_hit_rdata),
    .fill_resp_valid(c1_fill_v), .fill_resp_id(c1_fill_id), .fill_resp_rdata(c1_fill_rdata),
    .flush(1'b0), .squash_valid(1'b0), .squash_tag(5'd0), .rob_head(5'd0),
    .clean_all_req(c1_clean_req), .clean_all_done(c1_clean_done),
    .snoop_inv_valid(c1_snp_inv_v), .snoop_inv_laddr(c1_snp_inv_la),
    .coh_req_valid(d1_coh_req_v), .coh_req_ready(d1_coh_req_rdy),
    .coh_req_op(d1_coh_req_op), .coh_req_laddr(d1_coh_req_la), .coh_req_id(d1_coh_req_id),
    .snoop_valid(d1_snp_v), .snoop_op(d1_snp_op), .snoop_laddr(d1_snp_la),
    .snoop_resp_op(d1_snp_resp_op), .snoop_resp_data(d1_snp_resp_dat),
    .wb_valid(d1_wb_v), .wb_ready(d1_wb_rdy), .wb_laddr(d1_wb_la), .wb_data(d1_wb_dat),
    .coh_resp_valid(d1_resp_v), .coh_resp_id(d1_resp_id), .coh_resp_data(d1_resp_dat),
    .coh_resp_shared(d1_resp_sh), .coh_resp_from_c2c(d1_resp_c2c), .coh_resp_upgrade_ack(d1_resp_upg),
    .dbg_wr_commit_valid(d1_wr_v), .dbg_wr_commit_laddr(d1_wr_la), .dbg_wr_commit_data(d1_wr_dat),
    .dcache_idle(d1_idle), .dcache_writes_idle(d1_wr_idle), .dcache_any_miss(d1_any_m),
    .cache_hit(d1_hit_p), .cache_miss(d1_miss_p), .cache_wb(d1_wb_p),
    .cache_hum(d1_hum_p), .cache_mum_cyc(d1_mum_p), .cache_mshr_coalesce(d1_coal_p),
    .pmu_silent_e2m(d1_e2m_p), .pmu_upgrade_miss(d1_upg_p)
);

coh_interconnect #(
    .LADDR_W(LADDR_W), .M_ID_W(3), .ENABLE_MOESI_O(ENABLE_MOESI_O)
) u_coh (
    .clk(clk), .reset_n(reset_n),
    .d0_coh_req_valid(d0_coh_req_v), .d0_coh_req_ready(d0_coh_req_rdy),
    .d0_coh_req_op(d0_coh_req_op), .d0_coh_req_laddr(d0_coh_req_la), .d0_coh_req_id(d0_coh_req_id),
    .d0_snoop_valid(d0_snp_v), .d0_snoop_op(d0_snp_op), .d0_snoop_laddr(d0_snp_la),
    .d0_snoop_resp_op(d0_snp_resp_op), .d0_snoop_resp_data(d0_snp_resp_dat),
    .d0_wb_valid(d0_wb_v), .d0_wb_ready(d0_wb_rdy), .d0_wb_laddr(d0_wb_la), .d0_wb_data(d0_wb_dat),
    .d0_coh_resp_valid(d0_resp_v), .d0_coh_resp_id(d0_resp_id), .d0_coh_resp_data(d0_resp_dat),
    .d0_coh_resp_shared(d0_resp_sh), .d0_coh_resp_from_c2c(d0_resp_c2c), .d0_coh_resp_upgrade_ack(d0_resp_upg),
    .d1_coh_req_valid(d1_coh_req_v), .d1_coh_req_ready(d1_coh_req_rdy),
    .d1_coh_req_op(d1_coh_req_op), .d1_coh_req_laddr(d1_coh_req_la), .d1_coh_req_id(d1_coh_req_id),
    .d1_snoop_valid(d1_snp_v), .d1_snoop_op(d1_snp_op), .d1_snoop_laddr(d1_snp_la),
    .d1_snoop_resp_op(d1_snp_resp_op), .d1_snoop_resp_data(d1_snp_resp_dat),
    .d1_wb_valid(d1_wb_v), .d1_wb_ready(d1_wb_rdy), .d1_wb_laddr(d1_wb_la), .d1_wb_data(d1_wb_dat),
    .d1_coh_resp_valid(d1_resp_v), .d1_coh_resp_id(d1_resp_id), .d1_coh_resp_data(d1_resp_dat),
    .d1_coh_resp_shared(d1_resp_sh), .d1_coh_resp_from_c2c(d1_resp_c2c), .d1_coh_resp_upgrade_ack(d1_resp_upg),
    .dma_coh_req_valid(1'b0), .dma_coh_req_ready(dma_rdy_u), .dma_coh_req_is_write(1'b0),
    .dma_coh_req_laddr(16'd0), .dma_coh_req_wdata(128'd0), .dma_coh_req_wstrb(16'd0),
    .dma_coh_resp_valid(dma_rv_u), .dma_coh_resp_rdata(dma_rd_u),
    .dma_wr_commit_valid(dma_wv_u), .dma_wr_commit_laddr(dma_wla_u), .dma_wr_commit_data(dma_wdat_u),
    .i0_rd_valid(1'b0), .i0_rd_ready(i0_rdy_u), .i0_rd_laddr(16'd0), .i0_rresp_valid(i0_rv_u), .i0_rresp_data(i0_rd_u),
    .i1_rd_valid(1'b0), .i1_rd_ready(i1_rdy_u), .i1_rd_laddr(16'd0), .i1_rresp_valid(i1_rv_u), .i1_rresp_data(i1_rd_u),
    .l2_m0_req_valid(l2_m0_v), .l2_m0_req_ready(l2_m0_rdy), .l2_m0_req_is_write(l2_m0_we),
    .l2_m0_req_laddr(l2_m0_la), .l2_m0_req_wdata(l2_m0_wd), .l2_m0_req_id(l2_m0_id),
    .l2_m0_resp_valid(l2_m0_rv), .l2_m0_resp_id(l2_m0_rid), .l2_m0_resp_data(l2_m0_rd),
    .l2_m1_req_valid(l2_m1_v), .l2_m1_req_ready(l2_m1_rdy), .l2_m1_req_is_write(l2_m1_we),
    .l2_m1_req_laddr(l2_m1_la), .l2_m1_req_wdata(l2_m1_wd), .l2_m1_req_id(l2_m1_id),
    .l2_m1_resp_valid(l2_m1_rv), .l2_m1_resp_id(l2_m1_rid), .l2_m1_resp_data(l2_m1_rd),
    .bus_evt_valid(bus_evt_v), .bus_evt_src(bus_evt_src), .bus_evt_op(bus_evt_op),
    .bus_evt_laddr(bus_evt_la), .bus_evt_c2c(bus_evt_c2c), .bus_evt_shared(bus_evt_sh),
    .coh_idle(coh_idle),
    .pmu_coh_gets(pmu_gets), .pmu_coh_getm(pmu_getm), .pmu_coh_upgrade(pmu_upg),
    .pmu_coh_c2c_dirty(pmu_c2c), .pmu_coh_inval(pmu_inval), .pmu_coh_mesi_wb(pmu_mesi_wb),
    .pmu_coh_dma_rd(pmu_dma_rd_u), .pmu_coh_dma_wr(pmu_dma_wr_u)
);

l2_cache_shared #(
    .LADDR_W(LADDR_W), .SETS_PER_BANK(256), .M_ID_W(3), .HIT_LAT(3)
) u_l2 (
    .clk(clk), .reset_n(reset_n),
    .m0_req_valid(l2_m0_v), .m0_req_ready(l2_m0_rdy), .m0_req_is_write(l2_m0_we),
    .m0_req_laddr(l2_m0_la), .m0_req_wdata(l2_m0_wd), .m0_req_id(l2_m0_id),
    .m0_resp_valid(l2_m0_rv), .m0_resp_id(l2_m0_rid), .m0_resp_data(l2_m0_rd),
    .m1_req_valid(l2_m1_v), .m1_req_ready(l2_m1_rdy), .m1_req_is_write(l2_m1_we),
    .m1_req_laddr(l2_m1_la), .m1_req_wdata(l2_m1_wd), .m1_req_id(l2_m1_id),
    .m1_resp_valid(l2_m1_rv), .m1_resp_id(l2_m1_rid), .m1_resp_data(l2_m1_rd),
    .dram_rd_valid(dram_rd_v), .dram_rd_ready(dram_rd_rdy),
    .dram_rd_laddr(dram_rd_la), .dram_rd_id(dram_rd_id),
    .dram_rresp_valid(dram_rr_v), .dram_rresp_id(dram_rr_id), .dram_rresp_data(dram_rr_dat),
    .dram_wr_valid(dram_wr_v), .dram_wr_ready(dram_wr_rdy),
    .dram_wr_laddr(dram_wr_la), .dram_wr_data(dram_wr_dat),
    .l2_idle(l2_idle), .pmu_l2_hits(l2_hits_p), .pmu_l2_misses(l2_misses_p),
    .pmu_l2_wbs(l2_wbs_p), .pmu_l2_bank_conflict(l2_bc)
);

main_memory_dram #(
    .LINE_BITS(14), .LATENCY(8), .ID_W(2)
) u_dram (
    .clk(clk), .rst_n(reset_n), .jitter_en(jitter_en),
    .mem_rd_valid(dram_rd_v), .mem_rd_ready(dram_rd_rdy),
    .mem_rd_line_addr(dram_rd_la[13:0]), .mem_rd_id(dram_rd_id),
    .mem_rresp_valid(dram_rr_v), .mem_rresp_id(dram_rr_id), .mem_rresp_data(dram_rr_dat),
    .mem_wr_valid(dram_wr_v), .mem_wr_ready(dram_wr_rdy),
    .mem_wr_line_addr(dram_wr_la[13:0]), .mem_wr_data(dram_wr_dat),
    .pmu_dram_rd(dram_rd_p), .pmu_dram_wr(dram_wr_p)
);

wire [31:0] chk_errs, chk_gets, chk_getm, chk_upg, chk_c2c, chk_wr_commits;

coh_checker #(
    .LADDR_W(LADDR_W), .MEM_LINES(16384), .WATCHDOG_CYC(256)
) u_chk (
    .clk(clk), .reset_n(reset_n), .active(1'b1),
    .d0_wr_valid(d0_wr_v), .d0_wr_laddr(d0_wr_la), .d0_wr_data(d0_wr_dat),
    .d1_wr_valid(d1_wr_v), .d1_wr_laddr(d1_wr_la), .d1_wr_data(d1_wr_dat),
    .dma_wr_valid(1'b0), .dma_wr_laddr(16'd0), .dma_wr_data(128'd0),
    .bus_evt_valid(bus_evt_v), .bus_evt_src(bus_evt_src), .bus_evt_op(bus_evt_op),
    .bus_evt_laddr(bus_evt_la), .bus_evt_c2c(bus_evt_c2c), .bus_evt_shared(bus_evt_sh),
    .d0_idle(d0_idle), .d1_idle(d1_idle), .coh_idle(coh_idle),
    .d0_progress(c0_hit_v || c0_fill_v || c0_clean_done || d0_resp_v),
    .d1_progress(c1_hit_v || c1_fill_v || c1_clean_done || d1_resp_v),
    .chk_errors(chk_errs), .cnt_gets(chk_gets), .cnt_getm(chk_getm),
    .cnt_upgrade(chk_upg), .cnt_c2c(chk_c2c), .cnt_wr_commits(chk_wr_commits)
);

initial clk = 1'b0;
always #5 clk = ~clk;

// ----------------------------------------------------------------------------
// Every-Cycle Deep Coherence Invariant Checker (SWMR, Duplicate Tag, 128b Data!)
// ----------------------------------------------------------------------------
integer errs, cyc_cnt;
integer cnt_e2m, cnt_l2_wb_in, cnt_mesi_wb, cnt_o_state_seen, cnt_upg_promo;

function automatic [31:0] get_w(input [127:0] line, input [1:0] woff);
    begin
        case (woff)
            2'd0: get_w = line[31:0];
            2'd1: get_w = line[63:32];
            2'd2: get_w = line[95:64];
            2'd3: get_w = line[127:96];
        endcase
    end
endfunction

function automatic [2:0] lookup_d0_state(input [15:0] la);
    reg [6:0] s;
    reg [8:0] t;
    begin
        s = la[6:0]; t = la[15:7];
        if (u_d0.way0_state[s] != COH_I && u_d0.way0_tag[s] == t)
            lookup_d0_state = u_d0.way0_state[s];
        else if (u_d0.way1_state[s] != COH_I && u_d0.way1_tag[s] == t)
            lookup_d0_state = u_d0.way1_state[s];
        else
            lookup_d0_state = COH_I;
    end
endfunction

function automatic [2:0] lookup_d1_state(input [15:0] la);
    reg [6:0] s;
    reg [8:0] t;
    begin
        s = la[6:0]; t = la[15:7];
        if (u_d1.way0_state[s] != COH_I && u_d1.way0_tag[s] == t)
            lookup_d1_state = u_d1.way0_state[s];
        else if (u_d1.way1_state[s] != COH_I && u_d1.way1_tag[s] == t)
            lookup_d1_state = u_d1.way1_state[s];
        else
            lookup_d1_state = COH_I;
    end
endfunction

// Track active sets (0..31 plus hot sets 0,1) every cycle for fast 50,000-op runtime!
task check_line_invariants(input [15:0] la);
    reg [2:0] st0, st1;
    reg       wbb0_m, wbb1_m, mwb_m;
    integer   dirty_cnt;
    begin
        st0 = lookup_d0_state(la);
        st1 = lookup_d1_state(la);
        wbb0_m = (u_d0.wbb0_v && u_d0.wbb0_laddr == la) || (u_d0.wbb1_v && u_d0.wbb1_laddr == la);
        wbb1_m = (u_d1.wbb0_v && u_d1.wbb0_laddr == la) || (u_d1.wbb1_v && u_d1.wbb1_laddr == la);
        mwb_m  = u_coh.mwb_v && (u_coh.mwb_laddr == la);

        if (st0 == COH_O || st1 == COH_O)
            cnt_o_state_seen = cnt_o_state_seen + 1;

        // 1. SWMR Exclusivity
        if ((st0 == COH_M || st0 == COH_E) && (st1 != COH_I || wbb1_m)) begin
            $display("FAIL [SWMR]: cyc=%0d la=0x%04x D0=%0d D1=%0d wbb1=%0d", cyc_cnt, la, st0, st1, wbb1_m);
            errs = errs + 1;
        end
        if ((st1 == COH_M || st1 == COH_E) && (st0 != COH_I || wbb0_m)) begin
            $display("FAIL [SWMR]: cyc=%0d la=0x%04x D1=%0d D0=%0d wbb0=%0d", cyc_cnt, la, st1, st0, wbb0_m);
            errs = errs + 1;
        end
        if (st0 == COH_O && st1 == COH_O) begin
            $display("FAIL [SWMR]: cyc=%0d la=0x%04x BOTH D0 & D1 in O!", cyc_cnt, la);
            errs = errs + 1;
        end
        if (ENABLE_MOESI_O == 0 && (st0 == COH_O || st1 == COH_O)) begin
            $display("FAIL [MESI]: cyc=%0d la=0x%04x O state entered with ENABLE_MOESI_O=0!", cyc_cnt, la);
            errs = errs + 1;
        end

        // 2. Unique Dirty Owner (at most 1 dirty copy in cache/WBB across both cores)
        dirty_cnt = ((st0 == COH_M || st0 == COH_O) ? 1 : 0) +
                    ((st1 == COH_M || st1 == COH_O) ? 1 : 0) +
                    (wbb0_m ? 1 : 0) + (wbb1_m ? 1 : 0);
        if (dirty_cnt > 1) begin
            $display("FAIL [OWNER]: cyc=%0d la=0x%04x dirty_cnt=%0d (st0=%0d st1=%0d wbb0=%0d wbb1=%0d)",
                     cyc_cnt, la, dirty_cnt, st0, st1, wbb0_m, wbb1_m);
            errs = errs + 1;
        end
    end
endtask

task check_all_valid_ways;
    integer s;
    reg [15:0] la;
    begin
        // Check all WBB entries against Golden Shadow Memory
        if (u_d0.wbb0_v) begin
            if (u_d0.wbb0_data !== u_chk.shadow_mem[u_d0.wbb0_laddr[13:0]]) begin
                $display("FAIL [DATA-WBB0]: cyc=%0d la=0x%04x exp=0x%032x got=0x%032x",
                         cyc_cnt, u_d0.wbb0_laddr, u_chk.shadow_mem[u_d0.wbb0_laddr[13:0]], u_d0.wbb0_data);
                errs = errs + 1;
            end
            check_line_invariants(u_d0.wbb0_laddr);
        end
        if (u_d0.wbb1_v) begin
            if (u_d0.wbb1_data !== u_chk.shadow_mem[u_d0.wbb1_laddr[13:0]]) begin
                $display("FAIL [DATA-WBB0.1]: cyc=%0d la=0x%04x", cyc_cnt, u_d0.wbb1_laddr);
                errs = errs + 1;
            end
            check_line_invariants(u_d0.wbb1_laddr);
        end
        if (u_d1.wbb0_v) begin
            if (u_d1.wbb0_data !== u_chk.shadow_mem[u_d1.wbb0_laddr[13:0]]) begin
                $display("FAIL [DATA-WBB1]: cyc=%0d la=0x%04x exp=0x%032x got=0x%032x",
                         cyc_cnt, u_d1.wbb0_laddr, u_chk.shadow_mem[u_d1.wbb0_laddr[13:0]], u_d1.wbb0_data);
                errs = errs + 1;
            end
            check_line_invariants(u_d1.wbb0_laddr);
        end
        if (u_d1.wbb1_v) begin
            if (u_d1.wbb1_data !== u_chk.shadow_mem[u_d1.wbb1_laddr[13:0]]) begin
                $display("FAIL [DATA-WBB1.1]: cyc=%0d la=0x%04x", cyc_cnt, u_d1.wbb1_laddr);
                errs = errs + 1;
            end
            check_line_invariants(u_d1.wbb1_laddr);
        end

        // Check sets 0..31 (covers 100% of our directed & random working sets!)
        for (s = 0; s < 32; s = s + 1) begin
            // Duplicate Snoop Tag Equivalence on D0 & D1
            if (u_d0.way0_state[s] !== u_d0.snp_way0_state[s] || u_d0.way0_tag[s] !== u_d0.snp_way0_tag[s] ||
                u_d0.way1_state[s] !== u_d0.snp_way1_state[s] || u_d0.way1_tag[s] !== u_d0.snp_way1_tag[s] ||
                u_d1.way0_state[s] !== u_d1.snp_way0_state[s] || u_d1.way0_tag[s] !== u_d1.snp_way0_tag[s] ||
                u_d1.way1_state[s] !== u_d1.snp_way1_state[s] || u_d1.way1_tag[s] !== u_d1.snp_way1_tag[s]) begin
                $display("FAIL [DUP-TAG]: cyc=%0d set=%0d Main vs Snoop tag array diverged!", cyc_cnt, s);
                errs = errs + 1;
            end
            // No duplicate tag across way0/way1 in same cache
            if (u_d0.way0_state[s] != COH_I && u_d0.way1_state[s] != COH_I && u_d0.way0_tag[s] == u_d0.way1_tag[s]) begin
                $display("FAIL [DUP-WAY]: cyc=%0d D0 set=%0d duplicate tag=0x%03x!", cyc_cnt, s, u_d0.way0_tag[s]);
                errs = errs + 1;
            end
            if (u_d1.way0_state[s] != COH_I && u_d1.way1_state[s] != COH_I && u_d1.way0_tag[s] == u_d1.way1_tag[s]) begin
                $display("FAIL [DUP-WAY]: cyc=%0d D1 set=%0d duplicate tag=0x%03x!", cyc_cnt, s, u_d1.way0_tag[s]);
                errs = errs + 1;
            end
            // Golden 128-bit Data Value Invariant on EVERY valid way!
            if (u_d0.way0_state[s] != COH_I) begin
                la = {u_d0.way0_tag[s], s[6:0]};
                if (u_d0.way0_data[s] !== u_chk.shadow_mem[la[13:0]]) begin
                    $display("FAIL [DATA-D0W0]: cyc=%0d la=0x%04x st=%0d exp=0x%032x got=0x%032x",
                             cyc_cnt, la, u_d0.way0_state[s], u_chk.shadow_mem[la[13:0]], u_d0.way0_data[s]);
                    errs = errs + 1;
                end
                check_line_invariants(la);
            end
            if (u_d0.way1_state[s] != COH_I) begin
                la = {u_d0.way1_tag[s], s[6:0]};
                if (u_d0.way1_data[s] !== u_chk.shadow_mem[la[13:0]]) begin
                    $display("FAIL [DATA-D0W1]: cyc=%0d la=0x%04x st=%0d exp=0x%032x got=0x%032x",
                             cyc_cnt, la, u_d0.way1_state[s], u_chk.shadow_mem[la[13:0]], u_d0.way1_data[s]);
                    errs = errs + 1;
                end
                check_line_invariants(la);
            end
            if (u_d1.way0_state[s] != COH_I) begin
                la = {u_d1.way0_tag[s], s[6:0]};
                if (u_d1.way0_data[s] !== u_chk.shadow_mem[la[13:0]]) begin
                    $display("FAIL [DATA-D1W0]: cyc=%0d la=0x%04x st=%0d exp=0x%032x got=0x%032x",
                             cyc_cnt, la, u_d1.way0_state[s], u_chk.shadow_mem[la[13:0]], u_d1.way0_data[s]);
                    errs = errs + 1;
                end
                check_line_invariants(la);
            end
            if (u_d1.way1_state[s] != COH_I) begin
                la = {u_d1.way1_tag[s], s[6:0]};
                if (u_d1.way1_data[s] !== u_chk.shadow_mem[la[13:0]]) begin
                    $display("FAIL [DATA-D1W1]: cyc=%0d la=0x%04x st=%0d exp=0x%032x got=0x%032x",
                             cyc_cnt, la, u_d1.way1_state[s], u_chk.shadow_mem[la[13:0]], u_d1.way1_data[s]);
                    errs = errs + 1;
                end
                check_line_invariants(la);
            end
        end
    end
endtask

/* verilator lint_off BLKSEQ */
/* verilator lint_off SYNCASYNCNET */
always @(posedge clk) begin
    if (reset_n) begin
        cyc_cnt = cyc_cnt + 1;
        if (d0_e2m_p) cnt_e2m = cnt_e2m + 1;
        if (d1_e2m_p) cnt_e2m = cnt_e2m + 1;
        if (l2_m0_v && l2_m0_rdy && l2_m0_we) cnt_l2_wb_in = cnt_l2_wb_in + 1;
        if (l2_m1_v && l2_m1_rdy && l2_m1_we) cnt_l2_wb_in = cnt_l2_wb_in + 1;
        if (pmu_mesi_wb) cnt_mesi_wb = cnt_mesi_wb + 1;
        if (u_d0.snoop_valid && u_d0.snp_is_inv && u_d0.m0_valid && !u_d0.m0_sent && u_d0.m0_laddr == u_d0.snoop_laddr && u_d0.m0_coh_op == COH_REQ_UPGRADE)
            cnt_upg_promo = cnt_upg_promo + 1;
        if (u_d1.snoop_valid && u_d1.snp_is_inv && u_d1.m0_valid && !u_d1.m0_sent && u_d1.m0_laddr == u_d1.snoop_laddr && u_d1.m0_coh_op == COH_REQ_UPGRADE)
            cnt_upg_promo = cnt_upg_promo + 1;

        // Check every cycle (after NBA updates settle via #1 in separate monitor, or right before posedge!)
        if (errs < 10)
            check_all_valid_ways();
    end
end
/* verilator lint_on SYNCASYNCNET */
/* verilator lint_on BLKSEQ */

// ----------------------------------------------------------------------------
// Blocking Helper Tasks for Directed Tests
// ----------------------------------------------------------------------------
task wait_all_idle;
    integer wc;
    begin
        @(negedge clk);
        c0_req_v = 1'b0; c1_req_v = 1'b0;
        c0_clean_req = 1'b0; c1_clean_req = 1'b0;
        wc = 0;
        while ((!d0_idle || !d1_idle || !coh_idle || !l2_idle) && wc < 2000) begin
            @(posedge clk); #1;
            wc = wc + 1;
        end
        if (wc >= 2000) begin
            $display("FAIL [tb_coh]: Timeout waiting for subsystem idle! d0=%0d d1=%0d coh=%0d l2=%0d",
                     d0_idle, d1_idle, coh_idle, l2_idle);
            $fatal(1);
        end
    end
endtask

task do_c0_op(
    input        is_wr,
    input [3:0]  amo,
    input [19:0] addr,
    input [31:0] wdata,
    input [3:0]  wstrb,
    output [31:0] rdata
);
    reg got;
    integer wc;
    begin
        rdata = 32'd0;
        @(negedge clk);
        c0_req_v     = 1'b1;
        c0_req_we    = is_wr;
        c0_req_amo   = amo;
        c0_req_addr  = addr;
        c0_req_wdata = wdata;
        c0_req_wstrb = wstrb;
        c0_req_id    = 7'd1;
        #1;
        while (!c0_req_rdy) begin @(negedge clk); #1; end
        @(posedge clk);
        @(negedge clk);
        c0_req_v = 1'b0;
        if (!is_wr || (amo != AMO_NONE)) begin
            got = 1'b0; wc = 0;
            while (!got && wc < 500) begin
                #1;
                if (c0_hit_v)       begin rdata = c0_hit_rdata;  got = 1'b1; end
                else if (c0_fill_v) begin rdata = c0_fill_rdata; got = 1'b1; end
                else begin @(posedge clk); wc = wc + 1; end
            end
            if (!got) begin
                $display("FAIL [tb_coh]: Timeout waiting for C0 response addr=0x%05x", addr);
                $fatal(1);
            end
        end
    end
endtask

task do_c1_op(
    input        is_wr,
    input [3:0]  amo,
    input [19:0] addr,
    input [31:0] wdata,
    input [3:0]  wstrb,
    output [31:0] rdata
);
    reg got;
    integer wc;
    begin
        rdata = 32'd0;
        @(negedge clk);
        c1_req_v     = 1'b1;
        c1_req_we    = is_wr;
        c1_req_amo   = amo;
        c1_req_addr  = addr;
        c1_req_wdata = wdata;
        c1_req_wstrb = wstrb;
        c1_req_id    = 7'd1;
        #1;
        while (!c1_req_rdy) begin @(negedge clk); #1; end
        @(posedge clk);
        @(negedge clk);
        c1_req_v = 1'b0;
        if (!is_wr || (amo != AMO_NONE)) begin
            got = 1'b0; wc = 0;
            while (!got && wc < 500) begin
                #1;
                if (c1_hit_v)       begin rdata = c1_hit_rdata;  got = 1'b1; end
                else if (c1_fill_v) begin rdata = c1_fill_rdata; got = 1'b1; end
                else begin @(posedge clk); wc = wc + 1; end
            end
            if (!got) begin
                $display("FAIL [tb_coh]: Timeout waiting for C1 response addr=0x%05x", addr);
                $fatal(1);
            end
        end
    end
endtask

// ----------------------------------------------------------------------------
// Main Test Sequence
// ----------------------------------------------------------------------------
integer i, seed, ops_done, wc_t;
reg [31:0] rd0, rd1, exp0, exp1;
reg [31:0] c0_sum_cnt, c1_sum_cnt;
integer wb_before, c2c_before, upg_before;
reg        c0_pend_rd, c1_pend_rd, c0_pend_sc, c1_pend_sc;
reg [19:0] c0_pend_addr, c1_pend_addr;
reg        c0_is_wr_r, c1_is_wr_r;
reg [3:0]  c0_amo_r, c1_amo_r;
reg [19:0] c0_addr_r, c1_addr_r;
reg [31:0] c0_wdat_r, c1_wdat_r;
reg [3:0]  c0_strb_r, c1_strb_r;
reg        c0_need_resp, c1_need_resp;

initial begin
    errs             = 0;
    cyc_cnt          = 0;
    cnt_e2m          = 0;
    cnt_l2_wb_in     = 0;
    cnt_mesi_wb      = 0;
    cnt_o_state_seen = 0;
    cnt_upg_promo    = 0;
    reset_n          = 1'b0;
    jitter_en        = 1'b0;
    c0_req_v = 1'b0; c0_req_we = 1'b0; c0_req_amo = AMO_NONE; c0_req_addr = 20'd0;
    c0_req_wdata = 32'd0; c0_req_wstrb = 4'hF; c0_req_id = 7'd0; c0_clean_req = 1'b0;
    c1_req_v = 1'b0; c1_req_we = 1'b0; c1_req_amo = AMO_NONE; c1_req_addr = 20'd0;
    c1_req_wdata = 32'd0; c1_req_wstrb = 4'hF; c1_req_id = 7'd0; c1_clean_req = 1'b0;

    // Initialize 256 KB DRAM & Golden Shadow Memory identically
    for (i = 0; i < 16384; i = i + 1) begin
        u_dram.mem[i]       = {32'hD000_0000 ^ (i * 32'd7), 32'hC000_0000 ^ (i * 32'd5),
                               32'hB000_0000 ^ (i * 32'd3), 32'hA000_0000 ^ i[31:0]};
        u_chk.shadow_mem[i] = u_dram.mem[i];
    end

    repeat (3) @(posedge clk);
    @(negedge clk);
    reset_n = 1'b1;

    // ------------------------------------------------------------------------
    // TEST 1: Exclusive E Detection + Silent E -> M Store Upgrade (0 bus upg!)
    // ------------------------------------------------------------------------
    do_c0_op(1'b0, AMO_NONE, 20'h00010, 32'd0, 4'h0, rd0); // Line 1 -> E!
    wait_all_idle();
    if (lookup_d0_state(16'd1) !== COH_E) begin
        $display("FAIL [T1]: Expected D0 line 1 in E, got %0d", lookup_d0_state(16'd1));
        errs = errs + 1;
    end
    upg_before = chk_upg;
    do_c0_op(1'b1, AMO_NONE, 20'h00010, 32'h1122_3344, 4'hF, rd0); // Silent E -> M!
    wait_all_idle();
    if (lookup_d0_state(16'd1) !== COH_M || chk_upg != upg_before) begin
        $display("FAIL [T1]: Silent E->M failed! state=%0d upg_delta=%0d", lookup_d0_state(16'd1), chk_upg - upg_before);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------------
    // TEST 2: MOESI Dirty C2C Sharing (M -> O on peer GetS, 0 L2 WB in MOESI!)
    // ------------------------------------------------------------------------
    wb_before  = cnt_l2_wb_in;
    c2c_before = chk_c2c;
    do_c1_op(1'b0, AMO_NONE, 20'h00010, 32'd0, 4'h0, rd1); // Peer GetS -> C2C dirty!
    wait_all_idle();
    if (rd1 !== 32'h1122_3344 || chk_c2c <= c2c_before) begin
        $display("FAIL [T2]: C2C dirty read failed! rd1=0x%08x c2c_delta=%0d", rd1, chk_c2c - c2c_before);
        errs = errs + 1;
    end
    if (ENABLE_MOESI_O != 0) begin
        if (lookup_d0_state(16'd1) !== COH_O || lookup_d1_state(16'd1) !== COH_S || cnt_l2_wb_in != wb_before) begin
            $display("FAIL [T2-MOESI]: Expected D0=O D1=S 0 L2 WBs; got D0=%0d D1=%0d wb=%0d",
                     lookup_d0_state(16'd1), lookup_d1_state(16'd1), cnt_l2_wb_in - wb_before);
            errs = errs + 1;
        end
    end else begin
        if (lookup_d0_state(16'd1) !== COH_S || lookup_d1_state(16'd1) !== COH_S || cnt_l2_wb_in != wb_before + 1) begin
            $display("FAIL [T2-MESI]: Expected D0=S D1=S 1 L2 WB; got D0=%0d D1=%0d wb=%0d",
                     lookup_d0_state(16'd1), lookup_d1_state(16'd1), cnt_l2_wb_in - wb_before);
            errs = errs + 1;
        end
    end

    // ------------------------------------------------------------------------
    // TEST 3: Simultaneous Dual-Upgrade Race (OM_AD + SM_AD on line 1!)
    //   D0 writes word 0, D1 writes word 1 on the EXACT SAME CYCLE!
    //   Loser must auto-promote UPGRADE -> GETM and BOTH words must survive!
    // ------------------------------------------------------------------------
    @(negedge clk);
    c0_req_v = 1'b1; c0_req_we = 1'b1; c0_req_amo = AMO_NONE; c0_req_addr = 20'h00010; c0_req_wdata = 32'hAAAA_0001; c0_req_wstrb = 4'hF;
    c1_req_v = 1'b1; c1_req_we = 1'b1; c1_req_amo = AMO_NONE; c1_req_addr = 20'h00014; c1_req_wdata = 32'hBBBB_0002; c1_req_wstrb = 4'hF;
    @(posedge clk);
    @(negedge clk);
    c0_req_v = 1'b0; c1_req_v = 1'b0;
    wait_all_idle();

    do_c0_op(1'b0, AMO_NONE, 20'h00010, 32'd0, 4'h0, rd0);
    do_c1_op(1'b0, AMO_NONE, 20'h00014, 32'd0, 4'h0, rd1);
    wait_all_idle();
    if (rd0 !== 32'hAAAA_0001 || rd1 !== 32'hBBBB_0002) begin
        $display("FAIL [T3]: Dual-Upgrade race lost a store! rd0=0x%08x rd1=0x%08x", rd0, rd1);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------------
    // TEST 4: Pure SM_AD + SM_AD Race (Line 2 clean in BOTH D0=S & D1=S!)
    // ------------------------------------------------------------------------
    do_c0_op(1'b0, AMO_NONE, 20'h00020, 32'd0, 4'h0, rd0); // D0=E
    do_c1_op(1'b0, AMO_NONE, 20'h00020, 32'd0, 4'h0, rd1); // D0=S, D1=S
    wait_all_idle();
    @(negedge clk);
    c0_req_v = 1'b1; c0_req_we = 1'b1; c0_req_amo = AMO_NONE; c0_req_addr = 20'h00028; c0_req_wdata = 32'hCCCC_0003; c0_req_wstrb = 4'hF;
    c1_req_v = 1'b1; c1_req_we = 1'b1; c1_req_amo = AMO_NONE; c1_req_addr = 20'h0002C; c1_req_wdata = 32'hDDDD_0004; c1_req_wstrb = 4'hF;
    @(posedge clk);
    @(negedge clk);
    c0_req_v = 1'b0; c1_req_v = 1'b0;
    wait_all_idle();

    do_c1_op(1'b0, AMO_NONE, 20'h00028, 32'd0, 4'h0, rd0);
    do_c0_op(1'b0, AMO_NONE, 20'h0002C, 32'd0, 4'h0, rd1);
    wait_all_idle();
    if (rd0 !== 32'hCCCC_0003 || rd1 !== 32'hDDDD_0004) begin
        $display("FAIL [T4]: SM_AD+SM_AD race lost a store! rd0=0x%08x rd1=0x%08x", rd0, rd1);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------------
    // TEST 5: WBB Snoop Intervention (Dirty lines evicted into D0 WBB while
    //         D1 reads/writes those exact lines!)
    //   Set 0 lines: 0, 128 (0x800), 256 (0x1000) -> 3rd evicts line 0 to WBB!
    // ------------------------------------------------------------------------
    do_c0_op(1'b1, AMO_NONE, 20'h00000, 32'hFEED_F001, 4'hF, rd0); // Set 0 Way 0 = M
    do_c0_op(1'b1, AMO_NONE, 20'h00800, 32'hFEED_F002, 4'hF, rd0); // Set 0 Way 1 = M
    wait_all_idle();
    // Issue D0 miss to 0x1000 (evicts 0x0000 into WBB!) and immediately hammer 0x0000 from D1!
    @(negedge clk);
    c0_req_v = 1'b1; c0_req_we = 1'b1; c0_req_amo = AMO_NONE; c0_req_addr = 20'h01000; c0_req_wdata = 32'hFEED_F003; c0_req_wstrb = 4'hF;
    @(posedge clk);
    @(negedge clk);
    c0_req_v = 1'b0;
    do_c1_op(1'b0, AMO_NONE, 20'h00000, 32'd0, 4'h0, rd1);
    wait_all_idle();
    if (rd1 !== 32'hFEED_F001) begin
        $display("FAIL [T5]: WBB snoop intervention failed! exp=0xFEEDF001 got=0x%08x", rd1);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------------
    // TEST 6: Cross-Core LR.W / SC.W Reservation Invalidation & Success
    // ------------------------------------------------------------------------
    do_c0_op(1'b1, AMO_NONE, 20'h00030, 32'h0000_0055, 4'hF, rd0);
    wait_all_idle();
    do_c0_op(1'b0, AMO_LR, 20'h00030, 32'd0, 4'h0, rd0);          // D0 reserves line 3
    do_c1_op(1'b1, AMO_NONE, 20'h00030, 32'h0000_0099, 4'hF, rd1); // D1 invalidates D0!
    wait_all_idle();
    do_c0_op(1'b1, AMO_SC, 20'h00030, 32'hBAD0_BAD0, 4'hF, rd0);  // D0 SC MUST FAIL (1)!
    if (rd0 !== 32'd1) begin
        $display("FAIL [T6]: Contended SC.W should fail (1), got %0d!", rd0);
        errs = errs + 1;
    end
    do_c0_op(1'b0, AMO_LR, 20'h00030, 32'd0, 4'h0, rd0);          // Fresh LR -> reads 0x99
    do_c0_op(1'b1, AMO_SC, 20'h00030, 32'h0000_00AA, 4'hF, rd1);  // Uncontended SC -> MUST PASS (0)!
    wait_all_idle();
    if (rd0 !== 32'h0000_0099 || rd1 !== 32'd0) begin
        $display("FAIL [T6]: Uncontended LR/SC failed! lr=0x%08x sc=%0d", rd0, rd1);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------------
    // TEST 7: Contended AMOADD.W Hammer (200 x 2 Cores = +400 Exact!)
    // ------------------------------------------------------------------------
    do_c0_op(1'b1, AMO_NONE, 20'h00040, 32'd1000, 4'hF, rd0);
    wait_all_idle();
    c0_sum_cnt = 32'd0; c1_sum_cnt = 32'd0;
    c0_pend_rd = 1'b0;  c1_pend_rd = 1'b0;
    jitter_en  = 1'b1;

    while ((c0_sum_cnt < 200 || c1_sum_cnt < 200 || c0_pend_rd || c1_pend_rd) && errs == 0) begin
        @(negedge clk);
        c0_req_v = (c0_sum_cnt < 200) && !c0_pend_rd;
        c0_req_we = 1'b1; c0_req_amo = AMO_ADD; c0_req_addr = 20'h00040; c0_req_wdata = 32'd1; c0_req_wstrb = 4'hF; c0_req_id = 7'd2;

        c1_req_v = (c1_sum_cnt < 200) && !c1_pend_rd;
        c1_req_we = 1'b1; c1_req_amo = AMO_ADD; c1_req_addr = 20'h00040; c1_req_wdata = 32'd1; c1_req_wstrb = 4'hF; c1_req_id = 7'd2;
        #1;
        if (c0_req_v && c0_req_rdy) begin c0_pend_rd = 1'b1; c0_sum_cnt = c0_sum_cnt + 1; end
        if (c1_req_v && c1_req_rdy) begin c1_pend_rd = 1'b1; c1_sum_cnt = c1_sum_cnt + 1; end
        @(posedge clk);
        #1;
        if (c0_hit_v || c0_fill_v) c0_pend_rd = 1'b0;
        if (c1_hit_v || c1_fill_v) c1_pend_rd = 1'b0;
    end
    wait_all_idle();
    do_c0_op(1'b0, AMO_NONE, 20'h00040, 32'd0, 4'h0, rd0);
    if (rd0 !== 32'd1400) begin
        $display("FAIL [T7]: Contended AMOADD lost updates! exp=1400 got=%0d", rd0);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------------
    // TEST 8: FENCE.I Dirty Clean-All Sweep Under Live Peer Snooping!
    // ------------------------------------------------------------------------
    @(negedge clk);
    c0_clean_req = 1'b1;
    @(posedge clk);
    @(negedge clk);
    c0_clean_req = 1'b0;
    wc_t = 0;
    while (!c0_clean_done && wc_t < 1000) begin
        // Peer D1 actively reads line 4 during D0's FENCE.I clean-all sweep!
        @(negedge clk);
        c1_req_v = (wc_t == 5); c1_req_we = 1'b0; c1_req_amo = AMO_NONE; c1_req_addr = 20'h00040;
        @(posedge clk); #1;
        wc_t = wc_t + 1;
    end
    wait_all_idle();
    if (wc_t >= 1000) begin
        $display("FAIL [T8]: FENCE.I clean-all hung!");
        errs = errs + 1;
    end

    // ------------------------------------------------------------------------
    // TEST 9: 50,000-Op Adversarial Randomized Dual-Core Coherence Stress!
    //   Mixes: hot Set 0/1 conflict thrashing (tags 0..3 -> constant dirty
    //   evictions & WBB snoops!), shared working set (lines 0..15), loads,
    //   byte/half/word stores, LR/SC, AMOADD/SWAP/XOR, and DRAM Jitter!
    // ------------------------------------------------------------------------
    seed       = 32'hC0E4_2026;
    ops_done   = 0;
    c0_pend_rd = 1'b0; c1_pend_rd = 1'b0;
    c0_pend_sc = 1'b0; c1_pend_sc = 1'b0;
    c0_pend_addr = 20'd0; c1_pend_addr = 20'd0;
    jitter_en  = 1'b1;

    while (ops_done < NUM_RAND_OPS && errs == 0 && chk_errs == 0) begin
        @(negedge clk);
        // Generate random op for Core 0 (only 1 outstanding read/AMO per core here for tight response check,
        // while stores fire freely back-to-back to saturate 2 MSHRs + WBB!)
        seed = seed * 32'd1664525 + 32'd1013904223;
        if (seed[31:30] != 2'b00)
            // 75% hot Set 0/1 conflict addresses (4 tags -> guarantees nonstop 2-way evictions & WBB snoops!)
            c0_addr_r = {7'd0, seed[29:28], 6'd0, seed[27], seed[26:25], 2'b00};
        else
            // 25% shared lines 0..15
            c0_addr_r = {12'd0, seed[29:26], seed[25:24], 2'b00};
        c0_wdat_r = seed ^ 32'h1357_9BDF;
        c0_strb_r = (seed[23:22] == 2'd0) ? (4'b0001 << seed[21:20]) :
                    (seed[23:22] == 2'd1) ? (seed[21] ? 4'b1100 : 4'b0011) : 4'b1111;
        if (seed[19:17] == 3'd0) begin
            c0_is_wr_r = 1'b0; c0_amo_r = AMO_LR;
        end else if (seed[19:17] == 3'd1) begin
            c0_is_wr_r = 1'b1; c0_amo_r = AMO_SC; c0_strb_r = 4'hF;
        end else if (seed[19:17] == 3'd2) begin
            c0_is_wr_r = 1'b1; c0_amo_r = (seed[16] ? AMO_ADD : AMO_XOR); c0_strb_r = 4'hF;
        end else if (seed[19:18] == 2'b10) begin
            c0_is_wr_r = 1'b1; c0_amo_r = AMO_NONE;
        end else begin
            c0_is_wr_r = 1'b0; c0_amo_r = AMO_NONE;
        end
        c0_need_resp = (!c0_is_wr_r || c0_amo_r != AMO_NONE);
        c0_req_v     = !(c0_need_resp && c0_pend_rd);
        c0_req_we    = c0_is_wr_r;
        c0_req_amo   = c0_amo_r;
        c0_req_addr  = c0_addr_r;
        c0_req_wdata = c0_wdat_r;
        c0_req_wstrb = c0_strb_r;
        c0_req_id    = 7'd3;

        // Generate random op for Core 1
        seed = seed * 32'd1664525 + 32'd1013904223;
        if (seed[31:30] != 2'b00)
            c1_addr_r = {7'd0, seed[29:28], 6'd0, seed[27], seed[26:25], 2'b00};
        else
            c1_addr_r = {12'd0, seed[29:26], seed[25:24], 2'b00};
        c1_wdat_r = seed ^ 32'h2468_ACE0;
        c1_strb_r = (seed[23:22] == 2'd0) ? (4'b0001 << seed[21:20]) :
                    (seed[23:22] == 2'd1) ? (seed[21] ? 4'b1100 : 4'b0011) : 4'b1111;
        if (seed[19:17] == 3'd0) begin
            c1_is_wr_r = 1'b0; c1_amo_r = AMO_LR;
        end else if (seed[19:17] == 3'd1) begin
            c1_is_wr_r = 1'b1; c1_amo_r = AMO_SC; c1_strb_r = 4'hF;
        end else if (seed[19:17] == 3'd2) begin
            c1_is_wr_r = 1'b1; c1_amo_r = (seed[16] ? AMO_SWAP : AMO_MAX); c1_strb_r = 4'hF;
        end else if (seed[19:18] == 2'b10) begin
            c1_is_wr_r = 1'b1; c1_amo_r = AMO_NONE;
        end else begin
            c1_is_wr_r = 1'b0; c1_amo_r = AMO_NONE;
        end
        c1_need_resp = (!c1_is_wr_r || c1_amo_r != AMO_NONE);
        c1_req_v     = !(c1_need_resp && c1_pend_rd);
        c1_req_we    = c1_is_wr_r;
        c1_req_amo   = c1_amo_r;
        c1_req_addr  = c1_addr_r;
        c1_req_wdata = c1_wdat_r;
        c1_req_wstrb = c1_strb_r;
        c1_req_id    = 7'd3;

        #1;
        if (c0_req_v && c0_req_rdy) begin
            ops_done = ops_done + 1;
            if (c0_need_resp) begin
                c0_pend_rd   = 1'b1;
                c0_pend_sc   = (c0_amo_r == AMO_SC);
                c0_pend_addr = c0_addr_r;
            end
        end
        if (c1_req_v && c1_req_rdy) begin
            ops_done = ops_done + 1;
            if (c1_need_resp) begin
                c1_pend_rd   = 1'b1;
                c1_pend_sc   = (c1_amo_r == AMO_SC);
                c1_pend_addr = c1_addr_r;
            end
        end

        @(posedge clk);
        #1;
        if (c0_hit_v || c0_fill_v) begin
            c0_pend_rd = 1'b0;
            if (c0_pend_sc) begin
                if ((c0_hit_v ? c0_hit_rdata : c0_fill_rdata) > 32'd1) begin
                    $display("FAIL [RAND-SC0]: Invalid SC return value!");
                    errs = errs + 1;
                end
            end
        end
        if (c1_hit_v || c1_fill_v) begin
            c1_pend_rd = 1'b0;
            if (c1_pend_sc) begin
                if ((c1_hit_v ? c1_hit_rdata : c1_fill_rdata) > 32'd1) begin
                    $display("FAIL [RAND-SC1]: Invalid SC return value!");
                    errs = errs + 1;
                end
            end
        end
    end

    wait_all_idle();

    // Final Full-Hierarchy Coherence Check: flush both L1Ds with FENCE.I clean-all
    // and verify EVERY SINGLE line in our working set reads back exact Golden Shadow Memory!
    @(negedge clk); c0_clean_req = 1'b1; c1_clean_req = 1'b1;
    @(posedge clk);
    @(negedge clk); c0_clean_req = 1'b0; c1_clean_req = 1'b0;
    wait_all_idle();

    for (i = 0; i < 16; i = i + 1) begin
        do_c0_op(1'b0, AMO_NONE, {12'd0, i[3:0], 4'd0}, 32'd0, 4'h0, rd0);
        exp0 = get_w(u_chk.shadow_mem[i], 2'd0);
        if (rd0 !== exp0) begin
            $display("FAIL [FINAL-VERIFY]: line %0d exp=0x%08x got=0x%08x", i, exp0, rd0);
            errs = errs + 1;
        end
    end
    wait_all_idle();

    if (errs == 0 && chk_errs == 0 && chk_c2c > 100 && cnt_e2m > 10 && chk_upg > 50) begin
        $display("PASS [tb_coh_subsystem MOESI=%0d]: %0d ops (cyc=%0d) gets=%0d getm=%0d upg=%0d c2c=%0d e2m=%0d l2_wbs=%0d mesi_wbs=%0d o_cyc=%0d upg_promo=%0d wr_commits=%0d errs=0",
                 ENABLE_MOESI_O, ops_done, cyc_cnt, chk_gets, chk_getm, chk_upg, chk_c2c,
                 cnt_e2m, cnt_l2_wb_in, cnt_mesi_wb, cnt_o_state_seen, cnt_upg_promo, chk_wr_commits);
    end else begin
        $display("FAIL [tb_coh_subsystem MOESI=%0d]: errs=%0d chk_errs=%0d ops=%0d c2c=%0d e2m=%0d upg=%0d",
                 ENABLE_MOESI_O, errs, chk_errs, ops_done, chk_c2c, cnt_e2m, chk_upg);
        $fatal(1);
    end
    $finish;
end

endmodule
