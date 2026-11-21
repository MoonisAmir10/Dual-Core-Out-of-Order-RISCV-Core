`timescale 1ns / 1ps

// ============================================================================
// Single-Core 3-Level Memory Hierarchy RV32IMA_Zicsr_Zifencei SoC (Phase 2)
// Official 1-Core (1C) Baseline for Fair 1C vs. 2C Dual-Core Scaling!
//
// Hierarchy (Uniform 16-Byte / 128-Bit Lines End-to-End):
//   - Core 0: ooo2w_core (2-wide Tomasulo OoO, EX->EX bypass, Tournament+RAS)
//   - Private L1I: 4 KB (param 16 KB) 2-way 16B icache_2w (1-cyc hit, prefetcher)
//   - Private L1D: 4 KB (param 16 KB) 2-way 16B dcache_nb (1-cyc hit, 2 MSHRs, RV32A)
//   - Shared Unified L2: 32 KB (param 64 KB) 4-way 2-bank NINE l2_cache_shared
//     (3-cyc hit, 4 total MSHRs, Master 0=L1D, Master 1=L1I -> parallel bank hits!)
//   - Backing DRAM: 256 KB 4-outstanding pipelined main_memory_dram (default L16)
//   - Uncached MMIO: CLINT at 0x0200_0000 (msip0/1, 64-bit mtime, scratch[0..3])
// ============================================================================
module single_ooo2w_soc #(
    parameter [31:0]  CORE_ID         = 32'd0,
    parameter integer DRAM_LATENCY    = 16,  // Realistic 1 : 4 : 16 (L1 : L2 : DRAM)
    parameter integer L2_HIT_LAT      = 3,
    parameter integer ICACHE_SETS     = 128, // 4 KB L1I
    parameter integer DCACHE_SETS     = 128, // 4 KB L1D
    parameter integer L2_SETS_BANK    = 256, // 256 x 4 x 16B x 2 banks = 32 KB L2
    parameter integer DRAM_LINE_BITS  = 14,  // 256 KB unified DRAM (override for synth)
    parameter integer ENABLE_PREFETCH = 1
) (
    input  wire         clk,
    input  wire         reset_n,
    input  wire         dram_jitter_en,

    // 2-Wide RVFI Commit Trace outputs
    output wire         commit0_valid,
    output wire [31:0]  commit0_seq_id,
    output wire [31:0]  commit0_pc,
    output wire [31:0]  commit0_instr,
    output wire         commit0_rd_we,
    output wire [4:0]   commit0_rd_addr,
    output wire [31:0]  commit0_rd_wdata,
    output wire         commit0_mem_valid,
    output wire         commit0_mem_we,
    output wire [31:0]  commit0_mem_addr,
    output wire [3:0]   commit0_mem_wstrb,
    output wire [31:0]  commit0_mem_wdata,

    output wire         commit1_valid,
    output wire [31:0]  commit1_seq_id,
    output wire [31:0]  commit1_pc,
    output wire [31:0]  commit1_instr,
    output wire         commit1_rd_we,
    output wire [4:0]   commit1_rd_addr,
    output wire [31:0]  commit1_rd_wdata,
    output wire         commit1_mem_valid,
    output wire         commit1_mem_we,
    output wire [31:0]  commit1_mem_addr,
    output wire [3:0]   commit1_mem_wstrb,
    output wire [31:0]  commit1_mem_wdata,

    // OoO monitor hooks
    output wire         trace_disp0_valid,
    output wire [31:0]  trace_disp0_seq,
    output wire [31:0]  trace_disp0_pc,
    output wire [31:0]  trace_disp0_instr,
    output wire         trace_disp1_valid,
    output wire [31:0]  trace_disp1_seq,
    output wire [31:0]  trace_disp1_pc,
    output wire [31:0]  trace_disp1_instr,
    output wire [5:0]   trace_issue_valid,
    output wire [191:0] trace_issue_seq,
    output wire [1:0]   trace_cdb_valid,
    output wire [63:0]  trace_cdb_seq,
    output wire         trace_flush_valid,
    output wire [31:0]  trace_flush_after_seq,

    output wire         halted,
    output wire [31:0]  x14,
    output wire         clint_msip0,
    output wire         clint_msip1,

    // External AMBA APB4 Peripheral Pins (GPIO, UART, Timer/PWM)
    input  wire [31:0]  gpio_in,
    output wire [31:0]  gpio_out,
    output wire [31:0]  gpio_oe,
    input  wire         uart_rx,
    output wire         uart_tx,
    output wire         uart_tx_char_valid,
    output wire [7:0]   uart_tx_char_data,
    output wire [1:0]   pwm_out,

    // Full 3-Level Hierarchy Hardware PMU Counters
    output reg  [31:0]  pmu_cycles,
    output reg  [31:0]  pmu_instret,
    output reg  [31:0]  pmu_ic_hits,
    output reg  [31:0]  pmu_ic_misses,
    output reg  [31:0]  pmu_ic_pf_hits,
    output reg  [31:0]  pmu_dcache_hits,
    output reg  [31:0]  pmu_dcache_misses,
    output reg  [31:0]  pmu_dcache_wbs,
    output reg  [31:0]  pmu_dcache_hum,
    output reg  [31:0]  pmu_dcache_mum_cyc,
    output reg  [31:0]  pmu_l2_hits,
    output reg  [31:0]  pmu_l2_misses,
    output reg  [31:0]  pmu_l2_wbs,
    output reg  [31:0]  pmu_l2_bank_conflicts,
    output reg  [31:0]  pmu_dram_rds,
    output reg  [31:0]  pmu_dram_wrs,
    output reg  [31:0]  pmu_dual_commit_cyc,
    output reg  [31:0]  pmu_ex_bypass_hits,
    output reg  [31:0]  pmu_amo_commits,
    output reg  [31:0]  pmu_fence_commits
);

localparam integer AWIDTH    = 20;
localparam integer ID_W      = 7;
localparam integer LADDR_W   = AWIDTH - 4; // 16
localparam integer LINE_BITS = DRAM_LINE_BITS;

`include "soc_addr_map.vh"
localparam integer APB_NUM_SLAVES = 7;
localparam [15:0]  APB_SLV_MASK   = 16'b0000_0000_0110_1111; // S0=CLINT, S1=PLIC, S2=GPIO, S3=UART, S5=TIMER, S6=SOC_CTRL

// Core <-> L1I
wire        imem_req_valid, imem_valid0, imem_valid1;
wire [31:0] imem_pc0, imem_instr0, imem_pc1, imem_instr1;
wire        ic_flush, ic_invalidate_all, ic_idle_u;
wire        ic_hit_p, ic_miss_p, ic_pf_hit_p, ic_stall_u;

// Core <-> L1D / Standardized AMBA APB4 Uncached MMIO Fabric
wire        core_dc_valid, core_dc_ready, core_dc_we, core_dc_is_mmio;
wire [3:0]  core_dc_amo_op;
wire [AWIDTH-1:0] core_dc_addr;
wire [31:0] core_dc_paddr;
wire [31:0] core_dc_wdata;
wire [3:0]  core_dc_wstrb;
wire [ID_W-1:0] core_dc_id;
wire [4:0]  core_dc_rob_tag;

wire        dc_req_ready, mmio_req_ready;
wire        dc_hit_valid, mmio_resp_valid, mmio_resp_err, dc_fill_valid;
wire [ID_W-1:0] dc_hit_id, mmio_resp_id, dc_fill_id;
wire [31:0] dc_hit_rdata, mmio_resp_rdata, dc_fill_rdata;

wire        apb_m_psel, apb_m_penable, apb_m_pwrite, apb_m_pready, apb_m_pslverr;
wire [31:0] apb_m_paddr, apb_m_pwdata, apb_m_prdata;
wire [3:0]  apb_m_pstrb;
wire [APB_NUM_SLAVES-1:0]    apb_s_psel, apb_s_pready, apb_s_pslverr;
wire [APB_NUM_SLAVES*32-1:0] apb_s_prdata;
wire        apb_s_penable, apb_s_pwrite;
wire [31:0] apb_s_paddr, apb_s_pwdata;
wire [3:0]  apb_s_pstrb;

wire        clint_mtip0, clint_mtip1_u;
wire        plic_meip0;
wire        gpio_irq, uart_rx_irq, uart_tx_irq, timer_irq0, timer_irq1;
wire [15:1] soc_irq_inject;
wire [15:1] plic_irq_src = {8'd0, timer_irq1, timer_irq0, 2'b00, uart_tx_irq, uart_rx_irq, gpio_irq} | soc_irq_inject;
wire        core_halted, soc_sys_halt;
wire [31:0] soc_tohost_u;
wire        c1_mmio_rdy_u, c1_mmio_rv_u, c1_mmio_err_u;
wire [ID_W-1:0] c1_mmio_rid_u;
wire [31:0] c1_mmio_rd_u;

assign halted = core_halted || soc_sys_halt;

wire        dc_flush, dc_squash_valid, dc_clean_req, dc_clean_done;
wire [4:0]  dc_squash_tag, dc_rob_head;
wire        dc_idle_u, dc_writes_idle, dc_any_miss;
wire        dc_hit_p, dc_miss_p, dc_wb_p, dc_hum_p, dc_mum_p, dc_coal_u;

assign core_dc_ready = core_dc_is_mmio ? mmio_req_ready : dc_req_ready;

// Guaranteed mutually exclusive (LSQ holds younger cacheable loads while APB4 MMIO is active!)
wire            core_hit_valid = mmio_resp_valid || dc_hit_valid;
wire [ID_W-1:0] core_hit_id    = mmio_resp_valid ? mmio_resp_id    : dc_hit_id;
wire [31:0]     core_hit_rdata = mmio_resp_valid ? mmio_resp_rdata : dc_hit_rdata;
wire            core_hit_err   = mmio_resp_valid ? mmio_resp_err   : 1'b0;

// L1D <-> L2 Master 0 & L1I <-> L2 Master 1
wire        dc_rd_valid, dc_rd_ready, dc_rd_id;
wire [LADDR_W-1:0] dc_rd_laddr;
wire        dc_wr_valid, dc_wr_ready;
wire [LADDR_W-1:0] dc_wr_laddr;
wire [127:0] dc_wr_data;

wire        ic_rd_valid, ic_rd_ready;
wire [LADDR_W-1:0] ic_rd_laddr;

wire        l2_m0_valid = dc_wr_valid || dc_rd_valid;
wire        l2_m0_ready;
wire        l2_m0_we    = dc_wr_valid;
wire [LADDR_W-1:0] l2_m0_laddr = dc_wr_valid ? dc_wr_laddr : dc_rd_laddr;
wire [127:0] l2_m0_wdata = dc_wr_data;
wire [2:0]  l2_m0_id    = {2'b00, dc_rd_id};
wire        l2_m0_resp_v;
wire [2:0]  l2_m0_resp_id;
wire [127:0] l2_m0_resp_data;

assign dc_wr_ready = dc_wr_valid && l2_m0_ready;
assign dc_rd_ready = !dc_wr_valid && dc_rd_valid && l2_m0_ready;

wire        l2_m1_ready, l2_m1_resp_v;
wire [2:0]  l2_m1_resp_id_u;
wire [127:0] l2_m1_resp_data;
assign ic_rd_ready = l2_m1_ready;

// L2 <-> Pipelined 4-Outstanding Backing DRAM
wire        dram_rd_valid, dram_rd_ready, dram_rresp_valid;
wire [LADDR_W-1:0] dram_rd_laddr;
wire [1:0]  dram_rd_id, dram_rresp_id;
wire [127:0] dram_rresp_data;

wire        dram_wr_valid, dram_wr_ready;
wire [LADDR_W-1:0] dram_wr_laddr;
wire [127:0] dram_wr_data;

wire        l2_idle_u, l2_bconf_p, dram_rd_p, dram_wr_p;
wire [1:0]  l2_hits_p, l2_misses_p, l2_wbs_p;
wire [63:0] clint_mtime_u;

wire        st_u0, st_u1, st_u2, st_u3, st_u4, st_u5, st_u6, st_u7, st_u8, st_u9, st_u10;
wire        st_dcomm, st_amo, st_fence, st_trap_u;
wire [1:0]  st_v0, st_v1, st_v2, st_v3, st_v4;
wire [3:0]  st_ex_byp;

ooo2w_core #(
    .CORE_ID(CORE_ID),
    .RESET_PC(32'd0),
    .AWIDTH(AWIDTH),
    .ID_W(ID_W),
    .ENABLE_MMIO_PMA(1),
    .RAM_SIZE_BYTES(SOC_RAM_SIZE)
) u_core (
    .clk(clk), .reset_n(reset_n),
    .imem_req_valid(imem_req_valid),
    .imem_pc0(imem_pc0), .imem_valid0(imem_valid0), .imem_instr0(imem_instr0),
    .imem_pc1(imem_pc1), .imem_valid1(imem_valid1), .imem_instr1(imem_instr1),
    .ic_flush(ic_flush), .ic_invalidate_all(ic_invalidate_all),
    .dc_req_valid(core_dc_valid), .dc_req_ready(core_dc_ready),
    .dc_req_is_write(core_dc_we), .dc_req_is_mmio(core_dc_is_mmio),
    .dc_req_amo_op(core_dc_amo_op),
    .dc_req_addr(core_dc_addr), .dc_req_paddr(core_dc_paddr), .dc_req_wdata(core_dc_wdata), .dc_req_wstrb(core_dc_wstrb),
    .dc_req_id(core_dc_id), .dc_req_rob_tag(core_dc_rob_tag),
    .dc_hit_valid(core_hit_valid), .dc_hit_id(core_hit_id), .dc_hit_rdata(core_hit_rdata), .dc_hit_err(core_hit_err),
    .dc_fill_valid(dc_fill_valid), .dc_fill_id(dc_fill_id), .dc_fill_rdata(dc_fill_rdata), .dc_fill_err(1'b0),
    .dc_flush(dc_flush), .dc_squash_valid(dc_squash_valid),
    .dc_squash_tag(dc_squash_tag), .dc_rob_head(dc_rob_head),
    .dc_clean_all_req(dc_clean_req), .dc_clean_all_done(dc_clean_done),
    .dc_writes_idle(dc_writes_idle), .dc_any_miss(dc_any_miss),
    .coh_snoop_inv_valid(1'b0), .coh_snoop_inv_line(16'd0),
    .irq_msip(clint_msip0), .irq_mtip(clint_mtip0), .irq_meip(plic_meip0),
    .commit0_valid(commit0_valid), .commit0_seq_id(commit0_seq_id),
    .commit0_pc(commit0_pc), .commit0_instr(commit0_instr),
    .commit0_rd_we(commit0_rd_we), .commit0_rd_addr(commit0_rd_addr), .commit0_rd_wdata(commit0_rd_wdata),
    .commit0_mem_valid(commit0_mem_valid), .commit0_mem_we(commit0_mem_we),
    .commit0_mem_addr(commit0_mem_addr), .commit0_mem_wstrb(commit0_mem_wstrb), .commit0_mem_wdata(commit0_mem_wdata),
    .commit1_valid(commit1_valid), .commit1_seq_id(commit1_seq_id),
    .commit1_pc(commit1_pc), .commit1_instr(commit1_instr),
    .commit1_rd_we(commit1_rd_we), .commit1_rd_addr(commit1_rd_addr), .commit1_rd_wdata(commit1_rd_wdata),
    .commit1_mem_valid(commit1_mem_valid), .commit1_mem_we(commit1_mem_we),
    .commit1_mem_addr(commit1_mem_addr), .commit1_mem_wstrb(commit1_mem_wstrb), .commit1_mem_wdata(commit1_mem_wdata),
    .trace_disp0_valid(trace_disp0_valid), .trace_disp0_seq(trace_disp0_seq),
    .trace_disp0_pc(trace_disp0_pc), .trace_disp0_instr(trace_disp0_instr),
    .trace_disp1_valid(trace_disp1_valid), .trace_disp1_seq(trace_disp1_seq),
    .trace_disp1_pc(trace_disp1_pc), .trace_disp1_instr(trace_disp1_instr),
    .trace_issue_valid(trace_issue_valid), .trace_issue_seq(trace_issue_seq),
    .trace_cdb_valid(trace_cdb_valid), .trace_cdb_seq(trace_cdb_seq),
    .trace_flush_valid(trace_flush_valid), .trace_flush_after_seq(trace_flush_after_seq),
    .halted(core_halted), .x14(x14),
    .stat_rob_full_stall(st_u0), .stat_rs_full_stall(st_u1),
    .stat_lsq_full_stall(st_u2), .stat_cp_full_stall(st_u3),
    .stat_stlf_hit(st_u4),
    .stat_br_total_cnt(st_v0), .stat_br_mispred_cnt(st_v1),
    .stat_br_fast_squash(st_u5), .stat_war_waw_cnt(st_v2),
    .stat_div_overlap_cyc(st_u6), .stat_dcache_miss_overlap_cyc(st_u7),
    .stat_dual_disp(st_u8), .stat_dual_cdb(st_u9), .stat_dual_commit(st_dcomm),
    .stat_ex_byp_cnt(st_ex_byp), .stat_ras_pred_cnt(st_v3), .stat_ras_mispred_cnt(st_v4),
    .stat_replay_flush(st_u10), .stat_amo_commit(st_amo), .stat_fence_commit(st_fence),
    .stat_trap_taken(st_trap_u)
);

icache_2w #(
    .AWIDTH(AWIDTH),
    .NUM_SETS(ICACHE_SETS),
    .ENABLE_PREFETCH(ENABLE_PREFETCH)
) u_icache (
    .clk(clk), .reset_n(reset_n),
    .req_valid(imem_req_valid), .req_pc(imem_pc0),
    .flush(ic_flush), .invalidate_all(ic_invalidate_all),
    .resp_valid0(imem_valid0), .resp_instr0(imem_instr0),
    .resp_valid1(imem_valid1), .resp_instr1(imem_instr1),
    .mem_rd_valid(ic_rd_valid), .mem_rd_ready(ic_rd_ready),
    .mem_rd_line_addr(ic_rd_laddr),
    .mem_rresp_valid(l2_m1_resp_v), .mem_rresp_data(l2_m1_resp_data),
    .icache_idle(ic_idle_u),
    .pmu_ic_hit(ic_hit_p), .pmu_ic_miss(ic_miss_p),
    .pmu_ic_pf_hit(ic_pf_hit_p), .pmu_ic_stall(ic_stall_u)
);

dcache_nb #(
    .AWIDTH(AWIDTH),
    .NUM_SETS(DCACHE_SETS),
    .TAG_W(5),
    .ID_W(ID_W)
) u_dcache (
    .clk(clk), .reset_n(reset_n),
    .req_valid(core_dc_valid && !core_dc_is_mmio), .req_ready(dc_req_ready),
    .req_is_write(core_dc_we), .req_amo_op(core_dc_amo_op),
    .req_addr(core_dc_addr), .req_wdata(core_dc_wdata), .req_wstrb(core_dc_wstrb),
    .req_id(core_dc_id), .req_rob_tag(core_dc_rob_tag),
    .hit_resp_valid(dc_hit_valid), .hit_resp_id(dc_hit_id), .hit_resp_rdata(dc_hit_rdata),
    .fill_resp_valid(dc_fill_valid), .fill_resp_id(dc_fill_id), .fill_resp_rdata(dc_fill_rdata),
    .flush(dc_flush), .squash_valid(dc_squash_valid),
    .squash_tag(dc_squash_tag), .rob_head(dc_rob_head),
    .clean_all_req(dc_clean_req), .clean_all_done(dc_clean_done),
    .ext_resv_inv_valid(1'b0), .ext_resv_inv_laddr(16'd0),
    .mem_rd_valid(dc_rd_valid), .mem_rd_ready(dc_rd_ready),
    .mem_rd_line_addr(dc_rd_laddr), .mem_rd_id(dc_rd_id),
    .mem_rresp_valid(l2_m0_resp_v), .mem_rresp_id(l2_m0_resp_id[0]), .mem_rresp_data(l2_m0_resp_data),
    .mem_wr_valid(dc_wr_valid), .mem_wr_ready(dc_wr_ready),
    .mem_wr_line_addr(dc_wr_laddr), .mem_wr_data(dc_wr_data),
    .dcache_idle(dc_idle_u), .dcache_writes_idle(dc_writes_idle), .dcache_any_miss(dc_any_miss),
    .cache_hit(dc_hit_p), .cache_miss(dc_miss_p), .cache_wb(dc_wb_p),
    .cache_hum(dc_hum_p), .cache_mum_cyc(dc_mum_p), .cache_mshr_coalesce(dc_coal_u)
);

// Standardized AMBA APB4 Peripheral Fabric (Phase U2: S0=CLINT, S6=SoC Ctrl)
mmio_to_apb4_bridge #(.ID_W(ID_W)) u_apb_bridge (
    .clk(clk), .reset_n(reset_n),
    .c0_req_valid(core_dc_valid && core_dc_is_mmio), .c0_req_ready(mmio_req_ready),
    .c0_req_is_write(core_dc_we), .c0_req_addr(core_dc_paddr),
    .c0_req_wdata(core_dc_wdata), .c0_req_wstrb(core_dc_wstrb), .c0_req_id(core_dc_id),
    .c0_resp_valid(mmio_resp_valid), .c0_resp_id(mmio_resp_id),
    .c0_resp_rdata(mmio_resp_rdata), .c0_resp_err(mmio_resp_err),
    .c1_req_valid(1'b0), .c1_req_ready(c1_mmio_rdy_u),
    .c1_req_is_write(1'b0), .c1_req_addr(32'd0),
    .c1_req_wdata(32'd0), .c1_req_wstrb(4'd0), .c1_req_id({ID_W{1'b0}}),
    .c1_resp_valid(c1_mmio_rv_u), .c1_resp_id(c1_mmio_rid_u),
    .c1_resp_rdata(c1_mmio_rd_u), .c1_resp_err(c1_mmio_err_u),
    .m_psel(apb_m_psel), .m_penable(apb_m_penable), .m_pwrite(apb_m_pwrite),
    .m_paddr(apb_m_paddr), .m_pwdata(apb_m_pwdata), .m_pstrb(apb_m_pstrb),
    .m_pready(apb_m_pready), .m_prdata(apb_m_prdata), .m_pslverr(apb_m_pslverr)
);

assign apb_s_pready[APB_SLV_DMA]        = 1'b1;
assign apb_s_pslverr[APB_SLV_DMA]       = 1'b1;
assign apb_s_prdata[APB_SLV_DMA*32   +: 32] = 32'd0;

apb4_interconnect #(
    .NUM_SLAVES(APB_NUM_SLAVES),
    .SLAVE_PRESENT(APB_SLV_MASK)
) u_apb_xbar (
    .m_psel(apb_m_psel), .m_penable(apb_m_penable), .m_pwrite(apb_m_pwrite),
    .m_paddr(apb_m_paddr), .m_pwdata(apb_m_pwdata), .m_pstrb(apb_m_pstrb),
    .m_pready(apb_m_pready), .m_prdata(apb_m_prdata), .m_pslverr(apb_m_pslverr),
    .s_psel(apb_s_psel), .s_penable(apb_s_penable), .s_pwrite(apb_s_pwrite),
    .s_paddr(apb_s_paddr), .s_pwdata(apb_s_pwdata), .s_pstrb(apb_s_pstrb),
    .s_pready(apb_s_pready), .s_prdata(apb_s_prdata), .s_pslverr(apb_s_pslverr)
);

apb4_clint u_clint (
    .pclk(clk), .preset_n(reset_n),
    .psel(apb_s_psel[APB_SLV_CLINT]), .penable(apb_s_penable), .pwrite(apb_s_pwrite),
    .paddr(apb_s_paddr), .pwdata(apb_s_pwdata), .pstrb(apb_s_pstrb),
    .pready(apb_s_pready[APB_SLV_CLINT]),
    .prdata(apb_s_prdata[APB_SLV_CLINT*32 +: 32]),
    .pslverr(apb_s_pslverr[APB_SLV_CLINT]),
    .msip0_out(clint_msip0), .msip1_out(clint_msip1),
    .mtip0_out(clint_mtip0), .mtip1_out(clint_mtip1_u), .mtime_out(clint_mtime_u)
);

apb4_plic #(
    .NUM_SOURCES(16),
    .NUM_HARTS(1),
    .PRIO_BITS(3)
) u_plic (
    .pclk(clk), .preset_n(reset_n),
    .psel(apb_s_psel[APB_SLV_PLIC]), .penable(apb_s_penable), .pwrite(apb_s_pwrite),
    .paddr(apb_s_paddr), .pwdata(apb_s_pwdata), .pstrb(apb_s_pstrb),
    .pready(apb_s_pready[APB_SLV_PLIC]),
    .prdata(apb_s_prdata[APB_SLV_PLIC*32 +: 32]),
    .pslverr(apb_s_pslverr[APB_SLV_PLIC]),
    .irq_src_in(plic_irq_src),
    .meip_out(plic_meip0)
);

apb4_gpio u_gpio (
    .pclk(clk), .preset_n(reset_n),
    .psel(apb_s_psel[APB_SLV_GPIO]), .penable(apb_s_penable), .pwrite(apb_s_pwrite),
    .paddr(apb_s_paddr), .pwdata(apb_s_pwdata), .pstrb(apb_s_pstrb),
    .pready(apb_s_pready[APB_SLV_GPIO]),
    .prdata(apb_s_prdata[APB_SLV_GPIO*32 +: 32]),
    .pslverr(apb_s_pslverr[APB_SLV_GPIO]),
    .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_oe(gpio_oe), .irq_out(gpio_irq)
);

apb4_uart u_uart (
    .pclk(clk), .preset_n(reset_n),
    .psel(apb_s_psel[APB_SLV_UART]), .penable(apb_s_penable), .pwrite(apb_s_pwrite),
    .paddr(apb_s_paddr), .pwdata(apb_s_pwdata), .pstrb(apb_s_pstrb),
    .pready(apb_s_pready[APB_SLV_UART]),
    .prdata(apb_s_prdata[APB_SLV_UART*32 +: 32]),
    .pslverr(apb_s_pslverr[APB_SLV_UART]),
    .uart_rx(uart_rx), .uart_tx(uart_tx),
    .tx_char_valid(uart_tx_char_valid), .tx_char_data(uart_tx_char_data),
    .rx_irq_out(uart_rx_irq), .tx_irq_out(uart_tx_irq)
);

apb4_timer_pwm u_timer (
    .pclk(clk), .preset_n(reset_n),
    .psel(apb_s_psel[APB_SLV_TIMER]), .penable(apb_s_penable), .pwrite(apb_s_pwrite),
    .paddr(apb_s_paddr), .pwdata(apb_s_pwdata), .pstrb(apb_s_pstrb),
    .pready(apb_s_pready[APB_SLV_TIMER]),
    .prdata(apb_s_prdata[APB_SLV_TIMER*32 +: 32]),
    .pslverr(apb_s_pslverr[APB_SLV_TIMER]),
    .pwm_out(pwm_out), .irq_ch0_out(timer_irq0), .irq_ch1_out(timer_irq1)
);

apb4_soc_ctrl #(
    .SOC_ID(32'h2002_0002),
    .NUM_CORES(32'd1),
    .FEATURE_MASK(32'h0000_000F)
) u_soc_ctrl (
    .pclk(clk), .preset_n(reset_n),
    .psel(apb_s_psel[APB_SLV_SOC_CTRL]), .penable(apb_s_penable), .pwrite(apb_s_pwrite),
    .paddr(apb_s_paddr), .pwdata(apb_s_pwdata), .pstrb(apb_s_pstrb),
    .pready(apb_s_pready[APB_SLV_SOC_CTRL]),
    .prdata(apb_s_prdata[APB_SLV_SOC_CTRL*32 +: 32]),
    .pslverr(apb_s_pslverr[APB_SLV_SOC_CTRL]),
    .pmu_cycles_in(pmu_cycles),
    .pmu_c0_instret_in(pmu_instret),
    .pmu_c1_instret_in(32'd0),
    .pmu_coh_c2c_in(32'd0),
    .pmu_l2_hits_in(pmu_l2_hits),
    .pmu_l2_misses_in(pmu_l2_misses),
    .pmu_dram_rds_in(pmu_dram_rds),
    .pmu_dram_wrs_in(pmu_dram_wrs),
    .sys_halt_out(soc_sys_halt),
    .tohost_out(soc_tohost_u),
    .irq_inject_out(soc_irq_inject)
);

l2_cache_shared #(
    .LADDR_W(LADDR_W),
    .SETS_PER_BANK(L2_SETS_BANK),
    .M_ID_W(3),
    .HIT_LAT(L2_HIT_LAT)
) u_l2 (
    .clk(clk), .reset_n(reset_n),
    .m0_req_valid(l2_m0_valid), .m0_req_ready(l2_m0_ready),
    .m0_req_is_write(l2_m0_we), .m0_req_laddr(l2_m0_laddr),
    .m0_req_wdata(l2_m0_wdata), .m0_req_id(l2_m0_id),
    .m0_resp_valid(l2_m0_resp_v), .m0_resp_id(l2_m0_resp_id), .m0_resp_data(l2_m0_resp_data),
    .m1_req_valid(ic_rd_valid), .m1_req_ready(l2_m1_ready),
    .m1_req_is_write(1'b0), .m1_req_laddr(ic_rd_laddr),
    .m1_req_wdata(128'd0), .m1_req_id(3'd0),
    .m1_resp_valid(l2_m1_resp_v), .m1_resp_id(l2_m1_resp_id_u), .m1_resp_data(l2_m1_resp_data),
    .dram_rd_valid(dram_rd_valid), .dram_rd_ready(dram_rd_ready),
    .dram_rd_laddr(dram_rd_laddr), .dram_rd_id(dram_rd_id),
    .dram_rresp_valid(dram_rresp_valid), .dram_rresp_id(dram_rresp_id), .dram_rresp_data(dram_rresp_data),
    .dram_wr_valid(dram_wr_valid), .dram_wr_ready(dram_wr_ready),
    .dram_wr_laddr(dram_wr_laddr), .dram_wr_data(dram_wr_data),
    .l2_idle(l2_idle_u),
    .pmu_l2_hits(l2_hits_p), .pmu_l2_misses(l2_misses_p),
    .pmu_l2_wbs(l2_wbs_p), .pmu_l2_bank_conflict(l2_bconf_p)
);

main_memory_dram #(
    .LINE_BITS(LINE_BITS),
    .LATENCY(DRAM_LATENCY),
    .ID_W(2)
) u_dram (
    .clk(clk), .rst_n(reset_n), .jitter_en(dram_jitter_en),
    .mem_rd_valid(dram_rd_valid), .mem_rd_ready(dram_rd_ready),
    .mem_rd_line_addr(dram_rd_laddr[LINE_BITS-1:0]), .mem_rd_id(dram_rd_id),
    .mem_rresp_valid(dram_rresp_valid), .mem_rresp_id(dram_rresp_id), .mem_rresp_data(dram_rresp_data),
    .mem_wr_valid(dram_wr_valid), .mem_wr_ready(dram_wr_ready),
    .mem_wr_line_addr(dram_wr_laddr[LINE_BITS-1:0]), .mem_wr_data(dram_wr_data),
    .pmu_dram_rd(dram_rd_p), .pmu_dram_wr(dram_wr_p)
);

wire [31:0] commit_inc = {31'd0, commit0_valid} + {31'd0, commit1_valid};

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        pmu_cycles            <= 32'd0;
        pmu_instret           <= 32'd0;
        pmu_ic_hits           <= 32'd0;
        pmu_ic_misses         <= 32'd0;
        pmu_ic_pf_hits        <= 32'd0;
        pmu_dcache_hits       <= 32'd0;
        pmu_dcache_misses     <= 32'd0;
        pmu_dcache_wbs        <= 32'd0;
        pmu_dcache_hum        <= 32'd0;
        pmu_dcache_mum_cyc    <= 32'd0;
        pmu_l2_hits           <= 32'd0;
        pmu_l2_misses         <= 32'd0;
        pmu_l2_wbs            <= 32'd0;
        pmu_l2_bank_conflicts <= 32'd0;
        pmu_dram_rds          <= 32'd0;
        pmu_dram_wrs          <= 32'd0;
        pmu_dual_commit_cyc   <= 32'd0;
        pmu_ex_bypass_hits    <= 32'd0;
        pmu_amo_commits       <= 32'd0;
        pmu_fence_commits     <= 32'd0;
    end else if (!halted) begin
        pmu_cycles            <= pmu_cycles + 32'd1;
        pmu_instret           <= pmu_instret + commit_inc;
        if (ic_hit_p)    pmu_ic_hits           <= pmu_ic_hits + 32'd1;
        if (ic_miss_p)   pmu_ic_misses         <= pmu_ic_misses + 32'd1;
        if (ic_pf_hit_p) pmu_ic_pf_hits        <= pmu_ic_pf_hits + 32'd1;
        if (dc_hit_p)    pmu_dcache_hits       <= pmu_dcache_hits + 32'd1;
        if (dc_miss_p)   pmu_dcache_misses     <= pmu_dcache_misses + 32'd1;
        if (dc_wb_p)     pmu_dcache_wbs        <= pmu_dcache_wbs + 32'd1;
        if (dc_hum_p)    pmu_dcache_hum        <= pmu_dcache_hum + 32'd1;
        if (dc_mum_p)    pmu_dcache_mum_cyc    <= pmu_dcache_mum_cyc + 32'd1;
        pmu_l2_hits           <= pmu_l2_hits   + {30'd0, l2_hits_p};
        pmu_l2_misses         <= pmu_l2_misses + {30'd0, l2_misses_p};
        pmu_l2_wbs            <= pmu_l2_wbs    + {30'd0, l2_wbs_p};
        if (l2_bconf_p)  pmu_l2_bank_conflicts <= pmu_l2_bank_conflicts + 32'd1;
        if (dram_rd_p)   pmu_dram_rds          <= pmu_dram_rds + 32'd1;
        if (dram_wr_p)   pmu_dram_wrs          <= pmu_dram_wrs + 32'd1;
        if (st_dcomm)    pmu_dual_commit_cyc   <= pmu_dual_commit_cyc + 32'd1;
        pmu_ex_bypass_hits    <= pmu_ex_bypass_hits + {28'd0, st_ex_byp};
        if (st_amo)      pmu_amo_commits       <= pmu_amo_commits + 32'd1;
        if (st_fence)    pmu_fence_commits     <= pmu_fence_commits + 32'd1;
    end
end

endmodule
