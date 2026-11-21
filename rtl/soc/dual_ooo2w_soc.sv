`timescale 1ns / 1ps

// ============================================================================
// Dual-Core 2-Wide Superscalar Out-of-Order RV32IMA_Zicsr_Zifencei SoC (Phase 4)
// Peak Chip Throughput: 4 Instructions / Cycle (2 Cores x 2-Wide OoO)
//
// Full 3-Level Coherent Memory Hierarchy (Uniform 16-Byte / 128-Bit Lines):
//   - Core 0 (mhartid=0): ooo2w_core + 4KB L1I (icache_2w) + 4KB MOESI L1D (dcache_nb_coh)
//   - Core 1 (mhartid=1): ooo2w_core + 4KB L1I (icache_2w) + 4KB MOESI L1D (dcache_nb_coh)
//   - Coherent Interconnect: 3-Channel Split-Transaction MOESI Snooping Crossbar
//     (coh_interconnect, param ENABLE_MOESI_O=1 for MOESI / 0 for MESI ablation!)
//   - Shared Unified L2: 32 KB (param 64 KB) 4-Way 2-Bank NINE (l2_cache_shared)
//   - Backing DRAM: 256 KB 4-Outstanding Pipelined DRAM (main_memory_dram, L16 + jitter)
//   - Uncached CLINT MMIO Hub (0x0200_0000): shared round-robin 2-to-1 bypass to clint_mmio
//   - Ablation Hooks:
//       * ENABLE_MOESI_O      : 1=MOESI (M->O C2C), 0=MESI (M->S + L2 WB)
//       * ENABLE_SNOOP_REPLAY : 1=RVWMO LQ snoop-replay ON, 0=negative ablation proof!
//       * ENABLE_PREFETCH     : 1=L1I next-line prefetcher ON, 0=OFF
// ============================================================================
module dual_ooo2w_soc #(
    parameter integer DRAM_LATENCY        = 16,  // L1 : L2 : DRAM = 1 : 4 : 16
    parameter integer L2_HIT_LAT          = 3,
    parameter integer ICACHE_SETS         = 128, // 4 KB L1I per core
    parameter integer DCACHE_SETS         = 128, // 4 KB MOESI L1D per core
    parameter integer L2_SETS_BANK        = 256, // 32 KB shared 2-bank L2
    parameter integer DRAM_LINE_BITS      = 14,  // 256 KB unified DRAM (override for synth)
    parameter integer ENABLE_PREFETCH     = 1,
    parameter integer ENABLE_MOESI_O      = 1,
    parameter integer ENABLE_SNOOP_REPLAY = 1
) (
    input  wire         clk,
    input  wire         reset_n,
    input  wire         dram_jitter_en,

    // -------------------- Core 0: 2-Wide RVFI Commit Trace --------------------
    output wire         c0_commit0_valid,
    output wire [31:0]  c0_commit0_seq_id,
    output wire [31:0]  c0_commit0_pc,
    output wire [31:0]  c0_commit0_instr,
    output wire         c0_commit0_rd_we,
    output wire         c0_commit0_rd_is_fp,
    output wire [4:0]   c0_commit0_rd_addr,
    output wire [31:0]  c0_commit0_rd_wdata,
    output wire [4:0]   c0_commit0_fflags,
    output wire         c0_commit0_mem_valid,
    output wire         c0_commit0_mem_we,
    output wire [31:0]  c0_commit0_mem_addr,
    output wire [3:0]   c0_commit0_mem_wstrb,
    output wire [31:0]  c0_commit0_mem_wdata,

    output wire         c0_commit1_valid,
    output wire [31:0]  c0_commit1_seq_id,
    output wire [31:0]  c0_commit1_pc,
    output wire [31:0]  c0_commit1_instr,
    output wire         c0_commit1_rd_we,
    output wire         c0_commit1_rd_is_fp,
    output wire [4:0]   c0_commit1_rd_addr,
    output wire [31:0]  c0_commit1_rd_wdata,
    output wire [4:0]   c0_commit1_fflags,
    output wire         c0_commit1_mem_valid,
    output wire         c0_commit1_mem_we,
    output wire [31:0]  c0_commit1_mem_addr,
    output wire [3:0]   c0_commit1_mem_wstrb,
    output wire [31:0]  c0_commit1_mem_wdata,

    // -------------------- Core 1: 2-Wide RVFI Commit Trace --------------------
    output wire         c1_commit0_valid,
    output wire [31:0]  c1_commit0_seq_id,
    output wire [31:0]  c1_commit0_pc,
    output wire [31:0]  c1_commit0_instr,
    output wire         c1_commit0_rd_we,
    output wire         c1_commit0_rd_is_fp,
    output wire [4:0]   c1_commit0_rd_addr,
    output wire [31:0]  c1_commit0_rd_wdata,
    output wire [4:0]   c1_commit0_fflags,
    output wire         c1_commit0_mem_valid,
    output wire         c1_commit0_mem_we,
    output wire [31:0]  c1_commit0_mem_addr,
    output wire [3:0]   c1_commit0_mem_wstrb,
    output wire [31:0]  c1_commit0_mem_wdata,

    output wire         c1_commit1_valid,
    output wire [31:0]  c1_commit1_seq_id,
    output wire [31:0]  c1_commit1_pc,
    output wire [31:0]  c1_commit1_instr,
    output wire         c1_commit1_rd_we,
    output wire         c1_commit1_rd_is_fp,
    output wire [4:0]   c1_commit1_rd_addr,
    output wire [31:0]  c1_commit1_rd_wdata,
    output wire [4:0]   c1_commit1_fflags,
    output wire         c1_commit1_mem_valid,
    output wire         c1_commit1_mem_we,
    output wire [31:0]  c1_commit1_mem_addr,
    output wire [3:0]   c1_commit1_mem_wstrb,
    output wire [31:0]  c1_commit1_mem_wdata,

    // -------------------- Global Coherence Bus Trace & Checker Hooks --------------------
    output wire         bus_evt_valid,
    output wire [1:0]   bus_evt_src,
    output wire [1:0]   bus_evt_op,
    output wire [15:0]  bus_evt_laddr,
    output wire         bus_evt_c2c,
    output wire         bus_evt_shared,

    output wire         d0_wr_commit_valid,
    output wire [15:0]  d0_wr_commit_laddr,
    output wire [127:0] d0_wr_commit_data,
    output wire         d1_wr_commit_valid,
    output wire [15:0]  d1_wr_commit_laddr,
    output wire [127:0] d1_wr_commit_data,
    output wire         dma_wr_commit_valid,
    output wire [15:0]  dma_wr_commit_laddr,
    output wire [127:0] dma_wr_commit_data,
    output wire         d0_idle,
    output wire         d1_idle,
    output wire         coh_idle,
    output wire         l2_idle,

    // -------------------- Core Status, Results & CLINT Outputs --------------------
    output wire         c0_halted,
    output wire         c1_halted,
    output wire         all_halted,
    output wire [31:0]  c0_x14,
    output wire [31:0]  c1_x14,
    output wire         clint_msip0,
    output wire         clint_msip1,

    // -------------------- External AMBA APB4 Peripheral Pins --------------------
    input  wire [31:0]  gpio_in,
    output wire [31:0]  gpio_out,
    output wire [31:0]  gpio_oe,
    input  wire         uart_rx,
    output wire         uart_tx,
    output wire         uart_tx_char_valid,
    output wire [7:0]   uart_tx_char_data,
    output wire [1:0]   pwm_out,

    // -------------------- Chip, Core, Coherence, L2 & DRAM PMU Counters --------------------
    output reg  [31:0]  pmu_cycles,
    output reg  [31:0]  pmu_c0_cycles,
    output reg  [31:0]  pmu_c1_cycles,
    output reg  [31:0]  pmu_c0_instret,
    output reg  [31:0]  pmu_c1_instret,
    output reg  [31:0]  pmu_c0_ic_hits,
    output reg  [31:0]  pmu_c0_ic_misses,
    output reg  [31:0]  pmu_c1_ic_hits,
    output reg  [31:0]  pmu_c1_ic_misses,
    output reg  [31:0]  pmu_c0_dc_hits,
    output reg  [31:0]  pmu_c0_dc_misses,
    output reg  [31:0]  pmu_c1_dc_hits,
    output reg  [31:0]  pmu_c1_dc_misses,
    output reg  [31:0]  pmu_c0_replay,
    output reg  [31:0]  pmu_c1_replay,
    output reg  [31:0]  pmu_c0_amo,
    output reg  [31:0]  pmu_c1_amo,
    output reg  [31:0]  pmu_coh_gets,
    output reg  [31:0]  pmu_coh_getm,
    output reg  [31:0]  pmu_coh_upgrade,
    output reg  [31:0]  pmu_coh_c2c_dirty,
    output reg  [31:0]  pmu_coh_inval,
    output reg  [31:0]  pmu_coh_mesi_wb,
    output reg  [31:0]  pmu_coh_e2m,
    output reg  [31:0]  pmu_coh_dma_rds,
    output reg  [31:0]  pmu_coh_dma_wrs,
    output reg  [31:0]  pmu_l2_hits,
    output reg  [31:0]  pmu_l2_misses,
    output reg  [31:0]  pmu_l2_wbs,
    output reg  [31:0]  pmu_l2_bank_conflicts,
    output reg  [31:0]  pmu_dram_rds,
    output reg  [31:0]  pmu_dram_wrs
);

localparam integer AWIDTH    = 20;
localparam integer ID_W      = 7;
localparam integer LADDR_W   = AWIDTH - 4; // 16
localparam integer LINE_BITS = DRAM_LINE_BITS;

`include "soc_addr_map.vh"
localparam integer APB_NUM_SLAVES = 7;
localparam [15:0]  APB_SLV_MASK   = 16'b0000_0000_0111_1111; // S0=CLINT, S1=PLIC, S2=GPIO, S3=UART, S4=DMA, S5=TIMER, S6=SOC_CTRL

wire        soc_sys_halt;
wire [31:0] soc_tohost_u;
assign all_halted = (c0_halted && c1_halted) || soc_sys_halt;

// ============================================================================
// 1. Core 0 <-> L1I0 / MOESI L1D0 / AMBA APB4 MMIO Wires
// ============================================================================
wire        c0_imem_req_v, c0_imem_v0, c0_imem_v1, c0_ic_flush, c0_ic_inv_all;
wire [31:0] c0_imem_pc0, c0_imem_ins0, c0_imem_pc1, c0_imem_ins1;
wire        c0_ic_idle_u, c0_ic_hit_p, c0_ic_miss_p, c0_ic_pf_u, c0_ic_st_u;

wire        c0_dc_v, c0_dc_rdy, c0_dc_we, c0_dc_is_mmio;
wire [3:0]  c0_dc_amo;
wire [AWIDTH-1:0] c0_dc_addr;
wire [31:0] c0_dc_paddr;
wire [31:0] c0_dc_wdata;
wire [3:0]  c0_dc_wstrb;
wire [ID_W-1:0] c0_dc_id;
wire [4:0]  c0_dc_rob_tag;

wire        d0_req_rdy, d0_hit_v, d0_fill_v, c0_mmio_rdy, c0_mmio_rv, c0_mmio_err;
wire [ID_W-1:0] d0_hit_id, d0_fill_id, c0_mmio_rid;
wire [31:0] d0_hit_rd, d0_fill_rd, c0_mmio_rd;

wire        c0_dc_flush, c0_dc_sq_v, c0_dc_cl_req, d0_cl_done;
wire [4:0]  c0_dc_sq_tag, c0_dc_rob_hd;
wire        d0_wr_idle, d0_any_m, d0_snp_inv_v;
wire [LADDR_W-1:0] d0_snp_inv_la;
wire        d0_hit_p, d0_miss_p, d0_wb_p_u, d0_hum_u, d0_mum_u, d0_coal_u, d0_e2m_p, d0_upg_u;

// Unused OoO trace/PMU wires for Core 0
wire        c0_td0_v, c0_td1_v, c0_tf_v;
wire [31:0] c0_td0_s, c0_td0_p, c0_td0_i, c0_td1_s, c0_td1_p, c0_td1_i, c0_tf_s;
wire [5:0]  c0_ti_v; wire [191:0] c0_ti_s;
wire [1:0]  c0_tc_v; wire [63:0]  c0_tc_s;
wire        c0_s0, c0_s1, c0_s2, c0_s3, c0_s4, c0_s5, c0_s6, c0_s7, c0_s8, c0_s9, c0_s10;
wire        c0_replay_p, c0_amo_p, c0_fence_u;
wire [1:0]  c0_v0, c0_v1, c0_v2, c0_v3, c0_v4;
wire [3:0]  c0_exb_u;

// ============================================================================
// 2. Core 1 <-> L1I1 / MOESI L1D1 / CLINT MMIO Wires
// ============================================================================
wire        c1_imem_req_v, c1_imem_v0, c1_imem_v1, c1_ic_flush, c1_ic_inv_all;
wire [31:0] c1_imem_pc0, c1_imem_ins0, c1_imem_pc1, c1_imem_ins1;
wire        c1_ic_idle_u, c1_ic_hit_p, c1_ic_miss_p, c1_ic_pf_u, c1_ic_st_u;

wire        c1_dc_v, c1_dc_rdy, c1_dc_we, c1_dc_is_mmio;
wire [3:0]  c1_dc_amo;
wire [AWIDTH-1:0] c1_dc_addr;
wire [31:0] c1_dc_paddr;
wire [31:0] c1_dc_wdata;
wire [3:0]  c1_dc_wstrb;
wire [ID_W-1:0] c1_dc_id;
wire [4:0]  c1_dc_rob_tag;

wire        d1_req_rdy, d1_hit_v, d1_fill_v, c1_mmio_rdy, c1_mmio_rv, c1_mmio_err;
wire [ID_W-1:0] d1_hit_id, d1_fill_id, c1_mmio_rid;
wire [31:0] d1_hit_rd, d1_fill_rd, c1_mmio_rd;

wire        c1_dc_flush, c1_dc_sq_v, c1_dc_cl_req, d1_cl_done;
wire [4:0]  c1_dc_sq_tag, c1_dc_rob_hd;
wire        d1_wr_idle, d1_any_m, d1_snp_inv_v;
wire [LADDR_W-1:0] d1_snp_inv_la;
wire        d1_hit_p, d1_miss_p, d1_wb_p_u, d1_hum_u, d1_mum_u, d1_coal_u, d1_e2m_p, d1_upg_u;

wire        c1_td0_v, c1_td1_v, c1_tf_v;
wire [31:0] c1_td0_s, c1_td0_p, c1_td0_i, c1_td1_s, c1_td1_p, c1_td1_i, c1_tf_s;
wire [5:0]  c1_ti_v; wire [191:0] c1_ti_s;
wire [1:0]  c1_tc_v; wire [63:0]  c1_tc_s;
wire        c1_s0, c1_s1, c1_s2, c1_s3, c1_s4, c1_s5, c1_s6, c1_s7, c1_s8, c1_s9, c1_s10;
wire        c1_replay_p, c1_amo_p, c1_fence_u;
wire [1:0]  c1_v0, c1_v1, c1_v2, c1_v3, c1_v4;
wire [3:0]  c1_exb_u;

// ============================================================================
// 3. Standardized Dual-Core Uncached MMIO -> AMBA APB4 Fabric (Phase U2)
// ============================================================================
wire        apb_m_psel, apb_m_penable, apb_m_pwrite, apb_m_pready, apb_m_pslverr;
wire [31:0] apb_m_paddr, apb_m_pwdata, apb_m_prdata;
wire [3:0]  apb_m_pstrb;
wire [APB_NUM_SLAVES-1:0]    apb_s_psel, apb_s_pready, apb_s_pslverr;
wire [APB_NUM_SLAVES*32-1:0] apb_s_prdata;
wire        apb_s_penable, apb_s_pwrite;
wire [31:0] apb_s_paddr, apb_s_pwdata;
wire [3:0]  apb_s_pstrb;

wire        clint_mtip0, clint_mtip1;
wire [1:0]  plic_meip;
wire        gpio_irq, uart_rx_irq, uart_tx_irq, dma_ch0_irq, dma_ch1_irq, timer_irq0, timer_irq1;
wire [15:1] soc_irq_inject;
wire [15:1] plic_irq_src = {8'd0, timer_irq1, timer_irq0, dma_ch1_irq, dma_ch0_irq, uart_tx_irq, uart_rx_irq, gpio_irq} | soc_irq_inject;
wire [63:0] clint_mtime_u;

assign c0_dc_rdy = c0_dc_is_mmio ? c0_mmio_rdy : d0_req_rdy;
assign c1_dc_rdy = c1_dc_is_mmio ? c1_mmio_rdy : d1_req_rdy;

// Mutually exclusive: LSQ blocks younger cacheable loads while APB4 MMIO is active!
wire            c0_hit_v_mux   = c0_mmio_rv || d0_hit_v;
wire [ID_W-1:0] c0_hit_id_mux  = c0_mmio_rv ? c0_mmio_rid : d0_hit_id;
wire [31:0]     c0_hit_rd_mux  = c0_mmio_rv ? c0_mmio_rd  : d0_hit_rd;
wire            c0_hit_err_mux = c0_mmio_rv ? c0_mmio_err : 1'b0;

wire            c1_hit_v_mux   = c1_mmio_rv || d1_hit_v;
wire [ID_W-1:0] c1_hit_id_mux  = c1_mmio_rv ? c1_mmio_rid : d1_hit_id;
wire [31:0]     c1_hit_rd_mux  = c1_mmio_rv ? c1_mmio_rd  : d1_hit_rd;
wire            c1_hit_err_mux = c1_mmio_rv ? c1_mmio_err : 1'b0;

// ============================================================================
// 4. Interconnect <-> L1I/L1D <-> Shared L2 <-> Backing DRAM Wires
// ============================================================================
wire               d0_coh_req_v, d0_coh_req_rdy, d0_coh_req_id;
wire [1:0]         d0_coh_req_op;
wire [LADDR_W-1:0] d0_coh_req_la;
wire               d0_snp_v;
wire [1:0]         d0_snp_op, d0_snp_rop;
wire [LADDR_W-1:0] d0_snp_la;
wire [127:0]       d0_snp_rdat;
wire               d0_wb_v, d0_wb_rdy;
wire [LADDR_W-1:0] d0_wb_la;
wire [127:0]       d0_wb_dat;
wire               d0_resp_v, d0_resp_id, d0_resp_sh, d0_resp_c2c, d0_resp_upg;
wire [127:0]       d0_resp_dat;

wire               d1_coh_req_v, d1_coh_req_rdy, d1_coh_req_id;
wire [1:0]         d1_coh_req_op;
wire [LADDR_W-1:0] d1_coh_req_la;
wire               d1_snp_v;
wire [1:0]         d1_snp_op, d1_snp_rop;
wire [LADDR_W-1:0] d1_snp_la;
wire [127:0]       d1_snp_rdat;
wire               d1_wb_v, d1_wb_rdy;
wire [LADDR_W-1:0] d1_wb_la;
wire [127:0]       d1_wb_dat;
wire               d1_resp_v, d1_resp_id, d1_resp_sh, d1_resp_c2c, d1_resp_upg;
wire [127:0]       d1_resp_dat;

wire               i0_rd_v, i0_rd_rdy, i0_rr_v;
wire [LADDR_W-1:0] i0_rd_la;
wire [127:0]       i0_rr_dat;

wire               i1_rd_v, i1_rd_rdy, i1_rr_v;
wire [LADDR_W-1:0] i1_rd_la;
wire [127:0]       i1_rr_dat;

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

wire               coh_gets_p, coh_getm_p, coh_upg_p, coh_c2c_p, coh_inv_p, coh_mwb_p;
wire               coh_dma_rd_p, coh_dma_wr_p;
wire [1:0]         l2_hits_p, l2_misses_p, l2_wbs_p;
wire               l2_bc_p, dram_rd_p, dram_wr_p;

// AMBA AXI4 128-bit Coherent DMA Master <-> AXI4-to-MOESI Bridge Wires (Phase U7)
wire               dma_axi_arvalid, dma_axi_arready, dma_axi_rvalid, dma_axi_rready, dma_axi_rlast;
wire [31:0]        dma_axi_araddr;
wire [7:0]         dma_axi_arlen;
wire [2:0]         dma_axi_arsize;
wire [1:0]         dma_axi_arburst, dma_axi_rresp;
wire [127:0]       dma_axi_rdata;

wire               dma_axi_awvalid, dma_axi_awready, dma_axi_wvalid, dma_axi_wready, dma_axi_wlast;
wire               dma_axi_bvalid, dma_axi_bready;
wire [31:0]        dma_axi_awaddr;
wire [7:0]         dma_axi_awlen;
wire [2:0]         dma_axi_awsize;
wire [1:0]         dma_axi_awburst, dma_axi_bresp;
wire [127:0]       dma_axi_wdata;
wire [15:0]        dma_axi_wstrb;

wire               dma_coh_req_v, dma_coh_req_rdy, dma_coh_req_we, dma_coh_resp_v;
wire [LADDR_W-1:0] dma_coh_req_la;
wire [127:0]       dma_coh_req_wd, dma_coh_resp_rd;
wire [15:0]        dma_coh_req_ws;

// ============================================================================
// 5. Core 0 Subsystem (ooo2w_core #0 + icache_2w #0 + dcache_nb_coh #0)
// ============================================================================
wire c0_trap_u, c1_trap_u;

ooo2w_core #(
    .CORE_ID(32'd0), .RESET_PC(32'd0), .AWIDTH(AWIDTH), .ID_W(ID_W),
    .ENABLE_MMIO_PMA(1), .RAM_SIZE_BYTES(SOC_RAM_SIZE)
) u_core0 (
    .clk(clk), .reset_n(reset_n),
    .imem_req_valid(c0_imem_req_v),
    .imem_pc0(c0_imem_pc0), .imem_valid0(c0_imem_v0), .imem_instr0(c0_imem_ins0),
    .imem_pc1(c0_imem_pc1), .imem_valid1(c0_imem_v1), .imem_instr1(c0_imem_ins1),
    .ic_flush(c0_ic_flush), .ic_invalidate_all(c0_ic_inv_all),
    .dc_req_valid(c0_dc_v), .dc_req_ready(c0_dc_rdy),
    .dc_req_is_write(c0_dc_we), .dc_req_is_mmio(c0_dc_is_mmio),
    .dc_req_amo_op(c0_dc_amo),
    .dc_req_addr(c0_dc_addr), .dc_req_paddr(c0_dc_paddr), .dc_req_wdata(c0_dc_wdata), .dc_req_wstrb(c0_dc_wstrb),
    .dc_req_id(c0_dc_id), .dc_req_rob_tag(c0_dc_rob_tag),
    .dc_hit_valid(c0_hit_v_mux), .dc_hit_id(c0_hit_id_mux), .dc_hit_rdata(c0_hit_rd_mux), .dc_hit_err(c0_hit_err_mux),
    .dc_fill_valid(d0_fill_v), .dc_fill_id(d0_fill_id), .dc_fill_rdata(d0_fill_rd), .dc_fill_err(1'b0),
    .dc_flush(c0_dc_flush), .dc_squash_valid(c0_dc_sq_v),
    .dc_squash_tag(c0_dc_sq_tag), .dc_rob_head(c0_dc_rob_hd),
    .dc_clean_all_req(c0_dc_cl_req), .dc_clean_all_done(d0_cl_done),
    .dc_writes_idle(d0_wr_idle), .dc_any_miss(d0_any_m),
    .coh_snoop_inv_valid((ENABLE_SNOOP_REPLAY != 0) ? d0_snp_inv_v : 1'b0),
    .coh_snoop_inv_line(d0_snp_inv_la),
    .irq_msip(clint_msip0), .irq_mtip(clint_mtip0), .irq_meip(plic_meip[0]),
    .commit0_valid(c0_commit0_valid), .commit0_seq_id(c0_commit0_seq_id),
    .commit0_pc(c0_commit0_pc), .commit0_instr(c0_commit0_instr),
    .commit0_rd_we(c0_commit0_rd_we), .commit0_rd_is_fp(c0_commit0_rd_is_fp), .commit0_rd_addr(c0_commit0_rd_addr), .commit0_rd_wdata(c0_commit0_rd_wdata), .commit0_fflags(c0_commit0_fflags),
    .commit0_mem_valid(c0_commit0_mem_valid), .commit0_mem_we(c0_commit0_mem_we),
    .commit0_mem_addr(c0_commit0_mem_addr), .commit0_mem_wstrb(c0_commit0_mem_wstrb), .commit0_mem_wdata(c0_commit0_mem_wdata),
    .commit1_valid(c0_commit1_valid), .commit1_seq_id(c0_commit1_seq_id),
    .commit1_pc(c0_commit1_pc), .commit1_instr(c0_commit1_instr),
    .commit1_rd_we(c0_commit1_rd_we), .commit1_rd_is_fp(c0_commit1_rd_is_fp), .commit1_rd_addr(c0_commit1_rd_addr), .commit1_rd_wdata(c0_commit1_rd_wdata), .commit1_fflags(c0_commit1_fflags),
    .commit1_mem_valid(c0_commit1_mem_valid), .commit1_mem_we(c0_commit1_mem_we),
    .commit1_mem_addr(c0_commit1_mem_addr), .commit1_mem_wstrb(c0_commit1_mem_wstrb), .commit1_mem_wdata(c0_commit1_mem_wdata),
    .trace_disp0_valid(c0_td0_v), .trace_disp0_seq(c0_td0_s), .trace_disp0_pc(c0_td0_p), .trace_disp0_instr(c0_td0_i),
    .trace_disp1_valid(c0_td1_v), .trace_disp1_seq(c0_td1_s), .trace_disp1_pc(c0_td1_p), .trace_disp1_instr(c0_td1_i),
    .trace_issue_valid(c0_ti_v), .trace_issue_seq(c0_ti_s),
    .trace_cdb_valid(c0_tc_v), .trace_cdb_seq(c0_tc_s),
    .trace_flush_valid(c0_tf_v), .trace_flush_after_seq(c0_tf_s),
    .halted(c0_halted), .x14(c0_x14),
    .stat_rob_full_stall(c0_s0), .stat_rs_full_stall(c0_s1),
    .stat_lsq_full_stall(c0_s2), .stat_cp_full_stall(c0_s3),
    .stat_stlf_hit(c0_s4), .stat_br_total_cnt(c0_v0), .stat_br_mispred_cnt(c0_v1),
    .stat_br_fast_squash(c0_s5), .stat_war_waw_cnt(c0_v2),
    .stat_div_overlap_cyc(c0_s6), .stat_dcache_miss_overlap_cyc(c0_s7),
    .stat_dual_disp(c0_s8), .stat_dual_cdb(c0_s9), .stat_dual_commit(c0_s10),
    .stat_ex_byp_cnt(c0_exb_u), .stat_ras_pred_cnt(c0_v3), .stat_ras_mispred_cnt(c0_v4),
    .stat_replay_flush(c0_replay_p), .stat_amo_commit(c0_amo_p), .stat_fence_commit(c0_fence_u),
    .stat_trap_taken(c0_trap_u)
);

icache_2w #(
    .AWIDTH(AWIDTH), .NUM_SETS(ICACHE_SETS), .ENABLE_PREFETCH(ENABLE_PREFETCH)
) u_icache0 (
    .clk(clk), .reset_n(reset_n),
    .req_valid(c0_imem_req_v), .req_pc(c0_imem_pc0),
    .flush(c0_ic_flush), .invalidate_all(c0_ic_inv_all),
    .resp_valid0(c0_imem_v0), .resp_instr0(c0_imem_ins0),
    .resp_valid1(c0_imem_v1), .resp_instr1(c0_imem_ins1),
    .mem_rd_valid(i0_rd_v), .mem_rd_ready(i0_rd_rdy), .mem_rd_line_addr(i0_rd_la),
    .mem_rresp_valid(i0_rr_v), .mem_rresp_data(i0_rr_dat),
    .icache_idle(c0_ic_idle_u), .pmu_ic_hit(c0_ic_hit_p), .pmu_ic_miss(c0_ic_miss_p),
    .pmu_ic_pf_hit(c0_ic_pf_u), .pmu_ic_stall(c0_ic_st_u)
);

dcache_nb_coh #(
    .AWIDTH(AWIDTH), .NUM_SETS(DCACHE_SETS), .TAG_W(5), .ID_W(ID_W), .ENABLE_MOESI_O(ENABLE_MOESI_O)
) u_dcache0 (
    .clk(clk), .reset_n(reset_n),
    .req_valid(c0_dc_v && !c0_dc_is_mmio), .req_ready(d0_req_rdy),
    .req_is_write(c0_dc_we), .req_amo_op(c0_dc_amo),
    .req_addr(c0_dc_addr), .req_wdata(c0_dc_wdata), .req_wstrb(c0_dc_wstrb),
    .req_id(c0_dc_id), .req_rob_tag(c0_dc_rob_tag),
    .hit_resp_valid(d0_hit_v), .hit_resp_id(d0_hit_id), .hit_resp_rdata(d0_hit_rd),
    .fill_resp_valid(d0_fill_v), .fill_resp_id(d0_fill_id), .fill_resp_rdata(d0_fill_rd),
    .flush(c0_dc_flush), .squash_valid(c0_dc_sq_v), .squash_tag(c0_dc_sq_tag), .rob_head(c0_dc_rob_hd),
    .clean_all_req(c0_dc_cl_req), .clean_all_done(d0_cl_done),
    .snoop_inv_valid(d0_snp_inv_v), .snoop_inv_laddr(d0_snp_inv_la),
    .coh_req_valid(d0_coh_req_v), .coh_req_ready(d0_coh_req_rdy),
    .coh_req_op(d0_coh_req_op), .coh_req_laddr(d0_coh_req_la), .coh_req_id(d0_coh_req_id),
    .snoop_valid(d0_snp_v), .snoop_op(d0_snp_op), .snoop_laddr(d0_snp_la),
    .snoop_resp_op(d0_snp_rop), .snoop_resp_data(d0_snp_rdat),
    .wb_valid(d0_wb_v), .wb_ready(d0_wb_rdy), .wb_laddr(d0_wb_la), .wb_data(d0_wb_dat),
    .coh_resp_valid(d0_resp_v), .coh_resp_id(d0_resp_id), .coh_resp_data(d0_resp_dat),
    .coh_resp_shared(d0_resp_sh), .coh_resp_from_c2c(d0_resp_c2c), .coh_resp_upgrade_ack(d0_resp_upg),
    .dbg_wr_commit_valid(d0_wr_commit_valid), .dbg_wr_commit_laddr(d0_wr_commit_laddr), .dbg_wr_commit_data(d0_wr_commit_data),
    .dcache_idle(d0_idle), .dcache_writes_idle(d0_wr_idle), .dcache_any_miss(d0_any_m),
    .cache_hit(d0_hit_p), .cache_miss(d0_miss_p), .cache_wb(d0_wb_p_u),
    .cache_hum(d0_hum_u), .cache_mum_cyc(d0_mum_u), .cache_mshr_coalesce(d0_coal_u),
    .pmu_silent_e2m(d0_e2m_p), .pmu_upgrade_miss(d0_upg_u)
);

// ============================================================================
// 6. Core 1 Subsystem (ooo2w_core #1 + icache_2w #1 + dcache_nb_coh #1)
// ============================================================================
ooo2w_core #(
    .CORE_ID(32'd1), .RESET_PC(32'd0), .AWIDTH(AWIDTH), .ID_W(ID_W),
    .ENABLE_MMIO_PMA(1), .RAM_SIZE_BYTES(SOC_RAM_SIZE)
) u_core1 (
    .clk(clk), .reset_n(reset_n),
    .imem_req_valid(c1_imem_req_v),
    .imem_pc0(c1_imem_pc0), .imem_valid0(c1_imem_v0), .imem_instr0(c1_imem_ins0),
    .imem_pc1(c1_imem_pc1), .imem_valid1(c1_imem_v1), .imem_instr1(c1_imem_ins1),
    .ic_flush(c1_ic_flush), .ic_invalidate_all(c1_ic_inv_all),
    .dc_req_valid(c1_dc_v), .dc_req_ready(c1_dc_rdy),
    .dc_req_is_write(c1_dc_we), .dc_req_is_mmio(c1_dc_is_mmio),
    .dc_req_amo_op(c1_dc_amo),
    .dc_req_addr(c1_dc_addr), .dc_req_paddr(c1_dc_paddr), .dc_req_wdata(c1_dc_wdata), .dc_req_wstrb(c1_dc_wstrb),
    .dc_req_id(c1_dc_id), .dc_req_rob_tag(c1_dc_rob_tag),
    .dc_hit_valid(c1_hit_v_mux), .dc_hit_id(c1_hit_id_mux), .dc_hit_rdata(c1_hit_rd_mux), .dc_hit_err(c1_hit_err_mux),
    .dc_fill_valid(d1_fill_v), .dc_fill_id(d1_fill_id), .dc_fill_rdata(d1_fill_rd), .dc_fill_err(1'b0),
    .dc_flush(c1_dc_flush), .dc_squash_valid(c1_dc_sq_v),
    .dc_squash_tag(c1_dc_sq_tag), .dc_rob_head(c1_dc_rob_hd),
    .dc_clean_all_req(c1_dc_cl_req), .dc_clean_all_done(d1_cl_done),
    .dc_writes_idle(d1_wr_idle), .dc_any_miss(d1_any_m),
    .coh_snoop_inv_valid((ENABLE_SNOOP_REPLAY != 0) ? d1_snp_inv_v : 1'b0),
    .coh_snoop_inv_line(d1_snp_inv_la),
    .irq_msip(clint_msip1), .irq_mtip(clint_mtip1), .irq_meip(plic_meip[1]),
    .commit0_valid(c1_commit0_valid), .commit0_seq_id(c1_commit0_seq_id),
    .commit0_pc(c1_commit0_pc), .commit0_instr(c1_commit0_instr),
    .commit0_rd_we(c1_commit0_rd_we), .commit0_rd_is_fp(c1_commit0_rd_is_fp), .commit0_rd_addr(c1_commit0_rd_addr), .commit0_rd_wdata(c1_commit0_rd_wdata), .commit0_fflags(c1_commit0_fflags),
    .commit0_mem_valid(c1_commit0_mem_valid), .commit0_mem_we(c1_commit0_mem_we),
    .commit0_mem_addr(c1_commit0_mem_addr), .commit0_mem_wstrb(c1_commit0_mem_wstrb), .commit0_mem_wdata(c1_commit0_mem_wdata),
    .commit1_valid(c1_commit1_valid), .commit1_seq_id(c1_commit1_seq_id),
    .commit1_pc(c1_commit1_pc), .commit1_instr(c1_commit1_instr),
    .commit1_rd_we(c1_commit1_rd_we), .commit1_rd_is_fp(c1_commit1_rd_is_fp), .commit1_rd_addr(c1_commit1_rd_addr), .commit1_rd_wdata(c1_commit1_rd_wdata), .commit1_fflags(c1_commit1_fflags),
    .commit1_mem_valid(c1_commit1_mem_valid), .commit1_mem_we(c1_commit1_mem_we),
    .commit1_mem_addr(c1_commit1_mem_addr), .commit1_mem_wstrb(c1_commit1_mem_wstrb), .commit1_mem_wdata(c1_commit1_mem_wdata),
    .trace_disp0_valid(c1_td0_v), .trace_disp0_seq(c1_td0_s), .trace_disp0_pc(c1_td0_p), .trace_disp0_instr(c1_td0_i),
    .trace_disp1_valid(c1_td1_v), .trace_disp1_seq(c1_td1_s), .trace_disp1_pc(c1_td1_p), .trace_disp1_instr(c1_td1_i),
    .trace_issue_valid(c1_ti_v), .trace_issue_seq(c1_ti_s),
    .trace_cdb_valid(c1_tc_v), .trace_cdb_seq(c1_tc_s),
    .trace_flush_valid(c1_tf_v), .trace_flush_after_seq(c1_tf_s),
    .halted(c1_halted), .x14(c1_x14),
    .stat_rob_full_stall(c1_s0), .stat_rs_full_stall(c1_s1),
    .stat_lsq_full_stall(c1_s2), .stat_cp_full_stall(c1_s3),
    .stat_stlf_hit(c1_s4), .stat_br_total_cnt(c1_v0), .stat_br_mispred_cnt(c1_v1),
    .stat_br_fast_squash(c1_s5), .stat_war_waw_cnt(c1_v2),
    .stat_div_overlap_cyc(c1_s6), .stat_dcache_miss_overlap_cyc(c1_s7),
    .stat_dual_disp(c1_s8), .stat_dual_cdb(c1_s9), .stat_dual_commit(c1_s10),
    .stat_ex_byp_cnt(c1_exb_u), .stat_ras_pred_cnt(c1_v3), .stat_ras_mispred_cnt(c1_v4),
    .stat_replay_flush(c1_replay_p), .stat_amo_commit(c1_amo_p), .stat_fence_commit(c1_fence_u),
    .stat_trap_taken(c1_trap_u)
);

icache_2w #(
    .AWIDTH(AWIDTH), .NUM_SETS(ICACHE_SETS), .ENABLE_PREFETCH(ENABLE_PREFETCH)
) u_icache1 (
    .clk(clk), .reset_n(reset_n),
    .req_valid(c1_imem_req_v), .req_pc(c1_imem_pc0),
    .flush(c1_ic_flush), .invalidate_all(c1_ic_inv_all),
    .resp_valid0(c1_imem_v0), .resp_instr0(c1_imem_ins0),
    .resp_valid1(c1_imem_v1), .resp_instr1(c1_imem_ins1),
    .mem_rd_valid(i1_rd_v), .mem_rd_ready(i1_rd_rdy), .mem_rd_line_addr(i1_rd_la),
    .mem_rresp_valid(i1_rr_v), .mem_rresp_data(i1_rr_dat),
    .icache_idle(c1_ic_idle_u), .pmu_ic_hit(c1_ic_hit_p), .pmu_ic_miss(c1_ic_miss_p),
    .pmu_ic_pf_hit(c1_ic_pf_u), .pmu_ic_stall(c1_ic_st_u)
);

dcache_nb_coh #(
    .AWIDTH(AWIDTH), .NUM_SETS(DCACHE_SETS), .TAG_W(5), .ID_W(ID_W), .ENABLE_MOESI_O(ENABLE_MOESI_O)
) u_dcache1 (
    .clk(clk), .reset_n(reset_n),
    .req_valid(c1_dc_v && !c1_dc_is_mmio), .req_ready(d1_req_rdy),
    .req_is_write(c1_dc_we), .req_amo_op(c1_dc_amo),
    .req_addr(c1_dc_addr), .req_wdata(c1_dc_wdata), .req_wstrb(c1_dc_wstrb),
    .req_id(c1_dc_id), .req_rob_tag(c1_dc_rob_tag),
    .hit_resp_valid(d1_hit_v), .hit_resp_id(d1_hit_id), .hit_resp_rdata(d1_hit_rd),
    .fill_resp_valid(d1_fill_v), .fill_resp_id(d1_fill_id), .fill_resp_rdata(d1_fill_rd),
    .flush(c1_dc_flush), .squash_valid(c1_dc_sq_v), .squash_tag(c1_dc_sq_tag), .rob_head(c1_dc_rob_hd),
    .clean_all_req(c1_dc_cl_req), .clean_all_done(d1_cl_done),
    .snoop_inv_valid(d1_snp_inv_v), .snoop_inv_laddr(d1_snp_inv_la),
    .coh_req_valid(d1_coh_req_v), .coh_req_ready(d1_coh_req_rdy),
    .coh_req_op(d1_coh_req_op), .coh_req_laddr(d1_coh_req_la), .coh_req_id(d1_coh_req_id),
    .snoop_valid(d1_snp_v), .snoop_op(d1_snp_op), .snoop_laddr(d1_snp_la),
    .snoop_resp_op(d1_snp_rop), .snoop_resp_data(d1_snp_rdat),
    .wb_valid(d1_wb_v), .wb_ready(d1_wb_rdy), .wb_laddr(d1_wb_la), .wb_data(d1_wb_dat),
    .coh_resp_valid(d1_resp_v), .coh_resp_id(d1_resp_id), .coh_resp_data(d1_resp_dat),
    .coh_resp_shared(d1_resp_sh), .coh_resp_from_c2c(d1_resp_c2c), .coh_resp_upgrade_ack(d1_resp_upg),
    .dbg_wr_commit_valid(d1_wr_commit_valid), .dbg_wr_commit_laddr(d1_wr_commit_laddr), .dbg_wr_commit_data(d1_wr_commit_data),
    .dcache_idle(d1_idle), .dcache_writes_idle(d1_wr_idle), .dcache_any_miss(d1_any_m),
    .cache_hit(d1_hit_p), .cache_miss(d1_miss_p), .cache_wb(d1_wb_p_u),
    .cache_hum(d1_hum_u), .cache_mum_cyc(d1_mum_u), .cache_mshr_coalesce(d1_coal_u),
    .pmu_silent_e2m(d1_e2m_p), .pmu_upgrade_miss(d1_upg_u)
);

// ============================================================================
// 7. Standardized AMBA APB4 Peripheral Fabric, MOESI Crossbar, Shared L2 & DRAM
// ============================================================================
mmio_to_apb4_bridge #(.ID_W(ID_W)) u_apb_bridge (
    .clk(clk), .reset_n(reset_n),
    .c0_req_valid(c0_dc_v && c0_dc_is_mmio), .c0_req_ready(c0_mmio_rdy),
    .c0_req_is_write(c0_dc_we), .c0_req_addr(c0_dc_paddr),
    .c0_req_wdata(c0_dc_wdata), .c0_req_wstrb(c0_dc_wstrb), .c0_req_id(c0_dc_id),
    .c0_resp_valid(c0_mmio_rv), .c0_resp_id(c0_mmio_rid),
    .c0_resp_rdata(c0_mmio_rd), .c0_resp_err(c0_mmio_err),
    .c1_req_valid(c1_dc_v && c1_dc_is_mmio), .c1_req_ready(c1_mmio_rdy),
    .c1_req_is_write(c1_dc_we), .c1_req_addr(c1_dc_paddr),
    .c1_req_wdata(c1_dc_wdata), .c1_req_wstrb(c1_dc_wstrb), .c1_req_id(c1_dc_id),
    .c1_resp_valid(c1_mmio_rv), .c1_resp_id(c1_mmio_rid),
    .c1_resp_rdata(c1_mmio_rd), .c1_resp_err(c1_mmio_err),
    .m_psel(apb_m_psel), .m_penable(apb_m_penable), .m_pwrite(apb_m_pwrite),
    .m_paddr(apb_m_paddr), .m_pwdata(apb_m_pwdata), .m_pstrb(apb_m_pstrb),
    .m_pready(apb_m_pready), .m_prdata(apb_m_prdata), .m_pslverr(apb_m_pslverr)
);

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
    .mtip0_out(clint_mtip0), .mtip1_out(clint_mtip1), .mtime_out(clint_mtime_u)
);

apb4_plic #(
    .NUM_SOURCES(16),
    .NUM_HARTS(2),
    .PRIO_BITS(3)
) u_plic (
    .pclk(clk), .preset_n(reset_n),
    .psel(apb_s_psel[APB_SLV_PLIC]), .penable(apb_s_penable), .pwrite(apb_s_pwrite),
    .paddr(apb_s_paddr), .pwdata(apb_s_pwdata), .pstrb(apb_s_pstrb),
    .pready(apb_s_pready[APB_SLV_PLIC]),
    .prdata(apb_s_prdata[APB_SLV_PLIC*32 +: 32]),
    .pslverr(apb_s_pslverr[APB_SLV_PLIC]),
    .irq_src_in(plic_irq_src),
    .meip_out(plic_meip)
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

dma_axi4_coh u_dma (
    .clk(clk), .reset_n(reset_n),
    .psel(apb_s_psel[APB_SLV_DMA]), .penable(apb_s_penable), .pwrite(apb_s_pwrite),
    .paddr(apb_s_paddr), .pwdata(apb_s_pwdata), .pstrb(apb_s_pstrb),
    .pready(apb_s_pready[APB_SLV_DMA]),
    .prdata(apb_s_prdata[APB_SLV_DMA*32 +: 32]),
    .pslverr(apb_s_pslverr[APB_SLV_DMA]),
    .m_axi_arvalid(dma_axi_arvalid), .m_axi_arready(dma_axi_arready),
    .m_axi_araddr(dma_axi_araddr), .m_axi_arlen(dma_axi_arlen),
    .m_axi_arsize(dma_axi_arsize), .m_axi_arburst(dma_axi_arburst),
    .m_axi_rvalid(dma_axi_rvalid), .m_axi_rready(dma_axi_rready),
    .m_axi_rdata(dma_axi_rdata), .m_axi_rresp(dma_axi_rresp), .m_axi_rlast(dma_axi_rlast),
    .m_axi_awvalid(dma_axi_awvalid), .m_axi_awready(dma_axi_awready),
    .m_axi_awaddr(dma_axi_awaddr), .m_axi_awlen(dma_axi_awlen),
    .m_axi_awsize(dma_axi_awsize), .m_axi_awburst(dma_axi_awburst),
    .m_axi_wvalid(dma_axi_wvalid), .m_axi_wready(dma_axi_wready),
    .m_axi_wdata(dma_axi_wdata), .m_axi_wstrb(dma_axi_wstrb), .m_axi_wlast(dma_axi_wlast),
    .m_axi_bvalid(dma_axi_bvalid), .m_axi_bready(dma_axi_bready), .m_axi_bresp(dma_axi_bresp),
    .irq_ch0_out(dma_ch0_irq), .irq_ch1_out(dma_ch1_irq)
);

axi4_to_coh_bridge #(
    .LADDR_W(LADDR_W),
    .RAM_SIZE_BYTES(SOC_RAM_SIZE)
) u_axi_coh (
    .clk(clk), .reset_n(reset_n),
    .s_axi_arvalid(dma_axi_arvalid), .s_axi_arready(dma_axi_arready),
    .s_axi_araddr(dma_axi_araddr), .s_axi_arlen(dma_axi_arlen),
    .s_axi_arsize(dma_axi_arsize), .s_axi_arburst(dma_axi_arburst),
    .s_axi_rvalid(dma_axi_rvalid), .s_axi_rready(dma_axi_rready),
    .s_axi_rdata(dma_axi_rdata), .s_axi_rresp(dma_axi_rresp), .s_axi_rlast(dma_axi_rlast),
    .s_axi_awvalid(dma_axi_awvalid), .s_axi_awready(dma_axi_awready),
    .s_axi_awaddr(dma_axi_awaddr), .s_axi_awlen(dma_axi_awlen),
    .s_axi_awsize(dma_axi_awsize), .s_axi_awburst(dma_axi_awburst),
    .s_axi_wvalid(dma_axi_wvalid), .s_axi_wready(dma_axi_wready),
    .s_axi_wdata(dma_axi_wdata), .s_axi_wstrb(dma_axi_wstrb), .s_axi_wlast(dma_axi_wlast),
    .s_axi_bvalid(dma_axi_bvalid), .s_axi_bready(dma_axi_bready), .s_axi_bresp(dma_axi_bresp),
    .dma_coh_req_valid(dma_coh_req_v), .dma_coh_req_ready(dma_coh_req_rdy),
    .dma_coh_req_is_write(dma_coh_req_we), .dma_coh_req_laddr(dma_coh_req_la),
    .dma_coh_req_wdata(dma_coh_req_wd), .dma_coh_req_wstrb(dma_coh_req_ws),
    .dma_coh_resp_valid(dma_coh_resp_v), .dma_coh_resp_rdata(dma_coh_resp_rd)
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
    .NUM_CORES(32'd2),
    .FEATURE_MASK(32'h0000_003F)
) u_soc_ctrl (
    .pclk(clk), .preset_n(reset_n),
    .psel(apb_s_psel[APB_SLV_SOC_CTRL]), .penable(apb_s_penable), .pwrite(apb_s_pwrite),
    .paddr(apb_s_paddr), .pwdata(apb_s_pwdata), .pstrb(apb_s_pstrb),
    .pready(apb_s_pready[APB_SLV_SOC_CTRL]),
    .prdata(apb_s_prdata[APB_SLV_SOC_CTRL*32 +: 32]),
    .pslverr(apb_s_pslverr[APB_SLV_SOC_CTRL]),
    .pmu_cycles_in(pmu_cycles),
    .pmu_c0_instret_in(pmu_c0_instret),
    .pmu_c1_instret_in(pmu_c1_instret),
    .pmu_coh_c2c_in(pmu_coh_c2c_dirty),
    .pmu_l2_hits_in(pmu_l2_hits),
    .pmu_l2_misses_in(pmu_l2_misses),
    .pmu_dram_rds_in(pmu_dram_rds),
    .pmu_dram_wrs_in(pmu_dram_wrs),
    .sys_halt_out(soc_sys_halt),
    .tohost_out(soc_tohost_u),
    .irq_inject_out(soc_irq_inject)
);

coh_interconnect #(
    .LADDR_W(LADDR_W), .M_ID_W(3), .ENABLE_MOESI_O(ENABLE_MOESI_O)
) u_coh (
    .clk(clk), .reset_n(reset_n),
    .d0_coh_req_valid(d0_coh_req_v), .d0_coh_req_ready(d0_coh_req_rdy),
    .d0_coh_req_op(d0_coh_req_op), .d0_coh_req_laddr(d0_coh_req_la), .d0_coh_req_id(d0_coh_req_id),
    .d0_snoop_valid(d0_snp_v), .d0_snoop_op(d0_snp_op), .d0_snoop_laddr(d0_snp_la),
    .d0_snoop_resp_op(d0_snp_rop), .d0_snoop_resp_data(d0_snp_rdat),
    .d0_wb_valid(d0_wb_v), .d0_wb_ready(d0_wb_rdy), .d0_wb_laddr(d0_wb_la), .d0_wb_data(d0_wb_dat),
    .d0_coh_resp_valid(d0_resp_v), .d0_coh_resp_id(d0_resp_id), .d0_coh_resp_data(d0_resp_dat),
    .d0_coh_resp_shared(d0_resp_sh), .d0_coh_resp_from_c2c(d0_resp_c2c), .d0_coh_resp_upgrade_ack(d0_resp_upg),
    .d1_coh_req_valid(d1_coh_req_v), .d1_coh_req_ready(d1_coh_req_rdy),
    .d1_coh_req_op(d1_coh_req_op), .d1_coh_req_laddr(d1_coh_req_la), .d1_coh_req_id(d1_coh_req_id),
    .d1_snoop_valid(d1_snp_v), .d1_snoop_op(d1_snp_op), .d1_snoop_laddr(d1_snp_la),
    .d1_snoop_resp_op(d1_snp_rop), .d1_snoop_resp_data(d1_snp_rdat),
    .d1_wb_valid(d1_wb_v), .d1_wb_ready(d1_wb_rdy), .d1_wb_laddr(d1_wb_la), .d1_wb_data(d1_wb_dat),
    .d1_coh_resp_valid(d1_resp_v), .d1_coh_resp_id(d1_resp_id), .d1_coh_resp_data(d1_resp_dat),
    .d1_coh_resp_shared(d1_resp_sh), .d1_coh_resp_from_c2c(d1_resp_c2c), .d1_coh_resp_upgrade_ack(d1_resp_upg),
    .dma_coh_req_valid(dma_coh_req_v), .dma_coh_req_ready(dma_coh_req_rdy),
    .dma_coh_req_is_write(dma_coh_req_we), .dma_coh_req_laddr(dma_coh_req_la),
    .dma_coh_req_wdata(dma_coh_req_wd), .dma_coh_req_wstrb(dma_coh_req_ws),
    .dma_coh_resp_valid(dma_coh_resp_v), .dma_coh_resp_rdata(dma_coh_resp_rd),
    .dma_wr_commit_valid(dma_wr_commit_valid), .dma_wr_commit_laddr(dma_wr_commit_laddr),
    .dma_wr_commit_data(dma_wr_commit_data),
    .i0_rd_valid(i0_rd_v), .i0_rd_ready(i0_rd_rdy), .i0_rd_laddr(i0_rd_la),
    .i0_rresp_valid(i0_rr_v), .i0_rresp_data(i0_rr_dat),
    .i1_rd_valid(i1_rd_v), .i1_rd_ready(i1_rd_rdy), .i1_rd_laddr(i1_rd_la),
    .i1_rresp_valid(i1_rr_v), .i1_rresp_data(i1_rr_dat),
    .l2_m0_req_valid(l2_m0_v), .l2_m0_req_ready(l2_m0_rdy), .l2_m0_req_is_write(l2_m0_we),
    .l2_m0_req_laddr(l2_m0_la), .l2_m0_req_wdata(l2_m0_wd), .l2_m0_req_id(l2_m0_id),
    .l2_m0_resp_valid(l2_m0_rv), .l2_m0_resp_id(l2_m0_rid), .l2_m0_resp_data(l2_m0_rd),
    .l2_m1_req_valid(l2_m1_v), .l2_m1_req_ready(l2_m1_rdy), .l2_m1_req_is_write(l2_m1_we),
    .l2_m1_req_laddr(l2_m1_la), .l2_m1_req_wdata(l2_m1_wd), .l2_m1_req_id(l2_m1_id),
    .l2_m1_resp_valid(l2_m1_rv), .l2_m1_resp_id(l2_m1_rid), .l2_m1_resp_data(l2_m1_rd),
    .bus_evt_valid(bus_evt_valid), .bus_evt_src(bus_evt_src), .bus_evt_op(bus_evt_op),
    .bus_evt_laddr(bus_evt_laddr), .bus_evt_c2c(bus_evt_c2c), .bus_evt_shared(bus_evt_shared),
    .coh_idle(coh_idle),
    .pmu_coh_gets(coh_gets_p), .pmu_coh_getm(coh_getm_p), .pmu_coh_upgrade(coh_upg_p),
    .pmu_coh_c2c_dirty(coh_c2c_p), .pmu_coh_inval(coh_inv_p), .pmu_coh_mesi_wb(coh_mwb_p),
    .pmu_coh_dma_rd(coh_dma_rd_p), .pmu_coh_dma_wr(coh_dma_wr_p)
);

l2_cache_shared #(
    .LADDR_W(LADDR_W), .SETS_PER_BANK(L2_SETS_BANK), .M_ID_W(3), .HIT_LAT(L2_HIT_LAT)
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
    .pmu_l2_wbs(l2_wbs_p), .pmu_l2_bank_conflict(l2_bc_p)
);

main_memory_dram #(
    .LINE_BITS(LINE_BITS), .LATENCY(DRAM_LATENCY), .ID_W(2)
) u_dram (
    .clk(clk), .rst_n(reset_n), .jitter_en(dram_jitter_en),
    .mem_rd_valid(dram_rd_v), .mem_rd_ready(dram_rd_rdy),
    .mem_rd_line_addr(dram_rd_la[LINE_BITS-1:0]), .mem_rd_id(dram_rd_id),
    .mem_rresp_valid(dram_rr_v), .mem_rresp_id(dram_rr_id), .mem_rresp_data(dram_rr_dat),
    .mem_wr_valid(dram_wr_v), .mem_wr_ready(dram_wr_rdy),
    .mem_wr_line_addr(dram_wr_la[LINE_BITS-1:0]), .mem_wr_data(dram_wr_dat),
    .pmu_dram_rd(dram_rd_p), .pmu_dram_wr(dram_wr_p)
);

// ============================================================================
// 8. Hardware Performance Monitoring Unit (PMU) Counters
// ============================================================================
wire [31:0] c0_ret_inc = {31'd0, c0_commit0_valid} + {31'd0, c0_commit1_valid};
wire [31:0] c1_ret_inc = {31'd0, c1_commit0_valid} + {31'd0, c1_commit1_valid};

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        pmu_cycles            <= 32'd0;
        pmu_c0_cycles         <= 32'd0;
        pmu_c1_cycles         <= 32'd0;
        pmu_c0_instret        <= 32'd0;
        pmu_c1_instret        <= 32'd0;
        pmu_c0_ic_hits        <= 32'd0;
        pmu_c0_ic_misses      <= 32'd0;
        pmu_c1_ic_hits        <= 32'd0;
        pmu_c1_ic_misses      <= 32'd0;
        pmu_c0_dc_hits        <= 32'd0;
        pmu_c0_dc_misses      <= 32'd0;
        pmu_c1_dc_hits        <= 32'd0;
        pmu_c1_dc_misses      <= 32'd0;
        pmu_c0_replay         <= 32'd0;
        pmu_c1_replay         <= 32'd0;
        pmu_c0_amo            <= 32'd0;
        pmu_c1_amo            <= 32'd0;
        pmu_coh_gets          <= 32'd0;
        pmu_coh_getm          <= 32'd0;
        pmu_coh_upgrade       <= 32'd0;
        pmu_coh_c2c_dirty     <= 32'd0;
        pmu_coh_inval         <= 32'd0;
        pmu_coh_mesi_wb       <= 32'd0;
        pmu_coh_e2m           <= 32'd0;
        pmu_coh_dma_rds       <= 32'd0;
        pmu_coh_dma_wrs       <= 32'd0;
        pmu_l2_hits           <= 32'd0;
        pmu_l2_misses         <= 32'd0;
        pmu_l2_wbs            <= 32'd0;
        pmu_l2_bank_conflicts <= 32'd0;
        pmu_dram_rds          <= 32'd0;
        pmu_dram_wrs          <= 32'd0;
    end else if (!all_halted) begin
        pmu_cycles <= pmu_cycles + 32'd1;
        if (!c0_halted) begin
            pmu_c0_cycles  <= pmu_c0_cycles + 32'd1;
            pmu_c0_instret <= pmu_c0_instret + c0_ret_inc;
            if (c0_ic_hit_p)  pmu_c0_ic_hits   <= pmu_c0_ic_hits + 32'd1;
            if (c0_ic_miss_p) pmu_c0_ic_misses <= pmu_c0_ic_misses + 32'd1;
            if (d0_hit_p)     pmu_c0_dc_hits   <= pmu_c0_dc_hits + 32'd1;
            if (d0_miss_p)    pmu_c0_dc_misses <= pmu_c0_dc_misses + 32'd1;
            if (c0_replay_p)  pmu_c0_replay    <= pmu_c0_replay + 32'd1;
            if (c0_amo_p)     pmu_c0_amo       <= pmu_c0_amo + 32'd1;
        end
        if (!c1_halted) begin
            pmu_c1_cycles  <= pmu_c1_cycles + 32'd1;
            pmu_c1_instret <= pmu_c1_instret + c1_ret_inc;
            if (c1_ic_hit_p)  pmu_c1_ic_hits   <= pmu_c1_ic_hits + 32'd1;
            if (c1_ic_miss_p) pmu_c1_ic_misses <= pmu_c1_ic_misses + 32'd1;
            if (d1_hit_p)     pmu_c1_dc_hits   <= pmu_c1_dc_hits + 32'd1;
            if (d1_miss_p)    pmu_c1_dc_misses <= pmu_c1_dc_misses + 32'd1;
            if (c1_replay_p)  pmu_c1_replay    <= pmu_c1_replay + 32'd1;
            if (c1_amo_p)     pmu_c1_amo       <= pmu_c1_amo + 32'd1;
        end
        if (coh_gets_p) pmu_coh_gets      <= pmu_coh_gets + 32'd1;
        if (coh_getm_p) pmu_coh_getm      <= pmu_coh_getm + 32'd1;
        if (coh_upg_p)  pmu_coh_upgrade   <= pmu_coh_upgrade + 32'd1;
        if (coh_c2c_p)  pmu_coh_c2c_dirty <= pmu_coh_c2c_dirty + 32'd1;
        if (coh_inv_p)  pmu_coh_inval     <= pmu_coh_inval + 32'd1;
        if (coh_mwb_p)    pmu_coh_mesi_wb <= pmu_coh_mesi_wb + 32'd1;
        if (coh_dma_rd_p) pmu_coh_dma_rds <= pmu_coh_dma_rds + 32'd1;
        if (coh_dma_wr_p) pmu_coh_dma_wrs <= pmu_coh_dma_wrs + 32'd1;
        pmu_coh_e2m           <= pmu_coh_e2m + {31'd0, d0_e2m_p} + {31'd0, d1_e2m_p};
        pmu_l2_hits           <= pmu_l2_hits   + {30'd0, l2_hits_p};
        pmu_l2_misses         <= pmu_l2_misses + {30'd0, l2_misses_p};
        pmu_l2_wbs            <= pmu_l2_wbs    + {30'd0, l2_wbs_p};
        if (l2_bc_p)    pmu_l2_bank_conflicts <= pmu_l2_bank_conflicts + 32'd1;
        if (dram_rd_p)  pmu_dram_rds          <= pmu_dram_rds + 32'd1;
        if (dram_wr_p)  pmu_dram_wrs          <= pmu_dram_wrs + 32'd1;
    end
end

endmodule
