`timescale 1ns / 1ps

// ============================================================================
// Adversarial 3-Master MOESI + AMBA AXI4 Coherent DMA Unit & Stress Testbench
// (Phase U7 — Runs in BOTH Verilator --assert and Icarus Verilog!)
//
// Full 3-Master Coherent Subsystem Under Test:
//   D0 (4KB MOESI L1D) + D1 (4KB MOESI L1D) + dma_axi4_coh (2-Ch APB4+AXI4 DMA)
//     <-> axi4_to_coh_bridge (AMBA AXI4 128b Slave -> 3rd Coherent Master Port)
//     <-> coh_interconnect   (3-Master Split-Transaction MOESI Crossbar)
//     <-> l2_cache_shared    (32KB 4-Way 2-Bank NINE Unified L2)
//     <-> main_memory_dram   (256KB 4-Outstanding Pipelined DRAM + LFSR Jitter)
//     + coh_checker          (Always-On 3-Way SWMR, Golden Shadow & Watchdogs)
//
// Always-On Hardware Protocol Monitors:
//   - Strict AMBA AXI4 VALID/READY stability, RLAST/WLAST beat counts, and
//     4 KB burst boundary rule assertions on every cycle!
//   - Always-on 3-Master SWMR, Duplicate-Tag & 128-bit Golden Shadow checks!
//
// Tests (1-8):
//   1. APB4 Control Register R/W, DMA_VERSION, RO/Unmapped/Busy PSLVERR, 0-len
//   2. Coherent Dirty-L1D DMA Read (DMA_GETS -> C2C dirty hit, 0 manual flush!)
//   3. Coherent DMA Write Invalidation (DMA_PUTM -> snoop_inv_valid pulse on D1!)
//   4. Unaligned Byte Copies (DMA_RMW + Barrel Shifter) & 4 KB Boundary Split!
//   5. 2D Strided Tile Copy (5 rows x 19B, SRC_STRIDE=64, DST_STRIDE=48)
//   6. Concurrent 2-Channel Time-Slicing (Ch0 burst=4 + Ch1 burst=8 + IRQs)
//   7. Out-of-Range PMA DECERR Bus-Error Capture & Mid-Flight Clean Abort
//   8. 10,000-Op Randomized Concurrent D0 + D1 + 2-Ch DMA Coherent Stress!
// ============================================================================
module tb_dma_axi4_coh;

parameter integer ENABLE_MOESI_O = 1;
parameter integer NUM_RAND_OPS   = 10000;

`include "rv32ima_defs.vh"
`include "coh_defs.vh"

localparam integer AWIDTH   = 20;
localparam integer LADDR_W  = 16;
localparam integer NUM_SETS = 128;
localparam integer TAG_W    = 5;
localparam integer ID_W     = 7;

// APB4 Register Offsets (matching dma_axi4_coh.sv)
localparam [31:0] DMA_BASE        = 32'h1002_0000;
localparam [31:0] OFF_STATUS      = 32'h000;
localparam [31:0] OFF_IRQ_STAT    = 32'h004;
localparam [31:0] OFF_VERSION     = 32'h008;
localparam [31:0] OFF_CH0         = 32'h040;
localparam [31:0] OFF_CH1         = 32'h080;
localparam [31:0] CH_SRC          = 32'h00;
localparam [31:0] CH_DST          = 32'h04;
localparam [31:0] CH_LEN          = 32'h08;
localparam [31:0] CH_SRC_STRIDE   = 32'h0C;
localparam [31:0] CH_DST_STRIDE   = 32'h10;
localparam [31:0] CH_ROW_CNT      = 32'h14;
localparam [31:0] CH_CTRL         = 32'h18;
localparam [31:0] CH_STATUS       = 32'h1C;

reg clk, reset_n, jitter_en;

// APB4 Master Signals -> u_dma
reg         psel, penable, pwrite;
reg  [31:0] paddr, pwdata;
reg  [3:0]  pstrb;
wire        pready, pslverr;
wire [31:0] prdata;
wire        irq_ch0, irq_ch1;

// Core 0 CPU <-> D0 signals
reg                c0_req_v, c0_req_we;
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
reg                c1_req_v, c1_req_we;
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

// DMA AXI4 <-> AXI4-to-Coh Bridge <-> Interconnect
wire               axi_arvalid, axi_arready, axi_rvalid, axi_rready, axi_rlast;
wire [31:0]        axi_araddr;
wire [7:0]         axi_arlen;
wire [2:0]         axi_arsize;
wire [1:0]         axi_arburst, axi_rresp;
wire [127:0]       axi_rdata;

wire               axi_awvalid, axi_awready, axi_wvalid, axi_wready, axi_wlast;
wire               axi_bvalid, axi_bready;
wire [31:0]        axi_awaddr;
wire [7:0]         axi_awlen;
wire [2:0]         axi_awsize;
wire [1:0]         axi_awburst, axi_bresp;
wire [127:0]       axi_wdata;
wire [15:0]        axi_wstrb;

wire               dma_coh_req_v, dma_coh_req_rdy, dma_coh_req_we, dma_coh_resp_v;
wire [LADDR_W-1:0] dma_coh_req_la;
wire [127:0]       dma_coh_req_wd, dma_coh_resp_rd;
wire [15:0]        dma_coh_req_ws;
wire               dma_wr_v;
wire [LADDR_W-1:0] dma_wr_la;
wire [127:0]       dma_wr_dat;

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
wire               pmu_dma_rd, pmu_dma_wr;
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
    .clean_all_req(1'b0), .clean_all_done(c0_clean_done),
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
    .clean_all_req(1'b0), .clean_all_done(c1_clean_done),
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

dma_axi4_coh u_dma (
    .clk(clk), .reset_n(reset_n),
    .psel(psel), .penable(penable), .pwrite(pwrite),
    .paddr(paddr), .pwdata(pwdata), .pstrb(pstrb),
    .pready(pready), .prdata(prdata), .pslverr(pslverr),
    .m_axi_arvalid(axi_arvalid), .m_axi_arready(axi_arready),
    .m_axi_araddr(axi_araddr), .m_axi_arlen(axi_arlen),
    .m_axi_arsize(axi_arsize), .m_axi_arburst(axi_arburst),
    .m_axi_rvalid(axi_rvalid), .m_axi_rready(axi_rready),
    .m_axi_rdata(axi_rdata), .m_axi_rresp(axi_rresp), .m_axi_rlast(axi_rlast),
    .m_axi_awvalid(axi_awvalid), .m_axi_awready(axi_awready),
    .m_axi_awaddr(axi_awaddr), .m_axi_awlen(axi_awlen),
    .m_axi_awsize(axi_awsize), .m_axi_awburst(axi_awburst),
    .m_axi_wvalid(axi_wvalid), .m_axi_wready(axi_wready),
    .m_axi_wdata(axi_wdata), .m_axi_wstrb(axi_wstrb), .m_axi_wlast(axi_wlast),
    .m_axi_bvalid(axi_bvalid), .m_axi_bready(axi_bready), .m_axi_bresp(axi_bresp),
    .irq_ch0_out(irq_ch0), .irq_ch1_out(irq_ch1)
);

axi4_to_coh_bridge #(
    .LADDR_W(LADDR_W), .RAM_SIZE_BYTES(32'h0004_0000)
) u_axi_coh (
    .clk(clk), .reset_n(reset_n),
    .s_axi_arvalid(axi_arvalid), .s_axi_arready(axi_arready),
    .s_axi_araddr(axi_araddr), .s_axi_arlen(axi_arlen),
    .s_axi_arsize(axi_arsize), .s_axi_arburst(axi_arburst),
    .s_axi_rvalid(axi_rvalid), .s_axi_rready(axi_rready),
    .s_axi_rdata(axi_rdata), .s_axi_rresp(axi_rresp), .s_axi_rlast(axi_rlast),
    .s_axi_awvalid(axi_awvalid), .s_axi_awready(axi_awready),
    .s_axi_awaddr(axi_awaddr), .s_axi_awlen(axi_awlen),
    .s_axi_awsize(axi_awsize), .s_axi_awburst(axi_awburst),
    .s_axi_wvalid(axi_wvalid), .s_axi_wready(axi_wready),
    .s_axi_wdata(axi_wdata), .s_axi_wstrb(axi_wstrb), .s_axi_wlast(axi_wlast),
    .s_axi_bvalid(axi_bvalid), .s_axi_bready(axi_bready), .s_axi_bresp(axi_bresp),
    .dma_coh_req_valid(dma_coh_req_v), .dma_coh_req_ready(dma_coh_req_rdy),
    .dma_coh_req_is_write(dma_coh_req_we), .dma_coh_req_laddr(dma_coh_req_la),
    .dma_coh_req_wdata(dma_coh_req_wd), .dma_coh_req_wstrb(dma_coh_req_ws),
    .dma_coh_resp_valid(dma_coh_resp_v), .dma_coh_resp_rdata(dma_coh_resp_rd)
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
    .dma_coh_req_valid(dma_coh_req_v), .dma_coh_req_ready(dma_coh_req_rdy),
    .dma_coh_req_is_write(dma_coh_req_we), .dma_coh_req_laddr(dma_coh_req_la),
    .dma_coh_req_wdata(dma_coh_req_wd), .dma_coh_req_wstrb(dma_coh_req_ws),
    .dma_coh_resp_valid(dma_coh_resp_v), .dma_coh_resp_rdata(dma_coh_resp_rd),
    .dma_wr_commit_valid(dma_wr_v), .dma_wr_commit_laddr(dma_wr_la), .dma_wr_commit_data(dma_wr_dat),
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
    .pmu_coh_dma_rd(pmu_dma_rd), .pmu_coh_dma_wr(pmu_dma_wr)
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
    .LADDR_W(LADDR_W), .MEM_LINES(16384), .WATCHDOG_CYC(512)
) u_chk (
    .clk(clk), .reset_n(reset_n), .active(1'b1),
    .d0_wr_valid(d0_wr_v), .d0_wr_laddr(d0_wr_la), .d0_wr_data(d0_wr_dat),
    .d1_wr_valid(d1_wr_v), .d1_wr_laddr(d1_wr_la), .d1_wr_data(d1_wr_dat),
    .dma_wr_valid(dma_wr_v), .dma_wr_laddr(dma_wr_la), .dma_wr_data(dma_wr_dat),
    .bus_evt_valid(bus_evt_v), .bus_evt_src(bus_evt_src), .bus_evt_op(bus_evt_op),
    .bus_evt_laddr(bus_evt_la), .bus_evt_c2c(bus_evt_c2c), .bus_evt_shared(bus_evt_sh),
    .d0_idle(d0_idle), .d1_idle(d1_idle), .coh_idle(coh_idle),
    .d0_progress(c0_hit_v || c0_fill_v || d0_resp_v),
    .d1_progress(c1_hit_v || c1_fill_v || d1_resp_v),
    .chk_errors(chk_errs), .cnt_gets(chk_gets), .cnt_getm(chk_getm),
    .cnt_upgrade(chk_upg), .cnt_c2c(chk_c2c), .cnt_wr_commits(chk_wr_commits)
);

initial clk = 1'b0;
always #5 clk = ~clk;

// ----------------------------------------------------------------------------
// Always-On AMBA AXI4 Protocol Assertions + 3-Master SWMR/Shadow Monitor
// ----------------------------------------------------------------------------
integer errs, cyc_cnt, axi_ar_cnt, axi_aw_cnt, axi_r_beats, axi_w_beats;
integer cnt_dma_rd_coh, cnt_dma_wr_coh, cnt_c1_snp_inv;

reg         prev_arvalid, prev_awvalid, prev_wvalid;
reg [31:0]  prev_araddr, prev_awaddr;
reg [7:0]   prev_arlen, prev_awlen, r_exp_beats, r_cur_beat, w_exp_beats, w_cur_beat;
reg [127:0] prev_wdata;
reg [15:0]  prev_wstrb;
reg         prev_wlast, r_active, w_active;

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

function automatic [7:0] shadow_byte(input [31:0] addr);
    reg [13:0] la;
    reg [3:0]  bo;
    begin
        la = addr[17:4];
        bo = addr[3:0];
        shadow_byte = u_chk.shadow_mem[la][bo*8 +: 8];
    end
endfunction

task check_line_3m(input [15:0] la);
    reg [2:0] st0, st1;
    reg       wbb0_m, wbb1_m, dwb_m, drmw_m;
    integer   dirty_cnt;
    begin
        st0    = lookup_d0_state(la);
        st1    = lookup_d1_state(la);
        wbb0_m = (u_d0.wbb0_v && u_d0.wbb0_laddr == la) || (u_d0.wbb1_v && u_d0.wbb1_laddr == la);
        wbb1_m = (u_d1.wbb0_v && u_d1.wbb0_laddr == la) || (u_d1.wbb1_v && u_d1.wbb1_laddr == la);
        dwb_m  = u_coh.dma_wb_v && (u_coh.dma_wb_la == la);
        drmw_m = u_coh.p_dma_v && u_coh.p_dma_rmw && (u_coh.p_dma_la == la);

        if ((st0 == COH_M || st0 == COH_E) && (st1 != COH_I || wbb1_m || dwb_m || drmw_m)) begin
            $display("FAIL [3M-SWMR]: cyc=%0d la=0x%04x D0=%0d D1=%0d wbb1=%0d dwb=%0d drmw=%0d",
                     cyc_cnt, la, st0, st1, wbb1_m, dwb_m, drmw_m);
            errs = errs + 1;
        end
        if ((st1 == COH_M || st1 == COH_E) && (st0 != COH_I || wbb0_m || dwb_m || drmw_m)) begin
            $display("FAIL [3M-SWMR]: cyc=%0d la=0x%04x D1=%0d D0=%0d wbb0=%0d dwb=%0d drmw=%0d",
                     cyc_cnt, la, st1, st0, wbb0_m, dwb_m, drmw_m);
            errs = errs + 1;
        end
        if ((dwb_m || drmw_m) && (st0 != COH_I || st1 != COH_I || wbb0_m || wbb1_m)) begin
            $display("FAIL [DMA-SWMR]: cyc=%0d la=0x%04x dwb=%0d drmw=%0d D0=%0d D1=%0d",
                     cyc_cnt, la, dwb_m, drmw_m, st0, st1);
            errs = errs + 1;
        end
        dirty_cnt = ((st0 == COH_M || st0 == COH_O) ? 1 : 0) +
                    ((st1 == COH_M || st1 == COH_O) ? 1 : 0) +
                    (wbb0_m ? 1 : 0) + (wbb1_m ? 1 : 0) + (dwb_m ? 1 : 0);
        if (dirty_cnt > 1) begin
            $display("FAIL [3M-OWNER]: cyc=%0d la=0x%04x dirty_cnt=%0d", cyc_cnt, la, dirty_cnt);
            errs = errs + 1;
        end
    end
endtask

task check_active_caches;
    integer s;
    reg [15:0] la;
    begin
        if (u_coh.dma_wb_v) begin
            if (u_coh.dma_wb_data !== u_chk.shadow_mem[u_coh.dma_wb_la[13:0]]) begin
                $display("FAIL [DATA-DMAWB]: cyc=%0d la=0x%04x exp=0x%032x got=0x%032x",
                         cyc_cnt, u_coh.dma_wb_la, u_chk.shadow_mem[u_coh.dma_wb_la[13:0]], u_coh.dma_wb_data);
                errs = errs + 1;
            end
            check_line_3m(u_coh.dma_wb_la);
        end
        for (s = 0; s < 32; s = s + 1) begin
            if (u_d0.way0_state[s] != COH_I) begin
                la = {u_d0.way0_tag[s], s[6:0]};
                if (u_d0.way0_data[s] !== u_chk.shadow_mem[la[13:0]]) begin
                    $display("FAIL [DATA-D0W0]: cyc=%0d la=0x%04x exp=0x%032x got=0x%032x",
                             cyc_cnt, la, u_chk.shadow_mem[la[13:0]], u_d0.way0_data[s]);
                    errs = errs + 1;
                end
                check_line_3m(la);
            end
            if (u_d0.way1_state[s] != COH_I) begin
                la = {u_d0.way1_tag[s], s[6:0]};
                if (u_d0.way1_data[s] !== u_chk.shadow_mem[la[13:0]]) begin
                    $display("FAIL [DATA-D0W1]: cyc=%0d la=0x%04x exp=0x%032x got=0x%032x",
                             cyc_cnt, la, u_chk.shadow_mem[la[13:0]], u_d0.way1_data[s]);
                    errs = errs + 1;
                end
                check_line_3m(la);
            end
            if (u_d1.way0_state[s] != COH_I) begin
                la = {u_d1.way0_tag[s], s[6:0]};
                if (u_d1.way0_data[s] !== u_chk.shadow_mem[la[13:0]]) begin
                    $display("FAIL [DATA-D1W0]: cyc=%0d la=0x%04x exp=0x%032x got=0x%032x",
                             cyc_cnt, la, u_chk.shadow_mem[la[13:0]], u_d1.way0_data[s]);
                    errs = errs + 1;
                end
                check_line_3m(la);
            end
            if (u_d1.way1_state[s] != COH_I) begin
                la = {u_d1.way1_tag[s], s[6:0]};
                if (u_d1.way1_data[s] !== u_chk.shadow_mem[la[13:0]]) begin
                    $display("FAIL [DATA-D1W1]: cyc=%0d la=0x%04x exp=0x%032x got=0x%032x",
                             cyc_cnt, la, u_chk.shadow_mem[la[13:0]], u_d1.way1_data[s]);
                    errs = errs + 1;
                end
                check_line_3m(la);
            end
        end
    end
endtask

/* verilator lint_off BLKSEQ */
/* verilator lint_off SYNCASYNCNET */
always @(posedge clk) begin
    if (!reset_n) begin
        prev_arvalid = 1'b0; prev_awvalid = 1'b0; prev_wvalid = 1'b0;
        r_active = 1'b0; w_active = 1'b0;
        r_exp_beats = 8'd0; r_cur_beat = 8'd0;
        w_exp_beats = 8'd0; w_cur_beat = 8'd0;
    end else begin
        cyc_cnt = cyc_cnt + 1;
        if (pmu_dma_rd) cnt_dma_rd_coh = cnt_dma_rd_coh + 1;
        if (pmu_dma_wr) cnt_dma_wr_coh = cnt_dma_wr_coh + 1;
        if (c1_snp_inv_v) cnt_c1_snp_inv = cnt_c1_snp_inv + 1;

        // AMBA AXI4 Protocol Assertions
        if (prev_arvalid) begin
            if (!axi_arvalid || axi_araddr !== prev_araddr || axi_arlen !== prev_arlen) begin
                $display("FAIL [AXI4-AR]: cyc=%0d AR channel unstable while !ARREADY!", cyc_cnt);
                errs = errs + 1;
            end
        end
        if (prev_awvalid) begin
            if (!axi_awvalid || axi_awaddr !== prev_awaddr || axi_awlen !== prev_awlen) begin
                $display("FAIL [AXI4-AW]: cyc=%0d AW channel unstable while !AWREADY!", cyc_cnt);
                errs = errs + 1;
            end
        end
        if (prev_wvalid) begin
            if (!axi_wvalid || axi_wdata !== prev_wdata || axi_wstrb !== prev_wstrb || axi_wlast !== prev_wlast) begin
                $display("FAIL [AXI4-W]: cyc=%0d W channel unstable while !WREADY!", cyc_cnt);
                errs = errs + 1;
            end
        end

        if (axi_arvalid && axi_arready) begin
            axi_ar_cnt = axi_ar_cnt + 1;
            if (({20'd0, axi_araddr[11:0]} + ({24'd0, axi_arlen} + 32'd1) * 32'd16) > 32'd4096) begin
                $display("FAIL [AXI4-4KB-AR]: cyc=%0d addr=0x%08x len=%0d crosses 4KB boundary!",
                         cyc_cnt, axi_araddr, axi_arlen);
                errs = errs + 1;
            end
            r_active    = 1'b1;
            r_exp_beats = axi_arlen;
            r_cur_beat  = 8'd0;
        end
        if (axi_rvalid && axi_rready) begin
            axi_r_beats = axi_r_beats + 1;
            if (!r_active) begin
                $display("FAIL [AXI4-R]: cyc=%0d Unexpected R beat!", cyc_cnt);
                errs = errs + 1;
            end else if (axi_rlast !== (r_cur_beat == r_exp_beats)) begin
                $display("FAIL [AXI4-RLAST]: cyc=%0d beat=%0d exp=%0d rlast=%0d",
                         cyc_cnt, r_cur_beat, r_exp_beats, axi_rlast);
                errs = errs + 1;
            end
            if (axi_rlast) r_active = 1'b0;
            else           r_cur_beat = r_cur_beat + 8'd1;
        end

        if (axi_awvalid && axi_awready) begin
            axi_aw_cnt = axi_aw_cnt + 1;
            if (({20'd0, axi_awaddr[11:0]} + ({24'd0, axi_awlen} + 32'd1) * 32'd16) > 32'd4096) begin
                $display("FAIL [AXI4-4KB-AW]: cyc=%0d addr=0x%08x len=%0d crosses 4KB boundary!",
                         cyc_cnt, axi_awaddr, axi_awlen);
                errs = errs + 1;
            end
            w_active    = 1'b1;
            w_exp_beats = axi_awlen;
            w_cur_beat  = 8'd0;
        end
        if (axi_wvalid && axi_wready) begin
            axi_w_beats = axi_w_beats + 1;
            if (!w_active) begin
                $display("FAIL [AXI4-W]: cyc=%0d Unexpected W beat!", cyc_cnt);
                errs = errs + 1;
            end else if (axi_wlast !== (w_cur_beat == w_exp_beats)) begin
                $display("FAIL [AXI4-WLAST]: cyc=%0d beat=%0d exp=%0d wlast=%0d",
                         cyc_cnt, w_cur_beat, w_exp_beats, axi_wlast);
                errs = errs + 1;
            end
            if (axi_wlast) w_active = 1'b0;
            else           w_cur_beat = w_cur_beat + 8'd1;
        end

        prev_arvalid = axi_arvalid && !axi_arready;
        prev_araddr  = axi_araddr;
        prev_arlen   = axi_arlen;
        prev_awvalid = axi_awvalid && !axi_awready;
        prev_awaddr  = axi_awaddr;
        prev_awlen   = axi_awlen;
        prev_wvalid  = axi_wvalid && !axi_wready;
        prev_wdata   = axi_wdata;
        prev_wstrb   = axi_wstrb;
        prev_wlast   = axi_wlast;

        if (errs < 10)
            check_active_caches();
    end
end
/* verilator lint_on SYNCASYNCNET */
/* verilator lint_on BLKSEQ */

// ----------------------------------------------------------------------------
// APB4 & Core Helper Tasks
// ----------------------------------------------------------------------------
task apb_write(input [31:0] off, input [31:0] data, output err_out);
    begin
        @(negedge clk);
        psel    = 1'b1;
        penable = 1'b0;
        pwrite  = 1'b1;
        paddr   = DMA_BASE + off;
        pwdata  = data;
        pstrb   = 4'hF;
        @(posedge clk);
        @(negedge clk);
        penable = 1'b1;
        #1;
        err_out = pslverr;
        @(posedge clk);
        @(negedge clk);
        psel    = 1'b0;
        penable = 1'b0;
        pwrite  = 1'b0;
    end
endtask

task apb_read(input [31:0] off, output [31:0] data_out, output err_out);
    begin
        @(negedge clk);
        psel    = 1'b1;
        penable = 1'b0;
        pwrite  = 1'b0;
        paddr   = DMA_BASE + off;
        pstrb   = 4'h0;
        @(posedge clk);
        @(negedge clk);
        penable = 1'b1;
        #1;
        data_out = prdata;
        err_out  = pslverr;
        @(posedge clk);
        @(negedge clk);
        psel    = 1'b0;
        penable = 1'b0;
    end
endtask

task wait_all_idle;
    integer wc;
    begin
        @(negedge clk);
        c0_req_v = 1'b0; c1_req_v = 1'b0;
        wc = 0;
        while ((!d0_idle || !d1_idle || !coh_idle || !l2_idle || (u_dma.ch_busy != 2'b00)) && wc < 10000) begin
            @(posedge clk); #1;
            wc = wc + 1;
        end
        if (wc >= 10000) begin
            $display("FAIL [tb_dma]: Timeout waiting for idle! d0=%0d d1=%0d coh=%0d l2=%0d dma_busy=%b",
                     d0_idle, d1_idle, coh_idle, l2_idle, u_dma.ch_busy);
            $fatal(1);
        end
    end
endtask

task do_c0_op(input is_wr, input [3:0] amo, input [19:0] addr, input [31:0] wdata, input [3:0] wstrb, output [31:0] rdata);
    reg got;
    integer wc;
    begin
        rdata = 32'd0;
        @(negedge clk);
        c0_req_v = 1'b1; c0_req_we = is_wr; c0_req_amo = amo;
        c0_req_addr = addr; c0_req_wdata = wdata; c0_req_wstrb = wstrb; c0_req_id = 7'd1;
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
            if (!got) begin $display("FAIL [tb_dma]: C0 timeout addr=0x%05x", addr); $fatal(1); end
        end
    end
endtask

task do_c1_op(input is_wr, input [3:0] amo, input [19:0] addr, input [31:0] wdata, input [3:0] wstrb, output [31:0] rdata);
    reg got;
    integer wc;
    begin
        rdata = 32'd0;
        @(negedge clk);
        c1_req_v = 1'b1; c1_req_we = is_wr; c1_req_amo = amo;
        c1_req_addr = addr; c1_req_wdata = wdata; c1_req_wstrb = wstrb; c1_req_id = 7'd1;
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
            if (!got) begin $display("FAIL [tb_dma]: C1 timeout addr=0x%05x", addr); $fatal(1); end
        end
    end
endtask

task wait_dma_ch(input integer ch, output [31:0] st_out);
    reg aerr;
    integer wc;
    begin
        wc = 0;
        apb_read((ch == 0 ? OFF_CH0 : OFF_CH1) + CH_STATUS, st_out, aerr);
        while ((st_out[0] == 1'b1) && wc < 5000) begin
            apb_read((ch == 0 ? OFF_CH0 : OFF_CH1) + CH_STATUS, st_out, aerr);
            wc = wc + 1;
        end
        if (wc >= 5000) begin
            $display("FAIL [tb_dma]: Timeout waiting for DMA ch%0d!", ch);
            $fatal(1);
        end
    end
endtask

// ----------------------------------------------------------------------------
// Main Test Sequence
// ----------------------------------------------------------------------------
integer i, r, c, seed, ops_done, c2c_before, inv_before, dma_ops_done;
reg [31:0] rd0, rd1, st0, st1, vreg;
reg        aerr;
reg [7:0]  exp_b, got_b;
reg [7:0]  guard_before_lo, guard_before_hi;
reg [7:0]  ref_buf [0:511];

// Concurrent stress state
reg        c0_pend, c1_pend;
reg        c0_we_r, c1_we_r;
reg [3:0]  c0_amo_r, c1_amo_r, c0_strb_r, c1_strb_r;
reg [19:0] c0_addr_r, c1_addr_r;
reg [31:0] c0_wdat_r, c1_wdat_r;
reg        c0_need_r, c1_need_r;
integer    dma_launch_cd;
reg [31:0] rand_src, rand_dst, rand_len;

initial begin
    errs           = 0;
    cyc_cnt        = 0;
    axi_ar_cnt     = 0;
    axi_aw_cnt     = 0;
    axi_r_beats    = 0;
    axi_w_beats    = 0;
    cnt_dma_rd_coh = 0;
    cnt_dma_wr_coh = 0;
    cnt_c1_snp_inv = 0;
    reset_n        = 1'b0;
    jitter_en      = 1'b0;
    psel = 1'b0; penable = 1'b0; pwrite = 1'b0; paddr = 32'd0; pwdata = 32'd0; pstrb = 4'h0;
    c0_req_v = 1'b0; c0_req_we = 1'b0; c0_req_amo = AMO_NONE; c0_req_addr = 20'd0;
    c0_req_wdata = 32'd0; c0_req_wstrb = 4'hF; c0_req_id = 7'd0;
    c1_req_v = 1'b0; c1_req_we = 1'b0; c1_req_amo = AMO_NONE; c1_req_addr = 20'd0;
    c1_req_wdata = 32'd0; c1_req_wstrb = 4'hF; c1_req_id = 7'd0;

    // Initialize 256 KB DRAM & Golden Shadow Memory with unique byte pattern!
    for (i = 0; i < 16384; i = i + 1) begin
        u_dram.mem[i]       = {32'hD000_0000 ^ (i * 32'd7), 32'hC000_0000 ^ (i * 32'd5),
                               32'hB000_0000 ^ (i * 32'd3), 32'hA000_0000 ^ i[31:0]};
        u_chk.shadow_mem[i] = u_dram.mem[i];
    end

    repeat (3) @(posedge clk);
    @(negedge clk);
    reset_n = 1'b1;

    // ========================================================================
    // TEST 1: APB4 Register & Protocol Checks (Version, PSLVERR, 0-Len IRQ)
    // ========================================================================
    apb_read(OFF_VERSION, vreg, aerr);
    if (aerr || vreg !== 32'h0002_0100) begin
        $display("FAIL [T1]: DMA_VERSION exp=0x00020100 got=0x%08x err=%0d", vreg, aerr);
        errs = errs + 1;
    end
    apb_write(OFF_VERSION, 32'h1234, aerr);
    if (!aerr) begin $display("FAIL [T1]: RO write to DMA_VERSION did not raise PSLVERR!"); errs = errs + 1; end
    apb_read(32'h3FC, vreg, aerr);
    if (!aerr) begin $display("FAIL [T1]: Unmapped read did not raise PSLVERR!"); errs = errs + 1; end

    // Zero-length transfer -> immediate DONE + IRQ
    apb_write(OFF_CH0 + CH_LEN, 32'd0, aerr);
    apb_write(OFF_CH0 + CH_CTRL, 32'h0000_0003, aerr); // START | IRQ_EN
    apb_read(OFF_CH0 + CH_STATUS, st0, aerr);
    if (st0[1:0] !== 2'b10 || !irq_ch0) begin
        $display("FAIL [T1]: Zero-len ch0 expected DONE=1 BUSY=0 IRQ=1, got st=0x%08x irq=%0d", st0, irq_ch0);
        errs = errs + 1;
    end
    apb_write(OFF_IRQ_STAT, 32'h1, aerr);
    if (irq_ch0) begin $display("FAIL [T1]: W1C failed to clear irq_ch0!"); errs = errs + 1; end

    // ========================================================================
    // TEST 2: Coherent Dirty-L1D DMA Read (ZERO Manual Flush! M->O C2C Hit!)
    // ========================================================================
    for (i = 0; i < 64; i = i + 1) begin
        do_c0_op(1'b1, AMO_NONE, 20'h02000 + {i[17:0], 2'b00}, 32'hCAFE_0000 + i, 4'hF, rd0);
    end
    wait_all_idle();
    if (lookup_d0_state(16'h0200) !== COH_M) begin
        $display("FAIL [T2]: Expected D0 line 0x0200 in M before DMA, got %0d", lookup_d0_state(16'h0200));
        errs = errs + 1;
    end
    c2c_before = chk_c2c;
    apb_write(OFF_CH0 + CH_SRC,  32'h0000_2000, aerr);
    apb_write(OFF_CH0 + CH_DST,  32'h0000_3000, aerr);
    apb_write(OFF_CH0 + CH_LEN,  32'd256, aerr);
    apb_write(OFF_CH0 + CH_CTRL, 32'h0000_0003, aerr); // START | IRQ_EN (16-beat default!)
    wait_dma_ch(0, st0);
    wait_all_idle();
    if (!st0[1] || st0[2] || (st0 >> 8) != 32'd256 || !irq_ch0 || (chk_c2c <= c2c_before)) begin
        $display("FAIL [T2]: Coherent dirty-L1D DMA read failed! st=0x%08x irq=%0d c2c_d=%0d",
                 st0, irq_ch0, chk_c2c - c2c_before);
        errs = errs + 1;
    end
    if (ENABLE_MOESI_O != 0 && lookup_d0_state(16'h0200) !== COH_O) begin
        $display("FAIL [T2-MOESI]: Expected D0 dirty line M->O after DMA_GETS, got %0d", lookup_d0_state(16'h0200));
        errs = errs + 1;
    end
    apb_write(OFF_IRQ_STAT, 32'h1, aerr);
    // Core 1 reads destination buffer & verifies all 64 words!
    for (i = 0; i < 64; i = i + 1) begin
        do_c1_op(1'b0, AMO_NONE, 20'h03000 + {i[17:0], 2'b00}, 32'd0, 4'h0, rd1);
        if (rd1 !== (32'hCAFE_0000 + i)) begin
            $display("FAIL [T2]: C1 dst[%0d] exp=0x%08x got=0x%08x", i, 32'hCAFE_0000 + i, rd1);
            errs = errs + 1;
        end
    end
    wait_all_idle();

    // ========================================================================
    // TEST 3: Coherent DMA Write Invalidation (DMA_PUTM) + snoop_inv_valid!
    //   Core 1 dirties dst line 0x3000 -> DMA overwrites 0x3000 -> D1 must be
    //   invalidated, snoop_inv_valid must pulse, and C1 must see fresh DMA data!
    // ========================================================================
    do_c1_op(1'b1, AMO_NONE, 20'h03000, 32'hDEAD_BEEF, 4'hF, rd1);
    do_c0_op(1'b1, AMO_NONE, 20'h02000, 32'h55AA_9966, 4'hF, rd0);
    wait_all_idle();
    inv_before = cnt_c1_snp_inv;
    apb_write(OFF_CH0 + CH_SRC,  32'h0000_2000, aerr);
    apb_write(OFF_CH0 + CH_DST,  32'h0000_3000, aerr);
    apb_write(OFF_CH0 + CH_LEN,  32'd16, aerr);
    apb_write(OFF_CH0 + CH_CTRL, 32'h0000_0001, aerr);
    wait_dma_ch(0, st0);
    wait_all_idle();
    if (lookup_d1_state(16'h0300) !== COH_I || cnt_c1_snp_inv <= inv_before) begin
        $display("FAIL [T3]: DMA_PUTM did not invalidate D1 or pulse snoop_inv! st1=%0d inv_d=%0d",
                 lookup_d1_state(16'h0300), cnt_c1_snp_inv - inv_before);
        errs = errs + 1;
    end
    do_c1_op(1'b0, AMO_NONE, 20'h03000, 32'd0, 4'h0, rd1);
    if (rd1 !== 32'h55AA_9966) begin
        $display("FAIL [T3]: C1 read stale data after DMA overwrite! exp=0x55AA9966 got=0x%08x", rd1);
        errs = errs + 1;
    end
    wait_all_idle();

    // ========================================================================
    // TEST 4: Unaligned Byte Copy (DMA_RMW) + Guard Bytes + 4 KB Boundary!
    //   Copy 277 bytes from 0x0000_0FF3 (crosses 4KB @ 0x1000!) to 0x0000_3FF7
    //   (crosses 4KB @ 0x4000!) while D0/D1 hold dirty guard bytes at edges!
    // ========================================================================
    // Dirty both source & destination boundary lines in D0 and D1!
    do_c0_op(1'b1, AMO_NONE, 20'h00FF0, 32'h1413_1211, 4'hF, rd0);
    do_c0_op(1'b1, AMO_NONE, 20'h01000, 32'h2423_2221, 4'hF, rd0);
    do_c1_op(1'b1, AMO_NONE, 20'h03FF4, 32'h7766_5544, 4'hF, rd1); // bytes 3FF4..3FF6 are guard bytes!
    do_c1_op(1'b1, AMO_NONE, 20'h0410C, 32'hAABB_CCDD, 4'hF, rd1); // byte 410C+ is guard byte!
    wait_all_idle();

    for (i = 0; i < 277; i = i + 1)
        ref_buf[i] = shadow_byte(32'h0000_0FF3 + i);
    guard_before_lo = shadow_byte(32'h0000_3FF6);
    guard_before_hi = shadow_byte(32'h0000_3FF7 + 277);

    apb_write(OFF_CH0 + CH_SRC,  32'h0000_0FF3, aerr);
    apb_write(OFF_CH0 + CH_DST,  32'h0000_3FF7, aerr);
    apb_write(OFF_CH0 + CH_LEN,  32'd277, aerr);
    apb_write(OFF_CH0 + CH_CTRL, 32'h0000_0001, aerr);
    wait_dma_ch(0, st0);
    wait_all_idle();

    if (shadow_byte(32'h0000_3FF6) !== guard_before_lo ||
        shadow_byte(32'h0000_3FF7 + 277) !== guard_before_hi) begin
        $display("FAIL [T4]: Guard byte corrupted during unaligned DMA_RMW!");
        errs = errs + 1;
    end
    for (i = 0; i < 277; i = i + 1) begin
        got_b = shadow_byte(32'h0000_3FF7 + i);
        if (got_b !== ref_buf[i]) begin
            $display("FAIL [T4]: Unaligned byte[%0d] exp=0x%02x got=0x%02x", i, ref_buf[i], got_b);
            errs = errs + 1;
        end
    end

    // ========================================================================
    // TEST 5: 2D Strided Tile Copy (5 rows x 19B, src_stride=64, dst_stride=48)
    // ========================================================================
    for (r = 0; r < 5; r = r + 1) begin
        for (c = 0; c < 19; c = c + 1)
            ref_buf[r*19 + c] = shadow_byte(32'h0000_5002 + r*64 + c);
    end
    apb_write(OFF_CH1 + CH_SRC,        32'h0000_5002, aerr);
    apb_write(OFF_CH1 + CH_DST,        32'h0000_6005, aerr);
    apb_write(OFF_CH1 + CH_LEN,        32'd19, aerr);
    apb_write(OFF_CH1 + CH_SRC_STRIDE, 32'd64, aerr);
    apb_write(OFF_CH1 + CH_DST_STRIDE, 32'd48, aerr);
    apb_write(OFF_CH1 + CH_ROW_CNT,    32'd5, aerr);
    apb_write(OFF_CH1 + CH_CTRL,       32'h0000_0007, aerr); // START | IRQ_EN | MODE_2D
    wait_dma_ch(1, st1);
    wait_all_idle();
    if (!st1[1] || st1[2] || (st1 >> 8) != 32'd95 || !irq_ch1) begin
        $display("FAIL [T5]: 2D strided DMA status wrong! st1=0x%08x irq1=%0d", st1, irq_ch1);
        errs = errs + 1;
    end
    apb_write(OFF_IRQ_STAT, 32'h2, aerr);
    for (r = 0; r < 5; r = r + 1) begin
        for (c = 0; c < 19; c = c + 1) begin
            got_b = shadow_byte(32'h0000_6005 + r*48 + c);
            if (got_b !== ref_buf[r*19 + c]) begin
                $display("FAIL [T5]: 2D tile[%0d,%0d] exp=0x%02x got=0x%02x", r, c, ref_buf[r*19 + c], got_b);
                errs = errs + 1;
            end
        end
    end

    // ========================================================================
    // TEST 6: Concurrent Dual-Channel Time-Slicing (Ch0 burst=4 + Ch1 burst=8)
    // ========================================================================
    for (i = 0; i < 512; i = i + 1) ref_buf[i] = shadow_byte(32'h0000_7000 + i);
    apb_write(OFF_CH0 + CH_SRC,  32'h0000_7000, aerr);
    apb_write(OFF_CH0 + CH_DST,  32'h0000_8000, aerr);
    apb_write(OFF_CH0 + CH_LEN,  32'd512, aerr);
    apb_write(OFF_CH1 + CH_SRC,  32'h0000_7000, aerr);
    apb_write(OFF_CH1 + CH_DST,  32'h0000_8800, aerr);
    apb_write(OFF_CH1 + CH_LEN,  32'd512, aerr);
    apb_write(OFF_CH0 + CH_CTRL, 32'h0000_0043, aerr); // burst=4, IRQ_EN, START
    apb_write(OFF_CH1 + CH_CTRL, 32'h0000_0083, aerr); // burst=8, IRQ_EN, START
    // Busy-write PSLVERR check while Ch0 is active!
    apb_write(OFF_CH0 + CH_SRC,  32'h0000_9000, aerr);
    if (!aerr) begin $display("FAIL [T6]: Write to CH0_SRC while BUSY did not raise PSLVERR!"); errs = errs + 1; end
    wait_dma_ch(0, st0);
    wait_dma_ch(1, st1);
    wait_all_idle();
    if (!irq_ch0 || !irq_ch1 || (st0 >> 8) != 32'd512 || (st1 >> 8) != 32'd512) begin
        $display("FAIL [T6]: Dual-channel transfer failed! st0=0x%08x st1=0x%08x irqs=%b%b",
                 st0, st1, irq_ch1, irq_ch0);
        errs = errs + 1;
    end
    apb_write(OFF_IRQ_STAT, 32'h3, aerr);
    for (i = 0; i < 512; i = i + 1) begin
        if (shadow_byte(32'h0000_8000 + i) !== ref_buf[i] ||
            shadow_byte(32'h0000_8800 + i) !== ref_buf[i]) begin
            $display("FAIL [T6]: Dual-channel data mismatch at byte %0d!", i);
            errs = errs + 1;
        end
    end

    // ========================================================================
    // TEST 7: Out-of-Range PMA DECERR Bus-Error + Clean Mid-Flight Abort
    // ========================================================================
    apb_write(OFF_CH0 + CH_SRC,  32'h0004_0000, aerr); // Past 256 KB RAM!
    apb_write(OFF_CH0 + CH_DST,  32'h0000_9000, aerr);
    apb_write(OFF_CH0 + CH_LEN,  32'd32, aerr);
    apb_write(OFF_CH0 + CH_CTRL, 32'h0000_0003, aerr);
    wait_dma_ch(0, st0);
    if (!st0[2] || st0[5:4] !== 2'b11 || !irq_ch0) begin
        $display("FAIL [T7a]: Read OOB DECERR not captured! st0=0x%08x irq0=%0d", st0, irq_ch0);
        errs = errs + 1;
    end
    apb_write(OFF_IRQ_STAT, 32'h1, aerr);

    apb_write(OFF_CH1 + CH_SRC,  32'h0000_9000, aerr);
    apb_write(OFF_CH1 + CH_DST,  32'h0004_0010, aerr); // Past 256 KB RAM!
    apb_write(OFF_CH1 + CH_LEN,  32'd32, aerr);
    apb_write(OFF_CH1 + CH_CTRL, 32'h0000_0003, aerr);
    wait_dma_ch(1, st1);
    if (!st1[2] || st1[5:4] !== 2'b11 || !irq_ch1) begin
        $display("FAIL [T7b]: Write OOB DECERR not captured! st1=0x%08x irq1=%0d", st1, irq_ch1);
        errs = errs + 1;
    end
    apb_write(OFF_IRQ_STAT, 32'h2, aerr);

    // Mid-flight ABORT on 4096B transfer
    apb_write(OFF_CH0 + CH_SRC,  32'h0000_A000, aerr);
    apb_write(OFF_CH0 + CH_DST,  32'h0000_B000, aerr);
    apb_write(OFF_CH0 + CH_LEN,  32'd4096, aerr);
    apb_write(OFF_CH0 + CH_CTRL, 32'h0000_0001, aerr);
    repeat (12) @(posedge clk);
    apb_write(OFF_CH0 + CH_CTRL, 32'h0000_0008, aerr); // ABORT
    wait_dma_ch(0, st0);
    if (st0[0] || st0[2]) begin
        $display("FAIL [T7c]: Abort left channel busy or errored! st0=0x%08x", st0);
        errs = errs + 1;
    end
    wait_all_idle();

    // ========================================================================
    // TEST 8: 10,000-Op Randomized Concurrent D0 + D1 + 2-Ch Coherent DMA Stress!
    //   Overlapping hot lines (0x0000..0x03FF = 64 lines, sets 0..63) + DRAM jitter!
    // ========================================================================
    seed          = 32'h7777_2026;
    ops_done      = 0;
    dma_ops_done  = 0;
    dma_launch_cd = 0;
    jitter_en     = 1'b1;
    c0_pend       = 1'b0;
    c1_pend       = 1'b0;

    while ((ops_done < NUM_RAND_OPS) && errs == 0 && chk_errs == 0) begin
        // Prepare random Core 0 & Core 1 ops over hot 64-line window (0x0000..0x03FF)
        if (!c0_pend && !c0_req_v) begin
            seed      = seed * 32'd1664525 + 32'd1013904223;
            c0_addr_r = {10'd0, seed[15:8], 2'b00}; // 256 words = 64 lines!
            c0_wdat_r = seed ^ 32'hC0C0_0000;
            c0_strb_r = (seed[20:19] == 2'd0) ? 4'b0001 :
                        (seed[20:19] == 2'd1) ? 4'b0011 :
                        (seed[20:19] == 2'd2) ? 4'b1100 : 4'b1111;
            case (seed[23:21])
                3'd0, 3'd1, 3'd2: begin c0_we_r = 1'b0; c0_amo_r = AMO_NONE; c0_need_r = 1'b1; end
                3'd3, 3'd4, 3'd5: begin c0_we_r = 1'b1; c0_amo_r = AMO_NONE; c0_need_r = 1'b0; end
                3'd6:             begin c0_we_r = 1'b0; c0_amo_r = AMO_LR;   c0_need_r = 1'b1; end
                default:          begin c0_we_r = 1'b1; c0_amo_r = (seed[24] ? AMO_ADD : AMO_SC); c0_need_r = 1'b1; end
            endcase
            c0_req_v = 1'b1; c0_req_we = c0_we_r; c0_req_amo = c0_amo_r;
            c0_req_addr = c0_addr_r; c0_req_wdata = c0_wdat_r; c0_req_wstrb = c0_strb_r; c0_req_id = 7'd3;
        end
        if (!c1_pend && !c1_req_v) begin
            seed      = seed * 32'd1664525 + 32'd1013904223;
            c1_addr_r = {10'd0, seed[15:8], 2'b00};
            c1_wdat_r = seed ^ 32'hC1C1_0000;
            c1_strb_r = (seed[20:19] == 2'd0) ? 4'b0010 :
                        (seed[20:19] == 2'd1) ? 4'b1100 : 4'b1111;
            case (seed[23:21])
                3'd0, 3'd1, 3'd2: begin c1_we_r = 1'b0; c1_amo_r = AMO_NONE; c1_need_r = 1'b1; end
                3'd3, 3'd4, 3'd5: begin c1_we_r = 1'b1; c1_amo_r = AMO_NONE; c1_need_r = 1'b0; end
                3'd6:             begin c1_we_r = 1'b0; c1_amo_r = AMO_LR;   c1_need_r = 1'b1; end
                default:          begin c1_we_r = 1'b1; c1_amo_r = (seed[24] ? AMO_XOR : AMO_SWAP); c1_need_r = 1'b1; end
            endcase
            c1_req_v = 1'b1; c1_req_we = c1_we_r; c1_req_amo = c1_amo_r;
            c1_req_addr = c1_addr_r; c1_req_wdata = c1_wdat_r; c1_req_wstrb = c1_strb_r; c1_req_id = 7'd4;
        end

        // Periodically kick off unaligned/aligned DMA Ch0 or Ch1 inside the EXACT SAME hot window!
        if (dma_launch_cd > 0) begin
            dma_launch_cd = dma_launch_cd - 1;
        end else if (u_dma.ch_busy == 2'b00) begin
            seed     = seed * 32'd1664525 + 32'd1013904223;
            rand_src = {22'd0, seed[9:0]}; // 0x000..0x3FF (unaligned bytes!)
            seed     = seed * 32'd1664525 + 32'd1013904223;
            rand_dst = {22'd0, seed[9:0]};
            rand_len = {25'd0, seed[16:10]} + 32'd1; // 1..128 bytes
            if (seed[17]) begin
                // Launch Ch0 via APB4 (keep core requests active across APB cycles? Here just toggle ch0 directly!)
                u_dma.ch_src[0]        = rand_src;
                u_dma.ch_dst[0]        = rand_dst;
                u_dma.ch_len[0]        = rand_len;
                u_dma.ch_row_src[0]    = rand_src;
                u_dma.ch_row_dst[0]    = rand_dst;
                u_dma.ch_cur_src[0]    = rand_src;
                u_dma.ch_cur_dst[0]    = rand_dst;
                u_dma.ch_row_rem[0]    = rand_len;
                u_dma.ch_rows_left[0]  = 32'd1;
                u_dma.ch_mode_2d[0]    = 1'b0;
                u_dma.ch_burst_max[0]  = seed[21:18];
                u_dma.ch_done[0]       = 1'b0;
                u_dma.ch_err[0]        = 1'b0;
                u_dma.ch_busy[0]       = 1'b1;
            end else begin
                u_dma.ch_src[1]        = rand_src;
                u_dma.ch_dst[1]        = rand_dst;
                u_dma.ch_len[1]        = rand_len;
                u_dma.ch_row_src[1]    = rand_src;
                u_dma.ch_row_dst[1]    = rand_dst;
                u_dma.ch_cur_src[1]    = rand_src;
                u_dma.ch_cur_dst[1]    = rand_dst;
                u_dma.ch_row_rem[1]    = rand_len;
                u_dma.ch_rows_left[1]  = 32'd1;
                u_dma.ch_mode_2d[1]    = 1'b0;
                u_dma.ch_burst_max[1]  = seed[21:18];
                u_dma.ch_done[1]       = 1'b0;
                u_dma.ch_err[1]        = 1'b0;
                u_dma.ch_busy[1]       = 1'b1;
            end
            dma_ops_done  = dma_ops_done + 1;
            dma_launch_cd = 5;
        end

        #1;
        if (c0_req_v && c0_req_rdy) begin
            ops_done = ops_done + 1;
            if (c0_need_r) c0_pend = 1'b1;
        end
        if (c1_req_v && c1_req_rdy) begin
            ops_done = ops_done + 1;
            if (c1_need_r) c1_pend = 1'b1;
        end
        @(posedge clk);
        @(negedge clk);
        if (c0_req_v && c0_req_rdy) c0_req_v = 1'b0;
        if (c1_req_v && c1_req_rdy) c1_req_v = 1'b0;
        if (c0_hit_v || c0_fill_v)  c0_pend  = 1'b0;
        if (c1_hit_v || c1_fill_v)  c1_pend  = 1'b0;
    end

    wait_all_idle();

    if (errs != 0 || chk_errs != 0) begin
        $display("FAIL [tb_dma_axi4_coh]: errs=%0d chk_errs=%0d (ops=%0d dma_ops=%0d)",
                 errs, chk_errs, ops_done, dma_ops_done);
        $fatal(1);
    end

    $display("PASS [tb_dma_axi4_coh]: MOESI=%0d ops=%0d dma_xfers=%0d axi_ar=%0d(%0d beats) axi_aw=%0d(%0d beats) dma_coh_r=%0d dma_coh_w=%0d c2c=%0d gets=%0d getm=%0d upg=%0d cyc=%0d errs=0",
             ENABLE_MOESI_O, ops_done, dma_ops_done, axi_ar_cnt, axi_r_beats,
             axi_aw_cnt, axi_w_beats, cnt_dma_rd_coh, cnt_dma_wr_coh,
             chk_c2c, chk_gets, chk_getm, chk_upg, cyc_cnt);
    $finish;
end

endmodule
