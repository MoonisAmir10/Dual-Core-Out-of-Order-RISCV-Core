`timescale 1ns / 1ps

// ============================================================================
// 3-Channel Split-Transaction 3-Master MOESI Snooping Crossbar Interconnect
// (Phase 3: Dual-Core MOESI -> Phase U7: +3rd I/O-Coherent DMA Master Port!)
//
// Connects Dual Coherent L1Ds (D0, D1) + Dual L1Is (I0, I1) + 3rd Coherent
// Master (I/O-Coherent AXI4 DMA) to Dual-Master 2-Bank Shared L2 Cache:
//   - Channel A (Ordered Request / Broadcast Snoop):
//       * Fair 3-way round-robin arbiter across {D0, D1, DMA} (zero starvation!)
//       * Exact address-hazard table (4 L2-pending + 2 C2C-pending + MESI-WB +
//         DMA L2-pending + DMA WB) so non-colliding lines pipeline at full MLP
//         while SAME-LINE races serialize cleanly!
//   - Channel B / C (Non-Blocking Snoop Probe, C2C Dirty Handoff & Dirty WBBs):
//       * Registered snoop probe boundary -> ZERO cross-core comb loops!
//       * Core requests snoop peer L1D; DMA requests broadcast-snoop BOTH L1Ds!
//       * DMA_GETS (read): pulls dirty C2C from D0/D1/WBB (M->O in MOESI, or
//         M->S + MESI WB in MESI), else fetches from Shared L2 (L2ID_DMA=3'd2).
//       * DMA_PUTM (full-line write, wstrb==16'hFFFF): invalidates BOTH L1Ds &
//         WBBs (triggering automatic LQ snoop-replay on both cores!), writes
//         128-bit line cleanly to Shared L2, then returns write ACK.
//       * DMA_RMW (partial-line write, 0 < wstrb < 16'hFFFF): invalidates BOTH
//         L1Ds & WBBs, merges wstrb onto dirty C2C line (or fetched L2 line if
//         clean/absent), writes merged 128-bit line to Shared L2, returns ACK.
//   - Channel D (Lossless Non-Blocking Data / Upgrade-ACK Return):
//       * Priority L2 response + lossless 1-entry C2C/Upgrade holding buffer
//   - Global Channel A Serialization Bus Trace + Hardware Coherence PMU Pulses
// ============================================================================
module coh_interconnect #(
    parameter integer LADDR_W        = 16, // 16-bit 16B line address (1 MB window)
    parameter integer M_ID_W         = 3,  // L2 per-master ID width (matches l2_cache_shared)
    parameter integer ENABLE_MOESI_O = 1   // 1: MOESI (0 WB on M->O), 0: MESI (WB on M->S)
) (
    input  wire                clk,
    input  wire                reset_n,

    // -------------------- Core 0 Coherent L1D (D0) --------------------
    input  wire                d0_coh_req_valid,
    output wire                d0_coh_req_ready,
    input  wire [1:0]          d0_coh_req_op,
    input  wire [LADDR_W-1:0]  d0_coh_req_laddr,
    input  wire                d0_coh_req_id,

    output wire                d0_snoop_valid,
    output wire [1:0]          d0_snoop_op,
    output wire [LADDR_W-1:0]  d0_snoop_laddr,
    input  wire [1:0]          d0_snoop_resp_op,
    input  wire [127:0]        d0_snoop_resp_data,

    input  wire                d0_wb_valid,
    output wire                d0_wb_ready,
    input  wire [LADDR_W-1:0]  d0_wb_laddr,
    input  wire [127:0]        d0_wb_data,

    output wire                d0_coh_resp_valid,
    output wire                d0_coh_resp_id,
    output wire [127:0]        d0_coh_resp_data,
    output wire                d0_coh_resp_shared,
    output wire                d0_coh_resp_from_c2c,
    output wire                d0_coh_resp_upgrade_ack,

    // -------------------- Core 1 Coherent L1D (D1) --------------------
    input  wire                d1_coh_req_valid,
    output wire                d1_coh_req_ready,
    input  wire [1:0]          d1_coh_req_op,
    input  wire [LADDR_W-1:0]  d1_coh_req_laddr,
    input  wire                d1_coh_req_id,

    output wire                d1_snoop_valid,
    output wire [1:0]          d1_snoop_op,
    output wire [LADDR_W-1:0]  d1_snoop_laddr,
    input  wire [1:0]          d1_snoop_resp_op,
    input  wire [127:0]        d1_snoop_resp_data,

    input  wire                d1_wb_valid,
    output wire                d1_wb_ready,
    input  wire [LADDR_W-1:0]  d1_wb_laddr,
    input  wire [127:0]        d1_wb_data,

    output wire                d1_coh_resp_valid,
    output wire                d1_coh_resp_id,
    output wire [127:0]        d1_coh_resp_data,
    output wire                d1_coh_resp_shared,
    output wire                d1_coh_resp_from_c2c,
    output wire                d1_coh_resp_upgrade_ack,

    // -------------------- Master 2: I/O-Coherent DMA Port (Phase U7) --------------------
    input  wire                dma_coh_req_valid,
    output wire                dma_coh_req_ready,
    input  wire                dma_coh_req_is_write, // 0: DMA_GETS, 1: DMA_PUTM/DMA_RMW
    input  wire [LADDR_W-1:0]  dma_coh_req_laddr,
    input  wire [127:0]        dma_coh_req_wdata,
    input  wire [15:0]         dma_coh_req_wstrb,

    output reg                 dma_coh_resp_valid,
    output reg  [127:0]        dma_coh_resp_rdata,

    // Exact DMA Write Linearization Hook (for coh_checker & formal shadow memory)
    output wire                dma_wr_commit_valid,
    output wire [LADDR_W-1:0]  dma_wr_commit_laddr,
    output wire [127:0]        dma_wr_commit_data,

    // -------------------- Core 0 & Core 1 L1I (Read-Only) --------------------
    input  wire                i0_rd_valid,
    output wire                i0_rd_ready,
    input  wire [LADDR_W-1:0]  i0_rd_laddr,
    output wire                i0_rresp_valid,
    output wire [127:0]        i0_rresp_data,

    input  wire                i1_rd_valid,
    output wire                i1_rd_ready,
    input  wire [LADDR_W-1:0]  i1_rd_laddr,
    output wire                i1_rresp_valid,
    output wire [127:0]        i1_rresp_data,

    // -------------------- Shared 2-Bank L2 Master 0 (D0 + DMA + MESI WB) --------------------
    output reg                 l2_m0_req_valid,
    input  wire                l2_m0_req_ready,
    output reg                 l2_m0_req_is_write,
    output reg  [LADDR_W-1:0]  l2_m0_req_laddr,
    output reg  [127:0]        l2_m0_req_wdata,
    output reg  [M_ID_W-1:0]   l2_m0_req_id,

    input  wire                l2_m0_resp_valid,
    input  wire [M_ID_W-1:0]   l2_m0_resp_id,
    input  wire [127:0]        l2_m0_resp_data,

    // -------------------- Shared 2-Bank L2 Master 1 (D1 + I0 + I1) --------------------
    output reg                 l2_m1_req_valid,
    input  wire                l2_m1_req_ready,
    output reg                 l2_m1_req_is_write,
    output reg  [LADDR_W-1:0]  l2_m1_req_laddr,
    output reg  [127:0]        l2_m1_req_wdata,
    output reg  [M_ID_W-1:0]   l2_m1_req_id,

    input  wire                l2_m1_resp_valid,
    input  wire [M_ID_W-1:0]   l2_m1_resp_id,
    input  wire [127:0]        l2_m1_resp_data,

    // -------------------- Global Bus Trace, Status & Coherence PMU --------------------
    output wire                bus_evt_valid,
    output wire [1:0]          bus_evt_src,      // 0: Core 0, 1: Core 1, 2: Coherent DMA
    output wire [1:0]          bus_evt_op,       // GETS / GETM / UPGRADE
    output wire [LADDR_W-1:0]  bus_evt_laddr,
    output wire                bus_evt_c2c,
    output wire                bus_evt_shared,

    output wire                coh_idle,
    output wire                pmu_coh_gets,
    output wire                pmu_coh_getm,
    output wire                pmu_coh_upgrade,
    output wire                pmu_coh_c2c_dirty,
    output wire                pmu_coh_inval,
    output wire                pmu_coh_mesi_wb,
    output wire                pmu_coh_dma_rd,
    output wire                pmu_coh_dma_wr
`ifdef FORMAL
    ,output wire               f_c2c0_v,
    output wire                f_c2c0_from_c2c,
    output wire                f_c2c0_shared,
    output wire [LADDR_W-1:0]  f_c2c0_laddr,
    output wire [7:0]          f_c2c0_b0,
    output wire                f_c2c1_v,
    output wire                f_c2c1_from_c2c,
    output wire                f_c2c1_shared,
    output wire [LADDR_W-1:0]  f_c2c1_laddr,
    output wire [7:0]          f_c2c1_b0,
    output wire                f_p_dma_v,
    output wire                f_p_dma_rmw,
    output wire [LADDR_W-1:0]  f_p_dma_laddr,
    output wire                f_dma_wb_v,
    output wire [LADDR_W-1:0]  f_dma_wb_laddr,
    output wire [7:0]          f_dma_wb_b0
`endif
);

`include "coh_defs.vh"

// L2 Master ID encodings (M_ID_W = 3):
//   M0: 3'd0 -> D0 MSHR0, 3'd1 -> D0 MSHR1, 3'd2 -> Coherent DMA
//   M1: 3'd0 -> D1 MSHR0, 3'd1 -> D1 MSHR1, 3'd2 -> I0, 3'd3 -> I1
localparam [M_ID_W-1:0] L2ID_MSHR0 = 3'd0;
localparam [M_ID_W-1:0] L2ID_MSHR1 = 3'd1;
localparam [M_ID_W-1:0] L2ID_DMA   = 3'd2;
localparam [M_ID_W-1:0] L2ID_I0    = 3'd2;
localparam [M_ID_W-1:0] L2ID_I1    = 3'd3;

localparam [1:0] SRC_D0  = 2'd0;
localparam [1:0] SRC_D1  = 2'd1;
localparam [1:0] SRC_DMA = 2'd2;

// Byte-masked 128-bit line merge helper for DMA_RMW partial-line coherent writes
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

// ----------------------------------------------------------------------------
// 1. State Registers: Snoop Stage, 4 Core L2-Pending, 2 C2C, MESI WB & DMA
// ----------------------------------------------------------------------------
reg               snp_busy;
reg [1:0]         snp_src;       // 0: D0, 1: D1, 2: DMA
reg [1:0]         snp_op;
reg [LADDR_W-1:0] snp_laddr;
reg               snp_id;
reg               snp_dma_we;
reg [127:0]       snp_dma_wdata;
reg [15:0]        snp_dma_wstrb;

// Pending L2 reads per core/MSHR (0=D0.M0, 1=D0.M1, 2=D1.M0, 3=D1.M1)
reg [3:0]         p_v;
reg [3:0]         p_sent;
reg [3:0]         p_shared;
reg [LADDR_W-1:0] p0_la, p1_la, p2_la, p3_la;

// Pending L2 read for Coherent DMA (DMA_GETS clean miss OR DMA_RMW clean read)
reg               p_dma_v;
reg               p_dma_sent;
reg               p_dma_rmw;
reg [LADDR_W-1:0] p_dma_la;
reg [127:0]       p_dma_wdata;
reg [15:0]        p_dma_wstrb;

// 1-Entry Coherent DMA Writeback Buffer (sole dirty owner until L2 M0 accepts!)
reg               dma_wb_v;
reg [LADDR_W-1:0] dma_wb_la;
reg [127:0]       dma_wb_data;

// Lossless 1-entry C2C / Upgrade-ACK holding buffers per core
reg               c2c0_v, c2c0_id, c2c0_shared, c2c0_from_c2c, c2c0_upg_ack;
reg [LADDR_W-1:0] c2c0_laddr;
reg [127:0]       c2c0_data;

reg               c2c1_v, c2c1_id, c2c1_shared, c2c1_from_c2c, c2c1_upg_ack;
reg [LADDR_W-1:0] c2c1_laddr;
reg [127:0]       c2c1_data;

// 1-Entry MESI Shadow Writeback Buffer (used ONLY when ENABLE_MOESI_O == 0 on M->S!)
reg               mwb_v;
reg [LADDR_W-1:0] mwb_laddr;
reg [127:0]       mwb_data;

// Fair Round-Robin Arbiters
reg [1:0]         rr_chan_a; // 0: prefer D0, 1: prefer D1, 2: prefer DMA
reg               rr_i;      // 0: prefer I0, 1: prefer I1

// ----------------------------------------------------------------------------
// 2. Exact Address-Hazard Detection & Fair 3-Way Channel A Arbitration
// ----------------------------------------------------------------------------
wire d0_addr_busy = (p_v[0] && (p0_la == d0_coh_req_laddr)) ||
                    (p_v[1] && (p1_la == d0_coh_req_laddr)) ||
                    (p_v[2] && (p2_la == d0_coh_req_laddr)) ||
                    (p_v[3] && (p3_la == d0_coh_req_laddr)) ||
                    (c2c0_v && (c2c0_laddr == d0_coh_req_laddr)) ||
                    (c2c1_v && (c2c1_laddr == d0_coh_req_laddr)) ||
                    (mwb_v  && (mwb_laddr  == d0_coh_req_laddr)) ||
                    (p_dma_v  && (p_dma_la  == d0_coh_req_laddr)) ||
                    (dma_wb_v && (dma_wb_la == d0_coh_req_laddr));

wire d1_addr_busy = (p_v[0] && (p0_la == d1_coh_req_laddr)) ||
                    (p_v[1] && (p1_la == d1_coh_req_laddr)) ||
                    (p_v[2] && (p2_la == d1_coh_req_laddr)) ||
                    (p_v[3] && (p3_la == d1_coh_req_laddr)) ||
                    (c2c0_v && (c2c0_laddr == d1_coh_req_laddr)) ||
                    (c2c1_v && (c2c1_laddr == d1_coh_req_laddr)) ||
                    (mwb_v  && (mwb_laddr  == d1_coh_req_laddr)) ||
                    (p_dma_v  && (p_dma_la  == d1_coh_req_laddr)) ||
                    (dma_wb_v && (dma_wb_la == d1_coh_req_laddr));

wire dma_addr_busy = (p_v[0] && (p0_la == dma_coh_req_laddr)) ||
                     (p_v[1] && (p1_la == dma_coh_req_laddr)) ||
                     (p_v[2] && (p2_la == dma_coh_req_laddr)) ||
                     (p_v[3] && (p3_la == dma_coh_req_laddr)) ||
                     (c2c0_v && (c2c0_laddr == dma_coh_req_laddr)) ||
                     (c2c1_v && (c2c1_laddr == dma_coh_req_laddr)) ||
                     (mwb_v  && (mwb_laddr  == dma_coh_req_laddr)) ||
                     (p_dma_v  && (p_dma_la  == dma_coh_req_laddr)) ||
                     (dma_wb_v && (dma_wb_la == dma_coh_req_laddr));

wire d0_slot_busy = d0_coh_req_id ? p_v[1] : p_v[0];
wire d1_slot_busy = d1_coh_req_id ? p_v[3] : p_v[2];

wire d0_elig  = d0_coh_req_valid && !snp_busy && !c2c0_v && !c2c1_v && !mwb_v &&
                !d0_slot_busy && !d0_addr_busy;
wire d1_elig  = d1_coh_req_valid && !snp_busy && !c2c0_v && !c2c1_v && !mwb_v &&
                !d1_slot_busy && !d1_addr_busy;
wire dma_elig = dma_coh_req_valid && !snp_busy && !c2c0_v && !c2c1_v && !mwb_v &&
                !p_dma_v && !dma_wb_v && !dma_addr_busy;

reg a_pick_d0, a_pick_d1, a_pick_dma;
always @(*) begin
    a_pick_d0  = 1'b0;
    a_pick_d1  = 1'b0;
    a_pick_dma = 1'b0;
    case (rr_chan_a)
        2'd0: begin
            if (d0_elig)       a_pick_d0  = 1'b1;
            else if (d1_elig)  a_pick_d1  = 1'b1;
            else if (dma_elig) a_pick_dma = 1'b1;
        end
        2'd1: begin
            if (d1_elig)       a_pick_d1  = 1'b1;
            else if (dma_elig) a_pick_dma = 1'b1;
            else if (d0_elig)  a_pick_d0  = 1'b1;
        end
        default: begin // 2'd2
            if (dma_elig)      a_pick_dma = 1'b1;
            else if (d0_elig)  a_pick_d0  = 1'b1;
            else if (d1_elig)  a_pick_d1  = 1'b1;
        end
    endcase
end

assign d0_coh_req_ready  = a_pick_d0;
assign d1_coh_req_ready  = a_pick_d1;
assign dma_coh_req_ready = a_pick_dma;

// ----------------------------------------------------------------------------
// 3. Channel B Snoop Broadcast (Purely Registered -> 0 Cross-Core Comb Loops!)
// ----------------------------------------------------------------------------
wire snp_is_d0  = (snp_src == SRC_D0);
wire snp_is_d1  = (snp_src == SRC_D1);
wire snp_is_dma = (snp_src == SRC_DMA);

// Core 0 snoops D1; Core 1 snoops D0; Coherent DMA broadcast-snoops BOTH D0 & D1!
assign d0_snoop_valid = snp_busy && (snp_is_d1 || snp_is_dma);
assign d0_snoop_op    = snp_op;
assign d0_snoop_laddr = snp_laddr;

assign d1_snoop_valid = snp_busy && (snp_is_d0 || snp_is_dma);
assign d1_snoop_op    = snp_op;
assign d1_snoop_laddr = snp_laddr;

wire [1:0]   peer_resp_op   = snp_is_d1 ? d0_snoop_resp_op   : d1_snoop_resp_op;
wire [127:0] peer_resp_data = snp_is_d1 ? d0_snoop_resp_data : d1_snoop_resp_data;

// For DMA snoops, SWMR guarantees at most ONE of D0/D1 returns SNP_RESP_DATA
wire         dma_d0_c2c     = (d0_snoop_resp_op == SNP_RESP_DATA);
wire         dma_d1_c2c     = (d1_snoop_resp_op == SNP_RESP_DATA);
wire         dma_any_c2c    = dma_d0_c2c || dma_d1_c2c;
wire [127:0] dma_c2c_data   = dma_d0_c2c ? d0_snoop_resp_data : d1_snoop_resp_data;
wire         dma_any_hit    = (d0_snoop_resp_op != SNP_RESP_MISS) ||
                              (d1_snoop_resp_op != SNP_RESP_MISS);

wire snp_is_upg      = (snp_op == COH_REQ_UPGRADE);
wire snp_is_gets     = (snp_op == COH_REQ_GETS);
wire snp_core_c2c    = snp_busy && !snp_is_dma && !snp_is_upg && (peer_resp_op == SNP_RESP_DATA);
wire snp_core_l2     = snp_busy && !snp_is_dma && !snp_is_upg && (peer_resp_op != SNP_RESP_DATA);

wire snp_dma_full_wr = (snp_dma_wstrb == 16'hFFFF);
wire snp_dma_zero_wr = (snp_dma_wstrb == 16'h0000);
wire snp_dma_c2c_rd  = snp_busy && snp_is_dma && !snp_dma_we && dma_any_c2c;
wire snp_dma_l2_rd   = snp_busy && snp_is_dma && !snp_dma_we && !dma_any_c2c;
wire snp_dma_wr_now  = snp_busy && snp_is_dma &&  snp_dma_we && !snp_dma_zero_wr &&
                       (snp_dma_full_wr || dma_any_c2c);
wire snp_dma_rmw_l2  = snp_busy && snp_is_dma &&  snp_dma_we && !snp_dma_zero_wr &&
                       !snp_dma_full_wr && !dma_any_c2c;
wire snp_dma_wr_noop = snp_busy && snp_is_dma &&  snp_dma_we &&  snp_dma_zero_wr;

wire [127:0] snp_dma_wr_data = snp_dma_full_wr
                             ? snp_dma_wdata
                             : merge_line_bytes(dma_c2c_data, snp_dma_wdata, snp_dma_wstrb);

wire snp_need_mwb = (ENABLE_MOESI_O == 0) &&
                    ((snp_core_c2c && snp_is_gets) || snp_dma_c2c_rd);

// ----------------------------------------------------------------------------
// 4. Channel D Lossless Return Mux (L2 Response Priority 1, C2C/Upgrade Prio 2)
// ----------------------------------------------------------------------------
wire l2_d0_resp  = l2_m0_resp_valid && (l2_m0_resp_id[M_ID_W-1:1] == {(M_ID_W-1){1'b0}});
wire l2_dma_resp = l2_m0_resp_valid && (l2_m0_resp_id == L2ID_DMA);
wire l2_d1_resp  = l2_m1_resp_valid && (l2_m1_resp_id[M_ID_W-1:1] == {(M_ID_W-1){1'b0}});
wire d0_l2_mid   = l2_m0_resp_id[0];
wire d1_l2_mid   = l2_m1_resp_id[0];

assign d0_coh_resp_valid       = l2_d0_resp || c2c0_v;
assign d0_coh_resp_id          = l2_d0_resp ? d0_l2_mid : c2c0_id;
assign d0_coh_resp_data        = l2_d0_resp ? l2_m0_resp_data : c2c0_data;
assign d0_coh_resp_shared      = l2_d0_resp ? (d0_l2_mid ? p_shared[1] : p_shared[0]) : c2c0_shared;
assign d0_coh_resp_from_c2c    = l2_d0_resp ? 1'b0 : c2c0_from_c2c;
assign d0_coh_resp_upgrade_ack = l2_d0_resp ? 1'b0 : c2c0_upg_ack;

assign d1_coh_resp_valid       = l2_d1_resp || c2c1_v;
assign d1_coh_resp_id          = l2_d1_resp ? d1_l2_mid : c2c1_id;
assign d1_coh_resp_data        = l2_d1_resp ? l2_m1_resp_data : c2c1_data;
assign d1_coh_resp_shared      = l2_d1_resp ? (d1_l2_mid ? p_shared[3] : p_shared[2]) : c2c1_shared;
assign d1_coh_resp_from_c2c    = l2_d1_resp ? 1'b0 : c2c1_from_c2c;
assign d1_coh_resp_upgrade_ack = l2_d1_resp ? 1'b0 : c2c1_upg_ack;

// DMA RMW completion upon L2 read return (mutually exclusive with snp_dma_wr_now!)
wire         dma_rmw_wr_now  = l2_dma_resp && p_dma_v && p_dma_rmw;
wire [127:0] dma_rmw_wr_data = merge_line_bytes(l2_m0_resp_data, p_dma_wdata, p_dma_wstrb);

assign dma_wr_commit_valid = snp_dma_wr_now || dma_rmw_wr_now;
assign dma_wr_commit_laddr = dma_rmw_wr_now ? p_dma_la        : snp_laddr;
assign dma_wr_commit_data  = dma_rmw_wr_now ? dma_rmw_wr_data : snp_dma_wr_data;

// L1I responses from L2 Master 1
assign i0_rresp_valid = l2_m1_resp_valid && (l2_m1_resp_id == L2ID_I0);
assign i0_rresp_data  = l2_m1_resp_data;
assign i1_rresp_valid = l2_m1_resp_valid && (l2_m1_resp_id == L2ID_I1);
assign i1_rresp_data  = l2_m1_resp_data;

// ----------------------------------------------------------------------------
// 5. Downstream L2 Master 0 & Master 1 Request Arbitration
//    M0 Priority: MESI-WB > DMA-WB > D0 WBB > D0 MSHR0 Rd > D0 MSHR1 Rd > DMA Rd
//    M1 Priority: D1 WBB  > D1 MSHR0 Rd > D1 MSHR1 Rd > I0/I1 (RR, hazard-safe)
// ----------------------------------------------------------------------------
reg m0_sel_mwb, m0_sel_dmawb, m0_sel_d0wb, m0_sel_p0, m0_sel_p1, m0_sel_pdma;

always @(*) begin
    l2_m0_req_valid    = 1'b0;
    l2_m0_req_is_write = 1'b0;
    l2_m0_req_laddr    = {LADDR_W{1'b0}};
    l2_m0_req_wdata    = 128'd0;
    l2_m0_req_id       = L2ID_MSHR0;
    m0_sel_mwb         = 1'b0;
    m0_sel_dmawb       = 1'b0;
    m0_sel_d0wb        = 1'b0;
    m0_sel_p0          = 1'b0;
    m0_sel_p1          = 1'b0;
    m0_sel_pdma        = 1'b0;

    if (mwb_v) begin
        l2_m0_req_valid    = 1'b1;
        l2_m0_req_is_write = 1'b1;
        l2_m0_req_laddr    = mwb_laddr;
        l2_m0_req_wdata    = mwb_data;
        m0_sel_mwb         = 1'b1;
    end else if (dma_wb_v) begin
        l2_m0_req_valid    = 1'b1;
        l2_m0_req_is_write = 1'b1;
        l2_m0_req_laddr    = dma_wb_la;
        l2_m0_req_wdata    = dma_wb_data;
        m0_sel_dmawb       = 1'b1;
    end else if (d0_wb_valid) begin
        l2_m0_req_valid    = 1'b1;
        l2_m0_req_is_write = 1'b1;
        l2_m0_req_laddr    = d0_wb_laddr;
        l2_m0_req_wdata    = d0_wb_data;
        m0_sel_d0wb        = 1'b1;
    end else if (p_v[0] && !p_sent[0]) begin
        l2_m0_req_valid    = 1'b1;
        l2_m0_req_is_write = 1'b0;
        l2_m0_req_laddr    = p0_la;
        l2_m0_req_id       = L2ID_MSHR0;
        m0_sel_p0          = 1'b1;
    end else if (p_v[1] && !p_sent[1]) begin
        l2_m0_req_valid    = 1'b1;
        l2_m0_req_is_write = 1'b0;
        l2_m0_req_laddr    = p1_la;
        l2_m0_req_id       = L2ID_MSHR1;
        m0_sel_p1          = 1'b1;
    end else if (p_dma_v && !p_dma_sent) begin
        l2_m0_req_valid    = 1'b1;
        l2_m0_req_is_write = 1'b0;
        l2_m0_req_laddr    = p_dma_la;
        l2_m0_req_id       = L2ID_DMA;
        m0_sel_pdma        = 1'b1;
    end
end

assign d0_wb_ready = m0_sel_d0wb && l2_m0_req_ready;

// Gate L1I reads against any in-flight DMA write/RMW targeting the exact same line
wire i0_dma_haz = (dma_wb_v && (dma_wb_la == i0_rd_laddr)) ||
                  (p_dma_v && p_dma_rmw && (p_dma_la == i0_rd_laddr));
wire i1_dma_haz = (dma_wb_v && (dma_wb_la == i1_rd_laddr)) ||
                  (p_dma_v && p_dma_rmw && (p_dma_la == i1_rd_laddr));
wire i0_rd_elig = i0_rd_valid && !i0_dma_haz;
wire i1_rd_elig = i1_rd_valid && !i1_dma_haz;

reg m1_sel_d1wb, m1_sel_p2, m1_sel_p3, m1_sel_i0, m1_sel_i1;
wire i_pick_i1 = i1_rd_elig && (!i0_rd_elig || rr_i);

always @(*) begin
    l2_m1_req_valid    = 1'b0;
    l2_m1_req_is_write = 1'b0;
    l2_m1_req_laddr    = {LADDR_W{1'b0}};
    l2_m1_req_wdata    = 128'd0;
    l2_m1_req_id       = L2ID_MSHR0;
    m1_sel_d1wb        = 1'b0;
    m1_sel_p2          = 1'b0;
    m1_sel_p3          = 1'b0;
    m1_sel_i0          = 1'b0;
    m1_sel_i1          = 1'b0;

    if (d1_wb_valid) begin
        l2_m1_req_valid    = 1'b1;
        l2_m1_req_is_write = 1'b1;
        l2_m1_req_laddr    = d1_wb_laddr;
        l2_m1_req_wdata    = d1_wb_data;
        m1_sel_d1wb        = 1'b1;
    end else if (p_v[2] && !p_sent[2]) begin
        l2_m1_req_valid    = 1'b1;
        l2_m1_req_is_write = 1'b0;
        l2_m1_req_laddr    = p2_la;
        l2_m1_req_id       = L2ID_MSHR0;
        m1_sel_p2          = 1'b1;
    end else if (p_v[3] && !p_sent[3]) begin
        l2_m1_req_valid    = 1'b1;
        l2_m1_req_is_write = 1'b0;
        l2_m1_req_laddr    = p3_la;
        l2_m1_req_id       = L2ID_MSHR1;
        m1_sel_p3          = 1'b1;
    end else if (i0_rd_elig || i1_rd_elig) begin
        l2_m1_req_valid    = 1'b1;
        l2_m1_req_is_write = 1'b0;
        if (i_pick_i1) begin
            l2_m1_req_laddr = i1_rd_laddr;
            l2_m1_req_id    = L2ID_I1;
            m1_sel_i1       = 1'b1;
        end else begin
            l2_m1_req_laddr = i0_rd_laddr;
            l2_m1_req_id    = L2ID_I0;
            m1_sel_i0       = 1'b1;
        end
    end
end

assign d1_wb_ready = m1_sel_d1wb && l2_m1_req_ready;
assign i0_rd_ready = m1_sel_i0   && l2_m1_req_ready;
assign i1_rd_ready = m1_sel_i1   && l2_m1_req_ready;

// ----------------------------------------------------------------------------
// 6. Global Channel A Bus Trace, Status & Hardware PMU Telemetry
// ----------------------------------------------------------------------------
assign bus_evt_valid  = snp_busy;
assign bus_evt_src    = snp_src;
assign bus_evt_op     = snp_op;
assign bus_evt_laddr  = snp_laddr;
assign bus_evt_c2c    = snp_is_dma ? (snp_busy && dma_any_c2c) : snp_core_c2c;
assign bus_evt_shared = snp_is_dma ? dma_any_hit : (peer_resp_op != SNP_RESP_MISS);

assign coh_idle = !snp_busy && !p_v[0] && !p_v[1] && !p_v[2] && !p_v[3] &&
                  !c2c0_v && !c2c1_v && !mwb_v && !p_dma_v && !dma_wb_v &&
                  !dma_coh_resp_valid;

assign pmu_coh_gets      = snp_busy && (snp_op == COH_REQ_GETS);
assign pmu_coh_getm      = snp_busy && (snp_op == COH_REQ_GETM);
assign pmu_coh_upgrade   = snp_busy && (snp_op == COH_REQ_UPGRADE);
assign pmu_coh_c2c_dirty = snp_core_c2c || (snp_busy && snp_is_dma && dma_any_c2c);
assign pmu_coh_inval     = snp_busy && (snp_op == COH_REQ_GETM || snp_op == COH_REQ_UPGRADE) &&
                           (snp_is_dma ? dma_any_hit : (peer_resp_op != SNP_RESP_MISS));
assign pmu_coh_mesi_wb   = snp_need_mwb;
assign pmu_coh_dma_rd    = snp_busy && snp_is_dma && !snp_dma_we;
assign pmu_coh_dma_wr    = snp_busy && snp_is_dma &&  snp_dma_we;

// ----------------------------------------------------------------------------
// 7. Sequential State Updates
// ----------------------------------------------------------------------------
`ifdef FORMAL
initial begin
    snp_busy      = 1'b0;
    snp_src       = SRC_D0;
    snp_op        = COH_REQ_GETS;
    snp_laddr     = {LADDR_W{1'b0}};
    snp_id        = 1'b0;
    snp_dma_we    = 1'b0;
    snp_dma_wdata = 128'd0;
    snp_dma_wstrb = 16'd0;
    p_v           = 4'd0;
    p_sent        = 4'd0;
    p_shared      = 4'd0;
    p0_la         = {LADDR_W{1'b0}};
    p1_la         = {LADDR_W{1'b0}};
    p2_la         = {LADDR_W{1'b0}};
    p3_la         = {LADDR_W{1'b0}};
    p_dma_v       = 1'b0;
    p_dma_sent    = 1'b0;
    p_dma_rmw     = 1'b0;
    p_dma_la      = {LADDR_W{1'b0}};
    p_dma_wdata   = 128'd0;
    p_dma_wstrb   = 16'd0;
    dma_wb_v      = 1'b0;
    dma_wb_la     = {LADDR_W{1'b0}};
    dma_wb_data   = 128'd0;
    dma_coh_resp_valid = 1'b0;
    dma_coh_resp_rdata = 128'd0;
    c2c0_v = 1'b0; c2c0_id = 1'b0; c2c0_shared = 1'b0; c2c0_from_c2c = 1'b0; c2c0_upg_ack = 1'b0;
    c2c0_laddr = {LADDR_W{1'b0}}; c2c0_data = 128'd0;
    c2c1_v = 1'b0; c2c1_id = 1'b0; c2c1_shared = 1'b0; c2c1_from_c2c = 1'b0; c2c1_upg_ack = 1'b0;
    c2c1_laddr = {LADDR_W{1'b0}}; c2c1_data = 128'd0;
    mwb_v = 1'b0; mwb_laddr = {LADDR_W{1'b0}}; mwb_data = 128'd0;
    rr_chan_a = 2'd0;
    rr_i      = 1'b0;
end

always @(posedge clk) begin
`else
always @(posedge clk or negedge reset_n) begin
`endif
    if (!reset_n) begin
        snp_busy      <= 1'b0;
        snp_src       <= SRC_D0;
        snp_op        <= COH_REQ_GETS;
        snp_laddr     <= {LADDR_W{1'b0}};
        snp_id        <= 1'b0;
        snp_dma_we    <= 1'b0;
        snp_dma_wdata <= 128'd0;
        snp_dma_wstrb <= 16'd0;
        p_v           <= 4'd0;
        p_sent        <= 4'd0;
        p_shared      <= 4'd0;
        p0_la         <= {LADDR_W{1'b0}};
        p1_la         <= {LADDR_W{1'b0}};
        p2_la         <= {LADDR_W{1'b0}};
        p3_la         <= {LADDR_W{1'b0}};
        p_dma_v       <= 1'b0;
        p_dma_sent    <= 1'b0;
        p_dma_rmw     <= 1'b0;
        p_dma_la      <= {LADDR_W{1'b0}};
        p_dma_wdata   <= 128'd0;
        p_dma_wstrb   <= 16'd0;
        dma_wb_v      <= 1'b0;
        dma_wb_la     <= {LADDR_W{1'b0}};
        dma_wb_data   <= 128'd0;
        dma_coh_resp_valid <= 1'b0;
        dma_coh_resp_rdata <= 128'd0;
        c2c0_v <= 1'b0; c2c0_id <= 1'b0; c2c0_shared <= 1'b0; c2c0_from_c2c <= 1'b0; c2c0_upg_ack <= 1'b0;
        c2c0_laddr <= {LADDR_W{1'b0}}; c2c0_data <= 128'd0;
        c2c1_v <= 1'b0; c2c1_id <= 1'b0; c2c1_shared <= 1'b0; c2c1_from_c2c <= 1'b0; c2c1_upg_ack <= 1'b0;
        c2c1_laddr <= {LADDR_W{1'b0}}; c2c1_data <= 128'd0;
        mwb_v <= 1'b0; mwb_laddr <= {LADDR_W{1'b0}}; mwb_data <= 128'd0;
        rr_chan_a <= 2'd0;
        rr_i      <= 1'b0;
    end else begin
        dma_coh_resp_valid <= 1'b0;

        // 1. Drain C2C / Upgrade buffers when Channel D is not occupied by L2 response
        if (c2c0_v && !l2_d0_resp)
            c2c0_v <= 1'b0;
        if (c2c1_v && !l2_d1_resp)
            c2c1_v <= 1'b0;

        // 2. Drain MESI shadow WB & DMA WB buffers to L2 Master 0
        if (m0_sel_mwb && l2_m0_req_ready)
            mwb_v <= 1'b0;
        if (m0_sel_dmawb && l2_m0_req_ready) begin
            dma_wb_v           <= 1'b0;
            dma_coh_resp_valid <= 1'b1; // Write ACK strictly AFTER L2 accepts coherent write!
            dma_coh_resp_rdata <= 128'd0;
        end

        // 3. Mark L2 reads as sent & advance L1I RR pointer
        if (m0_sel_p0   && l2_m0_req_ready) p_sent[0]  <= 1'b1;
        if (m0_sel_p1   && l2_m0_req_ready) p_sent[1]  <= 1'b1;
        if (m0_sel_pdma && l2_m0_req_ready) p_dma_sent <= 1'b1;
        if (m1_sel_p2   && l2_m1_req_ready) p_sent[2]  <= 1'b1;
        if (m1_sel_p3   && l2_m1_req_ready) p_sent[3]  <= 1'b1;
        if ((m1_sel_i0 || m1_sel_i1) && l2_m1_req_ready)
            rr_i <= !m1_sel_i1;

        // 4. Clear L2-pending entries upon L2 response delivery on Channel D
        if (l2_d0_resp) begin
            if (!d0_l2_mid) begin p_v[0] <= 1'b0; p_sent[0] <= 1'b0; end
            else            begin p_v[1] <= 1'b0; p_sent[1] <= 1'b0; end
        end
        if (l2_d1_resp) begin
            if (!d1_l2_mid) begin p_v[2] <= 1'b0; p_sent[2] <= 1'b0; end
            else            begin p_v[3] <= 1'b0; p_sent[3] <= 1'b0; end
        end
        if (l2_dma_resp && p_dma_v) begin
            p_dma_v    <= 1'b0;
            p_dma_sent <= 1'b0;
            p_dma_rmw  <= 1'b0;
            if (p_dma_rmw) begin
                // Partial-line RMW read returned from L2 -> merge & push to DMA WB!
                dma_wb_v    <= 1'b1;
                dma_wb_la   <= p_dma_la;
                dma_wb_data <= dma_rmw_wr_data;
            end else begin
                // Clean DMA_GETS read returned from L2 -> return directly to DMA!
                dma_coh_resp_valid <= 1'b1;
                dma_coh_resp_rdata <= l2_m0_resp_data;
            end
        end

        // 5. Resolve Active Snoop Stage (S_SNOOP -> S_IDLE)
        if (snp_busy) begin
            snp_busy <= 1'b0;
            if (snp_is_dma) begin
                // --- Coherent DMA Snoop Resolution (Snooped BOTH D0 & D1) ---
                if (snp_dma_c2c_rd) begin
                    dma_coh_resp_valid <= 1'b1;
                    dma_coh_resp_rdata <= dma_c2c_data;
                    if (snp_need_mwb) begin
                        mwb_v     <= 1'b1;
                        mwb_laddr <= snp_laddr;
                        mwb_data  <= dma_c2c_data;
                    end
                end else if (snp_dma_l2_rd) begin
                    p_dma_v    <= 1'b1;
                    p_dma_sent <= 1'b0;
                    p_dma_rmw  <= 1'b0;
                    p_dma_la   <= snp_laddr;
                end else if (snp_dma_wr_now) begin
                    dma_wb_v    <= 1'b1;
                    dma_wb_la   <= snp_laddr;
                    dma_wb_data <= snp_dma_wr_data;
                end else if (snp_dma_rmw_l2) begin
                    p_dma_v     <= 1'b1;
                    p_dma_sent  <= 1'b0;
                    p_dma_rmw   <= 1'b1;
                    p_dma_la    <= snp_laddr;
                    p_dma_wdata <= snp_dma_wdata;
                    p_dma_wstrb <= snp_dma_wstrb;
                end else if (snp_dma_wr_noop) begin
                    dma_coh_resp_valid <= 1'b1;
                    dma_coh_resp_rdata <= 128'd0;
                end
            end else if (snp_is_upg) begin
                // Upgrade ACK -> 0 L2 read, 0 data transfer!
                if (snp_is_d0) begin
                    c2c0_v        <= 1'b1;
                    c2c0_id       <= snp_id;
                    c2c0_laddr    <= snp_laddr;
                    c2c0_data     <= 128'd0;
                    c2c0_shared   <= 1'b0;
                    c2c0_from_c2c <= 1'b0;
                    c2c0_upg_ack  <= 1'b1;
                end else begin
                    c2c1_v        <= 1'b1;
                    c2c1_id       <= snp_id;
                    c2c1_laddr    <= snp_laddr;
                    c2c1_data     <= 128'd0;
                    c2c1_shared   <= 1'b0;
                    c2c1_from_c2c <= 1'b0;
                    c2c1_upg_ack  <= 1'b1;
                end
            end else if (snp_core_c2c) begin
                // Dirty Cache-to-Cache Transfer directly from peer M/O/WBB!
                if (snp_is_d0) begin
                    c2c0_v        <= 1'b1;
                    c2c0_id       <= snp_id;
                    c2c0_laddr    <= snp_laddr;
                    c2c0_data     <= peer_resp_data;
                    c2c0_shared   <= snp_is_gets;
                    c2c0_from_c2c <= 1'b1;
                    c2c0_upg_ack  <= 1'b0;
                end else begin
                    c2c1_v        <= 1'b1;
                    c2c1_id       <= snp_id;
                    c2c1_laddr    <= snp_laddr;
                    c2c1_data     <= peer_resp_data;
                    c2c1_shared   <= snp_is_gets;
                    c2c1_from_c2c <= 1'b1;
                    c2c1_upg_ack  <= 1'b0;
                end
                // In MESI ablation mode (ENABLE_MOESI_O=0), M->S must write back to L2!
                if (snp_need_mwb) begin
                    mwb_v     <= 1'b1;
                    mwb_laddr <= snp_laddr;
                    mwb_data  <= peer_resp_data;
                end
            end else if (snp_core_l2) begin
                // Fetch clean line from Shared 2-Bank L2!
                if (snp_is_d0) begin
                    if (!snp_id) begin
                        p_v[0]      <= 1'b1;
                        p_sent[0]   <= 1'b0;
                        p_shared[0] <= (peer_resp_op == SNP_RESP_SHARED);
                        p0_la       <= snp_laddr;
                    end else begin
                        p_v[1]      <= 1'b1;
                        p_sent[1]   <= 1'b0;
                        p_shared[1] <= (peer_resp_op == SNP_RESP_SHARED);
                        p1_la       <= snp_laddr;
                    end
                end else begin
                    if (!snp_id) begin
                        p_v[2]      <= 1'b1;
                        p_sent[2]   <= 1'b0;
                        p_shared[2] <= (peer_resp_op == SNP_RESP_SHARED);
                        p2_la       <= snp_laddr;
                    end else begin
                        p_v[3]      <= 1'b1;
                        p_sent[3]   <= 1'b0;
                        p_shared[3] <= (peer_resp_op == SNP_RESP_SHARED);
                        p3_la       <= snp_laddr;
                    end
                end
            end
        end else if (a_pick_d0) begin
            // 6a. Latch Channel A Grant for D0 (S_IDLE -> S_SNOOP)
            snp_busy  <= 1'b1;
            snp_src   <= SRC_D0;
            snp_op    <= d0_coh_req_op;
            snp_laddr <= d0_coh_req_laddr;
            snp_id    <= d0_coh_req_id;
            rr_chan_a <= 2'd1;
        end else if (a_pick_d1) begin
            // 6b. Latch Channel A Grant for D1 (S_IDLE -> S_SNOOP)
            snp_busy  <= 1'b1;
            snp_src   <= SRC_D1;
            snp_op    <= d1_coh_req_op;
            snp_laddr <= d1_coh_req_laddr;
            snp_id    <= d1_coh_req_id;
            rr_chan_a <= 2'd2;
        end else if (a_pick_dma) begin
            // 6c. Latch Channel A Grant for Coherent DMA (S_IDLE -> S_SNOOP)
            snp_busy      <= 1'b1;
            snp_src       <= SRC_DMA;
            snp_op        <= dma_coh_req_is_write ? COH_REQ_GETM : COH_REQ_GETS;
            snp_laddr     <= dma_coh_req_laddr;
            snp_id        <= 1'b0;
            snp_dma_we    <= dma_coh_req_is_write;
            snp_dma_wdata <= dma_coh_req_wdata;
            snp_dma_wstrb <= dma_coh_req_wstrb;
            rr_chan_a     <= 2'd0;
        end
    end
end

`ifdef FORMAL
assign f_c2c0_v        = c2c0_v;
assign f_c2c0_from_c2c = c2c0_from_c2c;
assign f_c2c0_shared   = c2c0_shared;
assign f_c2c0_laddr    = c2c0_laddr;
assign f_c2c0_b0       = c2c0_data[7:0];
assign f_c2c1_v        = c2c1_v;
assign f_c2c1_from_c2c = c2c1_from_c2c;
assign f_c2c1_shared   = c2c1_shared;
assign f_c2c1_laddr    = c2c1_laddr;
assign f_c2c1_b0       = c2c1_data[7:0];
assign f_p_dma_v       = p_dma_v;
assign f_p_dma_rmw     = p_dma_rmw;
assign f_p_dma_laddr   = p_dma_la;
assign f_dma_wb_v      = dma_wb_v;
assign f_dma_wb_laddr  = dma_wb_la;
assign f_dma_wb_b0     = dma_wb_data[7:0];
`endif

endmodule
