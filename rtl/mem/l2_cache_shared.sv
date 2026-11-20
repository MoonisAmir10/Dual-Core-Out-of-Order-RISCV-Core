`timescale 1ns / 1ps

// ============================================================================
// Shared 32 KB (Param 64 KB) 4-Way 2-Bank Line-Interleaved Unified L2 Cache
// (Phase 2: Non-Inclusive NINE, 4 Total MSHRs, Dual Concurrent Master Ports!)
//
// Architecture:
//   - Interleaved on line_addr[0] (byte addr[4]):
//       * Bank 0: Even 16-byte lines (16 KB, 256 sets x 4 ways, 2 MSHRs, 2 WBB)
//       * Bank 1: Odd  16-byte lines (16 KB, 256 sets x 4 ways, 2 MSHRs, 2 WBB)
//   - 2x2 Master-to-Bank Crossbar: Master 0 & Master 1 accessing different
//     banks proceed simultaneously in 1 cycle with ZERO bank contention!
//   - Same-bank collision resolved by age-fair round-robin arbiter + PMU pulse
//   - Lossless 2x2 Response Crossbar back to Master 0 & Master 1
//   - Downstream 2-to-1 Pipelined DRAM Read Arbiter (2-bit ID -> 4 outstanding!)
//     and 2-to-1 Dirty Writeback Arbiter
// ============================================================================
module l2_cache_shared #(
    parameter integer LADDR_W       = 16,  // Full 16B line address width (1 MB window)
    parameter integer SETS_PER_BANK = 256, // 256 -> 32 KB total L2, 512 -> 64 KB total L2
    parameter integer M_ID_W        = 3,   // Per-master transaction ID width
    parameter integer HIT_LAT       = 3    // L2 hit latency in cycles
) (
    input  wire                clk,
    input  wire                reset_n,

    // Master 0 Port (Phase 2: L1D; Phase 3/4: Coherent Crossbar Port 0)
    input  wire                m0_req_valid,
    output wire                m0_req_ready,
    input  wire                m0_req_is_write,
    input  wire [LADDR_W-1:0]  m0_req_laddr,
    input  wire [127:0]        m0_req_wdata,
    input  wire [M_ID_W-1:0]   m0_req_id,

    output wire                m0_resp_valid,
    output wire [M_ID_W-1:0]   m0_resp_id,
    output wire [127:0]        m0_resp_data,

    // Master 1 Port (Phase 2: L1I; Phase 3/4: Coherent Crossbar Port 1)
    input  wire                m1_req_valid,
    output wire                m1_req_ready,
    input  wire                m1_req_is_write,
    input  wire [LADDR_W-1:0]  m1_req_laddr,
    input  wire [127:0]        m1_req_wdata,
    input  wire [M_ID_W-1:0]   m1_req_id,

    output wire                m1_resp_valid,
    output wire [M_ID_W-1:0]   m1_resp_id,
    output wire [127:0]        m1_resp_data,

    // Downstream Pipelined 128-bit DRAM Read Channel (up to 4 outstanding!)
    output wire                dram_rd_valid,
    input  wire                dram_rd_ready,
    output wire [LADDR_W-1:0]  dram_rd_laddr,
    output wire [1:0]          dram_rd_id,    // {bank_bit, bank_mshr_id}

    input  wire                dram_rresp_valid,
    input  wire [1:0]          dram_rresp_id,
    input  wire [127:0]        dram_rresp_data,

    // Downstream 128-bit DRAM Writeback Channel
    output wire                dram_wr_valid,
    input  wire                dram_wr_ready,
    output wire [LADDR_W-1:0]  dram_wr_laddr,
    output wire [127:0]        dram_wr_data,

    // Status & PMU Telemetry
    output wire                l2_idle,
    output wire [1:0]          pmu_l2_hits,
    output wire [1:0]          pmu_l2_misses,
    output wire [1:0]          pmu_l2_wbs,
    output wire                pmu_l2_bank_conflict
);

localparam integer BADDR_W = LADDR_W - 1;
localparam integer BID_W   = M_ID_W + 1; // MSB = master_id (0: m0, 1: m1)

// ----------------------------------------------------------------------------
// 1. Upstream 2x2 Master -> Bank Request Crossbar (Interleaved on laddr[0])
// ----------------------------------------------------------------------------
wire m0_to_b0 = m0_req_valid && !m0_req_laddr[0];
wire m0_to_b1 = m0_req_valid &&  m0_req_laddr[0];
wire m1_to_b0 = m1_req_valid && !m1_req_laddr[0];
wire m1_to_b1 = m1_req_valid &&  m1_req_laddr[0];

reg rr_b0, rr_b1; // 0: prefer M0, 1: prefer M1

wire b0_pick_m1 = m1_to_b0 && (!m0_to_b0 || rr_b0);
wire b0_pick_m0 = m0_to_b0 && !b0_pick_m1;

wire b1_pick_m1 = m1_to_b1 && (!m0_to_b1 || rr_b1);
wire b1_pick_m0 = m0_to_b1 && !b1_pick_m1;

wire                b0_req_valid    = b0_pick_m0 || b0_pick_m1;
wire                b0_req_ready;
wire                b0_req_is_write = b0_pick_m1 ? m1_req_is_write : m0_req_is_write;
wire [BADDR_W-1:0]  b0_req_baddr    = b0_pick_m1 ? m1_req_laddr[LADDR_W-1:1] : m0_req_laddr[LADDR_W-1:1];
wire [127:0]        b0_req_wdata    = b0_pick_m1 ? m1_req_wdata : m0_req_wdata;
wire [BID_W-1:0]    b0_req_id       = b0_pick_m1 ? {1'b1, m1_req_id} : {1'b0, m0_req_id};

wire                b1_req_valid    = b1_pick_m0 || b1_pick_m1;
wire                b1_req_ready;
wire                b1_req_is_write = b1_pick_m1 ? m1_req_is_write : m0_req_is_write;
wire [BADDR_W-1:0]  b1_req_baddr    = b1_pick_m1 ? m1_req_laddr[LADDR_W-1:1] : m0_req_laddr[LADDR_W-1:1];
wire [127:0]        b1_req_wdata    = b1_pick_m1 ? m1_req_wdata : m0_req_wdata;
wire [BID_W-1:0]    b1_req_id       = b1_pick_m1 ? {1'b1, m1_req_id} : {1'b0, m0_req_id};

assign m0_req_ready = (b0_pick_m0 && b0_req_ready) || (b1_pick_m0 && b1_req_ready);
assign m1_req_ready = (b0_pick_m1 && b0_req_ready) || (b1_pick_m1 && b1_req_ready);

assign pmu_l2_bank_conflict = (m0_to_b0 && m1_to_b0) || (m0_to_b1 && m1_to_b1);

// ----------------------------------------------------------------------------
// 2. Bank 0 & Bank 1 Instances
// ----------------------------------------------------------------------------
wire                b0_resp_valid, b0_resp_ready;
wire [BID_W-1:0]    b0_resp_id;
wire [127:0]        b0_resp_data;

wire                b1_resp_valid, b1_resp_ready;
wire [BID_W-1:0]    b1_resp_id;
wire [127:0]        b1_resp_data;

wire                b0_drd_v, b0_drd_rdy, b0_drd_id;
wire [BADDR_W-1:0]  b0_drd_baddr;
wire                b0_dwr_v, b0_dwr_rdy;
wire [BADDR_W-1:0]  b0_dwr_baddr;
wire [127:0]        b0_dwr_data;

wire                b1_drd_v, b1_drd_rdy, b1_drd_id;
wire [BADDR_W-1:0]  b1_drd_baddr;
wire                b1_dwr_v, b1_dwr_rdy;
wire [BADDR_W-1:0]  b1_dwr_baddr;
wire [127:0]        b1_dwr_data;

wire                b0_idle, b1_idle;
wire                b0_hit, b0_miss, b0_wb;
wire                b1_hit, b1_miss, b1_wb;

l2_bank #(
    .BADDR_W(BADDR_W),
    .NUM_SETS(SETS_PER_BANK),
    .ID_W(BID_W),
    .HIT_LAT(HIT_LAT)
) u_bank0 (
    .clk(clk), .reset_n(reset_n),
    .req_valid(b0_req_valid), .req_ready(b0_req_ready),
    .req_is_write(b0_req_is_write), .req_baddr(b0_req_baddr),
    .req_wdata(b0_req_wdata), .req_id(b0_req_id),
    .resp_valid(b0_resp_valid), .resp_ready(b0_resp_ready),
    .resp_id(b0_resp_id), .resp_data(b0_resp_data),
    .dram_rd_valid(b0_drd_v), .dram_rd_ready(b0_drd_rdy),
    .dram_rd_baddr(b0_drd_baddr), .dram_rd_id(b0_drd_id),
    .dram_rresp_valid(dram_rresp_valid && !dram_rresp_id[1]),
    .dram_rresp_id(dram_rresp_id[0]), .dram_rresp_data(dram_rresp_data),
    .dram_wr_valid(b0_dwr_v), .dram_wr_ready(b0_dwr_rdy),
    .dram_wr_baddr(b0_dwr_baddr), .dram_wr_data(b0_dwr_data),
    .bank_idle(b0_idle), .pmu_l2_hit(b0_hit), .pmu_l2_miss(b0_miss), .pmu_l2_wb(b0_wb)
);

l2_bank #(
    .BADDR_W(BADDR_W),
    .NUM_SETS(SETS_PER_BANK),
    .ID_W(BID_W),
    .HIT_LAT(HIT_LAT)
) u_bank1 (
    .clk(clk), .reset_n(reset_n),
    .req_valid(b1_req_valid), .req_ready(b1_req_ready),
    .req_is_write(b1_req_is_write), .req_baddr(b1_req_baddr),
    .req_wdata(b1_req_wdata), .req_id(b1_req_id),
    .resp_valid(b1_resp_valid), .resp_ready(b1_resp_ready),
    .resp_id(b1_resp_id), .resp_data(b1_resp_data),
    .dram_rd_valid(b1_drd_v), .dram_rd_ready(b1_drd_rdy),
    .dram_rd_baddr(b1_drd_baddr), .dram_rd_id(b1_drd_id),
    .dram_rresp_valid(dram_rresp_valid && dram_rresp_id[1]),
    .dram_rresp_id(dram_rresp_id[0]), .dram_rresp_data(dram_rresp_data),
    .dram_wr_valid(b1_dwr_v), .dram_wr_ready(b1_dwr_rdy),
    .dram_wr_baddr(b1_dwr_baddr), .dram_wr_data(b1_dwr_data),
    .bank_idle(b1_idle), .pmu_l2_hit(b1_hit), .pmu_l2_miss(b1_miss), .pmu_l2_wb(b1_wb)
);

// ----------------------------------------------------------------------------
// 3. Lossless 2x2 Bank -> Master Response Crossbar
// ----------------------------------------------------------------------------
wire b0_to_m0 = b0_resp_valid && !b0_resp_id[M_ID_W];
wire b0_to_m1 = b0_resp_valid &&  b0_resp_id[M_ID_W];
wire b1_to_m0 = b1_resp_valid && !b1_resp_id[M_ID_W];
wire b1_to_m1 = b1_resp_valid &&  b1_resp_id[M_ID_W];

reg rr_rm0, rr_rm1; // 0: prefer Bank 0, 1: prefer Bank 1

wire m0_pick_b1 = b1_to_m0 && (!b0_to_m0 || rr_rm0);
wire m0_pick_b0 = b0_to_m0 && !m0_pick_b1;

wire m1_pick_b1 = b1_to_m1 && (!b0_to_m1 || rr_rm1);
wire m1_pick_b0 = b0_to_m1 && !m1_pick_b1;

assign m0_resp_valid = m0_pick_b0 || m0_pick_b1;
assign m0_resp_id    = m0_pick_b1 ? b1_resp_id[M_ID_W-1:0] : b0_resp_id[M_ID_W-1:0];
assign m0_resp_data  = m0_pick_b1 ? b1_resp_data           : b0_resp_data;

assign m1_resp_valid = m1_pick_b0 || m1_pick_b1;
assign m1_resp_id    = m1_pick_b1 ? b1_resp_id[M_ID_W-1:0] : b0_resp_id[M_ID_W-1:0];
assign m1_resp_data  = m1_pick_b1 ? b1_resp_data           : b0_resp_data;

assign b0_resp_ready = m0_pick_b0 || m1_pick_b0;
assign b1_resp_ready = m0_pick_b1 || m1_pick_b1;

// ----------------------------------------------------------------------------
// 4. Downstream 2-to-1 DRAM Read & Writeback Arbiters
// ----------------------------------------------------------------------------
reg rr_drd, rr_dwr;

wire drd_pick_b1 = b1_drd_v && (!b0_drd_v || rr_drd);
wire drd_pick_b0 = b0_drd_v && !drd_pick_b1;

assign dram_rd_valid = drd_pick_b0 || drd_pick_b1;
assign dram_rd_laddr = drd_pick_b1 ? {b1_drd_baddr, 1'b1} : {b0_drd_baddr, 1'b0};
assign dram_rd_id    = drd_pick_b1 ? {1'b1, b1_drd_id}    : {1'b0, b0_drd_id};
assign b0_drd_rdy    = drd_pick_b0 && dram_rd_ready;
assign b1_drd_rdy    = drd_pick_b1 && dram_rd_ready;

wire dwr_pick_b1 = b1_dwr_v && (!b0_dwr_v || rr_dwr);
wire dwr_pick_b0 = b0_dwr_v && !dwr_pick_b1;

assign dram_wr_valid = dwr_pick_b0 || dwr_pick_b1;
assign dram_wr_laddr = dwr_pick_b1 ? {b1_dwr_baddr, 1'b1} : {b0_dwr_baddr, 1'b0};
assign dram_wr_data  = dwr_pick_b1 ? b1_dwr_data          : b0_dwr_data;
assign b0_dwr_rdy    = dwr_pick_b0 && dram_wr_ready;
assign b1_dwr_rdy    = dwr_pick_b1 && dram_wr_ready;

assign l2_idle       = b0_idle && b1_idle;
assign pmu_l2_hits   = {1'b0, b0_hit}  + {1'b0, b1_hit};
assign pmu_l2_misses = {1'b0, b0_miss} + {1'b0, b1_miss};
assign pmu_l2_wbs    = {1'b0, b0_wb}   + {1'b0, b1_wb};

// ----------------------------------------------------------------------------
// 5. Round-Robin Arbiter State Updates
// ----------------------------------------------------------------------------
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        rr_b0  <= 1'b0;
        rr_b1  <= 1'b0;
        rr_rm0 <= 1'b0;
        rr_rm1 <= 1'b0;
        rr_drd <= 1'b0;
        rr_dwr <= 1'b0;
    end else begin
        if (b0_req_valid && b0_req_ready) rr_b0  <= !b0_pick_m1;
        if (b1_req_valid && b1_req_ready) rr_b1  <= !b1_pick_m1;
        if (m0_resp_valid)                rr_rm0 <= !m0_pick_b1;
        if (m1_resp_valid)                rr_rm1 <= !m1_pick_b1;
        if (dram_rd_valid && dram_rd_ready) rr_drd <= !drd_pick_b1;
        if (dram_wr_valid && dram_wr_ready) rr_dwr <= !dwr_pick_b1;
    end
end

endmodule
