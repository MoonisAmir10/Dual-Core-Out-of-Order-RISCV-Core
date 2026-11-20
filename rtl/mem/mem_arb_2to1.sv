`timescale 1ns / 1ps

// ============================================================================
// 2-to-1 Split-Transaction 128-Bit Memory Read Arbiter (L1D + L1I -> Unified RAM)
// Used in Phase 1 single-core unified SoC (replaced by Shared L2 + Crossbar in P2/P3).
// Encodes master bit in mem_rd_id[1] (0: L1D, 1: L1I) and sub-ID in mem_rd_id[0].
// Round-robin fairness prevents starvation when both caches miss concurrently.
// ============================================================================
module mem_arb_2to1 #(
    parameter integer LADDR_W = 16 // AWIDTH - 4 (16 bits for 1 MB window)
) (
    input  wire               clk,
    input  wire               reset_n,

    // Port 0: L1 Data Cache (up to 2 MSHRs, 1-bit sub-ID)
    input  wire               dc_rd_valid,
    output wire               dc_rd_ready,
    input  wire [LADDR_W-1:0] dc_rd_line_addr,
    input  wire               dc_rd_id,
    output wire               dc_rresp_valid,
    output wire               dc_rresp_id,
    output wire [127:0]       dc_rresp_data,

    // Port 1: L1 Instruction Cache (1-entry miss/prefetch)
    input  wire               ic_rd_valid,
    output wire               ic_rd_ready,
    input  wire [LADDR_W-1:0] ic_rd_line_addr,
    output wire               ic_rresp_valid,
    output wire [127:0]       ic_rresp_data,

    // Downstream Pipelined Unified Memory Read Channel (2-bit ID)
    output wire               mem_rd_valid,
    input  wire               mem_rd_ready,
    output wire [LADDR_W-1:0] mem_rd_line_addr,
    output wire [1:0]         mem_rd_id,
    input  wire               mem_rresp_valid,
    input  wire [1:0]         mem_rresp_id,
    input  wire [127:0]       mem_rresp_data
);

reg rr_prio_ic; // 0: prefer L1D, 1: prefer L1I

wire pick_ic = ic_rd_valid && (!dc_rd_valid || rr_prio_ic);
wire pick_dc = dc_rd_valid && !pick_ic;

assign mem_rd_valid     = pick_dc || pick_ic;
assign mem_rd_line_addr = pick_ic ? ic_rd_line_addr : dc_rd_line_addr;
assign mem_rd_id        = pick_ic ? 2'b10 : {1'b0, dc_rd_id};

assign dc_rd_ready      = pick_dc && mem_rd_ready;
assign ic_rd_ready      = pick_ic && mem_rd_ready;

assign dc_rresp_valid   = mem_rresp_valid && !mem_rresp_id[1];
assign dc_rresp_id      = mem_rresp_id[0];
assign dc_rresp_data    = mem_rresp_data;

assign ic_rresp_valid   = mem_rresp_valid && mem_rresp_id[1];
assign ic_rresp_data    = mem_rresp_data;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        rr_prio_ic <= 1'b0;
    end else if (mem_rd_valid && mem_rd_ready) begin
        rr_prio_ic <= !pick_ic;
    end
end

endmodule
