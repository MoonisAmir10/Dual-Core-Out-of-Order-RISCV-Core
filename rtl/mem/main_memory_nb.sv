`timescale 1ns / 1ps

// ============================================================================
// Pipelined 128-Bit (16-Byte Line) Non-Blocking Backing Memory
// (Phase 1: Parameterized ID_W, 64 KB..256 KB unified image support)
// ============================================================================
module main_memory_nb #(
    parameter integer LINE_BITS = 12, // 12 -> 64 KB, 14 -> 256 KB
    parameter integer LATENCY   = 4,  // >= 1 clock cycles
    parameter integer ID_W      = 1   // Transaction ID width (1 for L1D, 2 for L1I+L1D)
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // Pipelined 128-bit line read channel (up to 2 outstanding)
    input  wire                  mem_rd_valid,
    output wire                  mem_rd_ready,
    input  wire [LINE_BITS-1:0]  mem_rd_line_addr,
    input  wire [ID_W-1:0]       mem_rd_id,

    output reg                   mem_rresp_valid,
    output reg  [ID_W-1:0]       mem_rresp_id,
    output reg  [127:0]          mem_rresp_data,

    // 128-bit line writeback channel
    input  wire                  mem_wr_valid,
    output wire                  mem_wr_ready,
    input  wire [LINE_BITS-1:0]  mem_wr_line_addr,
    input  wire [127:0]          mem_wr_data
);

localparam integer NUM_LINES = 1 << LINE_BITS;

reg [127:0] mem [0:NUM_LINES-1];

// 2-slot pipelined read tracker
reg                 r0_v, r1_v;
reg [ID_W-1:0]      r0_id, r1_id;
reg [LINE_BITS-1:0] r0_addr, r1_addr;
reg [7:0]           r0_cnt, r1_cnt;

// Write-port busy timer (array updates immediately on accept for coherence)
reg [7:0]           w_busy_cnt;

wire r0_done = r0_v && (r0_cnt == 8'd0);
wire r1_done = r1_v && (r1_cnt == 8'd0);
wire send_r0 = r0_done;
wire send_r1 = !r0_done && r1_done;

assign mem_rd_ready = !r0_v || !r1_v;
assign mem_wr_ready = (w_busy_cnt == 8'd0);

`ifndef SYNTHESIS
integer init_i;
initial begin
    for (init_i = 0; init_i < NUM_LINES; init_i = init_i + 1)
        mem[init_i] = 128'd0;
end
`endif

wire [7:0] init_lat = (LATENCY <= 1) ? 8'd0 : (LATENCY[7:0] - 8'd1);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r0_v            <= 1'b0;
        r1_v            <= 1'b0;
        r0_id           <= {ID_W{1'b0}};
        r1_id           <= {ID_W{1'b0}};
        r0_addr         <= {LINE_BITS{1'b0}};
        r1_addr         <= {LINE_BITS{1'b0}};
        r0_cnt          <= 8'd0;
        r1_cnt          <= 8'd0;
        w_busy_cnt      <= 8'd0;
        mem_rresp_valid <= 1'b0;
        mem_rresp_id    <= {ID_W{1'b0}};
        mem_rresp_data  <= 128'd0;
    end else begin
        mem_rresp_valid <= 1'b0;

        // 1. Progress active reads & emit at most 1 response/cycle
        if (send_r0) begin
            mem_rresp_valid <= 1'b1;
            mem_rresp_id    <= r0_id;
            mem_rresp_data  <= mem[r0_addr];
            r0_v            <= 1'b0;
        end else if (r0_v && (r0_cnt != 8'd0)) begin
            r0_cnt <= r0_cnt - 8'd1;
        end

        if (send_r1) begin
            mem_rresp_valid <= 1'b1;
            mem_rresp_id    <= r1_id;
            mem_rresp_data  <= mem[r1_addr];
            r1_v            <= 1'b0;
        end else if (r1_v && (r1_cnt != 8'd0)) begin
            r1_cnt <= r1_cnt - 8'd1;
        end

        // 2. Accept new pipelined read into a free slot
        if (mem_rd_valid && mem_rd_ready) begin
            if (!r0_v) begin
                r0_v    <= 1'b1;
                r0_id   <= mem_rd_id;
                r0_addr <= mem_rd_line_addr;
                r0_cnt  <= init_lat;
            end else begin
                r1_v    <= 1'b1;
                r1_id   <= mem_rd_id;
                r1_addr <= mem_rd_line_addr;
                r1_cnt  <= init_lat;
            end
        end

        // 3. Writeback port: commit immediately into array & enforce LATENCY busy
        if (w_busy_cnt != 8'd0) begin
            w_busy_cnt <= w_busy_cnt - 8'd1;
        end else if (mem_wr_valid) begin
            mem[mem_wr_line_addr] <= mem_wr_data;
            w_busy_cnt            <= init_lat;
        end
    end
end

endmodule
