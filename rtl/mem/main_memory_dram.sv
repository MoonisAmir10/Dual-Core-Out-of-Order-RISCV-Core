`timescale 1ns / 1ps

// ============================================================================
// Pipelined 128-Bit (16-Byte Line) 4-Outstanding Unified Backing DRAM (Phase 2)
//
// Features:
//   - Default: 256 KB (LINE_BITS=14 -> 16,384 x 128-bit lines)
//   - 4-Slot Pipelined Read Queue: sustains all 4 L2 MSHRs (2 banks x 2 MSHRs)
//     in flight simultaneously for true 4x DRAM Memory-Level Parallelism!
//   - Realistic configurable LATENCY (default L16 cycles) + optional runtime
//     LFSR jitter mode (+0..+7 cycles) to stress out-of-order fill interleavings
//   - Immediate coherent writeback array commit with realistic LATENCY busy spacing
// ============================================================================
module main_memory_dram #(
    parameter integer LINE_BITS = 14, // 14 -> 256 KB
    parameter integer LATENCY   = 16, // Realistic DRAM miss penalty vs 3-4 cyc L2
    parameter integer ID_W      = 2   // {bank_bit, bank_mshr_id}
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  jitter_en, // 1: add pseudo-random 0..7 cyc latency jitter

    // Pipelined 128-bit line read channel (up to 4 outstanding!)
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
    input  wire [127:0]          mem_wr_data,

    // Hardware PMU pulses
    output wire                  pmu_dram_rd,
    output wire                  pmu_dram_wr
);

localparam integer NUM_LINES = 1 << LINE_BITS;
localparam integer SLOTS     = 4;

reg [127:0] mem [0:NUM_LINES-1];

reg                 r_v    [0:SLOTS-1];
reg [ID_W-1:0]      r_id   [0:SLOTS-1];
reg [LINE_BITS-1:0] r_addr [0:SLOTS-1];
reg [7:0]           r_cnt  [0:SLOTS-1];

reg [7:0]           w_busy_cnt;
reg [15:0]          lfsr;

// Find free read slot & ready-to-respond read slot
wire [3:0] r_done = {
    (r_v[3] && (r_cnt[3] == 8'd0)),
    (r_v[2] && (r_cnt[2] == 8'd0)),
    (r_v[1] && (r_cnt[1] == 8'd0)),
    (r_v[0] && (r_cnt[0] == 8'd0))
};

wire has_free_slot = !r_v[0] || !r_v[1] || !r_v[2] || !r_v[3];
wire [1:0] free_slot = !r_v[0] ? 2'd0 : (!r_v[1] ? 2'd1 : (!r_v[2] ? 2'd2 : 2'd3));

wire has_done_slot = |r_done;
wire [1:0] done_slot = r_done[0] ? 2'd0 : (r_done[1] ? 2'd1 : (r_done[2] ? 2'd2 : 2'd3));

assign mem_rd_ready = has_free_slot;
assign mem_wr_ready = (w_busy_cnt == 8'd0);

assign pmu_dram_rd  = mem_rd_valid && mem_rd_ready;
assign pmu_dram_wr  = mem_wr_valid && mem_wr_ready;

wire [7:0] base_lat = (LATENCY <= 1) ? 8'd0 : (LATENCY[7:0] - 8'd1);
wire [7:0] eff_lat  = jitter_en ? (base_lat + {5'd0, lfsr[2:0]}) : base_lat;

`ifndef SYNTHESIS
integer init_i;
initial begin
    for (init_i = 0; init_i < NUM_LINES; init_i = init_i + 1)
        mem[init_i] = 128'd0;
end
`endif

integer k;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (k = 0; k < SLOTS; k = k + 1) begin
            r_v[k]    <= 1'b0;
            r_id[k]   <= {ID_W{1'b0}};
            r_addr[k] <= {LINE_BITS{1'b0}};
            r_cnt[k]  <= 8'd0;
        end
        w_busy_cnt      <= 8'd0;
        lfsr            <= 16'hACE1;
        mem_rresp_valid <= 1'b0;
        mem_rresp_id    <= {ID_W{1'b0}};
        mem_rresp_data  <= 128'd0;
    end else begin
        lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
        mem_rresp_valid <= 1'b0;

        // 1. Progress active reads & emit at most 1 response/cycle
        for (k = 0; k < SLOTS; k = k + 1) begin
            if (r_v[k] && (r_cnt[k] != 8'd0))
                r_cnt[k] <= r_cnt[k] - 8'd1;
        end

        if (has_done_slot) begin
            mem_rresp_valid <= 1'b1;
            mem_rresp_id    <= r_id[done_slot];
            mem_rresp_data  <= mem[r_addr[done_slot]];
            r_v[done_slot]  <= 1'b0;
        end

        // 2. Accept new pipelined read into a free slot
        if (mem_rd_valid && mem_rd_ready) begin
            r_v[free_slot]    <= 1'b1;
            r_id[free_slot]   <= mem_rd_id;
            r_addr[free_slot] <= mem_rd_line_addr;
            r_cnt[free_slot]  <= eff_lat;
        end

        // 3. Writeback port: commit immediately into array & enforce LATENCY busy
        if (w_busy_cnt != 8'd0) begin
            w_busy_cnt <= w_busy_cnt - 8'd1;
        end else if (mem_wr_valid) begin
            mem[mem_wr_line_addr] <= mem_wr_data;
            w_busy_cnt            <= base_lat;
        end
    end
end

endmodule
