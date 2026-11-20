`timescale 1ns / 1ps

// ============================================================================
// Private 2-Way Set-Associative 16-Byte Line L1 Instruction Cache (Phase 1)
//
// Features:
//   - Default: 4 KB (NUM_SETS=128 x 2 ways x 16-Byte line = 4,096 B)
//     (Parameterizable to 16 KB with NUM_SETS=512, AWIDTH=20 -> 1 MB window)
//   - 1-Cycle Hit Delivery of up to 2 Instructions/Cycle (same 16 B line when
//     req_pc[3:2] != 2'b11; clean 1-instruction delivery at line boundary!)
//   - 1-Entry Miss Controller with Mid-Miss Branch-Flush Kill
//   - FENCE.I Full Flash-Invalidation (drops any in-flight stale fill cleanly)
//   - Toggleable 1-Line Next-Line Stream Prefetcher (ENABLE_PREFETCH=1)
//   - 1D per-way arrays -> 100% Verilator, Icarus Verilog, and Yosys clean
// ============================================================================
module icache_2w #(
    parameter integer AWIDTH          = 20,  // 1 MB physical window
    parameter integer NUM_SETS        = 128, // 128 -> 4 KB, 512 -> 16 KB
    parameter integer ENABLE_PREFETCH = 1    // 1: next-line 1-entry stream buffer
) (
    input  wire                clk,
    input  wire                reset_n,

    // Fetch2W Front-End Interface
    input  wire                req_valid,
    input  wire [31:0]         req_pc,
    input  wire                flush,          // Branch mispredict / replay redirect
    input  wire                invalidate_all, // FENCE.I flash invalidation pulse

    output wire                resp_valid0,
    output wire [31:0]         resp_instr0,
    output wire                resp_valid1,
    output wire [31:0]         resp_instr1,

    // 128-Bit Backing Memory / L2 Read Interface
    output reg                 mem_rd_valid,
    input  wire                mem_rd_ready,
    output reg  [AWIDTH-5:0]   mem_rd_line_addr,

    input  wire                mem_rresp_valid,
    input  wire [127:0]        mem_rresp_data,

    // Status & PMU Observability Pulses
    output wire                icache_idle,
    output wire                pmu_ic_hit,
    output wire                pmu_ic_miss,
    output wire                pmu_ic_pf_hit,
    output wire                pmu_ic_stall
);

localparam integer IDX_W   = $clog2(NUM_SETS);
localparam integer LADDR_W = AWIDTH - 4;
localparam integer CTAG_W  = LADDR_W - IDX_W;

localparam [2:0] S_IDLE      = 3'd0;
localparam [2:0] S_MISS_REQ  = 3'd1;
localparam [2:0] S_MISS_WAIT = 3'd2;
localparam [2:0] S_PF_REQ    = 3'd3;
localparam [2:0] S_PF_WAIT   = 3'd4;

function automatic [31:0] get_word(input [127:0] line, input [1:0] woff);
    begin
        case (woff)
            2'd0: get_word = line[31:0];
            2'd1: get_word = line[63:32];
            2'd2: get_word = line[95:64];
            2'd3: get_word = line[127:96];
        endcase
    end
endfunction

// ----------------------------------------------------------------------------
// 1. Cache Tag/Data/LRU Arrays (1D per Way)
// ----------------------------------------------------------------------------
reg              way0_valid [0:NUM_SETS-1];
reg [CTAG_W-1:0] way0_tag   [0:NUM_SETS-1];
reg [127:0]      way0_data  [0:NUM_SETS-1];

reg              way1_valid [0:NUM_SETS-1];
reg [CTAG_W-1:0] way1_tag   [0:NUM_SETS-1];
reg [127:0]      way1_data  [0:NUM_SETS-1];

reg              lru_arr    [0:NUM_SETS-1]; // 1: Way 0 MRU (evict 1); 0: Way 1 MRU

// ----------------------------------------------------------------------------
// 2. Miss Controller & 1-Line Next-Line Prefetch Stream Buffer
// ----------------------------------------------------------------------------
reg [2:0]         state;
reg [LADDR_W-1:0] miss_laddr;
reg               miss_killed;

reg               pf_valid;
reg [LADDR_W-1:0] pf_laddr;
reg [127:0]       pf_data;

reg               pf_pend;
reg [LADDR_W-1:0] pf_pend_laddr;

// ----------------------------------------------------------------------------
// 3. Combinational Tag & Prefetch-Buffer Lookup
// ----------------------------------------------------------------------------
wire [LADDR_W-1:0] req_laddr = req_pc[AWIDTH-1:4];
wire [IDX_W-1:0]   req_set   = req_pc[4 +: IDX_W];
wire [CTAG_W-1:0]  req_ctag  = req_pc[AWIDTH-1 : 4 + IDX_W];
wire [1:0]         req_woff  = req_pc[3:2];

wire hit_w0  = way0_valid[req_set] && (way0_tag[req_set] == req_ctag);
wire hit_w1  = way1_valid[req_set] && (way1_tag[req_set] == req_ctag);
wire tag_hit = hit_w0 || hit_w1;
wire pf_hit  = (ENABLE_PREFETCH != 0) && pf_valid && (pf_laddr == req_laddr) && !tag_hit;

wire any_hit = req_valid && !flush && !invalidate_all && (tag_hit || pf_hit);

wire [127:0] hit_line = hit_w0 ? way0_data[req_set] :
                        hit_w1 ? way1_data[req_set] : pf_data;

assign resp_valid0 = any_hit;
assign resp_instr0 = any_hit ? get_word(hit_line, req_woff) : 32'h0000_0013;
assign resp_valid1 = any_hit && (req_woff != 2'b11);
assign resp_instr1 = resp_valid1 ? get_word(hit_line, req_woff + 2'd1) : 32'h0000_0013;

// Demand miss trigger
wire demand_miss = req_valid && !flush && !invalidate_all && !tag_hit && !pf_hit;

// Check if pending next-line prefetch is already resident in main cache or pf buffer
wire [IDX_W-1:0]  pfp_set  = pf_pend_laddr[IDX_W-1:0];
wire [CTAG_W-1:0] pfp_ctag = pf_pend_laddr[LADDR_W-1:IDX_W];
wire pfp_already_in = (way0_valid[pfp_set] && (way0_tag[pfp_set] == pfp_ctag)) ||
                      (way1_valid[pfp_set] && (way1_tag[pfp_set] == pfp_ctag)) ||
                      (pf_valid && (pf_laddr == pf_pend_laddr));

// Victim selection for refill or prefetch promotion
wire [IDX_W-1:0]  rf_set   = miss_laddr[IDX_W-1:0];
wire [CTAG_W-1:0] rf_ctag  = miss_laddr[LADDR_W-1:IDX_W];
wire              rf_vway  = (!way0_valid[rf_set]) ? 1'b0 :
                             (!way1_valid[rf_set]) ? 1'b1 : lru_arr[rf_set];

wire              pf_vway  = (!way0_valid[req_set]) ? 1'b0 :
                             (!way1_valid[req_set]) ? 1'b1 : lru_arr[req_set];

// Backing memory read request outputs
always @(*) begin
    mem_rd_valid     = 1'b0;
    mem_rd_line_addr = {LADDR_W{1'b0}};
    if (!invalidate_all) begin
        if (state == S_MISS_REQ && !flush) begin
            mem_rd_valid     = 1'b1;
            mem_rd_line_addr = miss_laddr;
        end else if (state == S_PF_REQ && !flush && !demand_miss) begin
            mem_rd_valid     = 1'b1;
            mem_rd_line_addr = miss_laddr;
        end
    end
end

assign icache_idle  = (state == S_IDLE);
assign pmu_ic_hit   = any_hit;
assign pmu_ic_miss  = (state == S_IDLE) && demand_miss;
assign pmu_ic_pf_hit = any_hit && pf_hit;
assign pmu_ic_stall = demand_miss;

// ----------------------------------------------------------------------------
// 4. Sequential Control & Array Updates
// ----------------------------------------------------------------------------
integer k;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (k = 0; k < NUM_SETS; k = k + 1) begin
            way0_valid[k] <= 1'b0;
            way0_tag[k]   <= {CTAG_W{1'b0}};
            way0_data[k]  <= 128'd0;
            way1_valid[k] <= 1'b0;
            way1_tag[k]   <= {CTAG_W{1'b0}};
            way1_data[k]  <= 128'd0;
            lru_arr[k]    <= 1'b0;
        end
        state         <= S_IDLE;
        miss_laddr    <= {LADDR_W{1'b0}};
        miss_killed   <= 1'b0;
        pf_valid      <= 1'b0;
        pf_laddr      <= {LADDR_W{1'b0}};
        pf_data       <= 128'd0;
        pf_pend       <= 1'b0;
        pf_pend_laddr <= {LADDR_W{1'b0}};
    end else if (invalidate_all) begin
        // FENCE.I: immediately clear all valid bits and kill any in-flight refill!
        for (k = 0; k < NUM_SETS; k = k + 1) begin
            way0_valid[k] <= 1'b0;
            way1_valid[k] <= 1'b0;
            lru_arr[k]    <= 1'b0;
        end
        pf_valid <= 1'b0;
        pf_pend  <= 1'b0;
        if (state == S_MISS_REQ || state == S_PF_REQ) begin
            state       <= S_IDLE;
            miss_killed <= 1'b0;
        end else if (state == S_MISS_WAIT || state == S_PF_WAIT) begin
            if (mem_rresp_valid) begin
                state       <= S_IDLE;
                miss_killed <= 1'b0;
            end else begin
                miss_killed <= 1'b1;
            end
        end
    end else begin
        // Update LRU on normal cache hits (or promote prefetch buffer hit into main cache!)
        if (any_hit) begin
            if (tag_hit) begin
                lru_arr[req_set] <= hit_w0 ? 1'b1 : 1'b0;
                // Arm next-line prefetch when crossing upper half of a line
                if ((ENABLE_PREFETCH != 0) && (req_woff[1] == 1'b1) && !pf_pend) begin
                    pf_pend       <= 1'b1;
                    pf_pend_laddr <= req_laddr + {{(LADDR_W-1){1'b0}}, 1'b1};
                end
            end else if (pf_hit) begin
                // Promote stream buffer line into LRU victim way & arm next sequential line!
                if (!pf_vway) begin
                    way0_valid[req_set] <= 1'b1;
                    way0_tag[req_set]   <= req_ctag;
                    way0_data[req_set]  <= pf_data;
                    lru_arr[req_set]    <= 1'b1;
                end else begin
                    way1_valid[req_set] <= 1'b1;
                    way1_tag[req_set]   <= req_ctag;
                    way1_data[req_set]  <= pf_data;
                    lru_arr[req_set]    <= 1'b0;
                end
                pf_valid      <= 1'b0;
                pf_pend       <= 1'b1;
                pf_pend_laddr <= req_laddr + {{(LADDR_W-1){1'b0}}, 1'b1};
            end
        end

        if (flush)
            pf_pend <= 1'b0;

        // Miss / Prefetch FSM
        case (state)
            S_IDLE: begin
                miss_killed <= 1'b0;
                if (demand_miss) begin
                    state      <= S_MISS_REQ;
                    miss_laddr <= req_laddr;
                end else if ((ENABLE_PREFETCH != 0) && pf_pend && !flush) begin
                    pf_pend <= 1'b0;
                    if (!pfp_already_in) begin
                        state      <= S_PF_REQ;
                        miss_laddr <= pf_pend_laddr;
                    end
                end
            end

            S_MISS_REQ: begin
                if (flush) begin
                    state <= S_IDLE;
                end else if (mem_rd_valid && mem_rd_ready) begin
                    state       <= S_MISS_WAIT;
                    miss_killed <= 1'b0;
                end
            end

            S_MISS_WAIT: begin
                if (flush)
                    miss_killed <= 1'b1;
                if (mem_rresp_valid) begin
                    // Always safe to install unless killed by FENCE.I (already handled above)
                    if (!miss_killed && !flush) begin
                        if (!rf_vway) begin
                            way0_valid[rf_set] <= 1'b1;
                            way0_tag[rf_set]   <= rf_ctag;
                            way0_data[rf_set]  <= mem_rresp_data;
                            lru_arr[rf_set]    <= 1'b1;
                        end else begin
                            way1_valid[rf_set] <= 1'b1;
                            way1_tag[rf_set]   <= rf_ctag;
                            way1_data[rf_set]  <= mem_rresp_data;
                            lru_arr[rf_set]    <= 1'b0;
                        end
                        if (ENABLE_PREFETCH != 0) begin
                            pf_pend       <= 1'b1;
                            pf_pend_laddr <= miss_laddr + {{(LADDR_W-1){1'b0}}, 1'b1};
                        end
                    end
                    state       <= S_IDLE;
                    miss_killed <= 1'b0;
                end
            end

            S_PF_REQ: begin
                if (flush) begin
                    state <= S_IDLE;
                end else if (demand_miss) begin
                    // Preempt unissued prefetch immediately in favor of demand miss!
                    state      <= S_MISS_REQ;
                    miss_laddr <= req_laddr;
                end else if (mem_rd_valid && mem_rd_ready) begin
                    state       <= S_PF_WAIT;
                    miss_killed <= 1'b0;
                end
            end

            S_PF_WAIT: begin
                if (flush)
                    miss_killed <= 1'b1;
                if (mem_rresp_valid) begin
                    if (!miss_killed && !flush) begin
                        pf_valid <= 1'b1;
                        pf_laddr <= miss_laddr;
                        pf_data  <= mem_rresp_data;
                    end
                    state       <= S_IDLE;
                    miss_killed <= 1'b0;
                end
            end

            default: begin
                state <= S_IDLE;
            end
        endcase
    end
end

endmodule
