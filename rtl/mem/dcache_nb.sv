`timescale 1ns / 1ps

// ============================================================================
// Parameterizable 2-Way Set-Associative Non-Blocking Data Cache (Upgrade U3)
//   - Default: 4 KB (NUM_SETS=128 x 2 ways x 16-Byte line = 4,096 B)
//     (Override NUM_SETS=512 -> 16 KB with zero RTL edits!)
//   - Write-Back, Write-Allocate, LRU replacement
//   - 1-Cycle Hit Latency (dedicated hit_resp channel)
//   - 2 MSHRs supporting:
//       * Hit-Under-Miss (HUM) & Hit-Under-2-Misses
//       * Miss-Under-Miss (MUM, MLP = 2) with pipelined backing memory
//       * Same-Line Secondary Load Coalescing (up to 4 waiters/MSHR)
//       * 16-Byte Write-Merge Buffers (write-allocate + store coalescing)
//       * Selective Branch-Squash Waiter Killing (harmless prefetch fill)
//   - 2-Entry Writeback Buffer (WBB) with exact address-hazard stalling
// ============================================================================
module dcache_nb #(
    parameter integer AWIDTH   = 16,  // 64 KB byte-addressable
    parameter integer NUM_SETS = 128, // 128 -> 4 KB, 256 -> 8 KB, 512 -> 16 KB
    parameter integer TAG_W    = 5,   // ROB tag width (32-entry ROB)
    parameter integer ID_W     = 6    // Load transaction ID ({lq_gen[2:0], lq_idx[2:0]})
) (
    input  wire                clk,
    input  wire                reset_n,

    // Split-Transaction CPU Request Channel (LSQ -> D$)
    input  wire                req_valid,
    output reg                 req_ready,
    input  wire                req_is_write,
    input  wire [AWIDTH-1:0]   req_addr,
    input  wire [31:0]         req_wdata,
    input  wire [3:0]          req_wstrb,
    input  wire [ID_W-1:0]     req_id,
    input  wire [TAG_W-1:0]    req_rob_tag,

    // Port A: 1-Cycle Load Hit Response (D$ -> LSQ)
    output reg                 hit_resp_valid,
    output reg  [ID_W-1:0]     hit_resp_id,
    output reg  [31:0]         hit_resp_rdata,

    // Port B: MSHR Miss-Fill Load Response (D$ -> LSQ)
    output reg                 fill_resp_valid,
    output reg  [ID_W-1:0]     fill_resp_id,
    output reg  [31:0]         fill_resp_rdata,

    // Selective Branch-Squash Kill (kills younger MSHR load waiters cleanly)
    input  wire                flush,
    input  wire                squash_valid,
    input  wire [TAG_W-1:0]    squash_tag,
    input  wire [TAG_W-1:0]    rob_head,

    // Pipelined 128-Bit Backing Memory Interface
    output reg                 mem_rd_valid,
    input  wire                mem_rd_ready,
    output reg  [AWIDTH-5:0]   mem_rd_line_addr,
    output reg                 mem_rd_id,

    input  wire                mem_rresp_valid,
    input  wire                mem_rresp_id,
    input  wire [127:0]        mem_rresp_data,

    output wire                mem_wr_valid,
    input  wire                mem_wr_ready,
    output wire [AWIDTH-5:0]   mem_wr_line_addr,
    output wire [127:0]        mem_wr_data,

    // Status & Hardware PMU Observability Pulses
    output wire                dcache_idle,
    output wire                dcache_writes_idle,
    output wire                dcache_any_miss,
    output wire                cache_hit,
    output wire                cache_miss,
    output wire                cache_wb,
    output wire                cache_hum,
    output wire                cache_mum_cyc,
    output wire                cache_mshr_coalesce
);

localparam integer IDX_W   = $clog2(NUM_SETS);        // 7 for 4 KB
localparam integer LADDR_W = AWIDTH - 4;              // 12 (16-byte line address)
localparam integer CTAG_W  = LADDR_W - IDX_W;         // 5 for 4 KB

// ----------------------------------------------------------------------------
// 1. Helper Functions (128-bit Line Word Extract & Byte-Masked Merge)
// ----------------------------------------------------------------------------
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

function automatic [127:0] merge_word(
    input [127:0] line,
    input [1:0]   woff,
    input [31:0]  wdata,
    input [3:0]   wstrb
);
    reg [127:0] res;
    reg [31:0]  cur, nw;
    begin
        res = line;
        cur = get_word(line, woff);
        nw  = {
            wstrb[3] ? wdata[31:24] : cur[31:24],
            wstrb[2] ? wdata[23:16] : cur[23:16],
            wstrb[1] ? wdata[15:8]  : cur[15:8],
            wstrb[0] ? wdata[7:0]   : cur[7:0]
        };
        case (woff)
            2'd0: res[31:0]   = nw;
            2'd1: res[63:32]  = nw;
            2'd2: res[95:64]  = nw;
            2'd3: res[127:96] = nw;
        endcase
        merge_word = res;
    end
endfunction

function automatic [15:0] expand_line_mask(input [1:0] woff, input [3:0] wstrb);
    begin
        case (woff)
            2'd0: expand_line_mask = {12'd0, wstrb};
            2'd1: expand_line_mask = {8'd0,  wstrb, 4'd0};
            2'd2: expand_line_mask = {4'd0,  wstrb, 8'd0};
            2'd3: expand_line_mask = {wstrb, 12'd0};
        endcase
    end
endfunction

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
// 2. Cache Arrays (1D Arrays per Way -> 100% Verilator/Icarus/Yosys Clean!)
// ----------------------------------------------------------------------------
reg              way0_valid [0:NUM_SETS-1];
reg              way0_dirty [0:NUM_SETS-1];
reg [CTAG_W-1:0] way0_tag   [0:NUM_SETS-1];
reg [127:0]      way0_data  [0:NUM_SETS-1];

reg              way1_valid [0:NUM_SETS-1];
reg              way1_dirty [0:NUM_SETS-1];
reg [CTAG_W-1:0] way1_tag   [0:NUM_SETS-1];
reg [127:0]      way1_data  [0:NUM_SETS-1];

reg              lru_arr    [0:NUM_SETS-1]; // 1: Way 0 MRU (evict 1); 0: Way 1 MRU (evict 0)

// ----------------------------------------------------------------------------
// 3. 2 MSHRs + Flat Waiter Sub-Queues (0..3 -> MSHR0, 4..7 -> MSHR1) + 2-Entry WBB
// ----------------------------------------------------------------------------
reg               m0_valid, m0_refilled, m0_mem_sent;
reg [LADDR_W-1:0] m0_laddr;
reg [15:0]        m0_st_mask;
reg [127:0]       m0_st_data;
reg [127:0]       m0_fill_line;

reg               m1_valid, m1_refilled, m1_mem_sent;
reg [LADDR_W-1:0] m1_laddr;
reg [15:0]        m1_st_mask;
reg [127:0]       m1_st_data;
reg [127:0]       m1_fill_line;

// Waiter slots 0..3 belong to MSHR0; 4..7 belong to MSHR1
reg               w_v    [0:7];
reg [ID_W-1:0]    w_id   [0:7];
reg [TAG_W-1:0]   w_tag  [0:7];
reg [1:0]         w_woff [0:7];

// 2-Entry Writeback Buffer (WBB)
reg               wbb0_v, wbb1_v;
reg [LADDR_W-1:0] wbb0_laddr, wbb1_laddr;
reg [127:0]       wbb0_data, wbb1_data;

// ----------------------------------------------------------------------------
// 4. Combinational Lookup, Hazard Detection & MSHR/WBB Arbitration
// ----------------------------------------------------------------------------
wire [LADDR_W-1:0] req_laddr = req_addr[AWIDTH-1:4];
wire [IDX_W-1:0]   req_set   = req_addr[4 +: IDX_W];
wire [CTAG_W-1:0]  req_ctag  = req_addr[AWIDTH-1 : 4 + IDX_W];
wire [1:0]         req_woff  = req_addr[3:2];

wire hit_w0  = way0_valid[req_set] && (way0_tag[req_set] == req_ctag);
wire hit_w1  = way1_valid[req_set] && (way1_tag[req_set] == req_ctag);
wire tag_hit = hit_w0 || hit_w1;

wire wbb_hit    = (wbb0_v && (wbb0_laddr == req_laddr)) || (wbb1_v && (wbb1_laddr == req_laddr));
wire m0_match   = m0_valid && (m0_laddr == req_laddr);
wire m1_match   = m1_valid && (m1_laddr == req_laddr);
wire mshr_match = m0_match || m1_match;

wire m0_has_waiter = w_v[0] || w_v[1] || w_v[2] || w_v[3];
wire m1_has_waiter = w_v[4] || w_v[5] || w_v[6] || w_v[7];
wire m0_has_wfree  = !w_v[0] || !w_v[1] || !w_v[2] || !w_v[3];
wire m1_has_wfree  = !w_v[4] || !w_v[5] || !w_v[6] || !w_v[7];

wire [1:0] m0_free_widx = !w_v[0] ? 2'd0 : (!w_v[1] ? 2'd1 : (!w_v[2] ? 2'd2 : 2'd3));
wire [1:0] m1_free_widx = !w_v[4] ? 2'd0 : (!w_v[5] ? 2'd1 : (!w_v[6] ? 2'd2 : 2'd3));

wire [1:0] wbb_free_cnt      = {1'b0, !wbb0_v} + {1'b0, !wbb1_v};
wire [1:0] unfilled_mshr_cnt = {1'b0, (m0_valid && !m0_refilled)} + {1'b0, (m1_valid && !m1_refilled)};
wire       same_set_mshr     = (m0_valid && (m0_laddr[IDX_W-1:0] == req_set)) ||
                               (m1_valid && (m1_laddr[IDX_W-1:0] == req_set));
wire       can_alloc_mshr    = (!m0_valid || !m1_valid) && (wbb_free_cnt > unfilled_mshr_cnt) && !same_set_mshr;

// Classify accepted CPU request cleanly
reg req_is_hit_act;
reg req_is_coalesce_act;
reg req_is_new_miss_act;

always @(*) begin
    req_ready           = 1'b0;
    req_is_hit_act      = 1'b0;
    req_is_coalesce_act = 1'b0;
    req_is_new_miss_act = 1'b0;

    if (req_valid && !mem_rresp_valid && !wbb_hit) begin
        if (m0_match) begin
            if (!m0_refilled) begin
                if (req_is_write) begin
                    // Allow store merge only when no older/speculative load waiters exist on MSHR0
                    if (!m0_has_waiter) begin
                        req_ready           = 1'b1;
                        req_is_coalesce_act = 1'b1;
                    end
                end else if (m0_has_wfree) begin
                    req_ready           = 1'b1;
                    req_is_coalesce_act = 1'b1;
                end
            end
        end else if (m1_match) begin
            if (!m1_refilled) begin
                if (req_is_write) begin
                    if (!m1_has_waiter) begin
                        req_ready           = 1'b1;
                        req_is_coalesce_act = 1'b1;
                    end
                end else if (m1_has_wfree) begin
                    req_ready           = 1'b1;
                    req_is_coalesce_act = 1'b1;
                end
            end
        end else if (tag_hit) begin
            req_ready      = 1'b1;
            req_is_hit_act = 1'b1;
        end else if (can_alloc_mshr) begin
            req_ready           = 1'b1;
            req_is_new_miss_act = 1'b1;
        end
    end
end

// Refill decode & victim selection (when mem_rresp_valid == 1)
wire [LADDR_W-1:0] rf_laddr  = mem_rresp_id ? m1_laddr   : m0_laddr;
wire [IDX_W-1:0]   rf_set    = rf_laddr[IDX_W-1:0];
wire [CTAG_W-1:0]  rf_ctag   = rf_laddr[LADDR_W-1:IDX_W];
wire [15:0]        rf_stmask = mem_rresp_id ? m1_st_mask : m0_st_mask;
wire [127:0]       rf_stdata = mem_rresp_id ? m1_st_data : m0_st_data;
wire [127:0]       rf_merged = merge_line_bytes(mem_rresp_data, rf_stdata, rf_stmask);

wire rf_vic_way   = (!way0_valid[rf_set]) ? 1'b0 :
                    (!way1_valid[rf_set]) ? 1'b1 :
                    lru_arr[rf_set];
wire rf_vic_valid = rf_vic_way ? way1_valid[rf_set] : way0_valid[rf_set];
wire rf_vic_dirty = rf_vic_way ? way1_dirty[rf_set] : way0_dirty[rf_set];
wire rf_need_wb   = mem_rresp_valid && rf_vic_valid && rf_vic_dirty;
wire [LADDR_W-1:0] rf_vic_laddr = {(rf_vic_way ? way1_tag[rf_set] : way0_tag[rf_set]), rf_set};
wire [127:0]       rf_vic_data  = rf_vic_way ? way1_data[rf_set] : way0_data[rf_set];

// Pick first surviving (non-killed) waiter for fill_resp_valid
wire [TAG_W-1:0] sq_br_age = squash_tag - rob_head;
wire [7:0] w_alive;
genvar gi;
generate
    for (gi = 0; gi < 8; gi = gi + 1) begin : gen_walive
        assign w_alive[gi] = w_v[gi] && !flush &&
                             !(squash_valid && ((w_tag[gi] - rob_head) > sq_br_age));
    end
endgenerate

// Waiter drain arbitration: prioritize refill-cycle MSHR, else already-refilled MSHR0/1
reg        drain_v;
reg        drain_is_m1;
reg [2:0]  drain_wslot;
reg [127:0] drain_line;

wire m0_rf_now = mem_rresp_valid && !mem_rresp_id && m0_valid;
wire m1_rf_now = mem_rresp_valid &&  mem_rresp_id && m1_valid;

wire m0_has_alive = |w_alive[3:0];
wire m1_has_alive = |w_alive[7:4];
wire [2:0] m0_first_alive = w_alive[0] ? 3'd0 : (w_alive[1] ? 3'd1 : (w_alive[2] ? 3'd2 : 3'd3));
wire [2:0] m1_first_alive = w_alive[4] ? 3'd4 : (w_alive[5] ? 3'd5 : (w_alive[6] ? 3'd6 : 3'd7));

always @(*) begin
    drain_v     = 1'b0;
    drain_is_m1 = 1'b0;
    drain_wslot = 3'd0;
    drain_line  = 128'd0;
    if (m0_rf_now && m0_has_alive) begin
        drain_v     = 1'b1;
        drain_is_m1 = 1'b0;
        drain_wslot = m0_first_alive;
        drain_line  = rf_merged;
    end else if (m1_rf_now && m1_has_alive) begin
        drain_v     = 1'b1;
        drain_is_m1 = 1'b1;
        drain_wslot = m1_first_alive;
        drain_line  = rf_merged;
    end else if (m0_valid && m0_refilled && m0_has_alive) begin
        drain_v     = 1'b1;
        drain_is_m1 = 1'b0;
        drain_wslot = m0_first_alive;
        drain_line  = m0_fill_line;
    end else if (m1_valid && m1_refilled && m1_has_alive) begin
        drain_v     = 1'b1;
        drain_is_m1 = 1'b1;
        drain_wslot = m1_first_alive;
        drain_line  = m1_fill_line;
    end
end

// Backing memory read request arbitration (MSHR0 -> MSHR1)
always @(*) begin
    mem_rd_valid     = 1'b0;
    mem_rd_line_addr = {LADDR_W{1'b0}};
    mem_rd_id        = 1'b0;
    if (m0_valid && !m0_mem_sent && !m0_refilled) begin
        mem_rd_valid     = 1'b1;
        mem_rd_line_addr = m0_laddr;
        mem_rd_id        = 1'b0;
    end else if (m1_valid && !m1_mem_sent && !m1_refilled) begin
        mem_rd_valid     = 1'b1;
        mem_rd_line_addr = m1_laddr;
        mem_rd_id        = 1'b1;
    end
end

// Backing memory WBB writeback port (WBB0 -> WBB1)
assign mem_wr_valid     = wbb0_v || wbb1_v;
assign mem_wr_line_addr = wbb0_v ? wbb0_laddr : wbb1_laddr;
assign mem_wr_data      = wbb0_v ? wbb0_data  : wbb1_data;

// Status & Hardware PMU Observability Pulses
assign dcache_any_miss     = (m0_valid && !m0_refilled) || (m1_valid && !m1_refilled);
assign dcache_writes_idle  = !(m0_valid && (|m0_st_mask)) &&
                             !(m1_valid && (|m1_st_mask)) &&
                             !wbb0_v && !wbb1_v;
assign dcache_idle         = !m0_valid && !m1_valid && !wbb0_v && !wbb1_v &&
                             !hit_resp_valid && !fill_resp_valid;

assign cache_hit           = req_is_hit_act;
assign cache_miss          = req_is_new_miss_act;
assign cache_wb            = rf_need_wb;
assign cache_hum           = req_is_hit_act && dcache_any_miss;
assign cache_mum_cyc       = (m0_valid && !m0_refilled) && (m1_valid && !m1_refilled);
assign cache_mshr_coalesce = req_is_coalesce_act && !req_is_write;

// Helper counts of remaining alive waiters after draining drain_wslot
wire [3:0] m0_alive_after = w_alive[3:0] & ~((drain_v && !drain_is_m1) ? (4'b0001 << drain_wslot[1:0]) : 4'b0000);
wire [3:0] m1_alive_after = w_alive[7:4] & ~((drain_v &&  drain_is_m1) ? (4'b0001 << drain_wslot[1:0]) : 4'b0000);

// ----------------------------------------------------------------------------
// 5. Sequential State Updates
// ----------------------------------------------------------------------------
wire [127:0] hit_cur_line = hit_w0 ? way0_data[req_set] : way1_data[req_set];
integer k;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (k = 0; k < NUM_SETS; k = k + 1) begin
            way0_valid[k] <= 1'b0;
            way0_dirty[k] <= 1'b0;
            way1_valid[k] <= 1'b0;
            way1_dirty[k] <= 1'b0;
            lru_arr[k]    <= 1'b0;
        end
        m0_valid     <= 1'b0; m0_refilled <= 1'b0; m0_mem_sent <= 1'b0;
        m0_laddr     <= {LADDR_W{1'b0}}; m0_st_mask <= 16'd0; m0_st_data <= 128'd0; m0_fill_line <= 128'd0;
        m1_valid     <= 1'b0; m1_refilled <= 1'b0; m1_mem_sent <= 1'b0;
        m1_laddr     <= {LADDR_W{1'b0}}; m1_st_mask <= 16'd0; m1_st_data <= 128'd0; m1_fill_line <= 128'd0;
        for (k = 0; k < 8; k = k + 1) begin
            w_v[k]    <= 1'b0;
            w_id[k]   <= {ID_W{1'b0}};
            w_tag[k]  <= {TAG_W{1'b0}};
            w_woff[k] <= 2'd0;
        end
        wbb0_v       <= 1'b0; wbb0_laddr <= {LADDR_W{1'b0}}; wbb0_data <= 128'd0;
        wbb1_v       <= 1'b0; wbb1_laddr <= {LADDR_W{1'b0}}; wbb1_data <= 128'd0;
        hit_resp_valid  <= 1'b0; hit_resp_id  <= {ID_W{1'b0}}; hit_resp_rdata  <= 32'd0;
        fill_resp_valid <= 1'b0; fill_resp_id <= {ID_W{1'b0}}; fill_resp_rdata <= 32'd0;
    end else begin
        hit_resp_valid  <= 1'b0;
        fill_resp_valid <= 1'b0;

        // 1. Kill squashed MSHR load waiters immediately (harmless line fill still completes!)
        for (k = 0; k < 8; k = k + 1) begin
            if (w_v[k] && !w_alive[k])
                w_v[k] <= 1'b0;
        end

        // 2. WBB Drain to Backing Memory
        if (mem_wr_valid && mem_wr_ready) begin
            if (wbb0_v)
                wbb0_v <= 1'b0;
            else if (wbb1_v)
                wbb1_v <= 1'b0;
        end

        // 3. Mark MSHR Read Request as Sent
        if (mem_rd_valid && mem_rd_ready) begin
            if (!mem_rd_id)
                m0_mem_sent <= 1'b1;
            else
                m1_mem_sent <= 1'b1;
        end

        // 4. Emit Fill Load Response for 1 Waiter & Release Completed MSHR
        if (drain_v) begin
            fill_resp_valid <= 1'b1;
            fill_resp_id    <= w_id[drain_wslot];
            fill_resp_rdata <= get_word(drain_line, w_woff[drain_wslot]);
            w_v[drain_wslot] <= 1'b0;
        end

        // Release refilled MSHRs that have zero remaining alive waiters
        if (m0_valid && m0_refilled && (m0_alive_after == 4'b0000)) begin
            m0_valid    <= 1'b0;
            m0_refilled <= 1'b0;
            m0_mem_sent <= 1'b0;
            m0_st_mask  <= 16'd0;
        end
        if (m1_valid && m1_refilled && (m1_alive_after == 4'b0000)) begin
            m1_valid    <= 1'b0;
            m1_refilled <= 1'b0;
            m1_mem_sent <= 1'b0;
            m1_st_mask  <= 16'd0;
        end

        // 5. Handle Incoming Backing Memory Refill (owns cache array this cycle!)
        if (m0_rf_now || m1_rf_now) begin
            // Install merged 128-bit line into victim way & mark MRU
            if (!rf_vic_way) begin
                way0_valid[rf_set] <= 1'b1;
                way0_dirty[rf_set] <= |rf_stmask;
                way0_tag[rf_set]   <= rf_ctag;
                way0_data[rf_set]  <= rf_merged;
                lru_arr[rf_set]    <= 1'b1; // Way 0 MRU
            end else begin
                way1_valid[rf_set] <= 1'b1;
                way1_dirty[rf_set] <= |rf_stmask;
                way1_tag[rf_set]   <= rf_ctag;
                way1_data[rf_set]  <= rf_merged;
                lru_arr[rf_set]    <= 1'b0; // Way 1 MRU
            end

            // Push dirty victim into guaranteed-free WBB slot
            if (rf_need_wb) begin
                if (!wbb0_v || (mem_wr_valid && mem_wr_ready && wbb0_v)) begin
                    wbb0_v     <= 1'b1;
                    wbb0_laddr <= rf_vic_laddr;
                    wbb0_data  <= rf_vic_data;
                end else begin
                    wbb1_v     <= 1'b1;
                    wbb1_laddr <= rf_vic_laddr;
                    wbb1_data  <= rf_vic_data;
                end
            end

            // Update MSHR status: free immediately if no waiters left, else hold line for draining
            if (m0_rf_now) begin
                m0_fill_line <= rf_merged;
                m0_st_mask   <= 16'd0;
                if (m0_alive_after == 4'b0000) begin
                    m0_valid    <= 1'b0;
                    m0_refilled <= 1'b0;
                    m0_mem_sent <= 1'b0;
                end else begin
                    m0_refilled <= 1'b1;
                end
            end else begin
                m1_fill_line <= rf_merged;
                m1_st_mask   <= 16'd0;
                if (m1_alive_after == 4'b0000) begin
                    m1_valid    <= 1'b0;
                    m1_refilled <= 1'b0;
                    m1_mem_sent <= 1'b0;
                end else begin
                    m1_refilled <= 1'b1;
                end
            end
        end else if (req_valid && req_ready) begin
            // 6. Serve Accepted CPU Request (Hit, Secondary MSHR Coalesce, or New MSHR Miss!)
            if (req_is_hit_act) begin
                lru_arr[req_set] <= hit_w0 ? 1'b1 : 1'b0;
                if (req_is_write) begin
                    if (hit_w0) begin
                        way0_data[req_set]  <= merge_word(hit_cur_line, req_woff, req_wdata, req_wstrb);
                        if (|req_wstrb) way0_dirty[req_set] <= 1'b1;
                    end else begin
                        way1_data[req_set]  <= merge_word(hit_cur_line, req_woff, req_wdata, req_wstrb);
                        if (|req_wstrb) way1_dirty[req_set] <= 1'b1;
                    end
                end else begin
                    hit_resp_valid <= 1'b1;
                    hit_resp_id    <= req_id;
                    hit_resp_rdata <= get_word(hit_cur_line, req_woff);
                end
            end else if (req_is_coalesce_act) begin
                if (m0_match) begin
                    if (req_is_write) begin
                        m0_st_mask <= m0_st_mask | expand_line_mask(req_woff, req_wstrb);
                        m0_st_data <= merge_word(m0_st_data, req_woff, req_wdata, req_wstrb);
                    end else begin
                        w_v[{1'b0, m0_free_widx}]    <= 1'b1;
                        w_id[{1'b0, m0_free_widx}]   <= req_id;
                        w_tag[{1'b0, m0_free_widx}]  <= req_rob_tag;
                        w_woff[{1'b0, m0_free_widx}] <= req_woff;
                    end
                end else begin
                    if (req_is_write) begin
                        m1_st_mask <= m1_st_mask | expand_line_mask(req_woff, req_wstrb);
                        m1_st_data <= merge_word(m1_st_data, req_woff, req_wdata, req_wstrb);
                    end else begin
                        w_v[{1'b1, m1_free_widx}]    <= 1'b1;
                        w_id[{1'b1, m1_free_widx}]   <= req_id;
                        w_tag[{1'b1, m1_free_widx}]  <= req_rob_tag;
                        w_woff[{1'b1, m1_free_widx}] <= req_woff;
                    end
                end
            end else if (req_is_new_miss_act) begin
                if (!m0_valid) begin
                    m0_valid    <= 1'b1;
                    m0_refilled <= 1'b0;
                    m0_mem_sent <= 1'b0;
                    m0_laddr    <= req_laddr;
                    if (req_is_write) begin
                        m0_st_mask <= expand_line_mask(req_woff, req_wstrb);
                        m0_st_data <= merge_word(128'd0, req_woff, req_wdata, req_wstrb);
                    end else begin
                        m0_st_mask <= 16'd0;
                        m0_st_data <= 128'd0;
                        w_v[0]     <= 1'b1;
                        w_id[0]    <= req_id;
                        w_tag[0]   <= req_rob_tag;
                        w_woff[0]  <= req_woff;
                    end
                end else begin
                    m1_valid    <= 1'b1;
                    m1_refilled <= 1'b0;
                    m1_mem_sent <= 1'b0;
                    m1_laddr    <= req_laddr;
                    if (req_is_write) begin
                        m1_st_mask <= expand_line_mask(req_woff, req_wstrb);
                        m1_st_data <= merge_word(128'd0, req_woff, req_wdata, req_wstrb);
                    end else begin
                        m1_st_mask <= 16'd0;
                        m1_st_data <= 128'd0;
                        w_v[4]     <= 1'b1;
                        w_id[4]    <= req_id;
                        w_tag[4]   <= req_rob_tag;
                        w_woff[4]  <= req_woff;
                    end
                end
            end
        end
    end
end

endmodule
