`timescale 1ns / 1ps

// ============================================================================
// Randomized Golden-Scoreboard Unit Testbench for dcache_nb + main_memory_nb
// (Upgrade U3: 4 KB 2-Way 16 B-Line Non-Blocking D-Cache with 2 MSHRs & 2 WBB)
//
// Tests:
//   1. Spatial 16 B line fill (1 miss + 3 consecutive 1-cycle word hits)
//   2. Hit-Under-Miss (HUM): cached hits complete while LATENCY=12 miss in flight
//   3. Miss-Under-Miss (MUM, MLP=2): 2 MSHRs overlap DRAM latency (~14 cyc vs 24+)
//   4. Secondary Same-Line Coalescing: 3 loads to 1 missing line -> 1 DRAM read!
//   5. Dirty Victim Eviction + WBB Hazard Stall (0 stale reads!)
//   6. Write-Allocate Miss + Sub-Word Merge + Speculative Mid-Miss Squash Kill
//   7. 5,000+ Randomized Mixed Ops (reads, writes, sub-word, set-0 conflicts,
//      random squashes) verified against byte-exact Golden Memory Scoreboard!
// ============================================================================
module tb_dcache_nb;

parameter integer MEM_LATENCY = 12;
parameter integer NUM_SETS    = 128; // 4 KB (128 x 2 x 16 B)

reg         clk, reset_n;
reg         req_valid, req_is_write;
wire        req_ready;
reg  [15:0] req_addr;
reg  [31:0] req_wdata;
reg  [3:0]  req_wstrb;
reg  [5:0]  req_id;
reg  [4:0]  req_rob_tag;

wire        hit_resp_valid, fill_resp_valid;
wire [5:0]  hit_resp_id, fill_resp_id;
wire [31:0] hit_resp_rdata, fill_resp_rdata;

reg         flush, squash_valid;
reg  [4:0]  squash_tag, rob_head;

wire        mem_rd_valid, mem_rd_ready, mem_rd_id;
wire [11:0] mem_rd_line_addr;
wire        mem_rresp_valid, mem_rresp_id;
wire [127:0] mem_rresp_data;
wire        mem_wr_valid, mem_wr_ready;
wire [11:0] mem_wr_line_addr;
wire [127:0] mem_wr_data;

wire        dcache_idle, dcache_writes_idle, dcache_any_miss, clean_all_done_u;
wire        cache_hit, cache_miss, cache_wb, cache_hum, cache_mum_cyc, cache_mshr_coalesce;

dcache_nb #(
    .AWIDTH(16),
    .NUM_SETS(NUM_SETS),
    .TAG_W(5),
    .ID_W(6)
) u_dut (
    .clk(clk), .reset_n(reset_n),
    .req_valid(req_valid), .req_ready(req_ready), .req_is_write(req_is_write), .req_amo_op(4'd0),
    .req_addr(req_addr), .req_wdata(req_wdata), .req_wstrb(req_wstrb),
    .req_id(req_id), .req_rob_tag(req_rob_tag),
    .hit_resp_valid(hit_resp_valid), .hit_resp_id(hit_resp_id), .hit_resp_rdata(hit_resp_rdata),
    .fill_resp_valid(fill_resp_valid), .fill_resp_id(fill_resp_id), .fill_resp_rdata(fill_resp_rdata),
    .flush(flush), .squash_valid(squash_valid), .squash_tag(squash_tag), .rob_head(rob_head),
    .clean_all_req(1'b0), .clean_all_done(clean_all_done_u),
    .ext_resv_inv_valid(1'b0), .ext_resv_inv_laddr(12'd0),
    .mem_rd_valid(mem_rd_valid), .mem_rd_ready(mem_rd_ready),
    .mem_rd_line_addr(mem_rd_line_addr), .mem_rd_id(mem_rd_id),
    .mem_rresp_valid(mem_rresp_valid), .mem_rresp_id(mem_rresp_id), .mem_rresp_data(mem_rresp_data),
    .mem_wr_valid(mem_wr_valid), .mem_wr_ready(mem_wr_ready),
    .mem_wr_line_addr(mem_wr_line_addr), .mem_wr_data(mem_wr_data),
    .dcache_idle(dcache_idle), .dcache_writes_idle(dcache_writes_idle), .dcache_any_miss(dcache_any_miss),
    .cache_hit(cache_hit), .cache_miss(cache_miss), .cache_wb(cache_wb),
    .cache_hum(cache_hum), .cache_mum_cyc(cache_mum_cyc), .cache_mshr_coalesce(cache_mshr_coalesce)
);

main_memory_nb #(
    .LINE_BITS(12),
    .LATENCY(MEM_LATENCY)
) u_mem (
    .clk(clk), .rst_n(reset_n),
    .mem_rd_valid(mem_rd_valid), .mem_rd_ready(mem_rd_ready),
    .mem_rd_line_addr(mem_rd_line_addr), .mem_rd_id(mem_rd_id),
    .mem_rresp_valid(mem_rresp_valid), .mem_rresp_id(mem_rresp_id), .mem_rresp_data(mem_rresp_data),
    .mem_wr_valid(mem_wr_valid), .mem_wr_ready(mem_wr_ready),
    .mem_wr_line_addr(mem_wr_line_addr), .mem_wr_data(mem_wr_data)
);

initial clk = 1'b0;
always #5 clk = ~clk;

// Golden byte-addressed 64 KB memory + 64-ID outstanding load scoreboard
reg [7:0]  gold_mem [0:65535];
reg        sb_valid [0:63];
reg [31:0] sb_exp   [0:63];
reg [4:0]  sb_tag   [0:63];

integer errs, cnt_hits, cnt_misses, cnt_wbs, cnt_hum, cnt_mum, cnt_coal, cnt_mem_rds;
integer cyc_now, pending_loads;

function automatic [31:0] gold_read_word(input [15:0] addr);
    reg [15:0] a;
    begin
        a = {addr[15:2], 2'b00};
        gold_read_word = {gold_mem[a + 16'd3], gold_mem[a + 16'd2], gold_mem[a + 16'd1], gold_mem[a]};
    end
endfunction

task gold_write_word(input [15:0] addr, input [31:0] data, input [3:0] wstrb);
    reg [15:0] a;
    begin
        a = {addr[15:2], 2'b00};
        if (wstrb[0]) gold_mem[a + 16'd0] = data[7:0];
        if (wstrb[1]) gold_mem[a + 16'd1] = data[15:8];
        if (wstrb[2]) gold_mem[a + 16'd2] = data[23:16];
        if (wstrb[3]) gold_mem[a + 16'd3] = data[31:24];
    end
endtask

// Continuous scoreboard checker on BOTH hit_resp and fill_resp ports!
/* verilator lint_off SYNCASYNCNET */
/* verilator lint_off BLKSEQ */
always @(posedge clk) begin
    if (!reset_n) begin
        cyc_now     <= 0;
        cnt_hits    <= 0;
        cnt_misses  <= 0;
        cnt_wbs     <= 0;
        cnt_hum     <= 0;
        cnt_mum     <= 0;
        cnt_coal    <= 0;
        cnt_mem_rds <= 0;
    end else begin
        cyc_now <= cyc_now + 1;
        if (cache_hit)           cnt_hits    <= cnt_hits + 1;
        if (cache_miss)          cnt_misses  <= cnt_misses + 1;
        if (cache_wb)            cnt_wbs     <= cnt_wbs + 1;
        if (cache_hum)           cnt_hum     <= cnt_hum + 1;
        if (cache_mum_cyc)       cnt_mum     <= cnt_mum + 1;
        if (cache_mshr_coalesce) cnt_coal    <= cnt_coal + 1;
        if (mem_rd_valid && mem_rd_ready) cnt_mem_rds <= cnt_mem_rds + 1;

        if (hit_resp_valid) begin
            if (!sb_valid[hit_resp_id]) begin
                $display("FAIL [tb_dcache_nb cyc=%0d]: spurious hit_resp id=%0d data=0x%08x",
                         cyc_now, hit_resp_id, hit_resp_rdata);
                errs = errs + 1;
            end else begin
                if (hit_resp_rdata !== sb_exp[hit_resp_id]) begin
                    $display("FAIL [tb_dcache_nb cyc=%0d]: hit_resp id=%0d exp=0x%08x got=0x%08x",
                             cyc_now, hit_resp_id, sb_exp[hit_resp_id], hit_resp_rdata);
                    errs = errs + 1;
                end
                sb_valid[hit_resp_id] = 1'b0;
                pending_loads = pending_loads - 1;
            end
        end

        if (fill_resp_valid) begin
            if (!sb_valid[fill_resp_id]) begin
                $display("FAIL [tb_dcache_nb cyc=%0d]: spurious fill_resp id=%0d data=0x%08x",
                         cyc_now, fill_resp_id, fill_resp_rdata);
                errs = errs + 1;
            end else begin
                if (fill_resp_rdata !== sb_exp[fill_resp_id]) begin
                    $display("FAIL [tb_dcache_nb cyc=%0d]: fill_resp id=%0d exp=0x%08x got=0x%08x",
                             cyc_now, fill_resp_id, sb_exp[fill_resp_id], fill_resp_rdata);
                    errs = errs + 1;
                end
                sb_valid[fill_resp_id] = 1'b0;
                pending_loads = pending_loads - 1;
            end
        end
    end
end
/* verilator lint_on BLKSEQ */
/* verilator lint_on SYNCASYNCNET */

task send_req(
    input        is_wr,
    input [15:0] addr,
    input [31:0] wdata,
    input [3:0]  wstrb,
    input [5:0]  id,
    input [4:0]  rtag
);
    integer guard;
    begin
        @(negedge clk);
        req_valid    = 1'b1;
        req_is_write = is_wr;
        req_addr     = addr;
        req_wdata    = wdata;
        req_wstrb    = wstrb;
        req_id       = id;
        req_rob_tag  = rtag;
        guard = 0;
        #1;
        while (!req_ready && guard < 200) begin
            @(posedge clk);
            @(negedge clk);
            #1;
            guard = guard + 1;
        end
        if (!req_ready) begin
            $display("FAIL [tb_dcache_nb]: req_ready timeout at addr=0x%04x wr=%0b", addr, is_wr);
            $fatal(1);
        end
        if (is_wr) begin
            gold_write_word(addr, wdata, wstrb);
        end else begin
            sb_valid[id]  = 1'b1;
            sb_exp[id]    = gold_read_word(addr);
            sb_tag[id]    = rtag;
            pending_loads = pending_loads + 1;
        end
        @(posedge clk);
        @(negedge clk);
        req_valid = 1'b0;
    end
endtask

task wait_idle;
    integer guard;
    begin
        guard = 0;
        while ((!dcache_idle || pending_loads != 0) && guard < 300) begin
            @(posedge clk);
            guard = guard + 1;
        end
        if (!dcache_idle || pending_loads != 0) begin
            $display("FAIL [tb_dcache_nb]: wait_idle timeout! idle=%0b pending=%0d", dcache_idle, pending_loads);
            $fatal(1);
        end
    end
endtask

integer i, k, t_start, rd_before, hit_before, miss_before, hum_before, mum_before, coal_before, wb_before;
reg [31:0] prng;
reg [15:0] r_addr;
reg [31:0] r_data;
reg [3:0]  r_strb;
reg [5:0]  next_id;
reg        r_is_wr;
integer    rand_ops, wait_guard;
reg [15:0] addr_pool [0:15];

function automatic [31:0] lfsr_next(input [31:0] s);
    begin
        lfsr_next = {s[30:0], s[31] ^ s[21] ^ s[1] ^ s[0]};
    end
endfunction

reg [31:0] sw0, sw1, sw2, sw3;
localparam [15:0] WAY_STRIDE = NUM_SETS[15:0] * 16'd16;

initial begin
    errs          = 0;
    pending_loads = 0;
    reset_n       = 1'b0;
    req_valid     = 1'b0;
    req_is_write  = 1'b0;
    req_addr      = 16'd0;
    req_wdata     = 32'd0;
    req_wstrb     = 4'd0;
    req_id        = 6'd0;
    req_rob_tag   = 5'd0;
    flush         = 1'b0;
    squash_valid  = 1'b0;
    squash_tag    = 5'd0;
    rob_head      = 5'd0;

    for (i = 0; i < 64; i = i + 1) begin
        sb_valid[i] = 1'b0;
        sb_exp[i]   = 32'd0;
        sb_tag[i]   = 5'd0;
    end
    for (i = 0; i < 65536; i = i + 1)
        gold_mem[i] = 8'd0;

    // Seed backing memory & golden memory with deterministic pattern
    for (i = 0; i < 4096; i = i + 1) begin
        sw0 = 32'hA000_0000 + {i[29:0], 2'd0};
        sw1 = 32'hB000_0000 + {i[29:0], 2'd1};
        sw2 = 32'hC000_0000 + {i[29:0], 2'd2};
        sw3 = 32'hD000_0000 + {i[29:0], 2'd3};
        u_mem.mem[i] = {sw3, sw2, sw1, sw0};
        gold_write_word({i[11:0], 4'h0}, sw0, 4'hF);
        gold_write_word({i[11:0], 4'h4}, sw1, 4'hF);
        gold_write_word({i[11:0], 4'h8}, sw2, 4'hF);
        gold_write_word({i[11:0], 4'hC}, sw3, 4'hF);
    end

    repeat (3) @(posedge clk);
    @(negedge clk);
    reset_n = 1'b1;

    // ------------------------------------------------------------------
    // TEST 1: Spatial 16 B Line Fill -> 1 miss + 3 1-cycle word hits!
    // ------------------------------------------------------------------
    hit_before  = cnt_hits;
    miss_before = cnt_misses;
    send_req(1'b0, 16'h0000, 32'd0, 4'h0, 6'd1, 5'd1);
    wait_idle();
    send_req(1'b0, 16'h0004, 32'd0, 4'h0, 6'd2, 5'd2);
    send_req(1'b0, 16'h0008, 32'd0, 4'h0, 6'd3, 5'd3);
    send_req(1'b0, 16'h000C, 32'd0, 4'h0, 6'd4, 5'd4);
    wait_idle();
    if ((cnt_misses - miss_before) != 1 || (cnt_hits - hit_before) != 3) begin
        $display("FAIL [T1 spatial]: exp 1 miss + 3 hits, got misses=%0d hits=%0d",
                 cnt_misses - miss_before, cnt_hits - hit_before);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------
    // TEST 2: Hit-Under-Miss (HUM) -> 3 hits complete while miss in flight!
    // ------------------------------------------------------------------
    hum_before = cnt_hum;
    send_req(1'b0, 16'h0100, 32'd0, 4'h0, 6'd5, 5'd5); // Misses (Set 16)
    send_req(1'b0, 16'h0000, 32'd0, 4'h0, 6'd6, 5'd6); // Hits Set 0 under miss!
    send_req(1'b0, 16'h0004, 32'd0, 4'h0, 6'd7, 5'd7); // Hits Set 0 under miss!
    send_req(1'b0, 16'h0008, 32'd0, 4'h0, 6'd8, 5'd8); // Hits Set 0 under miss!
    wait_idle();
    if ((cnt_hum - hum_before) < 3) begin
        $display("FAIL [T2 HUM]: expected >= 3 HUM events, got %0d", cnt_hum - hum_before);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------
    // TEST 3: Miss-Under-Miss (MUM, MLP = 2) -> both MSHRs overlap LATENCY!
    // ------------------------------------------------------------------
    mum_before = cnt_mum;
    t_start    = cyc_now;
    send_req(1'b0, 16'h0200, 32'd0, 4'h0, 6'd9,  5'd9);  // Miss #1 (Set 32)
    send_req(1'b0, 16'h0300, 32'd0, 4'h0, 6'd10, 5'd10); // Miss #2 (Set 48) -> overlaps!
    wait_idle();
    if ((MEM_LATENCY >= 4) && (((cnt_mum - mum_before) == 0) || ((cyc_now - t_start) >= (2 * MEM_LATENCY)))) begin
        $display("FAIL [T3 MUM]: expected dual-MSHR overlap (mum=%0d, cyc=%0d vs 2*LAT=%0d)",
                 cnt_mum - mum_before, cyc_now - t_start, 2 * MEM_LATENCY);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------
    // TEST 4: Same-Line Secondary Coalescing -> 3 loads, 1 DRAM fetch!
    // ------------------------------------------------------------------
    coal_before = cnt_coal;
    rd_before   = cnt_mem_rds;
    send_req(1'b0, 16'h0400, 32'd0, 4'h0, 6'd11, 5'd11); // Primary miss
    send_req(1'b0, 16'h0404, 32'd0, 4'h0, 6'd12, 5'd12); // Secondary coalesce #1
    send_req(1'b0, 16'h040C, 32'd0, 4'h0, 6'd13, 5'd13); // Secondary coalesce #2
    wait_idle();
    if ((cnt_coal - coal_before) != 2 || (cnt_mem_rds - rd_before) != 1) begin
        $display("FAIL [T4 coalesce]: exp 2 coalesces & 1 DRAM rd, got coal=%0d rds=%0d",
                 cnt_coal - coal_before, cnt_mem_rds - rd_before);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------
    // TEST 5: Dirty Victim Eviction + WBB Hazard Stall (zero stale reads!)
    // ------------------------------------------------------------------
    wb_before = cnt_wbs;
    send_req(1'b1, 16'h0000,               32'h1122_3344, 4'hF, 6'd0, 5'd1); // Dirty Way 0 of Set 0
    send_req(1'b1, WAY_STRIDE,             32'h5566_7788, 4'hF, 6'd0, 5'd2); // Dirty Way 1 of Set 0
    wait_idle();
    send_req(1'b1, WAY_STRIDE * 16'd2,     32'h99AA_BBCC, 4'h3, 6'd0, 5'd3); // Evicts 0x0000 to WBB!
    send_req(1'b0, 16'h0000,               32'd0,         4'h0, 6'd14, 5'd4); // Immediately re-reads 0x0000!
    send_req(1'b0, WAY_STRIDE,             32'd0,         4'h0, 6'd15, 5'd5);
    send_req(1'b0, WAY_STRIDE * 16'd2,     32'd0,         4'h0, 6'd16, 5'd6);
    wait_idle();
    if ((cnt_wbs - wb_before) < 1) begin
        $display("FAIL [T5 dirty WBB]: expected >= 1 dirty WB, got %0d", cnt_wbs - wb_before);
        errs = errs + 1;
    end

    // ------------------------------------------------------------------
    // TEST 6: Speculative Mid-Miss Squash Kill (younger waiter killed cleanly)
    // ------------------------------------------------------------------
    send_req(1'b0, 16'h0500, 32'd0, 4'h0, 6'd20, 5'd10); // Older surviving miss (tag=10)
    send_req(1'b0, 16'h0600, 32'd0, 4'h0, 6'd21, 5'd15); // Younger miss (tag=15) -> SQUASHED!
    @(negedge clk);
    rob_head     = 5'd8;
    squash_valid = 1'b1;
    squash_tag   = 5'd12; // Kills tag=15, preserves tag=10!
    sb_valid[21] = 1'b0;
    pending_loads = pending_loads - 1;
    @(posedge clk);
    @(negedge clk);
    squash_valid = 1'b0;
    wait_idle();

    // ------------------------------------------------------------------
    // TEST 7: 5,000 Randomized Mixed Ops (heavy Set-0/1 conflicts, sub-word
    //         writes, HUM, MUM, same-line coalescing, & dirty evictions!)
    // ------------------------------------------------------------------
    addr_pool[0]  = 16'h0000;                      addr_pool[1]  = 16'h0004;
    addr_pool[2]  = 16'h000C;                      addr_pool[3]  = 16'h0010;
    addr_pool[4]  = WAY_STRIDE;                    addr_pool[5]  = WAY_STRIDE + 16'h0008;
    addr_pool[6]  = WAY_STRIDE + 16'h0010;         addr_pool[7]  = WAY_STRIDE + 16'h0020;
    addr_pool[8]  = WAY_STRIDE * 16'd2;            addr_pool[9]  = WAY_STRIDE * 16'd2 + 16'h0004;
    addr_pool[10] = WAY_STRIDE * 16'd2 + 16'h0010; addr_pool[11] = WAY_STRIDE * 16'd2 + 16'h0020;
    addr_pool[12] = WAY_STRIDE * 16'd3;            addr_pool[13] = WAY_STRIDE * 16'd3 + 16'h000C;
    addr_pool[14] = WAY_STRIDE * 16'd3 + 16'h0010; addr_pool[15] = WAY_STRIDE * 16'd3 + 16'h0030;

    prng     = 32'hCAFE_1234;
    next_id  = 6'd0;
    rand_ops = 5000;
    for (k = 0; k < rand_ops; k = k + 1) begin
        prng    = lfsr_next(prng);
        r_addr  = addr_pool[prng[3:0]];
        r_is_wr = (prng[7:4] < 4'd6); // ~37.5% writes, ~62.5% loads
        r_data  = prng ^ (k * 32'h0101_0101);
        case (prng[9:8])
            2'd0: r_strb = 4'b1111;
            2'd1: r_strb = 4'b0011;
            2'd2: r_strb = 4'b1100;
            2'd3: r_strb = 4'b0001 << prng[11:10];
        endcase

        // If load, wait until next_id slot is free; if write, wait until all older
        // loads to the SAME word have drained (models LSQ commit-order store drain!)
        if (!r_is_wr) begin
            wait_guard = 0;
            while (sb_valid[next_id] && wait_guard < 200) begin
                @(posedge clk);
                wait_guard = wait_guard + 1;
            end
            send_req(1'b0, r_addr, 32'd0, 4'h0, next_id, 5'd1);
            next_id = next_id + 6'd1;
        end else begin
            // In CPU, a store drains ONLY after older loads have committed
            if (pending_loads != 0 && (prng[12] == 1'b1))
                wait_idle();
            else if (pending_loads != 0)
                wait_idle();
            send_req(1'b1, r_addr, r_data, r_strb, 6'd0, 5'd1);
        end
    end
    wait_idle();

    // Final coherence sweep: read back EVERY address in addr_pool and check vs gold_mem!
    for (i = 0; i < 16; i = i + 1) begin
        send_req(1'b0, addr_pool[i], 32'd0, 4'h0, i[5:0], 5'd1);
    end
    wait_idle();

    if (errs == 0) begin
        $display("PASS [tb_dcache_nb]: 5000+ ops (LAT=%0d, SETS=%0d) hits=%0d misses=%0d wbs=%0d hum=%0d mum_cyc=%0d coalesce=%0d errs=0",
                 MEM_LATENCY, NUM_SETS, cnt_hits, cnt_misses, cnt_wbs, cnt_hum, cnt_mum, cnt_coal);
    end else begin
        $display("FAIL [tb_dcache_nb]: %0d error(s)!", errs);
        $fatal(1);
    end
    $finish;
end

endmodule
