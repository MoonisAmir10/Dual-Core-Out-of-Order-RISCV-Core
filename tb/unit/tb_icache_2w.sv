`timescale 1ns / 1ps

// ============================================================================
// Standalone Unit Testbench for 2-Way 16B-Line L1 Instruction Cache (Phase 1)
// Verifies:
//   1. Cold miss fill & 1-cycle 2-instruction/cycle hit delivery (word0+1, word1+2, word2+3)
//   2. Line-boundary single-instruction delivery (PC[3:2]==2'b11 -> valid1=0)
//   3. Next-line stream prefetcher hit & promotion into cache array (0-miss sequential stream!)
//   4. 2-way LRU set-associative conflict eviction (3 lines mapping to set 0)
//   5. Mid-miss branch flush kill & redirect to new line
//   6. FENCE.I flash invalidation & updated memory fetch (self-modifying code proof!)
// ============================================================================
module tb_icache_2w;

localparam integer AWIDTH   = 20;
localparam integer NUM_SETS = 128; // 4 KB -> set stride = 128 * 16 = 2048 (0x800)
localparam integer LATENCY  = 4;

reg         clk;
reg         reset_n;
reg         req_valid;
reg  [31:0] req_pc;
reg         flush;
reg         invalidate_all;

wire        resp_valid0, resp_valid1;
wire [31:0] resp_instr0, resp_instr1;

wire        mem_rd_valid, mem_rd_ready;
wire [15:0] mem_rd_line_addr;
wire        mem_rresp_valid;
wire [127:0] mem_rresp_data;

wire        icache_idle, pmu_ic_hit, pmu_ic_miss, pmu_ic_pf_hit, pmu_ic_stall;

icache_2w #(
    .AWIDTH(AWIDTH),
    .NUM_SETS(NUM_SETS),
    .ENABLE_PREFETCH(1)
) dut (
    .clk(clk), .reset_n(reset_n),
    .req_valid(req_valid), .req_pc(req_pc),
    .flush(flush), .invalidate_all(invalidate_all),
    .resp_valid0(resp_valid0), .resp_instr0(resp_instr0),
    .resp_valid1(resp_valid1), .resp_instr1(resp_instr1),
    .mem_rd_valid(mem_rd_valid), .mem_rd_ready(mem_rd_ready),
    .mem_rd_line_addr(mem_rd_line_addr),
    .mem_rresp_valid(mem_rresp_valid), .mem_rresp_data(mem_rresp_data),
    .icache_idle(icache_idle),
    .pmu_ic_hit(pmu_ic_hit), .pmu_ic_miss(pmu_ic_miss),
    .pmu_ic_pf_hit(pmu_ic_pf_hit), .pmu_ic_stall(pmu_ic_stall)
);

wire mem_rresp_id_u, mem_wr_ready_u;
main_memory_nb #(
    .LINE_BITS(12),
    .LATENCY(LATENCY),
    .ID_W(1)
) u_mem (
    .clk(clk), .rst_n(reset_n),
    .mem_rd_valid(mem_rd_valid), .mem_rd_ready(mem_rd_ready),
    .mem_rd_line_addr(mem_rd_line_addr[11:0]), .mem_rd_id(1'b0),
    .mem_rresp_valid(mem_rresp_valid), .mem_rresp_id(mem_rresp_id_u),
    .mem_rresp_data(mem_rresp_data),
    .mem_wr_valid(1'b0), .mem_wr_ready(mem_wr_ready_u),
    .mem_wr_line_addr(12'd0), .mem_wr_data(128'd0)
);

initial clk = 1'b0;
always #5 clk = ~clk;

integer errors, checks, pf_hits, total_misses;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        pf_hits      <= 0;
        total_misses <= 0;
    end else begin
        if (pmu_ic_pf_hit) pf_hits      <= pf_hits + 1;
        if (pmu_ic_miss)   total_misses <= total_misses + 1;
    end
end

task step_clk;
    begin
        @(posedge clk);
        #1;
    end
endtask

task check(input cond, input [511:0] msg);
    begin
        checks = checks + 1;
        if (!cond) begin
            $display("FAIL [tb_icache_2w]: %0s (pc=0x%08x v0=%0d i0=0x%08x v1=%0d i1=0x%08x)",
                     msg, req_pc, resp_valid0, resp_instr0, resp_valid1, resp_instr1);
            errors = errors + 1;
        end
    end
endtask

task fetch_wait(input [31:0] pc_addr, output integer wait_cycs);
    begin
        req_valid = 1'b1;
        req_pc    = pc_addr;
        #1;
        wait_cycs = 0;
        while (!resp_valid0 && wait_cycs < 50) begin
            step_clk();
            wait_cycs = wait_cycs + 1;
        end
    end
endtask

integer l, wcycs;
reg [31:0] w0, w1, w2, w3;

initial begin
    errors         = 0;
    checks         = 0;
    reset_n        = 1'b0;
    req_valid      = 1'b0;
    req_pc         = 32'd0;
    flush          = 1'b0;
    invalidate_all = 1'b0;

    // Populate backing memory: word at byte address A holds 32'hA000_0000 | A
    for (l = 0; l < 1024; l = l + 1) begin
        w0 = 32'hA000_0000 | (l * 16 + 0);
        w1 = 32'hA000_0000 | (l * 16 + 4);
        w2 = 32'hA000_0000 | (l * 16 + 8);
        w3 = 32'hA000_0000 | (l * 16 + 12);
        u_mem.mem[l] = {w3, w2, w1, w0};
    end

    repeat (2) step_clk();
    reset_n = 1'b1;
    #1;

    // 1. Cold miss at 0x0000 -> fills line 0 & delivers 2 instructions (0x0000, 0x0004)
    fetch_wait(32'h0000_0000, wcycs);
    check(resp_valid0 && resp_valid1 && wcycs > 0, "Cold miss fill at 0x0000");
    check(resp_instr0 == 32'hA000_0000 && resp_instr1 == 32'hA000_0004, "Dual-fetch 0x0000+0x0004");
    step_clk();

    // 2. Immediate 0-stall hit at 0x0008 (word2+word3)
    fetch_wait(32'h0000_0008, wcycs);
    check(wcycs == 0 && resp_valid0 && resp_valid1, "1-cyc hit at 0x0008");
    check(resp_instr0 == 32'hA000_0008 && resp_instr1 == 32'hA000_000C, "Dual-fetch 0x0008+0x000C");
    step_clk();

    // 3. Line boundary at 0x000C -> valid0=1, valid1=0!
    fetch_wait(32'h0000_000C, wcycs);
    check(wcycs == 0 && resp_valid0 && !resp_valid1 && resp_instr0 == 32'hA000_000C, "Line boundary single fetch");
    step_clk();

    // Give background next-line prefetcher a few cycles to finish fetching line 1 (0x0010)
    req_valid = 1'b0;
    repeat (LATENCY + 2) step_clk();

    // 4. Sequential fetch at 0x0010 MUST hit in prefetch buffer with 0 miss stalls!
    fetch_wait(32'h0000_0010, wcycs);
    check(wcycs == 0 && pmu_ic_pf_hit && resp_valid0 && resp_valid1, "Next-line prefetcher 0-stall hit");
    check(resp_instr0 == 32'hA000_0010 && resp_instr1 == 32'hA000_0014, "Prefetch data 0x0010+0x0014");
    step_clk();

    // 5. 2-Way LRU Conflict Test on Set 0 (lines 0x0000, 0x0800, 0x1000)
    // Line 0x0000 is already in Set 0; bring in 0x0800 (way 1), then touch 0x0000 (MRU),
    // then bring in 0x1000 (evicts LRU 0x0800 while keeping 0x0000!)
    fetch_wait(32'h0000_0800, wcycs);
    check(wcycs > 0 && resp_instr0 == 32'hA000_0800, "Fill Set0 way1 0x0800");
    step_clk();

    fetch_wait(32'h0000_0000, wcycs);
    check(wcycs == 0 && resp_instr0 == 32'hA000_0000, "Re-touch 0x0000 MRU");
    step_clk();

    fetch_wait(32'h0000_1000, wcycs);
    check(wcycs > 0 && resp_instr0 == 32'hA000_1000, "Fill 0x1000 evicts LRU 0x0800");
    step_clk();

    fetch_wait(32'h0000_0000, wcycs);
    check(wcycs == 0 && resp_instr0 == 32'hA000_0000, "MRU 0x0000 survived eviction");
    step_clk();

    // 6. Mid-Miss Branch Flush Test: start miss to 0x2000, pulse flush 2 cycles later -> redirect to 0x0000
    req_valid = 1'b1;
    req_pc    = 32'h0000_2000;
    step_clk(); // IDLE -> MISS_REQ
    step_clk(); // MISS_REQ -> MISS_WAIT
    flush     = 1'b1;
    req_pc    = 32'h0000_0000;
    step_clk();
    flush     = 1'b0;
    #1;
    check(resp_valid0 && resp_instr0 == 32'hA000_0000, "Redirect to resident 0x0000 after mid-miss flush");
    req_valid = 1'b0;
    repeat (LATENCY + 2) step_clk();
    check(icache_idle, "Killed miss drained cleanly to IDLE");

    // 7. FENCE.I Self-Modifying Code Proof: modify memory word at 0x0000, pulse invalidate_all, re-fetch!
    u_mem.mem[0][31:0] = 32'hCAFE_BABE;
    invalidate_all     = 1'b1;
    step_clk();
    invalidate_all     = 1'b0;
    fetch_wait(32'h0000_0000, wcycs);
    check(wcycs > 0 && resp_valid0 && resp_instr0 == 32'hCAFE_BABE, "FENCE.I re-fetches patched code 0xCAFEBABE");
    step_clk();

    if (errors == 0)
        $display("PASS [tb_icache_2w]: %0d/%0d checks passed (pf_hits=%0d misses=%0d)",
                 checks, checks, pf_hits, total_misses);
    else begin
        $display("FAIL [tb_icache_2w]: %0d errors out of %0d checks!", errors, checks);
        $fatal(1);
    end
    $finish;
end

endmodule
