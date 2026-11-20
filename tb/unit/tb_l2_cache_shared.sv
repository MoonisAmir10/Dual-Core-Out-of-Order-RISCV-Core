`timescale 1ns / 1ps

// ============================================================================
// Dual-Master Randomized Golden-Scoreboard TB for l2_cache_shared + main_memory_dram
// (Phase 2: 32 KB 4-Way 2-Bank Line-Interleaved NINE L2 + 4-Outstanding L16 DRAM)
//
// Tests:
//   1. Parallel same-cycle dual-bank hits (M0 -> even Bank 0, M1 -> odd Bank 1, 0 conflicts!)
//   2. Same-bank conflict arbitration (M0 + M1 -> Bank 0 simultaneously, lossless!)
//   3. 4-Way Tree-PLRU conflict eviction (5 lines mapping to Bank 0 Set 0 -> dirty WB!)
//   4. 4x DRAM MLP: 4 simultaneous misses across both banks + secondary MSHR coalescing
//   5. 5,000+ Randomized Dual-Master Mixed Reads & Full-Line Dirty Writebacks with
//      DRAM latency jitter verified against a 128-bit Golden Memory Scoreboard!
// ============================================================================
module tb_l2_cache_shared;

localparam integer LADDR_W       = 16;
localparam integer SETS_PER_BANK = 256; // 32 KB total L2 (2 x 16 KB)
localparam integer M_ID_W        = 3;
localparam integer HIT_LAT       = 3;
localparam integer DRAM_LAT      = 12;

reg                 clk, reset_n, jitter_en;

reg                 m0_req_valid, m0_req_is_write;
wire                m0_req_ready;
reg  [LADDR_W-1:0]  m0_req_laddr;
reg  [127:0]        m0_req_wdata;
reg  [M_ID_W-1:0]   m0_req_id;
wire                m0_resp_valid;
wire [M_ID_W-1:0]   m0_resp_id;
wire [127:0]        m0_resp_data;

reg                 m1_req_valid, m1_req_is_write;
wire                m1_req_ready;
reg  [LADDR_W-1:0]  m1_req_laddr;
reg  [127:0]        m1_req_wdata;
reg  [M_ID_W-1:0]   m1_req_id;
wire                m1_resp_valid;
wire [M_ID_W-1:0]   m1_resp_id;
wire [127:0]        m1_resp_data;

wire                dram_rd_valid, dram_rd_ready, dram_rresp_valid;
wire [LADDR_W-1:0]  dram_rd_laddr;
wire [1:0]          dram_rd_id, dram_rresp_id;
wire [127:0]        dram_rresp_data;

wire                dram_wr_valid, dram_wr_ready;
wire [LADDR_W-1:0]  dram_wr_laddr;
wire [127:0]        dram_wr_data;

wire                l2_idle, l2_bconf, dram_rd_p, dram_wr_p;
wire [1:0]          l2_hits, l2_misses, l2_wbs;

l2_cache_shared #(
    .LADDR_W(LADDR_W),
    .SETS_PER_BANK(SETS_PER_BANK),
    .M_ID_W(M_ID_W),
    .HIT_LAT(HIT_LAT)
) u_l2 (
    .clk(clk), .reset_n(reset_n),
    .m0_req_valid(m0_req_valid), .m0_req_ready(m0_req_ready),
    .m0_req_is_write(m0_req_is_write), .m0_req_laddr(m0_req_laddr),
    .m0_req_wdata(m0_req_wdata), .m0_req_id(m0_req_id),
    .m0_resp_valid(m0_resp_valid), .m0_resp_id(m0_resp_id), .m0_resp_data(m0_resp_data),
    .m1_req_valid(m1_req_valid), .m1_req_ready(m1_req_ready),
    .m1_req_is_write(m1_req_is_write), .m1_req_laddr(m1_req_laddr),
    .m1_req_wdata(m1_req_wdata), .m1_req_id(m1_req_id),
    .m1_resp_valid(m1_resp_valid), .m1_resp_id(m1_resp_id), .m1_resp_data(m1_resp_data),
    .dram_rd_valid(dram_rd_valid), .dram_rd_ready(dram_rd_ready),
    .dram_rd_laddr(dram_rd_laddr), .dram_rd_id(dram_rd_id),
    .dram_rresp_valid(dram_rresp_valid), .dram_rresp_id(dram_rresp_id), .dram_rresp_data(dram_rresp_data),
    .dram_wr_valid(dram_wr_valid), .dram_wr_ready(dram_wr_ready),
    .dram_wr_laddr(dram_wr_laddr), .dram_wr_data(dram_wr_data),
    .l2_idle(l2_idle),
    .pmu_l2_hits(l2_hits), .pmu_l2_misses(l2_misses),
    .pmu_l2_wbs(l2_wbs), .pmu_l2_bank_conflict(l2_bconf)
);

main_memory_dram #(
    .LINE_BITS(14),
    .LATENCY(DRAM_LAT),
    .ID_W(2)
) u_dram (
    .clk(clk), .rst_n(reset_n), .jitter_en(jitter_en),
    .mem_rd_valid(dram_rd_valid), .mem_rd_ready(dram_rd_ready),
    .mem_rd_line_addr(dram_rd_laddr[13:0]), .mem_rd_id(dram_rd_id),
    .mem_rresp_valid(dram_rresp_valid), .mem_rresp_id(dram_rresp_id), .mem_rresp_data(dram_rresp_data),
    .mem_wr_valid(dram_wr_valid), .mem_wr_ready(dram_wr_ready),
    .mem_wr_line_addr(dram_wr_laddr[13:0]), .mem_wr_data(dram_wr_data),
    .pmu_dram_rd(dram_rd_p), .pmu_dram_wr(dram_wr_p)
);

initial clk = 1'b0;
always #5 clk = ~clk;

// Golden 128-bit memory model (16384 lines = 256 KB) + per-master outstanding read scoreboards
reg [127:0] gold_mem [0:16383];
reg         sb0_v    [0:7];
reg [127:0] sb0_exp  [0:7];
reg [15:0]  sb0_la   [0:7];

reg         sb1_v    [0:7];
reg [127:0] sb1_exp  [0:7];
reg [15:0]  sb1_la   [0:7];

integer errs, cnt_hits, cnt_misses, cnt_wbs, cnt_bconf, cnt_dual_acc, cnt_ops;
integer pending0, pending1;

function automatic has_inflight_addr(input [15:0] la);
    integer q;
    begin
        has_inflight_addr = 1'b0;
        for (q = 0; q < 8; q = q + 1) begin
            if ((sb0_v[q] && sb0_la[q] == la) || (sb1_v[q] && sb1_la[q] == la))
                has_inflight_addr = 1'b1;
        end
    end
endfunction

/* verilator lint_off BLKSEQ */
/* verilator lint_off SYNCASYNCNET */
always @(posedge clk) begin
    if (reset_n) begin
        cnt_hits   = cnt_hits   + {30'd0, l2_hits};
        cnt_misses = cnt_misses + {30'd0, l2_misses};
        cnt_wbs    = cnt_wbs    + {30'd0, l2_wbs};
        if (l2_bconf) cnt_bconf = cnt_bconf + 1;
        if (m0_req_valid && m0_req_ready && m1_req_valid && m1_req_ready)
            cnt_dual_acc = cnt_dual_acc + 1;

        if (m0_resp_valid) begin
            if (!sb0_v[m0_resp_id]) begin
                $display("FAIL [tb_l2]: Unexpected M0 response id=%0d", m0_resp_id);
                errs = errs + 1;
            end else begin
                if (m0_resp_data !== sb0_exp[m0_resp_id]) begin
                    $display("FAIL [tb_l2]: M0 data mismatch la=0x%04x exp=0x%032x got=0x%032x",
                             sb0_la[m0_resp_id], sb0_exp[m0_resp_id], m0_resp_data);
                    errs = errs + 1;
                end
                sb0_v[m0_resp_id] = 1'b0;
                pending0 = pending0 - 1;
            end
        end

        if (m1_resp_valid) begin
            if (!sb1_v[m1_resp_id]) begin
                $display("FAIL [tb_l2]: Unexpected M1 response id=%0d", m1_resp_id);
                errs = errs + 1;
            end else begin
                if (m1_resp_data !== sb1_exp[m1_resp_id]) begin
                    $display("FAIL [tb_l2]: M1 data mismatch la=0x%04x exp=0x%032x got=0x%032x",
                             sb1_la[m1_resp_id], sb1_exp[m1_resp_id], m1_resp_data);
                    errs = errs + 1;
                end
                sb1_v[m1_resp_id] = 1'b0;
                pending1 = pending1 - 1;
            end
        end
    end
end
/* verilator lint_on SYNCASYNCNET */
/* verilator lint_on BLKSEQ */

integer i, k, seed, wait_c;
reg [31:0] r0, r1, r2, r3;
reg [15:0] la0, la1;
reg [127:0] wd0, wd1;
reg        wr0, wr1;
reg [2:0]  id0, id1;
reg        f0, f1;

task wait_all_idle;
    begin
        m0_req_valid = 1'b0;
        m1_req_valid = 1'b0;
        wait_c = 0;
        while ((pending0 > 0 || pending1 > 0 || !l2_idle) && wait_c < 2000) begin
            @(posedge clk); #1;
            wait_c = wait_c + 1;
        end
        if (wait_c >= 2000) begin
            $display("FAIL [tb_l2]: Deadlock waiting for idle! pend0=%0d pend1=%0d l2_idle=%0d",
                     pending0, pending1, l2_idle);
            $fatal(1);
        end
    end
endtask

initial begin
    errs         = 0;
    cnt_hits     = 0;
    cnt_misses   = 0;
    cnt_wbs      = 0;
    cnt_bconf    = 0;
    cnt_dual_acc = 0;
    cnt_ops      = 0;
    pending0     = 0;
    pending1     = 0;
    reset_n      = 1'b0;
    jitter_en    = 1'b0;
    m0_req_valid = 1'b0; m0_req_is_write = 1'b0; m0_req_laddr = 16'd0; m0_req_wdata = 128'd0; m0_req_id = 3'd0;
    m1_req_valid = 1'b0; m1_req_is_write = 1'b0; m1_req_laddr = 16'd0; m1_req_wdata = 128'd0; m1_req_id = 3'd0;

    for (k = 0; k < 8; k = k + 1) begin
        sb0_v[k] = 1'b0; sb0_exp[k] = 128'd0; sb0_la[k] = 16'd0;
        sb1_v[k] = 1'b0; sb1_exp[k] = 128'd0; sb1_la[k] = 16'd0;
    end

    // Initialize 256 KB DRAM & golden memory with unique patterns
    for (i = 0; i < 16384; i = i + 1) begin
        r0 = 32'h1000_0000 ^ (i * 32'h0001_0003);
        r1 = 32'h2000_0000 ^ (i * 32'h0001_0005);
        r2 = 32'h3000_0000 ^ (i * 32'h0001_0007);
        r3 = 32'h4000_0000 ^ (i * 32'h0001_0009);
        gold_mem[i]   = {r3, r2, r1, r0};
        u_dram.mem[i] = {r3, r2, r1, r0};
    end

    repeat (3) @(posedge clk);
    @(negedge clk);
    reset_n = 1'b1;

    // ------------------------------------------------------------------------
    // Directed Test 1: 4-Way Tree-PLRU Dirty Eviction on Bank 0 Set 0
    //   Bank 0 lines have laddr[0]=0; Set 0 has laddr[8:1]=0 -> stride = 512 lines!
    //   Write 5 dirty lines (0, 512, 1024, 1536, 2048) -> 5th evicts dirty way 0 to DRAM!
    //   Then read back all 5 -> must match exact dirty values!
    // ------------------------------------------------------------------------
    for (i = 0; i < 5; i = i + 1) begin
        la0 = i[15:0] * 16'd512;
        wd0 = {32'hDEAD_0000 | i[31:0], 32'hBEEF_0000 | i[31:0], 32'hCAFE_0000 | i[31:0], 32'hFACE_0000 | i[31:0]};
        @(negedge clk);
        m0_req_valid    = 1'b1;
        m0_req_is_write = 1'b1;
        m0_req_laddr    = la0;
        m0_req_wdata    = wd0;
        m0_req_id       = 3'd0;
        #1;
        while (!m0_req_ready) begin
            @(negedge clk);
            #1;
        end
        gold_mem[la0[13:0]] = wd0;
        @(posedge clk);
    end
    @(negedge clk);
    m0_req_valid = 1'b0;
    wait_all_idle();

    if (cnt_wbs < 1) begin
        $display("FAIL [tb_l2]: Expected >= 1 dirty L2 eviction WB, got %0d", cnt_wbs);
        errs = errs + 1;
    end

    // Read all 5 lines back (including the evicted line 0 from DRAM!)
    for (i = 0; i < 5; i = i + 1) begin
        la0 = i[15:0] * 16'd512;
        @(negedge clk);
        m0_req_valid    = 1'b1;
        m0_req_is_write = 1'b0;
        m0_req_laddr    = la0;
        m0_req_id       = i[2:0];
        #1;
        while (!m0_req_ready) begin
            @(negedge clk);
            #1;
        end
        sb0_v[i[2:0]]   = 1'b1;
        sb0_la[i[2:0]]  = la0;
        sb0_exp[i[2:0]] = gold_mem[la0[13:0]];
        pending0        = pending0 + 1;
        @(posedge clk);
    end
    @(negedge clk);
    m0_req_valid = 1'b0;
    wait_all_idle();

    // ------------------------------------------------------------------------
    // Directed Test 2: Simultaneous Same-Cycle Parallel Hits on Bank 0 & Bank 1!
    //   Warm line 2 (Bank 0) & line 3 (Bank 1), then fire M0(line 2) + M1(line 3) together!
    // ------------------------------------------------------------------------
    @(negedge clk);
    m0_req_valid = 1'b1; m0_req_is_write = 1'b0; m0_req_laddr = 16'd2; m0_req_id = 3'd0;
    m1_req_valid = 1'b1; m1_req_is_write = 1'b0; m1_req_laddr = 16'd3; m1_req_id = 3'd0;
    sb0_v[0] = 1'b1; sb0_la[0] = 16'd2; sb0_exp[0] = gold_mem[2]; pending0 = pending0 + 1;
    sb1_v[0] = 1'b1; sb1_la[0] = 16'd3; sb1_exp[0] = gold_mem[3]; pending1 = pending1 + 1;
    @(posedge clk);
    @(negedge clk);
    m0_req_valid = 1'b0; m1_req_valid = 1'b0;
    wait_all_idle();

    // Now hit both banks simultaneously in 1 cycle!
    @(negedge clk);
    m0_req_valid = 1'b1; m0_req_is_write = 1'b0; m0_req_laddr = 16'd2; m0_req_id = 3'd1;
    m1_req_valid = 1'b1; m1_req_is_write = 1'b0; m1_req_laddr = 16'd3; m1_req_id = 3'd1;
    #1;
    if (!(m0_req_ready && m1_req_ready && !l2_bconf)) begin
        $display("FAIL [tb_l2]: Dual-bank parallel hit failed! rdy0=%0d rdy1=%0d bconf=%0d",
                 m0_req_ready, m1_req_ready, l2_bconf);
        errs = errs + 1;
    end
    sb0_v[1] = 1'b1; sb0_la[1] = 16'd2; sb0_exp[1] = gold_mem[2]; pending0 = pending0 + 1;
    sb1_v[1] = 1'b1; sb1_la[1] = 16'd3; sb1_exp[1] = gold_mem[3]; pending1 = pending1 + 1;
    @(posedge clk);
    @(negedge clk);
    m0_req_valid = 1'b0; m1_req_valid = 1'b0;
    wait_all_idle();

    // ------------------------------------------------------------------------
    // Randomized Dual-Master Stress: 5,000+ Concurrent Ops with DRAM Jitter!
    //   Mixes hot set-0 conflicts (forcing heavy 4-way PLRU dirty WBs!),
    //   same-bank collisions, parallel dual-bank hits, and 4x DRAM MLP!
    // ------------------------------------------------------------------------
    jitter_en = 1'b1;
    seed      = 32'h2026_CAFE;

    while (cnt_ops < 5000 && errs == 0) begin
        @(negedge clk);
        // Pick free ID for M0
        f0 = 1'b0; id0 = 3'd0;
        for (k = 0; k < 8; k = k + 1)
            if (!sb0_v[k] && !f0) begin f0 = 1'b1; id0 = k[2:0]; end
        // Pick free ID for M1
        f1 = 1'b0; id1 = 3'd0;
        for (k = 0; k < 8; k = k + 1)
            if (!sb1_v[k] && !f1) begin f1 = 1'b1; id1 = k[2:0]; end

        seed = seed * 32'd1664525 + 32'd1013904223;
        // 50% hot set-0/set-1 conflict addresses (stride 512), 50% small working set (0..63)
        if (seed[31])
            la0 = ({13'd0, seed[2:0]} * 16'd512) | {15'd0, seed[3]};
        else
            la0 = {10'd0, seed[5:0]};
        wr0 = (seed[10:8] == 3'd0); // ~12.5% dirty L1 writebacks
        wd0 = {seed ^ 32'hA5A5_1111, seed ^ 32'h5A5A_2222, seed ^ 32'h1234_3333, seed ^ 32'h9876_4444};

        seed = seed * 32'd1664525 + 32'd1013904223;
        if (seed[31])
            la1 = ({13'd0, seed[2:0]} * 16'd512) | {15'd0, seed[3]};
        else
            la1 = {10'd0, seed[5:0]};
        wr1 = (seed[10:8] == 3'd0);
        wd1 = {seed ^ 32'hF0F0_5555, seed ^ 32'h0F0F_6666, seed ^ 32'h7777_8888, seed ^ 32'h3333_9999};

        // Never issue a write to an address that currently has an in-flight read (preserves deterministic order)
        // and never issue both M0 & M1 writing the exact same line in the exact same cycle
        m0_req_valid    = 1'b0;
        m0_req_is_write = wr0;
        m0_req_laddr    = la0;
        m0_req_wdata    = wd0;
        m0_req_id       = id0;
        if (wr0) begin
            if (!has_inflight_addr(la0)) m0_req_valid = 1'b1;
        end else if (f0) begin
            m0_req_valid = 1'b1;
        end

        m1_req_valid    = 1'b0;
        m1_req_is_write = wr1;
        m1_req_laddr    = la1;
        m1_req_wdata    = wd1;
        m1_req_id       = id1;
        if (wr1) begin
            if (!has_inflight_addr(la1) && !(m0_req_valid && la0 == la1)) m1_req_valid = 1'b1;
        end else if (f1 && !(m0_req_valid && wr0 && la0 == la1)) begin
            m1_req_valid = 1'b1;
        end

        #1;
        if (m0_req_valid && m0_req_ready) begin
            cnt_ops = cnt_ops + 1;
            if (wr0) begin
                gold_mem[la0[13:0]] = wd0;
            end else begin
                sb0_v[id0]   = 1'b1;
                sb0_la[id0]  = la0;
                sb0_exp[id0] = gold_mem[la0[13:0]];
                pending0     = pending0 + 1;
            end
        end
        if (m1_req_valid && m1_req_ready) begin
            cnt_ops = cnt_ops + 1;
            if (wr1) begin
                gold_mem[la1[13:0]] = wd1;
            end else begin
                sb1_v[id1]   = 1'b1;
                sb1_la[id1]  = la1;
                sb1_exp[id1] = gold_mem[la1[13:0]];
                pending1     = pending1 + 1;
            end
        end
        @(posedge clk);
    end

    @(negedge clk);
    m0_req_valid = 1'b0;
    m1_req_valid = 1'b0;
    wait_all_idle();

    if (errs == 0 && cnt_dual_acc > 100 && cnt_bconf > 50 && cnt_wbs > 20) begin
        $display("PASS [tb_l2_cache_shared]: %0d ops (32KB 4-way 2-bank L2, DRAM=L%0d+jitter) hits=%0d misses=%0d wbs=%0d dual_bank_acc=%0d conflicts=%0d errs=0",
                 cnt_ops, DRAM_LAT, cnt_hits, cnt_misses, cnt_wbs, cnt_dual_acc, cnt_bconf);
    end else begin
        $display("FAIL [tb_l2_cache_shared]: errs=%0d ops=%0d dual_acc=%0d bconf=%0d wbs=%0d",
                 errs, cnt_ops, cnt_dual_acc, cnt_bconf, cnt_wbs);
        $fatal(1);
    end
    $finish;
end

endmodule
