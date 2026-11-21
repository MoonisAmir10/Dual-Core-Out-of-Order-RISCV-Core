`timescale 1ns / 1ps

// ============================================================================
// Self-Checking Single-Core 3-Level Hierarchy SoC Testbench (Phase 2)
// Tests Core + 4KB L1I + 4KB L1D + 32KB 4-Way 2-Bank L2 + 256KB DRAM + CLINT!
// ============================================================================
module tb_single_ooo2w_soc;

parameter [31:0]  CORE_ID         = 32'd0;
parameter integer DRAM_LATENCY    = 16;
parameter integer L2_HIT_LAT      = 3;
parameter integer ENABLE_PREFETCH = 1;

reg         clk, reset_n, dram_jitter_en;

wire        commit0_valid, commit1_valid;
wire [31:0] commit0_seq_id, commit0_pc, commit0_instr;
wire        commit0_rd_we;
wire [4:0]  commit0_rd_addr;
wire [31:0] commit0_rd_wdata;
wire        commit0_mem_valid, commit0_mem_we;
wire [31:0] commit0_mem_addr, commit0_mem_wdata;
wire [3:0]  commit0_mem_wstrb;

wire [31:0] commit1_seq_id, commit1_pc, commit1_instr;
wire        commit1_rd_we;
wire [4:0]  commit1_rd_addr;
wire [31:0] commit1_rd_wdata;
wire        commit1_mem_valid, commit1_mem_we;
wire [31:0] commit1_mem_addr, commit1_mem_wdata;
wire [3:0]  commit1_mem_wstrb;

wire        trace_disp0_valid, trace_disp1_valid;
wire [31:0] trace_disp0_seq, trace_disp0_pc, trace_disp0_instr;
wire [31:0] trace_disp1_seq, trace_disp1_pc, trace_disp1_instr;
wire [5:0]  trace_issue_valid;
wire [191:0] trace_issue_seq;
wire [1:0]  trace_cdb_valid;
wire [63:0] trace_cdb_seq;
wire        trace_flush_valid;
wire [31:0] trace_flush_after_seq;

wire        dut_halted, clint_msip0, clint_msip1;
wire [31:0] dut_x14;
reg  [31:0] gpio_in;
wire [31:0] gpio_out, gpio_oe;
wire        uart_tx, uart_tx_ch_v;
wire [7:0]  uart_tx_ch_d;
wire [1:0]  pwm_out;
wire [31:0] pmu_cycles, pmu_instret;
wire [31:0] pmu_ic_hits, pmu_ic_misses, pmu_ic_pf_hits;
wire [31:0] pmu_dcache_hits, pmu_dcache_misses, pmu_dcache_wbs, pmu_dcache_hum, pmu_dcache_mum_cyc;
wire [31:0] pmu_l2_hits, pmu_l2_misses, pmu_l2_wbs, pmu_l2_bank_conflicts;
wire [31:0] pmu_dram_rds, pmu_dram_wrs, pmu_dual_commit_cyc, pmu_ex_bypass_hits;
wire [31:0] pmu_amo_commits, pmu_fence_commits;

wire [31:0] mon_cycles, mon_ooo_issue, mon_ooo_cdb, mon_spec_squash, mon_inv_errors;

single_ooo2w_soc #(
    .CORE_ID(CORE_ID),
    .DRAM_LATENCY(DRAM_LATENCY),
    .L2_HIT_LAT(L2_HIT_LAT),
    .ICACHE_SETS(128),
    .DCACHE_SETS(128),
    .L2_SETS_BANK(256),
    .ENABLE_PREFETCH(ENABLE_PREFETCH)
) u_dut (
    .clk(clk), .reset_n(reset_n), .dram_jitter_en(dram_jitter_en),
    .commit0_valid(commit0_valid), .commit0_seq_id(commit0_seq_id),
    .commit0_pc(commit0_pc), .commit0_instr(commit0_instr),
    .commit0_rd_we(commit0_rd_we), .commit0_rd_addr(commit0_rd_addr), .commit0_rd_wdata(commit0_rd_wdata),
    .commit0_mem_valid(commit0_mem_valid), .commit0_mem_we(commit0_mem_we),
    .commit0_mem_addr(commit0_mem_addr), .commit0_mem_wstrb(commit0_mem_wstrb), .commit0_mem_wdata(commit0_mem_wdata),
    .commit1_valid(commit1_valid), .commit1_seq_id(commit1_seq_id),
    .commit1_pc(commit1_pc), .commit1_instr(commit1_instr),
    .commit1_rd_we(commit1_rd_we), .commit1_rd_addr(commit1_rd_addr), .commit1_rd_wdata(commit1_rd_wdata),
    .commit1_mem_valid(commit1_mem_valid), .commit1_mem_we(commit1_mem_we),
    .commit1_mem_addr(commit1_mem_addr), .commit1_mem_wstrb(commit1_mem_wstrb), .commit1_mem_wdata(commit1_mem_wdata),
    .trace_disp0_valid(trace_disp0_valid), .trace_disp0_seq(trace_disp0_seq),
    .trace_disp0_pc(trace_disp0_pc), .trace_disp0_instr(trace_disp0_instr),
    .trace_disp1_valid(trace_disp1_valid), .trace_disp1_seq(trace_disp1_seq),
    .trace_disp1_pc(trace_disp1_pc), .trace_disp1_instr(trace_disp1_instr),
    .trace_issue_valid(trace_issue_valid), .trace_issue_seq(trace_issue_seq),
    .trace_cdb_valid(trace_cdb_valid), .trace_cdb_seq(trace_cdb_seq),
    .trace_flush_valid(trace_flush_valid), .trace_flush_after_seq(trace_flush_after_seq),
    .halted(dut_halted), .x14(dut_x14),
    .clint_msip0(clint_msip0), .clint_msip1(clint_msip1),
    .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_oe(gpio_oe),
    .uart_rx(1'b1), .uart_tx(uart_tx),
    .uart_tx_char_valid(uart_tx_ch_v), .uart_tx_char_data(uart_tx_ch_d),
    .pwm_out(pwm_out),
    .pmu_cycles(pmu_cycles), .pmu_instret(pmu_instret),
    .pmu_ic_hits(pmu_ic_hits), .pmu_ic_misses(pmu_ic_misses), .pmu_ic_pf_hits(pmu_ic_pf_hits),
    .pmu_dcache_hits(pmu_dcache_hits), .pmu_dcache_misses(pmu_dcache_misses),
    .pmu_dcache_wbs(pmu_dcache_wbs), .pmu_dcache_hum(pmu_dcache_hum), .pmu_dcache_mum_cyc(pmu_dcache_mum_cyc),
    .pmu_l2_hits(pmu_l2_hits), .pmu_l2_misses(pmu_l2_misses),
    .pmu_l2_wbs(pmu_l2_wbs), .pmu_l2_bank_conflicts(pmu_l2_bank_conflicts),
    .pmu_dram_rds(pmu_dram_rds), .pmu_dram_wrs(pmu_dram_wrs),
    .pmu_dual_commit_cyc(pmu_dual_commit_cyc), .pmu_ex_bypass_hits(pmu_ex_bypass_hits),
    .pmu_amo_commits(pmu_amo_commits), .pmu_fence_commits(pmu_fence_commits)
);

ooo_trace_mon2w #(
    .MAX_SEQ(32768),
    .NUM_ISSUE_PORT(6),
    .NUM_CDB_PORT(2),
    .WATCHDOG_CYC(2048)
) u_mon (
    .clk(clk), .reset_n(reset_n), .active(!dut_halted),
    .disp0_valid(trace_disp0_valid), .disp0_seq_id(trace_disp0_seq),
    .disp0_pc(trace_disp0_pc), .disp0_instr(trace_disp0_instr),
    .disp1_valid(trace_disp1_valid), .disp1_seq_id(trace_disp1_seq),
    .disp1_pc(trace_disp1_pc), .disp1_instr(trace_disp1_instr),
    .issue_valid(trace_issue_valid), .issue_seq_id(trace_issue_seq),
    .cdb_valid(trace_cdb_valid), .cdb_seq_id(trace_cdb_seq),
    .commit0_valid(commit0_valid), .commit0_seq_id(commit0_seq_id),
    .commit1_valid(commit1_valid), .commit1_seq_id(commit1_seq_id),
    .flush_valid(trace_flush_valid), .flush_after_seq_id(trace_flush_after_seq),
    .cycle_count(mon_cycles), .ooo_issue_pairs(mon_ooo_issue),
    .ooo_cdb_pairs(mon_ooo_cdb), .spec_squashed_issued(mon_spec_squash),
    .inv_errors(mon_inv_errors)
);

initial clk = 1'b0;
always #5 clk = ~clk;

reg [31:0] hex_words [0:16383];

integer trace_fd, uart_fd, commit_idx, cmp_errors, max_cycles, sim_cyc;
integer stim_cnt, pwm0_hi, pwm0_edges, pwm1_hi, pwm1_edges, uart_bytes;
reg     has_trace, has_uart, gpio_stim_en, expect_x14_en, prev_pwm0, prev_pwm1;
reg [31:0] expect_x14;
reg [8*256-1:0] hex_file, trace_file, uart_file;

task check_commit_lane(
    input integer lane,
    input [31:0]  c_pc,
    input [31:0]  c_instr,
    input         c_rd_we,
    input [4:0]   c_rd_addr,
    input [31:0]  c_rd_wdata,
    input         c_mem_valid,
    input         c_mem_we,
    input [31:0]  c_mem_addr,
    input [3:0]   c_mem_wstrb,
    input [31:0]  c_mem_wdata
);
    integer rc, g_step, g_rd_we, g_rd_addr, g_mv, g_mw, g_hlt;
    reg [31:0] g_pc, g_instr, g_rd_wdata, g_maddr, g_mwdata;
    reg [3:0]  g_mwstrb;
    begin
        if (has_trace) begin
            rc = $fscanf(trace_fd, "%d %h %h %d %d %h %d %d %h %h %h %d\n",
                         g_step, g_pc, g_instr, g_rd_we, g_rd_addr, g_rd_wdata,
                         g_mv, g_mw, g_maddr, g_mwstrb, g_mwdata, g_hlt);
            if (rc != 12) begin
                $display("FAIL [single_soc]: Golden trace EOF early at commit %0d", commit_idx);
                cmp_errors = cmp_errors + 1;
            end else begin
                if (c_pc !== g_pc || c_instr !== g_instr) begin
                    $display("FAIL [single_soc]: commit %0d lane%0d PC/INS exp=(0x%08x,0x%08x) got=(0x%08x,0x%08x)",
                             commit_idx, lane, g_pc, g_instr, c_pc, c_instr);
                    cmp_errors = cmp_errors + 1;
                end
                if (c_rd_we !== g_rd_we[0] ||
                    (g_rd_we[0] && (c_rd_addr !== g_rd_addr[4:0] || c_rd_wdata !== g_rd_wdata))) begin
                    $display("FAIL [single_soc]: commit %0d lane%0d RD pc=0x%08x exp=(we=%0d r%0d=0x%08x) got=(we=%0d r%0d=0x%08x)",
                             commit_idx, lane, g_pc, g_rd_we, g_rd_addr, g_rd_wdata, c_rd_we, c_rd_addr, c_rd_wdata);
                    cmp_errors = cmp_errors + 1;
                end
                if (c_mem_valid !== g_mv[0] ||
                    (g_mv[0] && (c_mem_we !== g_mw[0] || c_mem_addr !== g_maddr)) ||
                    (g_mv[0] && g_mw[0] && (c_mem_wstrb !== g_mwstrb || c_mem_wdata !== g_mwdata))) begin
                    $display("FAIL [single_soc]: commit %0d lane%0d MEM pc=0x%08x exp=(v=%0d w=%0d a=0x%08x s=%0x d=0x%08x) got=(v=%0d w=%0d a=0x%08x s=%0x d=0x%08x)",
                             commit_idx, lane, g_pc, g_mv, g_mw, g_maddr, g_mwstrb, g_mwdata,
                             c_mem_valid, c_mem_we, c_mem_addr, c_mem_wstrb, c_mem_wdata);
                    cmp_errors = cmp_errors + 1;
                end
            end
        end
        commit_idx = commit_idx + 1;
    end
endtask

/* verilator lint_off BLKSEQ */
/* verilator lint_off SYNCASYNCNET */
always @(posedge clk) begin
    if (reset_n && !dut_halted) begin
        if (commit0_valid)
            check_commit_lane(0, commit0_pc, commit0_instr, commit0_rd_we, commit0_rd_addr, commit0_rd_wdata,
                              commit0_mem_valid, commit0_mem_we, commit0_mem_addr, commit0_mem_wstrb, commit0_mem_wdata);
        if (commit1_valid)
            check_commit_lane(1, commit1_pc, commit1_instr, commit1_rd_we, commit1_rd_addr, commit1_rd_wdata,
                              commit1_mem_valid, commit1_mem_we, commit1_mem_addr, commit1_mem_wstrb, commit1_mem_wdata);

        // Peripheral harness: UART console capture, PWM waveform monitor & GPIO stimulus
        if (uart_tx_ch_v) begin
            uart_bytes = uart_bytes + 1;
            if (has_uart) $fwrite(uart_fd, "%c", uart_tx_ch_d);
        end
        if (pwm_out[0]) pwm0_hi = pwm0_hi + 1;
        if (pwm_out[0] && !prev_pwm0) pwm0_edges = pwm0_edges + 1;
        if (pwm_out[1]) pwm1_hi = pwm1_hi + 1;
        if (pwm_out[1] && !prev_pwm1) pwm1_edges = pwm1_edges + 1;
        prev_pwm0 = pwm_out[0];
        prev_pwm1 = pwm_out[1];

        if (gpio_stim_en && gpio_out[15]) begin
            stim_cnt = stim_cnt + 1;
            if (stim_cnt == 64)        gpio_in = 32'h0000_0001; // Rising edge pin 0
            else if (stim_cnt == 1800) gpio_in = 32'h0000_0003; // Rising edge pin 1
            else if (stim_cnt == 3200) gpio_in = 32'h0000_0006; // Rising edge pin 2 (pin 0 low)
        end
    end
end
/* verilator lint_on SYNCASYNCNET */
/* verilator lint_on BLKSEQ */

integer wi, li;

initial begin
    reset_n        = 1'b0;
    dram_jitter_en = 1'b0;
    gpio_in        = 32'd0;
    commit_idx     = 0;
    cmp_errors     = 0;
    stim_cnt       = 0;
    pwm0_hi        = 0;
    pwm0_edges     = 0;
    pwm1_hi        = 0;
    pwm1_edges     = 0;
    uart_bytes     = 0;
    prev_pwm0      = 1'b0;
    prev_pwm1      = 1'b0;
    has_trace      = 1'b0;
    has_uart       = 1'b0;
    gpio_stim_en   = 1'b0;
    expect_x14_en  = 1'b0;
    expect_x14     = 32'd0;
    max_cycles     = 200000;

    for (wi = 0; wi < 16384; wi = wi + 1)
        hex_words[wi] = 32'd0;

    if (!$value$plusargs("HEX=%s", hex_file)) begin
        $display("FAIL [tb_single_ooo2w_soc]: Missing +HEX=<file>");
        $fatal(1);
    end
    $readmemh(hex_file, hex_words);

    for (li = 0; li < 4096; li = li + 1)
        u_dut.u_dram.mem[li] = {hex_words[li*4+3], hex_words[li*4+2], hex_words[li*4+1], hex_words[li*4+0]};
    for (li = 4096; li < 16384; li = li + 1)
        u_dut.u_dram.mem[li] = 128'd0;

    if ($value$plusargs("TRACE=%s", trace_file)) begin
        trace_fd = $fopen(trace_file, "r");
        if (trace_fd == 0) begin
            $display("FAIL [tb_single_ooo2w_soc]: Cannot open trace %0s", trace_file);
            $fatal(1);
        end
        has_trace = 1'b1;
    end
    if ($value$plusargs("UART_OUT=%s", uart_file)) begin
        uart_fd = $fopen(uart_file, "w");
        if (uart_fd == 0) begin
            $display("FAIL [tb_single_ooo2w_soc]: Cannot open UART_OUT %0s", uart_file);
            $fatal(1);
        end
        has_uart = 1'b1;
    end
    if ($test$plusargs("GPIO_STIM")) gpio_stim_en = 1'b1;
    if ($value$plusargs("MAX_CYCLES=%d", max_cycles)) begin end
    if ($value$plusargs("EXPECT_X14=%h", expect_x14)) expect_x14_en = 1'b1;
    if ($test$plusargs("JITTER")) dram_jitter_en = 1'b1;

    repeat (2) @(posedge clk);
    @(negedge clk);
    reset_n = 1'b1;

    sim_cyc = 0;
    while (!dut_halted && sim_cyc < max_cycles) begin
        @(posedge clk);
        sim_cyc = sim_cyc + 1;
        if (cmp_errors > 10) begin
            $display("FAIL [tb_single_ooo2w_soc]: Aborting after >10 RVFI mismatches");
            $fatal(1);
        end
    end

    if (has_trace) $fclose(trace_fd);
    if (has_uart)  $fclose(uart_fd);

    if (has_uart || gpio_stim_en)
        $display("PERIPH_STATS: gpio_out=0x%08x gpio_oe=0x%08x pwm0_hi=%0d pwm0_edges=%0d pwm1_hi=%0d pwm1_edges=%0d uart_bytes=%0d",
                 gpio_out, gpio_oe, pwm0_hi, pwm0_edges, pwm1_hi, pwm1_edges, uart_bytes);

    if (!dut_halted) begin
        $display("FAIL [tb_single_ooo2w_soc]: Timeout after %0d cycles (commits=%0d)", max_cycles, commit_idx);
        $fatal(1);
    end
    if (cmp_errors != 0 || mon_inv_errors != 0) begin
        $display("FAIL [tb_single_ooo2w_soc]: cmp_errors=%0d mon_inv_errors=%0d", cmp_errors, mon_inv_errors);
        $fatal(1);
    end
    if (expect_x14_en && (dut_x14 !== expect_x14)) begin
        $display("FAIL [tb_single_ooo2w_soc]: x14 exp=0x%08x got=0x%08x", expect_x14, dut_x14);
        $fatal(1);
    end

    $display("PASS [single_ooo2w_L2]: file=%0s commits=%0d cycles=%0d IPC=%0.3f ic=%0d/%0d(pf=%0d) dc=%0d/%0d(wb=%0d,hum=%0d,mum=%0d) l2_h=%0d l2_m=%0d l2_wb=%0d l2_bc=%0d dram_r=%0d dram_w=%0d dual_c=%0d ex_byp=%0d x14=0x%08x",
             hex_file, commit_idx, pmu_cycles,
             (pmu_cycles > 0) ? (1.0 * pmu_instret / pmu_cycles) : 0.0,
             pmu_ic_hits, pmu_ic_misses, pmu_ic_pf_hits,
             pmu_dcache_hits, pmu_dcache_misses, pmu_dcache_wbs, pmu_dcache_hum, pmu_dcache_mum_cyc,
             pmu_l2_hits, pmu_l2_misses, pmu_l2_wbs, pmu_l2_bank_conflicts,
             pmu_dram_rds, pmu_dram_wrs, pmu_dual_commit_cyc, pmu_ex_bypass_hits, dut_x14);
    $finish;
end

endmodule
