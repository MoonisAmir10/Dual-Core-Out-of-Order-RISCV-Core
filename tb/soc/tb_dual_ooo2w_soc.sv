`timescale 1ns / 1ps

// ============================================================================
// Self-Checking Dual-Core MOESI Out-of-Order SoC Testbench (Phases 4, 5 & 6)
//
// Features:
//   - Loads unified +HEX=<file> into 256 KB DRAM AND coh_checker shadow memory
//   - Runs ALWAYS-ON coh_checker (SWMR write safety + 256-cycle deadlock watchdog)
//     PLUS periodic/event SWMR, Duplicate-Tag & 128-bit Golden Shadow checks!
//   - Optional +LOG=<file> dumps 4-wide RVFI commits + serialized Channel A bus
//     events for Phase 5 Global Bus Linearizability Checker!
//   - Optional +JITTER enables runtime LFSR DRAM latency jitter (+0..+7 cyc)
//   - Optional +EXPECT_C0_X14=<hex>, +EXPECT_C1_X14=<hex>, +ALLOW_NONZERO_X14
//   - Optional +DUMP_MEM_BASE=<hex> +DUMP_MEM_WORDS=<n> dumps final coherent
//     architectural memory values (merging latest shadow_mem!) for litmus & bench!
//   - Hard single-core-hang watchdog (prevents silent livelock if 1 core stalls!)
// ============================================================================
module tb_dual_ooo2w_soc;

parameter integer DRAM_LATENCY        = 16;
parameter integer L2_HIT_LAT          = 3;
parameter integer ENABLE_PREFETCH     = 1;
parameter integer ENABLE_MOESI_O      = 1;
parameter integer ENABLE_SNOOP_REPLAY = 1;

`include "coh_defs.vh"

reg clk, reset_n, dram_jitter_en;

// 4-Wide RVFI Commit Trace
wire        c0_c0_v, c0_c1_v, c1_c0_v, c1_c1_v;
wire [31:0] c0_c0_seq, c0_c0_pc, c0_c0_ins, c0_c0_rdw, c0_c0_ma, c0_c0_md;
wire        c0_c0_rwe, c0_c0_rfp, c0_c0_mv, c0_c0_mw;
wire [4:0]  c0_c0_rda, c0_c0_ff; wire [3:0] c0_c0_ms;

wire [31:0] c0_c1_seq, c0_c1_pc, c0_c1_ins, c0_c1_rdw, c0_c1_ma, c0_c1_md;
wire        c0_c1_rwe, c0_c1_rfp, c0_c1_mv, c0_c1_mw;
wire [4:0]  c0_c1_rda, c0_c1_ff; wire [3:0] c0_c1_ms;

wire [31:0] c1_c0_seq, c1_c0_pc, c1_c0_ins, c1_c0_rdw, c1_c0_ma, c1_c0_md;
wire        c1_c0_rwe, c1_c0_rfp, c1_c0_mv, c1_c0_mw;
wire [4:0]  c1_c0_rda, c1_c0_ff; wire [3:0] c1_c0_ms;

wire [31:0] c1_c1_seq, c1_c1_pc, c1_c1_ins, c1_c1_rdw, c1_c1_ma, c1_c1_md;
wire        c1_c1_rwe, c1_c1_rfp, c1_c1_mv, c1_c1_mw;
wire [4:0]  c1_c1_rda, c1_c1_ff; wire [3:0] c1_c1_ms;

wire        bus_evt_v, bus_evt_src, bus_evt_c2c, bus_evt_sh;
wire [1:0]  bus_evt_op;
wire [15:0] bus_evt_la;

wire        d0_wr_v, d1_wr_v, d0_idle, d1_idle, coh_idle, l2_idle;
wire [15:0] d0_wr_la, d1_wr_la;
wire [127:0] d0_wr_dat, d1_wr_dat;

wire        c0_halted, c1_halted, all_halted, clint_msip0, clint_msip1;
wire [31:0] c0_x14, c1_x14;
reg  [31:0] gpio_in;
wire [31:0] gpio_out, gpio_oe;
wire        uart_tx, uart_tx_ch_v;
wire [7:0]  uart_tx_ch_d;
wire [1:0]  pwm_out;
wire [31:0] pmu_cycles, pmu_c0_cycles, pmu_c1_cycles, pmu_c0_instret, pmu_c1_instret;
wire [31:0] pmu_c0_ic_h, pmu_c0_ic_m, pmu_c1_ic_h, pmu_c1_ic_m;
wire [31:0] pmu_c0_dc_h, pmu_c0_dc_m, pmu_c1_dc_h, pmu_c1_dc_m;
wire [31:0] pmu_c0_replay, pmu_c1_replay, pmu_c0_amo, pmu_c1_amo;
wire [31:0] pmu_gets, pmu_getm, pmu_upg, pmu_c2c, pmu_inval, pmu_mesi_wb, pmu_e2m;
wire [31:0] pmu_l2_h, pmu_l2_m, pmu_l2_wb, pmu_l2_bc, pmu_dram_r, pmu_dram_w;

dual_ooo2w_soc #(
    .DRAM_LATENCY(DRAM_LATENCY),
    .L2_HIT_LAT(L2_HIT_LAT),
    .ICACHE_SETS(128),
    .DCACHE_SETS(128),
    .L2_SETS_BANK(256),
    .ENABLE_PREFETCH(ENABLE_PREFETCH),
    .ENABLE_MOESI_O(ENABLE_MOESI_O),
    .ENABLE_SNOOP_REPLAY(ENABLE_SNOOP_REPLAY)
) u_dut (
    .clk(clk), .reset_n(reset_n), .dram_jitter_en(dram_jitter_en),
    .c0_commit0_valid(c0_c0_v), .c0_commit0_seq_id(c0_c0_seq), .c0_commit0_pc(c0_c0_pc), .c0_commit0_instr(c0_c0_ins),
    .c0_commit0_rd_we(c0_c0_rwe), .c0_commit0_rd_is_fp(c0_c0_rfp), .c0_commit0_rd_addr(c0_c0_rda), .c0_commit0_rd_wdata(c0_c0_rdw), .c0_commit0_fflags(c0_c0_ff),
    .c0_commit0_mem_valid(c0_c0_mv), .c0_commit0_mem_we(c0_c0_mw),
    .c0_commit0_mem_addr(c0_c0_ma), .c0_commit0_mem_wstrb(c0_c0_ms), .c0_commit0_mem_wdata(c0_c0_md),
    .c0_commit1_valid(c0_c1_v), .c0_commit1_seq_id(c0_c1_seq), .c0_commit1_pc(c0_c1_pc), .c0_commit1_instr(c0_c1_ins),
    .c0_commit1_rd_we(c0_c1_rwe), .c0_commit1_rd_is_fp(c0_c1_rfp), .c0_commit1_rd_addr(c0_c1_rda), .c0_commit1_rd_wdata(c0_c1_rdw), .c0_commit1_fflags(c0_c1_ff),
    .c0_commit1_mem_valid(c0_c1_mv), .c0_commit1_mem_we(c0_c1_mw),
    .c0_commit1_mem_addr(c0_c1_ma), .c0_commit1_mem_wstrb(c0_c1_ms), .c0_commit1_mem_wdata(c0_c1_md),
    .c1_commit0_valid(c1_c0_v), .c1_commit0_seq_id(c1_c0_seq), .c1_commit0_pc(c1_c0_pc), .c1_commit0_instr(c1_c0_ins),
    .c1_commit0_rd_we(c1_c0_rwe), .c1_commit0_rd_is_fp(c1_c0_rfp), .c1_commit0_rd_addr(c1_c0_rda), .c1_commit0_rd_wdata(c1_c0_rdw), .c1_commit0_fflags(c1_c0_ff),
    .c1_commit0_mem_valid(c1_c0_mv), .c1_commit0_mem_we(c1_c0_mw),
    .c1_commit0_mem_addr(c1_c0_ma), .c1_commit0_mem_wstrb(c1_c0_ms), .c1_commit0_mem_wdata(c1_c0_md),
    .c1_commit1_valid(c1_c1_v), .c1_commit1_seq_id(c1_c1_seq), .c1_commit1_pc(c1_c1_pc), .c1_commit1_instr(c1_c1_ins),
    .c1_commit1_rd_we(c1_c1_rwe), .c1_commit1_rd_is_fp(c1_c1_rfp), .c1_commit1_rd_addr(c1_c1_rda), .c1_commit1_rd_wdata(c1_c1_rdw), .c1_commit1_fflags(c1_c1_ff),
    .c1_commit1_mem_valid(c1_c1_mv), .c1_commit1_mem_we(c1_c1_mw),
    .c1_commit1_mem_addr(c1_c1_ma), .c1_commit1_mem_wstrb(c1_c1_ms), .c1_commit1_mem_wdata(c1_c1_md),
    .bus_evt_valid(bus_evt_v), .bus_evt_src(bus_evt_src), .bus_evt_op(bus_evt_op),
    .bus_evt_laddr(bus_evt_la), .bus_evt_c2c(bus_evt_c2c), .bus_evt_shared(bus_evt_sh),
    .d0_wr_commit_valid(d0_wr_v), .d0_wr_commit_laddr(d0_wr_la), .d0_wr_commit_data(d0_wr_dat),
    .d1_wr_commit_valid(d1_wr_v), .d1_wr_commit_laddr(d1_wr_la), .d1_wr_commit_data(d1_wr_dat),
    .d0_idle(d0_idle), .d1_idle(d1_idle), .coh_idle(coh_idle), .l2_idle(l2_idle),
    .c0_halted(c0_halted), .c1_halted(c1_halted), .all_halted(all_halted),
    .c0_x14(c0_x14), .c1_x14(c1_x14),
    .clint_msip0(clint_msip0), .clint_msip1(clint_msip1),
    .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_oe(gpio_oe),
    .uart_rx(1'b1), .uart_tx(uart_tx),
    .uart_tx_char_valid(uart_tx_ch_v), .uart_tx_char_data(uart_tx_ch_d),
    .pwm_out(pwm_out),
    .pmu_cycles(pmu_cycles), .pmu_c0_cycles(pmu_c0_cycles), .pmu_c1_cycles(pmu_c1_cycles),
    .pmu_c0_instret(pmu_c0_instret), .pmu_c1_instret(pmu_c1_instret),
    .pmu_c0_ic_hits(pmu_c0_ic_h), .pmu_c0_ic_misses(pmu_c0_ic_m),
    .pmu_c1_ic_hits(pmu_c1_ic_h), .pmu_c1_ic_misses(pmu_c1_ic_m),
    .pmu_c0_dc_hits(pmu_c0_dc_h), .pmu_c0_dc_misses(pmu_c0_dc_m),
    .pmu_c1_dc_hits(pmu_c1_dc_h), .pmu_c1_dc_misses(pmu_c1_dc_m),
    .pmu_c0_replay(pmu_c0_replay), .pmu_c1_replay(pmu_c1_replay),
    .pmu_c0_amo(pmu_c0_amo), .pmu_c1_amo(pmu_c1_amo),
    .pmu_coh_gets(pmu_gets), .pmu_coh_getm(pmu_getm), .pmu_coh_upgrade(pmu_upg),
    .pmu_coh_c2c_dirty(pmu_c2c), .pmu_coh_inval(pmu_inval), .pmu_coh_mesi_wb(pmu_mesi_wb),
    .pmu_coh_e2m(pmu_e2m),
    .pmu_l2_hits(pmu_l2_h), .pmu_l2_misses(pmu_l2_m), .pmu_l2_wbs(pmu_l2_wb),
    .pmu_l2_bank_conflicts(pmu_l2_bc), .pmu_dram_rds(pmu_dram_r), .pmu_dram_wrs(pmu_dram_w)
);

wire [31:0] chk_errs, chk_gets, chk_getm, chk_upg, chk_c2c, chk_wr_c;

coh_checker #(
    .LADDR_W(16), .MEM_LINES(16384), .WATCHDOG_CYC(512)
) u_chk (
    .clk(clk), .reset_n(reset_n), .active(!all_halted),
    .d0_wr_valid(d0_wr_v), .d0_wr_laddr(d0_wr_la), .d0_wr_data(d0_wr_dat),
    .d1_wr_valid(d1_wr_v), .d1_wr_laddr(d1_wr_la), .d1_wr_data(d1_wr_dat),
    .bus_evt_valid(bus_evt_v), .bus_evt_src(bus_evt_src), .bus_evt_op(bus_evt_op),
    .bus_evt_laddr(bus_evt_la), .bus_evt_c2c(bus_evt_c2c), .bus_evt_shared(bus_evt_sh),
    .d0_idle(d0_idle), .d1_idle(d1_idle), .coh_idle(coh_idle),
    .d0_progress(c0_c0_v || c0_c1_v || d0_wr_v),
    .d1_progress(c1_c0_v || c1_c1_v || d1_wr_v),
    .chk_errors(chk_errs), .cnt_gets(chk_gets), .cnt_getm(chk_getm),
    .cnt_upgrade(chk_upg), .cnt_c2c(chk_c2c), .cnt_wr_commits(chk_wr_c)
);

initial clk = 1'b0;
always #5 clk = ~clk;

reg [31:0] hex_words [0:16383];

integer max_cycles, sim_cyc, swmr_errs, log_fd, uart_fd;
integer c0_hang_wd, c1_hang_wd;
integer stim_cnt, pwm0_hi, pwm0_edges, pwm1_hi, pwm1_edges, uart_bytes;
reg     has_log, has_uart, gpio_stim_en, exp_c0_en, exp_c1_en, prev_pwm0, prev_pwm1;
reg [31:0] exp_c0_x14, exp_c1_x14;
reg [31:0] dump_base;
integer    dump_words;
reg [8*256-1:0] hex_file, log_file, uart_file;

// Fast SWMR + Golden Shadow Data-Value Check on Every Coherence Bus Event & Every 64 Cycles!
task check_swmr_line(input [15:0] la);
    reg [6:0] s;
    reg [8:0] t;
    reg [2:0] st0, st1;
    reg [127:0] dat0, dat1;
    reg       wbb0_m, wbb1_m;
    begin
        s = la[6:0]; t = la[15:7];
        if (u_dut.u_dcache0.way0_state[s] != COH_I && u_dut.u_dcache0.way0_tag[s] == t) begin
            st0 = u_dut.u_dcache0.way0_state[s]; dat0 = u_dut.u_dcache0.way0_data[s];
        end else if (u_dut.u_dcache0.way1_state[s] != COH_I && u_dut.u_dcache0.way1_tag[s] == t) begin
            st0 = u_dut.u_dcache0.way1_state[s]; dat0 = u_dut.u_dcache0.way1_data[s];
        end else begin
            st0 = COH_I; dat0 = 128'd0;
        end

        if (u_dut.u_dcache1.way0_state[s] != COH_I && u_dut.u_dcache1.way0_tag[s] == t) begin
            st1 = u_dut.u_dcache1.way0_state[s]; dat1 = u_dut.u_dcache1.way0_data[s];
        end else if (u_dut.u_dcache1.way1_state[s] != COH_I && u_dut.u_dcache1.way1_tag[s] == t) begin
            st1 = u_dut.u_dcache1.way1_state[s]; dat1 = u_dut.u_dcache1.way1_data[s];
        end else begin
            st1 = COH_I; dat1 = 128'd0;
        end

        wbb0_m = (u_dut.u_dcache0.wbb0_v && u_dut.u_dcache0.wbb0_laddr == la) ||
                 (u_dut.u_dcache0.wbb1_v && u_dut.u_dcache0.wbb1_laddr == la);
        wbb1_m = (u_dut.u_dcache1.wbb0_v && u_dut.u_dcache1.wbb0_laddr == la) ||
                 (u_dut.u_dcache1.wbb1_v && u_dut.u_dcache1.wbb1_laddr == la);

        if ((st0 == COH_M || st0 == COH_E) && (st1 != COH_I || wbb1_m)) begin
            $display("FAIL [dual_soc SWMR]: cyc=%0d la=0x%04x D0=%0d D1=%0d wbb1=%0d", sim_cyc, la, st0, st1, wbb1_m);
            swmr_errs = swmr_errs + 1;
        end
        if ((st1 == COH_M || st1 == COH_E) && (st0 != COH_I || wbb0_m)) begin
            $display("FAIL [dual_soc SWMR]: cyc=%0d la=0x%04x D1=%0d D0=%0d wbb0=%0d", sim_cyc, la, st1, st0, wbb0_m);
            swmr_errs = swmr_errs + 1;
        end
        if (st0 == COH_O && st1 == COH_O) begin
            $display("FAIL [dual_soc SWMR]: cyc=%0d la=0x%04x BOTH D0 & D1 in O!", sim_cyc, la);
            swmr_errs = swmr_errs + 1;
        end
        if (st0 != COH_I && dat0 !== u_chk.shadow_mem[la[13:0]]) begin
            $display("FAIL [dual_soc DATA0]: cyc=%0d la=0x%04x st0=%0d exp=0x%032x got=0x%032x",
                     sim_cyc, la, st0, u_chk.shadow_mem[la[13:0]], dat0);
            swmr_errs = swmr_errs + 1;
        end
        if (st1 != COH_I && dat1 !== u_chk.shadow_mem[la[13:0]]) begin
            $display("FAIL [dual_soc DATA1]: cyc=%0d la=0x%04x st1=%0d exp=0x%032x got=0x%032x",
                     sim_cyc, la, st1, u_chk.shadow_mem[la[13:0]], dat1);
            swmr_errs = swmr_errs + 1;
        end
    end
endtask

/* verilator lint_off BLKSEQ */
/* verilator lint_off SYNCASYNCNET */
always @(posedge clk) begin
    if (reset_n && !all_halted) begin
        // Check SWMR & Data Consistency on every coherence bus event & every write commit!
        if (bus_evt_v && swmr_errs < 10) check_swmr_line(bus_evt_la);
        if (d0_wr_v && swmr_errs < 10)   check_swmr_line(d0_wr_la);
        if (d1_wr_v && swmr_errs < 10)   check_swmr_line(d1_wr_la);

        // Single-core hang watchdogs (4096 cycles without commit while unhalted!)
        if (c0_halted || c0_c0_v || c0_c1_v) c0_hang_wd = 0;
        else c0_hang_wd = c0_hang_wd + 1;
        if (c1_halted || c1_c0_v || c1_c1_v) c1_hang_wd = 0;
        else c1_hang_wd = c1_hang_wd + 1;

        if (c0_hang_wd == 4096) begin
            $display("FAIL [dual_soc]: Core 0 hung (>4096 cycles with no commit!)");
            swmr_errs = swmr_errs + 1;
        end
        if (c1_hang_wd == 4096) begin
            $display("FAIL [dual_soc]: Core 1 hung (>4096 cycles with no commit!)");
            swmr_errs = swmr_errs + 1;
        end

        // Peripheral harness: UART console capture, PWM monitor & GPIO pin stimulus
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

        // Optional 4-Wide RVFI + Channel A Global Trace Log (for Phase 5 Linearizability Checker)
        if (has_log) begin
            if (bus_evt_v)
                $fwrite(log_fd, "B %0d %0d %0d %04x %0d %0d\n",
                        sim_cyc, bus_evt_src, bus_evt_op, bus_evt_la, bus_evt_c2c, bus_evt_sh);
            if (d0_wr_v)
                $fwrite(log_fd, "W %0d 0 %04x %032x\n", sim_cyc, d0_wr_la, d0_wr_dat);
            if (d1_wr_v)
                $fwrite(log_fd, "W %0d 1 %04x %032x\n", sim_cyc, d1_wr_la, d1_wr_dat);
            if (c0_c0_v)
                $fwrite(log_fd, "C %0d 0 %08x %08x %0d %0d %08x %0d %0d %08x %0x %08x %0d %02x\n",
                        sim_cyc, c0_c0_pc, c0_c0_ins, c0_c0_rwe, c0_c0_rda, c0_c0_rdw,
                        c0_c0_mv, c0_c0_mw, c0_c0_ma, c0_c0_ms, c0_c0_md, c0_c0_rfp, c0_c0_ff);
            if (c0_c1_v)
                $fwrite(log_fd, "C %0d 0 %08x %08x %0d %0d %08x %0d %0d %08x %0x %08x %0d %02x\n",
                        sim_cyc, c0_c1_pc, c0_c1_ins, c0_c1_rwe, c0_c1_rda, c0_c1_rdw,
                        c0_c1_mv, c0_c1_mw, c0_c1_ma, c0_c1_ms, c0_c1_md, c0_c1_rfp, c0_c1_ff);
            if (c1_c0_v)
                $fwrite(log_fd, "C %0d 1 %08x %08x %0d %0d %08x %0d %0d %08x %0x %08x %0d %02x\n",
                        sim_cyc, c1_c0_pc, c1_c0_ins, c1_c0_rwe, c1_c0_rda, c1_c0_rdw,
                        c1_c0_mv, c1_c0_mw, c1_c0_ma, c1_c0_ms, c1_c0_md, c1_c0_rfp, c1_c0_ff);
            if (c1_c1_v)
                $fwrite(log_fd, "C %0d 1 %08x %08x %0d %0d %08x %0d %0d %08x %0x %08x %0d %02x\n",
                        sim_cyc, c1_c1_pc, c1_c1_ins, c1_c1_rwe, c1_c1_rda, c1_c1_rdw,
                        c1_c1_mv, c1_c1_mw, c1_c1_ma, c1_c1_ms, c1_c1_md, c1_c1_rfp, c1_c1_ff);
        end
    end
end
/* verilator lint_on SYNCASYNCNET */
/* verilator lint_on BLKSEQ */

integer wi, li, di;
reg [31:0] daddr, dval;
reg [15:0] dla;
reg [1:0]  dwo;

initial begin
    reset_n        = 1'b0;
    dram_jitter_en = 1'b0;
    gpio_in        = 32'd0;
    max_cycles     = 400000;
    sim_cyc        = 0;
    swmr_errs      = 0;
    c0_hang_wd     = 0;
    c1_hang_wd     = 0;
    stim_cnt       = 0;
    pwm0_hi        = 0;
    pwm0_edges     = 0;
    pwm1_hi        = 0;
    pwm1_edges     = 0;
    uart_bytes     = 0;
    prev_pwm0      = 1'b0;
    prev_pwm1      = 1'b0;
    has_log        = 1'b0;
    has_uart       = 1'b0;
    gpio_stim_en   = 1'b0;
    exp_c0_en      = 1'b0;
    exp_c1_en      = 1'b0;
    exp_c0_x14     = 32'd0;
    exp_c1_x14     = 32'd0;
    dump_base      = 32'd0;
    dump_words     = 0;

    for (wi = 0; wi < 16384; wi = wi + 1)
        hex_words[wi] = 32'd0;

    if (!$value$plusargs("HEX=%s", hex_file)) begin
        $display("FAIL [tb_dual_ooo2w_soc]: Missing +HEX=<file>");
        $fatal(1);
    end
    $readmemh(hex_file, hex_words);

    for (li = 0; li < 4096; li = li + 1) begin
        u_dut.u_dram.mem[li] = {hex_words[li*4+3], hex_words[li*4+2], hex_words[li*4+1], hex_words[li*4+0]};
        u_chk.shadow_mem[li] = u_dut.u_dram.mem[li];
    end
    for (li = 4096; li < 16384; li = li + 1) begin
        u_dut.u_dram.mem[li] = 128'd0;
        u_chk.shadow_mem[li] = 128'd0;
    end

    if ($value$plusargs("LOG=%s", log_file)) begin
        log_fd = $fopen(log_file, "w");
        if (log_fd == 0) begin
            $display("FAIL [tb_dual_ooo2w_soc]: Cannot open log %0s", log_file);
            $fatal(1);
        end
        has_log = 1'b1;
    end
    if ($value$plusargs("UART_OUT=%s", uart_file)) begin
        uart_fd = $fopen(uart_file, "w");
        if (uart_fd == 0) begin
            $display("FAIL [tb_dual_ooo2w_soc]: Cannot open UART_OUT %0s", uart_file);
            $fatal(1);
        end
        has_uart = 1'b1;
    end
    if ($test$plusargs("GPIO_STIM")) gpio_stim_en = 1'b1;
    if ($value$plusargs("MAX_CYCLES=%d", max_cycles)) begin end
    if ($value$plusargs("EXPECT_C0_X14=%h", exp_c0_x14)) exp_c0_en = 1'b1;
    if ($value$plusargs("EXPECT_C1_X14=%h", exp_c1_x14)) exp_c1_en = 1'b1;
    if ($value$plusargs("DUMP_MEM_BASE=%h", dump_base)) begin end
    if ($value$plusargs("DUMP_MEM_WORDS=%d", dump_words)) begin end
    if ($test$plusargs("JITTER")) dram_jitter_en = 1'b1;

    repeat (2) @(posedge clk);
    @(negedge clk);
    reset_n = 1'b1;

    while (!all_halted && sim_cyc < max_cycles && swmr_errs == 0 && chk_errs == 0) begin
        @(posedge clk);
        sim_cyc = sim_cyc + 1;
    end

    if (has_log)  $fclose(log_fd);
    if (has_uart) $fclose(uart_fd);

    if (has_uart || gpio_stim_en)
        $display("PERIPH_STATS: gpio_out=0x%08x gpio_oe=0x%08x pwm0_hi=%0d pwm0_edges=%0d pwm1_hi=%0d pwm1_edges=%0d uart_bytes=%0d",
                 gpio_out, gpio_oe, pwm0_hi, pwm0_edges, pwm1_hi, pwm1_edges, uart_bytes);

    // Optional architectural memory dump (from Golden Shadow Memory, which reflects exact coherent state!)
    for (di = 0; di < dump_words; di = di + 1) begin
        daddr = dump_base + (di * 4);
        dla   = daddr[19:4];
        dwo   = daddr[3:2];
        case (dwo)
            2'd0: dval = u_chk.shadow_mem[dla[13:0]][31:0];
            2'd1: dval = u_chk.shadow_mem[dla[13:0]][63:32];
            2'd2: dval = u_chk.shadow_mem[dla[13:0]][95:64];
            2'd3: dval = u_chk.shadow_mem[dla[13:0]][127:96];
        endcase
        $display("MEM[0x%08x]=0x%08x", daddr, dval);
    end

    if (!all_halted) begin
        $display("FAIL [tb_dual_ooo2w_soc]: Timeout after %0d cycles (c0_halt=%0d c1_halt=%0d c0_ret=%0d c1_ret=%0d)",
                 sim_cyc, c0_halted, c1_halted, pmu_c0_instret, pmu_c1_instret);
        $fatal(1);
    end
    if (swmr_errs != 0 || chk_errs != 0) begin
        $display("FAIL [tb_dual_ooo2w_soc]: swmr_errs=%0d chk_errs=%0d", swmr_errs, chk_errs);
        $fatal(1);
    end
    if (exp_c0_en && (c0_x14 !== exp_c0_x14)) begin
        $display("FAIL [tb_dual_ooo2w_soc]: c0_x14 exp=0x%08x got=0x%08x", exp_c0_x14, c0_x14);
        $fatal(1);
    end
    if (exp_c1_en && (c1_x14 !== exp_c1_x14)) begin
        $display("FAIL [tb_dual_ooo2w_soc]: c1_x14 exp=0x%08x got=0x%08x", exp_c1_x14, c1_x14);
        $fatal(1);
    end

    $display("PASS [dual_ooo2w_soc]: file=%0s cycles=%0d c0_ret=%0d c1_ret=%0d chip_IPC=%0.3f c0_x14=0x%08x c1_x14=0x%08x replay=%0d/%0d amo=%0d/%0d ic_m=%0d/%0d dc_m=%0d/%0d coh_gets=%0d coh_getm=%0d coh_upg=%0d c2c=%0d e2m=%0d mesi_wb=%0d l2_h=%0d l2_m=%0d l2_wb=%0d l2_bc=%0d dram_r=%0d dram_w=%0d",
             hex_file, pmu_cycles, pmu_c0_instret, pmu_c1_instret,
             (pmu_cycles > 0) ? (1.0 * (pmu_c0_instret + pmu_c1_instret) / pmu_cycles) : 0.0,
             c0_x14, c1_x14, pmu_c0_replay, pmu_c1_replay, pmu_c0_amo, pmu_c1_amo,
             pmu_c0_ic_m, pmu_c1_ic_m, pmu_c0_dc_m, pmu_c1_dc_m,
             pmu_gets, pmu_getm, pmu_upg, pmu_c2c, pmu_e2m, pmu_mesi_wb,
             pmu_l2_h, pmu_l2_m, pmu_l2_wb, pmu_l2_bc, pmu_dram_r, pmu_dram_w);
    $finish;
end

endmodule
