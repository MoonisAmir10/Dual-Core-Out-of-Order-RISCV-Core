`timescale 1ns / 1ps

// ============================================================================
// Self-Checking 2-Wide Superscalar Lockstep Golden ISS + OoO Proof TB
// ============================================================================
module tb_ooo2w_soc;

parameter integer MEM_LATENCY = 4;

reg clk;
reg reset_n;

wire         commit0_valid, commit1_valid;
wire [31:0]  commit0_seq_id, commit0_pc, commit0_instr;
wire         commit0_rd_we;
wire [4:0]   commit0_rd_addr;
wire [31:0]  commit0_rd_wdata;
wire         commit0_mem_valid, commit0_mem_we;
wire [31:0]  commit0_mem_addr, commit0_mem_wdata;
wire [3:0]   commit0_mem_wstrb;

wire [31:0]  commit1_seq_id, commit1_pc, commit1_instr;
wire         commit1_rd_we;
wire [4:0]   commit1_rd_addr;
wire [31:0]  commit1_rd_wdata;
wire         commit1_mem_valid, commit1_mem_we;
wire [31:0]  commit1_mem_addr, commit1_mem_wdata;
wire [3:0]   commit1_mem_wstrb;

wire         trace_disp0_valid, trace_disp1_valid;
wire [31:0]  trace_disp0_seq, trace_disp0_pc, trace_disp0_instr;
wire [31:0]  trace_disp1_seq, trace_disp1_pc, trace_disp1_instr;
wire [5:0]   trace_issue_valid;
wire [191:0] trace_issue_seq;
wire [1:0]   trace_cdb_valid;
wire [63:0]  trace_cdb_seq;
wire         trace_flush_valid;
wire [31:0]  trace_flush_after_seq;

wire         dut_halted;
wire [31:0]  dut_x14;
wire [31:0]  pmu_cycles, pmu_instret;
wire [31:0]  pmu_dcache_hits, pmu_dcache_misses, pmu_dcache_wbs, pmu_stlf_hits;
wire [31:0]  pmu_dcache_hum, pmu_dcache_mum_cyc, pmu_mshr_coalesce;
wire [31:0]  pmu_br_total, pmu_br_mispred, pmu_br_fast_squash;
wire [31:0]  pmu_stall_rob_full, pmu_stall_rs_full, pmu_stall_lsq_full, pmu_stall_cp_full;
wire [31:0]  pmu_war_waw_renamed, pmu_div_overlap_cyc, pmu_dcache_miss_overlap_cyc;
wire [31:0]  pmu_dual_disp_cyc, pmu_dual_cdb_cyc, pmu_dual_commit_cyc, pmu_ex_bypass_hits;
wire [31:0]  pmu_ras_preds, pmu_ras_mispreds;

wire         iss_halted_u;
wire [31:0]  iss_cc_u, iss_ec_u;
wire [31:0]  mon_cycles, mon_ooo_issue, mon_ooo_cdb, mon_spec_squash, mon_inv_errors;

ooo2w_soc #(
    .IMEM_WORDS(1024),
    .MEM_LATENCY(MEM_LATENCY)
) u_dut (
    .clk(clk), .reset_n(reset_n),
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
    .pmu_cycles(pmu_cycles), .pmu_instret(pmu_instret),
    .pmu_dcache_hits(pmu_dcache_hits), .pmu_dcache_misses(pmu_dcache_misses),
    .pmu_dcache_wbs(pmu_dcache_wbs), .pmu_dcache_hum(pmu_dcache_hum),
    .pmu_dcache_mum_cyc(pmu_dcache_mum_cyc), .pmu_mshr_coalesce(pmu_mshr_coalesce),
    .pmu_stlf_hits(pmu_stlf_hits),
    .pmu_br_total(pmu_br_total), .pmu_br_mispred(pmu_br_mispred), .pmu_br_fast_squash(pmu_br_fast_squash),
    .pmu_stall_rob_full(pmu_stall_rob_full), .pmu_stall_rs_full(pmu_stall_rs_full),
    .pmu_stall_lsq_full(pmu_stall_lsq_full), .pmu_stall_cp_full(pmu_stall_cp_full),
    .pmu_war_waw_renamed(pmu_war_waw_renamed), .pmu_div_overlap_cyc(pmu_div_overlap_cyc),
    .pmu_dcache_miss_overlap_cyc(pmu_dcache_miss_overlap_cyc),
    .pmu_dual_disp_cyc(pmu_dual_disp_cyc), .pmu_dual_cdb_cyc(pmu_dual_cdb_cyc),
    .pmu_dual_commit_cyc(pmu_dual_commit_cyc), .pmu_ex_bypass_hits(pmu_ex_bypass_hits),
    .pmu_ras_preds(pmu_ras_preds), .pmu_ras_mispreds(pmu_ras_mispreds)
);

// Reusable Golden RV32IM ISS stepped 0, 1, or 2 times/cycle in lockstep!
rv32im_iss #(
    .IMEM_WORDS(1024),
    .DMEM_BYTES(16384),
    .CHECK_EN(0),
    .VERBOSE(0)
) u_iss (
    .clk(clk), .reset_n(reset_n),
    .commit_valid(1'b0), .commit_pc(32'd0), .commit_instr(32'd0),
    .commit_rd_we(1'b0), .commit_rd_addr(5'd0), .commit_rd_wdata(32'd0),
    .commit_mem_valid(1'b0), .commit_mem_we(1'b0), .commit_mem_addr(32'd0),
    .commit_mem_wstrb(4'd0), .commit_mem_wdata(32'd0),
    .halted(iss_halted_u), .commit_count(iss_cc_u), .error_count(iss_ec_u)
);

ooo_trace_mon2w #(
    .MAX_SEQ(4096),
    .NUM_ISSUE_PORT(6),
    .NUM_CDB_PORT(2),
    .WATCHDOG_CYC(512)
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
    .cycle_count(mon_cycles), .ooo_issue_pairs(mon_ooo_issue), .ooo_cdb_pairs(mon_ooo_cdb),
    .spec_squashed_issued(mon_spec_squash), .inv_errors(mon_inv_errors)
);

// 2-Wide Lockstep Commit Checker against u_iss.step_golden()
reg [31:0] iss_error_count;
reg [31:0] iss_commit_count;

task automatic check_one_commit(
    input [31:0] c_pc,
    input [31:0] c_instr,
    input        c_rd_we,
    input [4:0]  c_rd_addr,
    input [31:0] c_rd_wdata,
    input        c_mem_valid,
    input        c_mem_we,
    input [31:0] c_mem_addr,
    input [3:0]  c_mem_wstrb,
    input [31:0] c_mem_wdata,
    input integer lane
);
    begin
        if (u_iss.halted) begin
            $display("ISS2W FAIL [step %0d L%0d]: committed after EBREAK! pc=0x%08x",
                     iss_commit_count, lane, c_pc);
            iss_error_count = iss_error_count + 32'd1;
        end else begin
            u_iss.step_golden();
            if (c_pc !== u_iss.exp_pc || c_instr !== u_iss.exp_instr) begin
                $display("ISS2W FAIL [step %0d L%0d]: PC/INSTR exp(0x%08x,0x%08x) got(0x%08x,0x%08x)",
                         iss_commit_count, lane, u_iss.exp_pc, u_iss.exp_instr, c_pc, c_instr);
                iss_error_count = iss_error_count + 32'd1;
            end
            if (c_rd_we !== u_iss.exp_rd_we ||
                (u_iss.exp_rd_we && (c_rd_addr !== u_iss.exp_rd_addr || c_rd_wdata !== u_iss.exp_rd_wdata))) begin
                $display("ISS2W FAIL [step %0d L%0d PC=0x%08x]: RD exp(we=%0b x%0d=0x%08x) got(we=%0b x%0d=0x%08x)",
                         iss_commit_count, lane, u_iss.exp_pc, u_iss.exp_rd_we, u_iss.exp_rd_addr, u_iss.exp_rd_wdata,
                         c_rd_we, c_rd_addr, c_rd_wdata);
                iss_error_count = iss_error_count + 32'd1;
            end
            if (c_mem_valid !== u_iss.exp_mem_valid ||
                (u_iss.exp_mem_valid && (c_mem_we !== u_iss.exp_mem_we || c_mem_addr !== u_iss.exp_mem_addr)) ||
                (u_iss.exp_mem_valid && u_iss.exp_mem_we &&
                 (c_mem_wstrb !== u_iss.exp_mem_wstrb || c_mem_wdata !== u_iss.exp_mem_wdata))) begin
                $display("ISS2W FAIL [step %0d L%0d PC=0x%08x]: MEM mismatch exp(v=%0b we=%0b a=0x%08x s=%1x d=0x%08x) got(v=%0b we=%0b a=0x%08x s=%1x d=0x%08x)",
                         iss_commit_count, lane, u_iss.exp_pc,
                         u_iss.exp_mem_valid, u_iss.exp_mem_we, u_iss.exp_mem_addr, u_iss.exp_mem_wstrb, u_iss.exp_mem_wdata,
                         c_mem_valid, c_mem_we, c_mem_addr, c_mem_wstrb, c_mem_wdata);
                iss_error_count = iss_error_count + 32'd1;
            end
            iss_commit_count = iss_commit_count + 32'd1;
        end
    end
endtask

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        iss_error_count  <= 32'd0;
        iss_commit_count <= 32'd0;
    end else begin
        if (commit0_valid) begin
            check_one_commit(commit0_pc, commit0_instr, commit0_rd_we, commit0_rd_addr, commit0_rd_wdata,
                             commit0_mem_valid, commit0_mem_we, commit0_mem_addr, commit0_mem_wstrb, commit0_mem_wdata, 0);
        end
        if (commit1_valid) begin
            check_one_commit(commit1_pc, commit1_instr, commit1_rd_we, commit1_rd_addr, commit1_rd_wdata,
                             commit1_mem_valid, commit1_mem_we, commit1_mem_addr, commit1_mem_wstrb, commit1_mem_wdata, 1);
        end
    end
end

initial clk = 1'b0;
always #5 clk = ~clk;

string hex_file, konata_file;
integer max_cycles, gantt_max, konata_max, cyc, r_idx, reg_errs;
reg req_ooo_issue, req_ooo_cdb, req_spec_squash, req_ipc_gt1, show_gantt, dump_konata_en;
reg [31:0] p_iss, p_cdb, p_sq, p_err;
real ipc_val;

initial begin
    if (!$value$plusargs("HEX=%s", hex_file))
        hex_file = "build/phase0_all45.hex";
    if (!$value$plusargs("MAX_CYCLES=%d", max_cycles))
        max_cycles = 80000;
    if (!$value$plusargs("GANTT_MAX=%d", gantt_max))
        gantt_max = 32;
    if (!$value$plusargs("KONATA_MAX=%d", konata_max))
        konata_max = 256;
    dump_konata_en  = $value$plusargs("KONATA=%s", konata_file);
    req_ooo_issue   = $test$plusargs("REQ_OOO_ISSUE");
    req_ooo_cdb     = $test$plusargs("REQ_OOO_CDB");
    req_spec_squash = $test$plusargs("REQ_SPEC_SQUASH");
    req_ipc_gt1     = $test$plusargs("REQ_IPC_GT1");
    show_gantt      = $test$plusargs("GANTT");

    reset_n = 1'b0;
    u_dut.u_imem.load_hex(hex_file);
    u_iss.load_hex(hex_file);

    repeat (3) @(posedge clk);
    @(negedge clk);
    reset_n = 1'b1;

    cyc = 0;
    while (!dut_halted && cyc < max_cycles) begin
        @(posedge clk);
        cyc = cyc + 1;
        if (iss_error_count > 32'd5 || mon_inv_errors > 32'd5) begin
            $display("FAIL [ooo2w_soc]: aborting early iss_err=%0d mon_err=%0d file=%s",
                     iss_error_count, mon_inv_errors, hex_file);
            $fatal(1);
        end
    end

    repeat (2) @(posedge clk);

    if (!dut_halted || !u_iss.halted) begin
        $display("FAIL [ooo2w_soc]: timeout! dut_halted=%0b iss_halted=%0b cyc=%0d commits=%0d file=%s",
                 dut_halted, u_iss.halted, cyc, iss_commit_count, hex_file);
        $fatal(1);
    end

    reg_errs = 0;
    for (r_idx = 1; r_idx < 32; r_idx = r_idx + 1) begin
        if (u_dut.u_core.u_arf.reg_array[r_idx] !== u_iss.regs[r_idx]) begin
            $display("FAIL [ooo2w_soc ARF]: x%0d dut=0x%08x iss=0x%08x",
                     r_idx, u_dut.u_core.u_arf.reg_array[r_idx], u_iss.regs[r_idx]);
            reg_errs = reg_errs + 1;
        end
    end

    if (show_gantt)
        u_mon.dump_gantt(1, gantt_max);
    if (dump_konata_en)
        u_mon.dump_konata(konata_file, konata_max);

    u_mon.compute_and_check_proofs(req_ooo_issue, req_ooo_cdb, req_spec_squash, p_iss, p_cdb, p_sq, p_err);
    ipc_val = (pmu_cycles > 0) ? ($itor(pmu_instret) / $itor(pmu_cycles)) : 0.0;

    if (req_ipc_gt1 && (ipc_val <= 1.0)) begin
        $display("FAIL [ooo2w_soc]: expected superscalar IPC > 1.0, got %0.3f!", ipc_val);
        $fatal(1);
    end

    if (iss_error_count != 32'd0 || reg_errs != 0 || p_err != 32'd0) begin
        $display("FAIL [ooo2w_soc]: iss_err=%0d reg_err=%0d mon_err=%0d ooo_iss=%0d ooo_cdb=%0d spec_sq=%0d file=%s",
                 iss_error_count, reg_errs, p_err, p_iss, p_cdb, p_sq, hex_file);
        $fatal(1);
    end

    $display("PASS [ooo2w_soc]: file=%s commits=%0d cycles=%0d IPC=%0.3f ooo_issue=%0d ooo_cdb=%0d spec_sq=%0d fast_sq=%0d dual_d=%0d dual_w=%0d dual_c=%0d ex_byp=%0d ras=%0d/%0d hits=%0d misses=%0d wbs=%0d hum=%0d mum=%0d coal=%0d stlf=%0d war_waw=%0d div_ovl=%0d dcm_ovl=%0d br=%0d/%0d x14=0x%08x",
             hex_file, pmu_instret, pmu_cycles, ipc_val,
             p_iss, p_cdb, p_sq, pmu_br_fast_squash,
             pmu_dual_disp_cyc, pmu_dual_cdb_cyc, pmu_dual_commit_cyc, pmu_ex_bypass_hits,
             pmu_ras_mispreds, pmu_ras_preds,
             pmu_dcache_hits, pmu_dcache_misses, pmu_dcache_wbs,
             pmu_dcache_hum, pmu_dcache_mum_cyc, pmu_mshr_coalesce, pmu_stlf_hits,
             pmu_war_waw_renamed, pmu_div_overlap_cyc, pmu_dcache_miss_overlap_cyc,
             pmu_br_mispred, pmu_br_total, dut_x14);
    $finish;
end

endmodule
