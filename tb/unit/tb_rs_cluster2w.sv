`timescale 1ns / 1ps

// ============================================================================
// Standalone Unit Testbench for rs_cluster2w (Upgrade U1)
// Verifies:
//   1. 1-Cycle Speculative Issue-Stage Tag Wakeup + EX->EX Bypass (ALU0 & ALU1)
//   2. Delayed Register Capture when woken consumer does NOT issue immediately
//      (alu_out_val overwritten next cycle -> consumer must still get old value!)
//   3. Dispatch-on-Wakeup (consumer dispatches on the exact cycle producer issues)
//   4. Selective Squash killing younger bypassed op while preserving older op
// ============================================================================
module tb_rs_cluster2w;

reg        clk, reset_n, flush, squash_valid;
reg  [4:0] squash_tag, rob_head;
wire       free_ge1, free_ge2;

reg        disp0_valid, disp0_asel, disp0_bsel, disp0_qj_valid, disp0_qk_valid;
reg  [3:0] disp0_op;
reg  [4:0] disp0_rob_tag, disp0_qj, disp0_qk;
reg [31:0] disp0_seq_id, disp0_pc, disp0_imm, disp0_vj, disp0_vk;

reg        disp1_valid, disp1_asel, disp1_bsel, disp1_qj_valid, disp1_qk_valid;
reg  [3:0] disp1_op;
reg  [4:0] disp1_rob_tag, disp1_qj, disp1_qk;
reg [31:0] disp1_seq_id, disp1_pc, disp1_imm, disp1_vj, disp1_vk;

reg        cdb0_valid, cdb1_valid;
reg  [4:0] cdb0_tag, cdb1_tag;
reg [31:0] cdb0_val, cdb1_val;

reg        wk0_valid, wk1_valid;
reg  [4:0] wk0_tag, wk1_tag;
reg [31:0] byp_alu0_val, byp_alu1_val;

wire       issue0_valid, issue1_valid;
reg        fu0_ready, fu1_ready;
wire [3:0] issue0_op, issue1_op;
wire [4:0] issue0_rob_tag, issue1_rob_tag;
wire [31:0] issue0_seq_id, issue0_pc, issue0_imm, issue0_vj, issue0_vk;
wire [31:0] issue1_seq_id, issue1_pc, issue1_imm, issue1_vj, issue1_vk;
wire       issue0_asel, issue0_bsel, issue1_asel, issue1_bsel;
wire [2:0] stat_ex_byp_cnt;

rs_cluster2w #(.ENTRIES(8), .TAG_W(5), .OP_W(4), .DUAL_ISSUE(1)) dut (
    .clk(clk), .reset_n(reset_n), .flush(flush),
    .squash_valid(squash_valid), .squash_tag(squash_tag), .rob_head(rob_head),
    .free_ge1(free_ge1), .free_ge2(free_ge2),
    .disp0_valid(disp0_valid), .disp0_op(disp0_op), .disp0_rob_tag(disp0_rob_tag),
    .disp0_seq_id(disp0_seq_id), .disp0_pc(disp0_pc), .disp0_imm(disp0_imm),
    .disp0_asel(disp0_asel), .disp0_bsel(disp0_bsel),
    .disp0_vj(disp0_vj), .disp0_qj(disp0_qj), .disp0_qj_valid(disp0_qj_valid),
    .disp0_vk(disp0_vk), .disp0_qk(disp0_qk), .disp0_qk_valid(disp0_qk_valid),
    .disp1_valid(disp1_valid), .disp1_op(disp1_op), .disp1_rob_tag(disp1_rob_tag),
    .disp1_seq_id(disp1_seq_id), .disp1_pc(disp1_pc), .disp1_imm(disp1_imm),
    .disp1_asel(disp1_asel), .disp1_bsel(disp1_bsel),
    .disp1_vj(disp1_vj), .disp1_qj(disp1_qj), .disp1_qj_valid(disp1_qj_valid),
    .disp1_vk(disp1_vk), .disp1_qk(disp1_qk), .disp1_qk_valid(disp1_qk_valid),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .cdb2_valid(1'b0), .cdb2_tag(5'd0), .cdb2_val(32'd0),
    .wk0_valid(wk0_valid), .wk0_tag(wk0_tag), .wk1_valid(wk1_valid), .wk1_tag(wk1_tag),
    .byp_alu0_val(byp_alu0_val), .byp_alu1_val(byp_alu1_val),
    .issue0_valid(issue0_valid), .fu0_ready(fu0_ready), .issue0_op(issue0_op),
    .issue0_rob_tag(issue0_rob_tag), .issue0_seq_id(issue0_seq_id), .issue0_pc(issue0_pc),
    .issue0_imm(issue0_imm), .issue0_asel(issue0_asel), .issue0_bsel(issue0_bsel),
    .issue0_vj(issue0_vj), .issue0_vk(issue0_vk),
    .issue1_valid(issue1_valid), .fu1_ready(fu1_ready), .issue1_op(issue1_op),
    .issue1_rob_tag(issue1_rob_tag), .issue1_seq_id(issue1_seq_id), .issue1_pc(issue1_pc),
    .issue1_imm(issue1_imm), .issue1_asel(issue1_asel), .issue1_bsel(issue1_bsel),
    .issue1_vj(issue1_vj), .issue1_vk(issue1_vk),
    .stat_ex_byp_cnt(stat_ex_byp_cnt)
);

initial clk = 1'b0;
always #5 clk = ~clk;

integer errs;

task clear_inputs;
    begin
        flush = 1'b0; squash_valid = 1'b0; squash_tag = 5'd0;
        disp0_valid = 1'b0; disp0_op = 4'd0; disp0_rob_tag = 5'd0;
        disp0_seq_id = 32'd0; disp0_pc = 32'd0; disp0_imm = 32'd0;
        disp0_asel = 1'b0; disp0_bsel = 1'b0;
        disp0_vj = 32'd0; disp0_qj = 5'd0; disp0_qj_valid = 1'b0;
        disp0_vk = 32'd0; disp0_qk = 5'd0; disp0_qk_valid = 1'b0;

        disp1_valid = 1'b0; disp1_op = 4'd0; disp1_rob_tag = 5'd0;
        disp1_seq_id = 32'd0; disp1_pc = 32'd0; disp1_imm = 32'd0;
        disp1_asel = 1'b0; disp1_bsel = 1'b0;
        disp1_vj = 32'd0; disp1_qj = 5'd0; disp1_qj_valid = 1'b0;
        disp1_vk = 32'd0; disp1_qk = 5'd0; disp1_qk_valid = 1'b0;

        cdb0_valid = 1'b0; cdb0_tag = 5'd0; cdb0_val = 32'd0;
        cdb1_valid = 1'b0; cdb1_tag = 5'd0; cdb1_val = 32'd0;
        wk0_valid  = 1'b0; wk0_tag  = 5'd0;
        wk1_valid  = 1'b0; wk1_tag  = 5'd0;
        byp_alu0_val = 32'd0; byp_alu1_val = 32'd0;
        fu0_ready = 1'b0; fu1_ready = 1'b0;
    end
endtask

task check(input cond, input string msg);
    begin
        if (!cond) begin
            $display("FAIL [tb_rs_cluster2w]: %s", msg);
            errs = errs + 1;
        end
    end
endtask

initial begin
    errs = 0;
    reset_n = 1'b0;
    rob_head = 5'd0;
    clear_inputs();
    repeat (2) @(posedge clk);
    @(negedge clk);
    reset_n = 1'b1;

    // ------------------------------------------------------------------
    // TEST 1: Dual 1-cycle EX->EX wakeup & immediate issue next cycle!
    //   Dispatch two consumers waiting on tags 1 (ALU0) and 2 (ALU1)
    // ------------------------------------------------------------------
    @(negedge clk);
    disp0_valid = 1'b1; disp0_rob_tag = 5'd3; disp0_seq_id = 32'd10;
    disp0_qj = 5'd1; disp0_qj_valid = 1'b1;
    disp0_vk = 32'h0000_0005; disp0_qk_valid = 1'b0;

    disp1_valid = 1'b1; disp1_rob_tag = 5'd4; disp1_seq_id = 32'd11;
    disp1_vj = 32'h0000_0007; disp1_qj_valid = 1'b0;
    disp1_qk = 5'd2; disp1_qk_valid = 1'b1;
    @(posedge clk);

    // Cycle T+1: Broadcast tag-only wakeup wk0=1, wk1=2 (neither ready yet)
    @(negedge clk);
    clear_inputs();
    check(!issue0_valid && !issue1_valid, "T1: consumers must wait before wk");
    wk0_valid = 1'b1; wk0_tag = 5'd1;
    wk1_valid = 1'b1; wk1_tag = 5'd2;
    @(posedge clk);

    // Cycle T+2: Both consumers MUST issue immediately using byp_alu0_val & byp_alu1_val!
    @(negedge clk);
    clear_inputs();
    byp_alu0_val = 32'h1111_2222;
    byp_alu1_val = 32'h3333_4444;
    fu0_ready = 1'b1; fu1_ready = 1'b1;
    #1;
    check(issue0_valid && issue0_rob_tag == 5'd3 && issue0_vj == 32'h1111_2222 && issue0_vk == 32'd5,
          "T1: issue0 EX->EX bypass from ALU0");
    check(issue1_valid && issue1_rob_tag == 5'd4 && issue1_vj == 32'd7 && issue1_vk == 32'h3333_4444,
          "T1: issue1 EX->EX bypass from ALU1");
    check(stat_ex_byp_cnt == 3'd2, "T1: stat_ex_byp_cnt == 2");
    @(posedge clk);

    // ------------------------------------------------------------------
    // TEST 2: Non-issued delayed capture (byp_alu0_val overwritten at T+2!)
    // ------------------------------------------------------------------
    @(negedge clk);
    clear_inputs();
    disp0_valid = 1'b1; disp0_rob_tag = 5'd6; disp0_seq_id = 32'd20;
    disp0_qj = 5'd5; disp0_qj_valid = 1'b1;
    disp0_vk = 32'h0000_0009; disp0_qk_valid = 1'b0;
    @(posedge clk);

    // Wake tag 5 via wk0
    @(negedge clk);
    clear_inputs();
    wk0_valid = 1'b1; wk0_tag = 5'd5;
    @(posedge clk);

    // Next cycle: fu0_ready = 0 (blocked!) while byp_alu0_val = 0xCAFE_BABE
    @(negedge clk);
    clear_inputs();
    byp_alu0_val = 32'hCAFE_BABE;
    fu0_ready = 1'b0; fu1_ready = 1'b0;
    #1;
    check(issue0_valid && issue0_vj == 32'hCAFE_BABE, "T2: ready with live bypass");
    @(posedge clk);

    // Cycle after: byp_alu0_val overwritten with garbage! Must still hold 0xCAFE_BABE!
    @(negedge clk);
    clear_inputs();
    byp_alu0_val = 32'hDEAD_BEEF;
    fu0_ready = 1'b1;
    #1;
    check(issue0_valid && issue0_rob_tag == 5'd6 && issue0_vj == 32'hCAFE_BABE,
          "T2: delayed capture preserved 0xCAFE_BABE");
    check(stat_ex_byp_cnt == 3'd0, "T2: captured reg read no longer counts as bypass");
    @(posedge clk);

    // ------------------------------------------------------------------
    // TEST 3: Dispatch-on-Wakeup (consumer dispatches while wk1 fires!)
    // ------------------------------------------------------------------
    @(negedge clk);
    clear_inputs();
    disp0_valid = 1'b1; disp0_rob_tag = 5'd9; disp0_seq_id = 32'd30;
    disp0_qj = 5'd8; disp0_qj_valid = 1'b1;
    disp0_qk = 5'd8; disp0_qk_valid = 1'b1;
    wk1_valid = 1'b1; wk1_tag = 5'd8;
    @(posedge clk);

    @(negedge clk);
    clear_inputs();
    byp_alu1_val = 32'h55AA_66BB;
    fu0_ready = 1'b1;
    #1;
    check(issue0_valid && issue0_rob_tag == 5'd9 &&
          issue0_vj == 32'h55AA_66BB && issue0_vk == 32'h55AA_66BB,
          "T3: dispatch-on-wakeup dual-operand ALU1 bypass");
    check(stat_ex_byp_cnt == 3'd2, "T3: both operands bypassed == 2");
    @(posedge clk);

    // ------------------------------------------------------------------
    // TEST 4: Selective squash kills younger bypassed op, preserves older
    // ------------------------------------------------------------------
    @(negedge clk);
    clear_inputs();
    rob_head = 5'd10;
    disp0_valid = 1'b1; disp0_rob_tag = 5'd11; disp0_seq_id = 32'd40;
    disp0_qj = 5'd10; disp0_qj_valid = 1'b1;
    disp1_valid = 1'b1; disp1_rob_tag = 5'd14; disp1_seq_id = 32'd41;
    disp1_qj = 5'd10; disp1_qj_valid = 1'b1;
    wk0_valid = 1'b1; wk0_tag = 5'd10;
    @(posedge clk);

    // Squash at tag 12 (kills tag 14, preserves tag 11!)
    @(negedge clk);
    clear_inputs();
    rob_head = 5'd10;
    squash_valid = 1'b1; squash_tag = 5'd12;
    byp_alu0_val = 32'h0000_0777;
    fu0_ready = 1'b0; fu1_ready = 1'b0;
    #1;
    check(issue0_valid && issue0_rob_tag == 5'd11 && !issue1_valid,
          "T4: older tag 11 visible, younger tag 14 masked on squash");
    @(posedge clk);

    // Next cycle: only tag 11 remains, with captured 0x777!
    @(negedge clk);
    clear_inputs();
    rob_head = 5'd10;
    byp_alu0_val = 32'hBAD0_0BAD;
    fu0_ready = 1'b1; fu1_ready = 1'b1;
    #1;
    check(issue0_valid && issue0_rob_tag == 5'd11 && issue0_vj == 32'h0000_0777 && !issue1_valid,
          "T4: older tag 11 survived squash with captured 0x777; younger tag 14 gone");
    @(posedge clk);

    if (errs == 0)
        $display("PASS [tb_rs_cluster2w]: all 4 U1 wakeup + EX->EX bypass + capture + squash checks passed!");
    else begin
        $display("FAIL [tb_rs_cluster2w]: %0d error(s)!", errs);
        $fatal(1);
    end
    $finish;
end

endmodule
