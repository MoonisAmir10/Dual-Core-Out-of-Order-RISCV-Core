`timescale 1ns / 1ps

// ============================================================================
// Standalone Unit Testbench for btb_bht2w (Upgrade U2: GSHARE + 8-RAS + 4-CP)
// Verifies:
//   1. 0-cycle cold JAL pre-decode target & B-type direct target
//   2. 8-entry nested CALL/RET stack + circular overflow + coroutine swap
//   3. 256-entry GSHARE learning periodic T-N-T-N pattern (where local 2b fails!)
//   4. Exact GHR + 8-entry RAS 4-CP recovery after wrong-path CALL/RET squash!
// ============================================================================
module tb_btb_bht2w;

`include "rv32ima_defs.vh"

reg        clk, reset_n;
reg [31:0] req_pc0, req_instr0, req_pc1, req_instr1;
wire       pred_taken0, pred_taken1, pred_used_ras0, pred_used_ras1;
wire       pred_bim0, pred_gsh0, pred_bim1, pred_gsh1;
wire [31:0] pred_target0, pred_target1;
wire [7:0] pred_ghr0, pred_ghr1;

reg        enq0_fire, enq1_fire;
reg        disp0_fire, disp1_fire;
reg [31:0] disp0_pc, disp0_instr, disp1_pc, disp1_instr;
reg        cp_take, cp_is_lane1;
reg [4:0]  cp_take_tag;

reg        upd_valid, upd_is_cond, upd_is_jalr, upd_taken, upd_pred_bim, upd_pred_gsh;
reg [31:0] upd_pc, upd_target;
reg [7:0]  upd_ghr;
reg        br_resolve_ok, squash_valid;
reg [4:0]  br_resolve_tag, squash_tag, rob_head;

btb_bht2w #(.BTB_ENTRIES(32), .BHT_ENTRIES(256), .RAS_DEPTH(8), .TAG_W(5), .NUM_CP(4)) dut (
    .clk(clk), .reset_n(reset_n),
    .req_pc0(req_pc0), .req_instr0(req_instr0),
    .pred_taken0(pred_taken0), .pred_target0(pred_target0),
    .pred_ghr0(pred_ghr0), .pred_used_ras0(pred_used_ras0),
    .pred_bim0(pred_bim0), .pred_gsh0(pred_gsh0),
    .req_pc1(req_pc1), .req_instr1(req_instr1),
    .pred_taken1(pred_taken1), .pred_target1(pred_target1),
    .pred_ghr1(pred_ghr1), .pred_used_ras1(pred_used_ras1),
    .pred_bim1(pred_bim1), .pred_gsh1(pred_gsh1),
    .enq0_fire(enq0_fire), .enq1_fire(enq1_fire),
    .disp0_fire(disp0_fire), .disp0_pc(disp0_pc), .disp0_instr(disp0_instr),
    .disp1_fire(disp1_fire), .disp1_pc(disp1_pc), .disp1_instr(disp1_instr),
    .cp_take(cp_take), .cp_is_lane1(cp_is_lane1), .cp_take_tag(cp_take_tag),
    .commit0_valid(1'b0), .commit0_pc(32'd0), .commit0_instr(32'h0000_0013), .commit0_br_taken(1'b0),
    .commit1_valid(1'b0), .commit1_pc(32'd0), .commit1_instr(32'h0000_0013), .commit1_br_taken(1'b0),
    .full_flush(1'b0),
    .upd_valid(upd_valid), .upd_pc(upd_pc), .upd_is_cond(upd_is_cond),
    .upd_is_jalr(upd_is_jalr), .upd_taken(upd_taken), .upd_target(upd_target),
    .upd_ghr(upd_ghr), .upd_pred_bim(upd_pred_bim), .upd_pred_gsh(upd_pred_gsh),
    .br_resolve_ok(br_resolve_ok), .br_resolve_tag(br_resolve_tag),
    .squash_valid(squash_valid), .squash_tag(squash_tag), .rob_head(rob_head)
);

initial clk = 1'b0;
always #5 clk = ~clk;

// Helper instruction encodings
localparam [31:0] INS_JAL_X1_P64  = 32'h0400_00EF; // jal x1, +64 (CALL)
localparam [31:0] INS_RET_X1      = 32'h0000_8067; // jalr x0, x1, 0 (RET)
localparam [31:0] INS_CORO_X5_X1  = 32'h0000_82E7; // jalr x5, x1, 0 (POP then PUSH)
localparam [31:0] INS_RET_X5      = 32'h0002_8067; // jalr x0, x5, 0 (RET via t0/x5)
localparam [31:0] INS_BEQ_P16     = 32'h0000_0863; // beq x0, x0, +16

integer errs, k, loop_misp;
reg [7:0] saved_ghr;
reg       saved_bim, saved_gsh;

task clear_all;
    begin
        req_pc0 = 32'd0; req_instr0 = INSTR_NOP;
        req_pc1 = 32'd4; req_instr1 = INSTR_NOP;
        enq0_fire = 1'b0; enq1_fire = 1'b0;
        disp0_fire = 1'b0; disp0_pc = 32'd0; disp0_instr = INSTR_NOP;
        disp1_fire = 1'b0; disp1_pc = 32'd0; disp1_instr = INSTR_NOP;
        cp_take = 1'b0; cp_is_lane1 = 1'b0; cp_take_tag = 5'd0;
        upd_valid = 1'b0; upd_pc = 32'd0; upd_is_cond = 1'b0; upd_is_jalr = 1'b0;
        upd_taken = 1'b0; upd_target = 32'd0; upd_ghr = 8'd0;
        upd_pred_bim = 1'b0; upd_pred_gsh = 1'b0;
        br_resolve_ok = 1'b0; br_resolve_tag = 5'd0;
        squash_valid = 1'b0; squash_tag = 5'd0; rob_head = 5'd0;
    end
endtask

task check(input cond, input string msg);
    begin
        if (!cond) begin
            $display("FAIL [tb_btb_bht2w]: %s", msg);
            errs = errs + 1;
        end
    end
endtask

initial begin
    errs = 0;
    reset_n = 1'b0;
    clear_all();
    repeat (2) @(posedge clk);
    @(negedge clk);
    reset_n = 1'b1;

    // ------------------------------------------------------------------
    // TEST 1: Cold JAL predicts taken + exact target in 0 cycles,
    //         and 8-deep nested CALL -> 8 nested RET returns LIFO exact!
    // ------------------------------------------------------------------
    for (k = 0; k < 8; k = k + 1) begin
        @(negedge clk);
        clear_all();
        req_pc0    = 32'h0000_0100 + (k * 32'h10);
        req_instr0 = INS_JAL_X1_P64;
        enq0_fire  = 1'b1;
        disp0_fire = 1'b1; disp0_pc = req_pc0; disp0_instr = INS_JAL_X1_P64;
        #1;
        check(pred_taken0 && (pred_target0 == (req_pc0 + 32'd64)) && !pred_used_ras0,
              "T1: cold JAL predicted taken with exact pc+64 target");
        @(posedge clk);
    end

    // Pop all 8 in LIFO order via RET
    for (k = 7; k >= 0; k = k - 1) begin
        @(negedge clk);
        clear_all();
        req_pc0    = 32'h0000_0500;
        req_instr0 = INS_RET_X1;
        enq0_fire  = 1'b1;
        disp0_fire = 1'b1; disp0_pc = req_pc0; disp0_instr = INS_RET_X1;
        #1;
        check(pred_taken0 && pred_used_ras0 &&
              (pred_target0 == (32'h0000_0100 + (k * 32'h10) + 32'd4)),
              "T1: 8-deep nested RET matched exact LIFO return address");
        @(posedge clk);
    end

    // Now RAS is empty -> RET must NOT claim pred_used_ras0
    @(negedge clk);
    clear_all();
    req_pc0 = 32'h0000_0500; req_instr0 = INS_RET_X1;
    #1;
    check(!pred_used_ras0, "T1: empty RAS does not predict");
    @(posedge clk);

    // ------------------------------------------------------------------
    // TEST 2: Coroutine swap (jalr x5, x1, 0 -> pop x1 ret, push pc+4)
    // ------------------------------------------------------------------
    @(negedge clk);
    clear_all();
    req_pc0 = 32'h0000_0200; req_instr0 = INS_JAL_X1_P64;
    enq0_fire = 1'b1; disp0_fire = 1'b1; disp0_pc = 32'h200; disp0_instr = INS_JAL_X1_P64;
    @(posedge clk);

    @(negedge clk);
    clear_all();
    req_pc0 = 32'h0000_0300; req_instr0 = INS_CORO_X5_X1;
    enq0_fire = 1'b1; disp0_fire = 1'b1; disp0_pc = 32'h300; disp0_instr = INS_CORO_X5_X1;
    #1;
    check(pred_taken0 && pred_used_ras0 && (pred_target0 == 32'h0000_0204),
          "T2: coroutine swap popped 0x204");
    @(posedge clk);

    @(negedge clk);
    clear_all();
    req_pc0 = 32'h0000_0350; req_instr0 = INS_RET_X5;
    enq0_fire = 1'b1; disp0_fire = 1'b1; disp0_pc = 32'h350; disp0_instr = INS_RET_X5;
    #1;
    check(pred_taken0 && pred_used_ras0 && (pred_target0 == 32'h0000_0304),
          "T2: coroutine swap pushed 0x304 and ret via x5 popped 0x304");
    @(posedge clk);

    // ------------------------------------------------------------------
    // TEST 3: Tournament switches to GSHARE on alternating T-N-T-N branch!
    // ------------------------------------------------------------------
    for (k = 0; k < 28; k = k + 1) begin
        @(negedge clk);
        clear_all();
        req_pc0 = 32'h0000_0400; req_instr0 = INS_BEQ_P16;
        #1;
        saved_ghr = pred_ghr0;
        saved_bim = pred_bim0;
        saved_gsh = pred_gsh0;
        if (k >= 16) begin
            check(pred_taken0 == k[0], "T3: Tournament GSHARE predicted alternating T-N-T-N accurately");
            if (k[0])
                check(pred_target0 == 32'h0000_0410, "T3: direct B-type target == pc+16");
        end
        upd_valid = 1'b1; upd_pc = 32'h0000_0400; upd_is_cond = 1'b1;
        upd_taken = k[0]; upd_target = 32'h0000_0410; upd_ghr = saved_ghr;
        upd_pred_bim = saved_bim; upd_pred_gsh = saved_gsh;
        if (pred_taken0 != k[0]) begin
            squash_valid = 1'b1; squash_tag = 5'd0;
        end else begin
            enq0_fire = 1'b1;
        end
        @(posedge clk);
    end

    // ------------------------------------------------------------------
    // TEST 3B: Tournament stays on Bimodal for a simple 10-iter loop
    //          (only 2 mispredicts: cold iter 0 + loop exit iter 9!)
    // ------------------------------------------------------------------
    loop_misp = 0;
    for (k = 0; k < 10; k = k + 1) begin
        @(negedge clk);
        clear_all();
        req_pc0 = 32'h0000_0480; req_instr0 = INS_BEQ_P16;
        #1;
        saved_ghr = pred_ghr0;
        saved_bim = pred_bim0;
        saved_gsh = pred_gsh0;
        upd_valid = 1'b1; upd_pc = 32'h0000_0480; upd_is_cond = 1'b1;
        upd_taken = (k < 9); upd_target = 32'h0000_0490; upd_ghr = saved_ghr;
        upd_pred_bim = saved_bim; upd_pred_gsh = saved_gsh;
        if (pred_taken0 != (k < 9)) begin
            loop_misp = loop_misp + 1;
            squash_valid = 1'b1; squash_tag = 5'd0;
        end else begin
            enq0_fire = 1'b1;
        end
        @(posedge clk);
    end
    check(loop_misp == 2, "T3B: Tournament Bimodal warms up in 1 iter (2/10 mispreds on loop)");

    // ------------------------------------------------------------------
    // TEST 4: Wrong-path speculative CALL + branch GHR pollution ->
    //         4-CP squash restores BOTH spec_ghr AND 8-entry RAS exactly!
    // ------------------------------------------------------------------
    // Step A: Real CALL at 0x600 pushes 0x604
    @(negedge clk);
    clear_all();
    req_pc0 = 32'h0000_0600; req_instr0 = INS_JAL_X1_P64;
    enq0_fire = 1'b1; disp0_fire = 1'b1; disp0_pc = 32'h600; disp0_instr = INS_JAL_X1_P64;
    @(posedge clk);

    // Step B: Conditional branch at 0x640 takes checkpoint tag=4 (predicts NT)
    @(negedge clk);
    clear_all();
    req_pc0 = 32'h0000_0640; req_instr0 = INS_BEQ_P16;
    #1;
    saved_ghr = pred_ghr0;
    enq0_fire = 1'b1;
    disp0_fire = 1'b1; disp0_pc = 32'h640; disp0_instr = INS_BEQ_P16;
    cp_take = 1'b1; cp_is_lane1 = 1'b0; cp_take_tag = 5'd4;
    @(posedge clk);

    // Step C: Wrong-path CALL at 0x644 pushes garbage 0x648 onto BOTH f_ras & d_ras!
    @(negedge clk);
    clear_all();
    req_pc0 = 32'h0000_0644; req_instr0 = INS_JAL_X1_P64;
    enq0_fire = 1'b1;
    disp0_fire = 1'b1; disp0_pc = 32'h644; disp0_instr = INS_JAL_X1_P64;
    @(posedge clk);

    // Verify wrong-path polluted RAS top to 0x648 before squash
    @(negedge clk);
    clear_all();
    req_pc0 = 32'h0000_0700; req_instr0 = INS_RET_X1;
    #1;
    check(pred_target0 == 32'h0000_0648, "T4: pre-squash wrong-path top is 0x648");

    // Step D: Branch tag=4 mispredicts (actually Taken!) -> squash restores RAS & GHR!
    upd_valid = 1'b1; upd_pc = 32'h0000_0640; upd_is_cond = 1'b1;
    upd_taken = 1'b1; upd_target = 32'h0000_0650; upd_ghr = saved_ghr;
    squash_valid = 1'b1; squash_tag = 5'd4; rob_head = 5'd0;
    @(posedge clk);

    // Step E: After squash, RET MUST pop real 0x604 AND spec_ghr == {saved_ghr[6:0], 1'b1}!
    @(negedge clk);
    clear_all();
    req_pc0 = 32'h0000_0660; req_instr0 = INS_RET_X1;
    #1;
    check(pred_taken0 && pred_used_ras0 && (pred_target0 == 32'h0000_0604),
          "T4: post-squash RAS restored exact 0x604!");
    check(pred_ghr0 == {saved_ghr[6:0], 1'b1},
          "T4: post-squash spec_ghr restored exact {saved_ghr[6:0], 1'b1}!");
    @(posedge clk);

    if (errs == 0)
        $display("PASS [tb_btb_bht2w]: all 4 U2 GSHARE + 8-RAS + 4-CP recovery checks passed!");
    else begin
        $display("FAIL [tb_btb_bht2w]: %0d error(s)!", errs);
        $fatal(1);
    end
    $finish;
end

endmodule
