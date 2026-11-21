`timescale 1ns / 1ps

// ============================================================================
// Standalone IEEE-754-2008 / RISC-V RV32F Verification Testbench (Phase U5)
//
// Verifies ALL 3 FPU Execution Units against Exact Rational Oracle Vectors:
//   1. fpu_fma_pipe : 4-stage II=1 streaming across FMADD/FMSUB/FNMSUB/FNMADD/
//                     FADD/FSUB/FMUL (5 rounding modes + 5 fflags!)
//   2. fpu_div_sqrt : Non-blocking iterative FDIV.S & FSQRT.S
//   3. fpu_misc     : 1-cycle FSGNJ*/FMIN/FMAX/FEQ/FLT/FLE/FCLASS/FCVT*/FMV*
//   4. OoO Pipeline Hazard Suite:
//        - Random out_ready backpressure on fpu_fma_pipe & fpu_div_sqrt
//        - Selective ROB-tag age-based branch squash (incl. wrap-around!)
//        - Full synchronous flush abort
// ============================================================================
module tb_fpu_ieee754;

`include "fpu_defs.vh"

localparam integer TAG_W = 5;

reg clk, reset_n;
initial clk = 1'b0;
always #5 clk = ~clk;

// ----------------------------------------------------------------------------
// 1. DUT Instantiations
// ----------------------------------------------------------------------------
reg              fma_flush, fma_sq_v;
reg  [TAG_W-1:0] fma_sq_tag, fma_rob_head;
reg              fma_in_v;
wire             fma_in_rdy;
reg  [2:0]       fma_in_op, fma_in_rm;
reg  [31:0]      fma_in_a, fma_in_b, fma_in_c, fma_in_seq;
reg  [TAG_W-1:0] fma_in_tag;
wire             fma_out_v;
reg              fma_out_rdy;
wire [31:0]      fma_out_res, fma_out_seq;
wire [4:0]       fma_out_flg;
wire [TAG_W-1:0] fma_out_tag;

fpu_fma_pipe #(.TAG_W(TAG_W)) u_fma (
    .clk(clk), .reset_n(reset_n),
    .flush(fma_flush), .squash_valid(fma_sq_v), .squash_tag(fma_sq_tag), .rob_head(fma_rob_head),
    .in_valid(fma_in_v), .in_ready(fma_in_rdy),
    .in_op(fma_in_op), .in_rm(fma_in_rm),
    .in_rs1(fma_in_a), .in_rs2(fma_in_b), .in_rs3(fma_in_c),
    .in_tag(fma_in_tag), .in_seq(fma_in_seq),
    .out_valid(fma_out_v), .out_ready(fma_out_rdy),
    .out_result(fma_out_res), .out_fflags(fma_out_flg),
    .out_tag(fma_out_tag), .out_seq(fma_out_seq)
);

reg              div_flush, div_sq_v;
reg  [TAG_W-1:0] div_sq_tag, div_rob_head;
reg              div_in_v;
wire             div_in_rdy;
reg              div_in_op;
reg  [2:0]       div_in_rm;
reg  [31:0]      div_in_a, div_in_b, div_in_seq;
reg  [TAG_W-1:0] div_in_tag;
wire             div_out_v;
reg              div_out_rdy;
wire [31:0]      div_out_res, div_out_seq;
wire [4:0]       div_out_flg;
wire [TAG_W-1:0] div_out_tag;

fpu_div_sqrt #(.TAG_W(TAG_W)) u_div (
    .clk(clk), .reset_n(reset_n),
    .flush(div_flush), .squash_valid(div_sq_v), .squash_tag(div_sq_tag), .rob_head(div_rob_head),
    .in_valid(div_in_v), .in_ready(div_in_rdy),
    .in_op(div_in_op), .in_rm(div_in_rm),
    .in_rs1(div_in_a), .in_rs2(div_in_b),
    .in_tag(div_in_tag), .in_seq(div_in_seq),
    .out_valid(div_out_v), .out_ready(div_out_rdy),
    .out_result(div_out_res), .out_fflags(div_out_flg),
    .out_tag(div_out_tag), .out_seq(div_out_seq)
);

reg  [3:0]  misc_op;
reg  [2:0]  misc_rm;
reg  [31:0] misc_a, misc_b;
wire [31:0] misc_res;
wire [4:0]  misc_flg;

fpu_misc u_misc (
    .in_op(misc_op), .in_rm(misc_rm),
    .in_rs1(misc_a), .in_rs2(misc_b),
    .out_result(misc_res), .out_fflags(misc_flg)
);

// ----------------------------------------------------------------------------
// 2. Test Helpers & Counters
// ----------------------------------------------------------------------------
integer fma_cnt, div_cnt, misc_cnt, haz_cnt, errors;
integer max_vec;
reg [8*256-1:0] fma_vec_file, div_vec_file, misc_vec_file;

task step_clk;
    begin
        @(posedge clk);
        #1;
    end
endtask

task check_haz(input cond, input [1023:0] msg);
    begin
        haz_cnt = haz_cnt + 1;
        if (!cond) begin
            $display("FAIL [tb_fpu_ieee754 HAZARD]: %0s", msg);
            errors = errors + 1;
        end
    end
endtask

// FIFO scoreboard for pipelined II=1 FMA streaming verification
localparam integer SB_DEPTH = 16;
reg [31:0] sb_exp_res [0:SB_DEPTH-1];
reg [4:0]  sb_exp_flg [0:SB_DEPTH-1];
reg [31:0] sb_a       [0:SB_DEPTH-1];
reg [31:0] sb_b       [0:SB_DEPTH-1];
reg [31:0] sb_c       [0:SB_DEPTH-1];
reg [2:0]  sb_op      [0:SB_DEPTH-1];
reg [2:0]  sb_rm      [0:SB_DEPTH-1];
reg [31:0] sb_seq     [0:SB_DEPTH-1];
integer    sb_head, sb_tail;

integer fd, rc, wd, cyc_stall;
reg [31:0] v_op, v_rm, v_a, v_b, v_c, v_exp_res, v_exp_flg;
reg        eof_hit;

initial begin
    errors   = 0;
    fma_cnt  = 0;
    div_cnt  = 0;
    misc_cnt = 0;
    haz_cnt  = 0;
    max_vec  = 0;

    reset_n      = 1'b0;
    fma_flush    = 1'b0; fma_sq_v = 1'b0; fma_sq_tag = 5'd0; fma_rob_head = 5'd0;
    fma_in_v     = 1'b0; fma_in_op = 3'd0; fma_in_rm = 3'd0;
    fma_in_a     = 32'd0; fma_in_b = 32'd0; fma_in_c = 32'd0; fma_in_tag = 5'd0; fma_in_seq = 32'd0;
    fma_out_rdy  = 1'b1;

    div_flush    = 1'b0; div_sq_v = 1'b0; div_sq_tag = 5'd0; div_rob_head = 5'd0;
    div_in_v     = 1'b0; div_in_op = 1'b0; div_in_rm = 3'd0;
    div_in_a     = 32'd0; div_in_b = 32'd0; div_in_tag = 5'd0; div_in_seq = 32'd0;
    div_out_rdy  = 1'b1;

    misc_op = 4'd0; misc_rm = 3'd0; misc_a = 32'd0; misc_b = 32'd0;

    if ($value$plusargs("MAX_VEC=%d", max_vec)) begin end

    repeat (2) step_clk();
    reset_n = 1'b1;
    step_clk();

    // ========================================================================
    // A. FPU MISC / COMPARE / CONVERT VECTOR SUITE
    // ========================================================================
    if ($value$plusargs("MISC_VEC=%s", misc_vec_file)) begin
        fd = $fopen(misc_vec_file, "r");
        if (fd == 0) begin
            $display("FAIL [tb_fpu_ieee754]: Cannot open MISC_VEC %0s", misc_vec_file);
            $fatal(1);
        end
        eof_hit = 1'b0;
        while (!eof_hit && (max_vec == 0 || misc_cnt < max_vec)) begin
            rc = $fscanf(fd, "%h %h %h %h %h %h\n", v_op, v_rm, v_a, v_b, v_exp_res, v_exp_flg);
            if (rc != 6) begin
                eof_hit = 1'b1;
            end else begin
                misc_op = v_op[3:0];
                misc_rm = v_rm[2:0];
                misc_a  = v_a;
                misc_b  = v_b;
                #1;
                if (misc_res !== v_exp_res || misc_flg !== v_exp_flg[4:0]) begin
                    if (errors < 20)
                        $display("FAIL [fpu_misc #%0d]: op=%0d rm=%0d a=%08x b=%08x exp=(%08x,%02x) got=(%08x,%02x)",
                                 misc_cnt, v_op, v_rm, v_a, v_b, v_exp_res, v_exp_flg[4:0], misc_res, misc_flg);
                    errors = errors + 1;
                end
                misc_cnt = misc_cnt + 1;
            end
        end
        $fclose(fd);
    end

    // ========================================================================
    // B. FPU FMA 4-STAGE PIPELINE VECTOR SUITE (Back-to-Back II=1 + Random Backpressure!)
    // ========================================================================
    if ($value$plusargs("FMA_VEC=%s", fma_vec_file)) begin
        fd = $fopen(fma_vec_file, "r");
        if (fd == 0) begin
            $display("FAIL [tb_fpu_ieee754]: Cannot open FMA_VEC %0s", fma_vec_file);
            $fatal(1);
        end
        sb_head = 0;
        sb_tail = 0;
        eof_hit = 1'b0;
        cyc_stall = 0;

        while (!eof_hit || (sb_head != sb_tail)) begin
            // Periodically inject backpressure stalls (every 64th cycle) to stress pipe_en!
            cyc_stall = cyc_stall + 1;
            fma_out_rdy = (cyc_stall[5:0] != 6'd31);

            if (!eof_hit && (max_vec == 0 || fma_cnt < max_vec) && fma_in_rdy) begin
                rc = $fscanf(fd, "%h %h %h %h %h %h %h\n",
                             v_op, v_rm, v_a, v_b, v_c, v_exp_res, v_exp_flg);
                if (rc != 7) begin
                    eof_hit  = 1'b1;
                    fma_in_v = 1'b0;
                end else begin
                    fma_in_v   = 1'b1;
                    fma_in_op  = v_op[2:0];
                    fma_in_rm  = v_rm[2:0];
                    fma_in_a   = v_a;
                    fma_in_b   = v_b;
                    fma_in_c   = v_c;
                    fma_in_tag = fma_cnt[TAG_W-1:0];
                    fma_in_seq = fma_cnt;

                    sb_exp_res[sb_tail % SB_DEPTH] = v_exp_res;
                    sb_exp_flg[sb_tail % SB_DEPTH] = v_exp_flg[4:0];
                    sb_a[sb_tail % SB_DEPTH]       = v_a;
                    sb_b[sb_tail % SB_DEPTH]       = v_b;
                    sb_c[sb_tail % SB_DEPTH]       = v_c;
                    sb_op[sb_tail % SB_DEPTH]      = v_op[2:0];
                    sb_rm[sb_tail % SB_DEPTH]      = v_rm[2:0];
                    sb_seq[sb_tail % SB_DEPTH]     = fma_cnt;
                    sb_tail = sb_tail + 1;
                    fma_cnt = fma_cnt + 1;
                end
            end else if (eof_hit || (max_vec != 0 && fma_cnt >= max_vec)) begin
                eof_hit  = 1'b1;
                fma_in_v = 1'b0;
            end

            // Check any accepted output BEFORE advancing the clock edge
            if (fma_out_v && fma_out_rdy) begin
                if (sb_head == sb_tail) begin
                    $display("FAIL [fpu_fma_pipe]: Unexpected extra output seq=%0d!", fma_out_seq);
                    errors = errors + 1;
                end else begin
                    if (fma_out_res !== sb_exp_res[sb_head % SB_DEPTH] ||
                        fma_out_flg !== sb_exp_flg[sb_head % SB_DEPTH] ||
                        fma_out_seq !== sb_seq[sb_head % SB_DEPTH]) begin
                        if (errors < 20)
                            $display("FAIL [fpu_fma #%0d]: op=%0d rm=%0d a=%08x b=%08x c=%08x exp=(%08x,%02x,s=%0d) got=(%08x,%02x,s=%0d)",
                                     sb_seq[sb_head % SB_DEPTH],
                                     sb_op[sb_head % SB_DEPTH], sb_rm[sb_head % SB_DEPTH],
                                     sb_a[sb_head % SB_DEPTH], sb_b[sb_head % SB_DEPTH], sb_c[sb_head % SB_DEPTH],
                                     sb_exp_res[sb_head % SB_DEPTH], sb_exp_flg[sb_head % SB_DEPTH], sb_seq[sb_head % SB_DEPTH],
                                     fma_out_res, fma_out_flg, fma_out_seq);
                        errors = errors + 1;
                    end
                    sb_head = sb_head + 1;
                end
            end

            step_clk();
        end
        fma_in_v    = 1'b0;
        fma_out_rdy = 1'b1;
        $fclose(fd);
    end

    // ========================================================================
    // C. FPU ITERATIVE FDIV.S & FSQRT.S VECTOR SUITE
    // ========================================================================
    if ($value$plusargs("DIV_VEC=%s", div_vec_file)) begin
        fd = $fopen(div_vec_file, "r");
        if (fd == 0) begin
            $display("FAIL [tb_fpu_ieee754]: Cannot open DIV_VEC %0s", div_vec_file);
            $fatal(1);
        end
        eof_hit = 1'b0;
        while (!eof_hit && (max_vec == 0 || div_cnt < max_vec)) begin
            rc = $fscanf(fd, "%h %h %h %h %h %h\n", v_op, v_rm, v_a, v_b, v_exp_res, v_exp_flg);
            if (rc != 6) begin
                eof_hit = 1'b1;
            end else begin
                while (!div_in_rdy) step_clk();
                div_in_v   = 1'b1;
                div_in_op  = v_op[0];
                div_in_rm  = v_rm[2:0];
                div_in_a   = v_a;
                div_in_b   = v_b;
                div_in_tag = div_cnt[TAG_W-1:0];
                div_in_seq = div_cnt;
                step_clk();
                div_in_v   = 1'b0;

                wd = 0;
                while (!div_out_v && wd < 64) begin
                    step_clk();
                    wd = wd + 1;
                end
                if (!div_out_v) begin
                    $display("FAIL [fpu_div_sqrt #%0d]: Watchdog timeout!", div_cnt);
                    errors = errors + 1;
                end else if (div_out_res !== v_exp_res || div_out_flg !== v_exp_flg[4:0]) begin
                    if (errors < 20)
                        $display("FAIL [fpu_div_sqrt #%0d]: op=%0d rm=%0d a=%08x b=%08x exp=(%08x,%02x) got=(%08x,%02x)",
                                 div_cnt, v_op[0], v_rm, v_a, v_b, v_exp_res, v_exp_flg[4:0], div_out_res, div_out_flg);
                    errors = errors + 1;
                end
                // Occasionally stall out_ready for 1 cycle before accepting!
                if ((div_cnt & 63) == 17) begin
                    div_out_rdy = 1'b0;
                    step_clk();
                    check_haz(div_out_v && div_out_res === v_exp_res, "fpu_div_sqrt holds output on !out_ready");
                    div_out_rdy = 1'b1;
                end
                step_clk();
                div_cnt = div_cnt + 1;
            end
        end
        $fclose(fd);
    end

    // ========================================================================
    // D. OoO SELECTIVE ROB-TAG BRANCH SQUASH & FLUSH TESTS (Wrap-Around!)
    // ========================================================================
    // 1. FMA Selective Squash: rob_head=30, branch at squash_tag=1 (age=3).
    //    Launch 3 back-to-back ops:
    //      - tag=31 (age=1 <= 3): OLDER -> MUST SURVIVE! (2.0 * 3.0 + 1.0 = 7.0 = 0x40E0_0000)
    //      - tag=1  (age=3 <= 3): EQUAL -> MUST SURVIVE! (4.0 * 5.0 + 0.0 = 20.0 = 0x41A0_0000)
    //      - tag=2  (age=4 >  3): YOUNGER -> MUST BE KILLED! (1.0 + 1.0)
    fma_out_rdy  = 1'b1;
    fma_rob_head = 5'd30;
    fma_sq_tag   = 5'd1;

    fma_in_v = 1'b1; fma_in_op = FMA_OP_FMADD; fma_in_rm = FPU_RM_RNE;
    fma_in_a = 32'h4000_0000; fma_in_b = 32'h4040_0000; fma_in_c = 32'h3F80_0000;
    fma_in_tag = 5'd31; fma_in_seq = 32'd101;
    step_clk(); // tag 31 -> S1

    fma_in_a = 32'h4080_0000; fma_in_b = 32'h40A0_0000; fma_in_c = 32'h0000_0000;
    fma_in_tag = 5'd1; fma_in_seq = 32'd102;
    step_clk(); // tag 31 -> S2, tag 1 -> S1

    fma_in_a = 32'h3F80_0000; fma_in_b = 32'h3F80_0000; fma_in_c = 32'h3F80_0000;
    fma_in_tag = 5'd2; fma_in_seq = 32'd103;
    fma_sq_v = 1'b1; // Squash younger than tag=1 while tag=2 is entering!
    step_clk(); // tag 31 -> S3, tag 1 -> S2, tag 2 KILLED at S1!
    fma_in_v = 1'b0;
    fma_sq_v = 1'b0;

    step_clk(); // tag 31 -> S4 (out_valid!), tag 1 -> S3
    check_haz(fma_out_v && fma_out_tag == 5'd31 && fma_out_res == 32'h40E0_0000,
              "FMA older tag=31 survived wrap-around selective squash with exact 7.0");
    step_clk(); // tag 1 -> S4 (out_valid!)
    check_haz(fma_out_v && fma_out_tag == 5'd1 && fma_out_res == 32'h41A0_0000,
              "FMA equal tag=1 survived wrap-around selective squash with exact 20.0");
    step_clk(); // tag 2 MUST have been killed!
    check_haz(!fma_out_v, "FMA younger tag=2 killed cleanly by wrap-around selective squash");

    // 2. FDIV Selective Squash: older survives, younger aborts immediately back to IDLE!
    div_rob_head = 5'd28;
    div_sq_tag   = 5'd30;
    div_in_v = 1'b1; div_in_op = FDIV_OP_DIV; div_in_rm = FPU_RM_RNE;
    div_in_a = 32'h4110_0000; div_in_b = 32'h4040_0000; // 9.0 / 3.0 = 3.0 (0x4040_0000)
    div_in_tag = 5'd29; div_in_seq = 32'd201; // age=1 <= 2 (OLDER!)
    step_clk();
    div_in_v = 1'b0;
    repeat (5) step_clk();
    div_sq_v = 1'b1;
    step_clk();
    div_sq_v = 1'b0;
    wd = 0; while (!div_out_v && wd < 40) begin step_clk(); wd = wd + 1; end
    check_haz(div_out_v && div_out_tag == 5'd29 && div_out_res == 32'h4040_0000,
              "FDIV older tag=29 survived selective branch squash with exact 3.0");
    step_clk();

    // Launch younger FDIV (tag=0, age=4 > 2) and squash mid-flight!
    div_in_v = 1'b1; div_in_op = FDIV_OP_SQRT; div_in_rm = FPU_RM_RNE;
    div_in_a = 32'h4000_0000; div_in_b = 32'd0;
    div_in_tag = 5'd0; div_in_seq = 32'd202;
    step_clk();
    div_in_v = 1'b0;
    repeat (4) step_clk();
    check_haz(!div_in_rdy, "FDIV busy mid-flight before squash");
    div_sq_v = 1'b1;
    step_clk();
    div_sq_v = 1'b0;
    check_haz(div_in_rdy && !div_out_v, "FDIV younger tag=0 aborted immediately to IDLE on squash");

    // 3. Full Synchronous Flush on FMA
    fma_in_v = 1'b1; fma_in_tag = 5'd5; fma_in_seq = 32'd301;
    step_clk();
    fma_in_v = 1'b0;
    fma_flush = 1'b1;
    step_clk();
    fma_flush = 1'b0;
    repeat (4) begin
        check_haz(!fma_out_v, "FMA flush cleared all in-flight stages");
        step_clk();
    end

    if (errors == 0) begin
        $display("PASS [tb_fpu_ieee754]: FMA=%0d DIV_SQRT=%0d MISC=%0d HAZARDS=%0d (TOTAL=%0d vectors, 0 mismatches!)",
                 fma_cnt, div_cnt, misc_cnt, haz_cnt, fma_cnt + div_cnt + misc_cnt + haz_cnt);
    end else begin
        $display("FAIL [tb_fpu_ieee754]: errors=%0d (FMA=%0d DIV_SQRT=%0d MISC=%0d HAZARDS=%0d)",
                 errors, fma_cnt, div_cnt, misc_cnt, haz_cnt);
        $fatal(1);
    end
    $finish;
end

endmodule
