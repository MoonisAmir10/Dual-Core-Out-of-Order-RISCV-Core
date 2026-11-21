`timescale 1ns / 1ps

// ============================================================================
// 1-Cycle Combinational IEEE-754-2008 / RISC-V RV32F Misc, Compare & Convert
//
// Operations (in_op[3:0]):
//   - Sign Injection : FSGNJ.S, FSGNJN.S, FSGNJX.S (no NaN canonicalization)
//   - Min / Max      : FMIN.S, FMAX.S (RISC-V v2.2 -0.0 < +0.0, sNaN NV only)
//   - Compare        : FEQ.S (quiet), FLT.S / FLE.S (signaling)
//   - Classify       : FCLASS.S (10-bit one-hot mask)
//   - Float -> Int   : FCVT.W.S, FCVT.WU.S (round-before-range-check, exact NV/NX)
//   - Int -> Float   : FCVT.S.W, FCVT.S.WU (all 5 rounding modes, exact NX)
//   - Raw Move       : FMV.X.W, FMV.W.X
// ============================================================================
module fpu_misc (
    input  wire [3:0]  in_op,
    input  wire [2:0]  in_rm,
    input  wire [31:0] in_rs1,
    input  wire [31:0] in_rs2,

    output reg  [31:0] out_result,
    output reg  [4:0]  out_fflags
);

`include "fpu_defs.vh"

function automatic [4:0] clz32(input [31:0] v);
    integer k;
    reg found;
    begin
        clz32 = 5'd0;
        found = 1'b0;
        for (k = 31; k >= 0; k = k - 1) begin
            if (!found && v[k]) begin
                clz32 = 5'd31 - k[4:0];
                found = 1'b1;
            end
        end
    end
endfunction

function automatic round_inc(
    input       sign,
    input [2:0] rm,
    input       lsb,
    input       rb,
    input       sb
);
    begin
        case (rm)
            FPU_RM_RNE: round_inc = rb && (sb || lsb);
            FPU_RM_RTZ: round_inc = 1'b0;
            FPU_RM_RDN: round_inc = sign && (rb || sb);
            FPU_RM_RUP: round_inc = (!sign) && (rb || sb);
            FPU_RM_RMM: round_inc = rb;
            default:    round_inc = 1'b0;
        endcase
    end
endfunction

// ----------------------------------------------------------------------------
// 1. Operand Classification
// ----------------------------------------------------------------------------
wire        a_s = in_rs1[31]; wire [7:0] a_e = in_rs1[30:23]; wire [22:0] a_f = in_rs1[22:0];
wire        b_s = in_rs2[31]; wire [7:0] b_e = in_rs2[30:23]; wire [22:0] b_f = in_rs2[22:0];

wire a_e0  = (a_e == 8'h00); wire a_eff = (a_e == 8'hFF); wire a_f0 = (a_f == 23'd0);
wire b_e0  = (b_e == 8'h00); wire b_eff = (b_e == 8'hFF); wire b_f0 = (b_f == 23'd0);

wire a_zero = a_e0  &&  a_f0; wire a_sub = a_e0  && !a_f0; wire a_norm = !a_e0 && !a_eff;
wire b_zero = b_e0  &&  b_f0;
wire a_inf  = a_eff &&  a_f0; wire a_nan = a_eff && !a_f0; wire a_snan = a_nan && !a_f[22]; wire a_qnan = a_nan && a_f[22];
wire b_inf  = b_eff &&  b_f0; wire b_nan = b_eff && !b_f0; wire b_snan = b_nan && !b_f[22];

// Total numerical comparison for non-NaN operands
wire both_zero = a_zero && b_zero;
wire a_eq_b    = both_zero || (in_rs1 == in_rs2);
wire mag_lt    = (in_rs1[30:0] < in_rs2[30:0]);
wire mag_gt    = (in_rs1[30:0] > in_rs2[30:0]);

// Strict numeric < (treating -0.0 == +0.0)
wire a_lt_b_num = both_zero ? 1'b0 :
                  (a_s != b_s) ? a_s :
                  (a_s ? mag_gt : mag_lt);

// FMIN/FMAX order (RISC-V v2.2: -0.0 is strictly less than +0.0!)
wire a_lt_b_minmax = both_zero ? (a_s && !b_s) : a_lt_b_num;

// FCLASS.S 10-bit one-hot mask
wire [9:0] fclass_mask = {
    a_qnan,                  // [9] quiet NaN
    a_snan,                  // [8] signaling NaN
    (!a_s && a_inf),         // [7] +Inf
    (!a_s && a_norm),        // [6] +normal
    (!a_s && a_sub),         // [5] +subnormal
    (!a_s && a_zero),        // [4] +0
    ( a_s && a_zero),        // [3] -0
    ( a_s && a_sub),         // [2] -subnormal
    ( a_s && a_norm),        // [1] -normal
    ( a_s && a_inf)          // [0] -Inf
};

// ----------------------------------------------------------------------------
// 2. Float -> Integer Conversion (FCVT.W.S / FCVT.WU.S)
// ----------------------------------------------------------------------------
wire signed [9:0] cvt_exp = $signed({2'd0, a_e}) - 10'sd127;
wire [23:0]       cvt_sig = a_e0 ? {1'b0, a_f} : {1'b1, a_f};

reg [31:0] cvt_int_part;
reg        cvt_rb;
reg        cvt_sb;
reg [55:0] cvt_sh_buf;
reg [4:0]  cvt_rsh;
reg [3:0]  cvt_lsh;

always @(*) begin
    cvt_int_part = 32'd0;
    cvt_rb       = 1'b0;
    cvt_sb       = 1'b0;
    cvt_sh_buf   = 56'd0;
    cvt_rsh      = 5'd0;
    cvt_lsh      = 4'd0;

    if (a_zero) begin
        cvt_int_part = 32'd0;
        cvt_rb       = 1'b0;
        cvt_sb       = 1'b0;
    end else if (cvt_exp < -10'sd1) begin
        // 0 < |a| < 0.5 -> int_part=0, round_bit=0, sticky=1
        cvt_int_part = 32'd0;
        cvt_rb       = 1'b0;
        cvt_sb       = 1'b1;
    end else if (cvt_exp == -10'sd1) begin
        // 0.5 <= |a| < 1.0 -> int_part=0, round_bit=1, sticky=(a_f != 0)
        cvt_int_part = 32'd0;
        cvt_rb       = 1'b1;
        cvt_sb       = !a_f0;
    end else if (cvt_exp < 10'sd23) begin
        // 0 <= exp <= 22: binary point inside cvt_sig[23:0]
        // Shift {cvt_sig, 32'd0} right by (23 - exp) -> top 24 bits = int_part, next bit = rb, rest = sb
        cvt_rsh      = 5'd23 - cvt_exp[4:0];
        cvt_sh_buf   = {cvt_sig, 32'd0} >> cvt_rsh;
        cvt_int_part = {8'd0, cvt_sh_buf[55:32]};
        cvt_rb       = cvt_sh_buf[31];
        cvt_sb       = |cvt_sh_buf[30:0];
    end else if (cvt_exp <= 10'sd31) begin
        // 23 <= exp <= 31: exact integer (no fractional bits!)
        cvt_lsh      = cvt_exp[3:0] - 4'd7; // exp - 23 in [0..8]
        cvt_int_part = {8'd0, cvt_sig} << cvt_lsh;
        cvt_rb       = 1'b0;
        cvt_sb       = 1'b0;
    end else begin
        // exp >= 32: out of 32-bit integer range (handled below)
        cvt_int_part = 32'hFFFF_FFFF;
        cvt_rb       = 1'b0;
        cvt_sb       = 1'b0;
    end
end

wire        cvt_inexact = cvt_rb || cvt_sb;
wire        cvt_inc     = round_inc(a_s, in_rm, cvt_int_part[0], cvt_rb, cvt_sb);
wire [32:0] cvt_mag_rnd = {1'b0, cvt_int_part} + {32'd0, cvt_inc};

reg [31:0] f2i_w_res, f2i_wu_res;
reg [4:0]  f2i_w_flg, f2i_wu_flg;

always @(*) begin
    // FCVT.W.S (signed int32)
    if (a_nan) begin
        f2i_w_res = 32'h7FFF_FFFF;
        f2i_w_flg = 5'b10000; // NV
    end else if (a_inf) begin
        f2i_w_res = a_s ? 32'h8000_0000 : 32'h7FFF_FFFF;
        f2i_w_flg = 5'b10000; // NV
    end else if (a_s) begin
        if (cvt_exp >= 10'sd32 || (cvt_exp == 10'sd31 && !a_f0)) begin
            f2i_w_res = 32'h8000_0000;
            f2i_w_flg = 5'b10000; // NV
        end else if (cvt_exp == 10'sd31) begin
            // Exact -2^31
            f2i_w_res = 32'h8000_0000;
            f2i_w_flg = 5'd0;
        end else begin
            f2i_w_res = (~cvt_mag_rnd[31:0]) + 32'd1;
            f2i_w_flg = {4'b0000, cvt_inexact};
        end
    end else begin
        if (cvt_exp >= 10'sd31) begin
            f2i_w_res = 32'h7FFF_FFFF;
            f2i_w_flg = 5'b10000; // NV
        end else begin
            f2i_w_res = cvt_mag_rnd[31:0];
            f2i_w_flg = {4'b0000, cvt_inexact};
        end
    end

    // FCVT.WU.S (unsigned uint32 — rounds BEFORE checking range!)
    if (a_nan) begin
        f2i_wu_res = 32'hFFFF_FFFF;
        f2i_wu_flg = 5'b10000; // NV
    end else if (a_inf) begin
        f2i_wu_res = a_s ? 32'd0 : 32'hFFFF_FFFF;
        f2i_wu_flg = 5'b10000; // NV
    end else if (a_s) begin
        if (cvt_exp >= 10'sd32 || (cvt_mag_rnd != 33'd0)) begin
            // Rounded result is strictly negative (< 0) -> clamp 0, NV=1, NX=0
            f2i_wu_res = 32'd0;
            f2i_wu_flg = 5'b10000; // NV
        end else begin
            // Rounded result is 0 (e.g. -0.0, or -0.3 under RNE/RTZ/RUP!) -> NV=0, NX=inexact
            f2i_wu_res = 32'd0;
            f2i_wu_flg = {4'b0000, cvt_inexact};
        end
    end else begin
        if (cvt_exp >= 10'sd32) begin
            f2i_wu_res = 32'hFFFF_FFFF;
            f2i_wu_flg = 5'b10000; // NV
        end else begin
            f2i_wu_res = cvt_mag_rnd[31:0];
            f2i_wu_flg = {4'b0000, cvt_inexact};
        end
    end
end

// ----------------------------------------------------------------------------
// 3. Integer -> Float Conversion (FCVT.S.W / FCVT.S.WU)
// ----------------------------------------------------------------------------
wire        i2f_signed = (in_op == FMISC_OP_CVT_SW);
wire        i2f_sign   = i2f_signed && in_rs1[31];
wire [31:0] i2f_mag    = i2f_sign ? (~in_rs1 + 32'd1) : in_rs1;
wire [4:0]  i2f_lz     = clz32(i2f_mag);
wire [31:0] i2f_norm   = i2f_mag << i2f_lz;
wire [7:0]  i2f_exp_u  = 8'd158 - {3'd0, i2f_lz}; // 127 + (31 - lz)
wire [23:0] i2f_sig24  = i2f_norm[31:8];
wire        i2f_rb     = i2f_norm[7];
wire        i2f_sb     = |i2f_norm[6:0];
wire        i2f_nx     = i2f_rb || i2f_sb;
wire        i2f_inc    = round_inc(i2f_sign, in_rm, i2f_sig24[0], i2f_rb, i2f_sb);
wire [24:0] i2f_sum    = {1'b0, i2f_sig24} + {24'd0, i2f_inc};

wire [31:0] i2f_res = (in_rs1 == 32'd0) ? 32'd0 :
                      i2f_sum[24]       ? {i2f_sign, (i2f_exp_u + 8'd1), 23'd0} :
                                          {i2f_sign, i2f_exp_u, i2f_sum[22:0]};
wire [4:0]  i2f_flg = (in_rs1 == 32'd0) ? 5'd0 : {4'b0000, i2f_nx};

// ----------------------------------------------------------------------------
// 4. Final Output Mux
// ----------------------------------------------------------------------------
always @(*) begin
    out_result = 32'd0;
    out_fflags = 5'd0;

    case (in_op)
        FMISC_OP_SGNJ: begin
            out_result = {b_s, in_rs1[30:0]};
            out_fflags = 5'd0;
        end
        FMISC_OP_SGNJN: begin
            out_result = {~b_s, in_rs1[30:0]};
            out_fflags = 5'd0;
        end
        FMISC_OP_SGNJX: begin
            out_result = {a_s ^ b_s, in_rs1[30:0]};
            out_fflags = 5'd0;
        end
        FMISC_OP_MIN: begin
            out_fflags = {(a_snan || b_snan), 4'b0000};
            if (a_nan && b_nan)      out_result = FPU_CANON_QNAN;
            else if (a_nan)          out_result = in_rs2;
            else if (b_nan)          out_result = in_rs1;
            else                     out_result = a_lt_b_minmax ? in_rs1 : in_rs2;
        end
        FMISC_OP_MAX: begin
            out_fflags = {(a_snan || b_snan), 4'b0000};
            if (a_nan && b_nan)      out_result = FPU_CANON_QNAN;
            else if (a_nan)          out_result = in_rs2;
            else if (b_nan)          out_result = in_rs1;
            else                     out_result = a_lt_b_minmax ? in_rs2 : in_rs1;
        end
        FMISC_OP_EQ: begin
            out_fflags = {(a_snan || b_snan), 4'b0000};
            out_result = (a_nan || b_nan) ? 32'd0 : {31'd0, a_eq_b};
        end
        FMISC_OP_LT: begin
            out_fflags = {(a_nan || b_nan), 4'b0000};
            out_result = (a_nan || b_nan) ? 32'd0 : {31'd0, a_lt_b_num};
        end
        FMISC_OP_LE: begin
            out_fflags = {(a_nan || b_nan), 4'b0000};
            out_result = (a_nan || b_nan) ? 32'd0 : {31'd0, (a_lt_b_num || a_eq_b)};
        end
        FMISC_OP_CLASS: begin
            out_result = {22'd0, fclass_mask};
            out_fflags = 5'd0;
        end
        FMISC_OP_CVT_WS: begin
            out_result = f2i_w_res;
            out_fflags = f2i_w_flg;
        end
        FMISC_OP_CVT_WUS: begin
            out_result = f2i_wu_res;
            out_fflags = f2i_wu_flg;
        end
        FMISC_OP_CVT_SW,
        FMISC_OP_CVT_SWU: begin
            out_result = i2f_res;
            out_fflags = i2f_flg;
        end
        FMISC_OP_MV_XW,
        FMISC_OP_MV_WX: begin
            out_result = in_rs1;
            out_fflags = 5'd0;
        end
        default: begin
            out_result = 32'd0;
            out_fflags = 5'd0;
        end
    endcase
end

endmodule
