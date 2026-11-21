`timescale 1ns / 1ps

// ============================================================================
// 4-Stage Fully Pipelined IEEE-754-2008 / RISC-V RV32F Single-Precision FMA
//
// Operations (in_op[2:0]):
//   FMADD.S, FMSUB.S, FNMSUB.S, FNMADD.S, FADD.S, FSUB.S, FMUL.S
//
// Microarchitecture & Compliance:
//   - Latency    : 4 clock cycles (II = 1 op/cycle fully pipelined!)
//   - Single-rounding 76-bit aligned accumulator (ZERO double-rounding!)
//   - Full IEEE-754-2008 subnormal input & output support
//   - All 5 rounding modes: RNE, RTZ, RDN, RUP, RMM
//   - Exact RISC-V F flags: NV (incl. 0*Inf+qNaN), OF, UF (tininess AFTER
//     rounding!), NX, and canonical qNaN (0x7FC0_0000)
//   - Clean backpressure (out_ready) + flush + selective ROB-tag branch squash
// ============================================================================
module fpu_fma_pipe #(
    parameter integer TAG_W = 5
) (
    input  wire             clk,
    input  wire             reset_n,
    input  wire             flush,
    input  wire             squash_valid,
    input  wire [TAG_W-1:0] squash_tag,
    input  wire [TAG_W-1:0] rob_head,

    input  wire             in_valid,
    output wire             in_ready,
    input  wire [2:0]       in_op,
    input  wire [2:0]       in_rm,
    input  wire [31:0]      in_rs1,
    input  wire [31:0]      in_rs2,
    input  wire [31:0]      in_rs3,
    input  wire [TAG_W-1:0] in_tag,
    input  wire [31:0]      in_seq,

    output wire             out_valid,
    input  wire             out_ready,
    output reg  [31:0]      out_result,
    output reg  [4:0]       out_fflags,
    output reg  [TAG_W-1:0] out_tag,
    output reg  [31:0]      out_seq
);

`include "fpu_defs.vh"

// Helper: 24-bit Leading-Zero Counter (for subnormal input normalization)
function automatic [4:0] clz24(input [23:0] v);
    integer k;
    reg found;
    begin
        clz24 = 5'd24;
        found = 1'b0;
        for (k = 23; k >= 0; k = k - 1) begin
            if (!found && v[k]) begin
                clz24 = 5'd23 - k[4:0];
                found = 1'b1;
            end
        end
    end
endfunction

// Helper: 77-bit Leading-Zero Counter (for post-add/sub normalization)
function automatic [6:0] clz77(input [76:0] v);
    integer k;
    reg found;
    begin
        clz77 = 7'd77;
        found = 1'b0;
        for (k = 76; k >= 0; k = k - 1) begin
            if (!found && v[k]) begin
                clz77 = 7'd76 - k[6:0];
                found = 1'b1;
            end
        end
    end
endfunction

// Helper: IEEE-754-2008 5-Mode Rounding Increment
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

// Stage 4 valid declared early for pipe_en
reg s4_valid;

wire pipe_en = !s4_valid || out_ready;
assign in_ready  = pipe_en;
assign out_valid = s4_valid;

// ============================================================================
// Combinational Pre-S1: Operand Steering, Unpack, Subnormal Norm & Special Cases
// ============================================================================
wire is_add_sub = (in_op == FMA_OP_FADD) || (in_op == FMA_OP_FSUB);
wire is_mul_op  = (in_op == FMA_OP_FMUL);
wire neg_prod   = (in_op == FMA_OP_FNMSUB) || (in_op == FMA_OP_FNMADD);
wire neg_c      = (in_op == FMA_OP_FMSUB)  || (in_op == FMA_OP_FNMADD) || (in_op == FMA_OP_FSUB);

wire [31:0] op_a = in_rs1;
wire [31:0] op_b = is_add_sub ? 32'h3F80_0000 : in_rs2; // +1.0 for FADD/FSUB
wire        mul_psign_in = in_rs1[31] ^ in_rs2[31];
// For FMUL, steer addend c = signed zero matching exact product sign (a_sign ^ b_sign)
wire [31:0] op_c = is_add_sub ? in_rs2 :
                   is_mul_op  ? {mul_psign_in, 31'd0} :
                                in_rs3;

wire        a_s = op_a[31]; wire [7:0] a_e = op_a[30:23]; wire [22:0] a_f = op_a[22:0];
wire        b_s = op_b[31]; wire [7:0] b_e = op_b[30:23]; wire [22:0] b_f = op_b[22:0];
wire        c_s = op_c[31]; wire [7:0] c_e = op_c[30:23]; wire [22:0] c_f = op_c[22:0];

wire a_exp_0 = (a_e == 8'h00); wire a_exp_ff = (a_e == 8'hFF);
wire b_exp_0 = (b_e == 8'h00); wire b_exp_ff = (b_e == 8'hFF);
wire c_exp_0 = (c_e == 8'h00); wire c_exp_ff = (c_e == 8'hFF);

wire a_f0 = (a_f == 23'd0); wire b_f0 = (b_f == 23'd0); wire c_f0 = (c_f == 23'd0);

wire a_zero = a_exp_0 &&  a_f0; wire a_sub = a_exp_0 && !a_f0;
wire b_zero = b_exp_0 &&  b_f0; wire b_sub = b_exp_0 && !b_f0;
wire c_zero = c_exp_0 &&  c_f0; wire c_sub = c_exp_0 && !c_f0;

wire a_inf  = a_exp_ff &&  a_f0; wire a_nan = a_exp_ff && !a_f0; wire a_snan = a_nan && !a_f[22];
wire b_inf  = b_exp_ff &&  b_f0; wire b_nan = b_exp_ff && !b_f0; wire b_snan = b_nan && !b_f[22];
wire c_inf  = c_exp_ff &&  c_f0; wire c_nan = c_exp_ff && !c_f0; wire c_snan = c_nan && !c_f[22];

wire p_sign_in = a_s ^ b_s ^ neg_prod;
wire c_sign_in = c_s ^ neg_c;

wire any_nan      = a_nan || b_nan || c_nan;
wire any_snan     = a_snan || b_snan || c_snan;
wire mul_zero_inf = (a_zero && b_inf) || (a_inf && b_zero);
wire p_inf_in     = (a_inf || b_inf) && !mul_zero_inf && !a_nan && !b_nan;
wire p_zero_in    = (a_zero || b_zero) && !mul_zero_inf && !a_nan && !b_nan;
wire inf_sub_inf  = p_inf_in && c_inf && (p_sign_in != c_sign_in);

reg        sp_valid_in;
reg [31:0] sp_res_in;
reg [4:0]  sp_flags_in;

always @(*) begin
    sp_valid_in = 1'b0;
    sp_res_in   = 32'd0;
    sp_flags_in = 5'd0;
    if (any_nan || mul_zero_inf) begin
        // RISC-V F spec: 0*Inf + qNaN MUST raise NV=1!
        sp_valid_in = 1'b1;
        sp_res_in   = FPU_CANON_QNAN;
        sp_flags_in = {(any_snan || mul_zero_inf), 4'b0000};
    end else if (inf_sub_inf) begin
        sp_valid_in = 1'b1;
        sp_res_in   = FPU_CANON_QNAN;
        sp_flags_in = 5'b10000; // NV
    end else if (p_inf_in) begin
        sp_valid_in = 1'b1;
        sp_res_in   = {p_sign_in, 8'hFF, 23'd0};
        sp_flags_in = 5'd0;
    end else if (c_inf) begin
        sp_valid_in = 1'b1;
        sp_res_in   = {c_sign_in, 8'hFF, 23'd0};
        sp_flags_in = 5'd0;
    end else if (p_zero_in && c_zero) begin
        sp_valid_in = 1'b1;
        if (p_sign_in == c_sign_in)
            sp_res_in = {p_sign_in, 31'd0};
        else
            sp_res_in = {(in_rm == FPU_RM_RDN), 31'd0};
        sp_flags_in = 5'd0;
    end
end

// Normalize significands & compute signed 11-bit unbiased exponents [-149 .. +127]
wire [4:0] a_lz = clz24({1'b0, a_f});
wire [4:0] b_lz = clz24({1'b0, b_f});
wire [4:0] c_lz = clz24({1'b0, c_f});

wire [23:0] a_sig_in = a_zero ? 24'd0 : (a_sub ? ({1'b0, a_f} << a_lz) : {1'b1, a_f});
wire [23:0] b_sig_in = b_zero ? 24'd0 : (b_sub ? ({1'b0, b_f} << b_lz) : {1'b1, b_f});
wire [23:0] c_sig_in = c_zero ? 24'd0 : (c_sub ? ({1'b0, c_f} << c_lz) : {1'b1, c_f});

wire signed [10:0] a_exp_in = a_sub ? (-11'sd126 - $signed({6'd0, a_lz})) : ($signed({3'd0, a_e}) - 11'sd127);
wire signed [10:0] b_exp_in = b_sub ? (-11'sd126 - $signed({6'd0, b_lz})) : ($signed({3'd0, b_e}) - 11'sd127);
wire signed [10:0] c_exp_in = c_sub ? (-11'sd126 - $signed({6'd0, c_lz})) : ($signed({3'd0, c_e}) - 11'sd127);
wire signed [10:0] p_exp_in = a_exp_in + b_exp_in;

// ============================================================================
// Stage 1 Registers: Normalized Operands & Special-Case Metadata
// ============================================================================
reg               s1_valid;
reg               s1_sp_valid;
reg [31:0]        s1_sp_res;
reg [4:0]         s1_sp_flags;
reg [2:0]         s1_rm;
reg               s1_p_sign, s1_c_sign;
reg               s1_p_zero, s1_c_zero;
reg [23:0]        s1_a_sig, s1_b_sig, s1_c_sig;
reg signed [10:0] s1_p_exp, s1_c_exp;
reg [TAG_W-1:0]   s1_tag;
reg [31:0]        s1_seq;

// ============================================================================
// Combinational S1 -> S2: 24x24 Exact Product & 76-Bit Alignment Shifter
// ============================================================================
wire [47:0] s1_p_sig = {24'd0, s1_a_sig} * {24'd0, s1_b_sig};

// Alignment exponent difference: sh = (p_exp + 27) - c_exp
wire signed [10:0] s1_sh_diff = (s1_p_exp + 11'sd27) - s1_c_exp;

reg signed [10:0] s1_grid_exp;
reg [75:0]        s1_p_aligned;
reg [75:0]        s1_c_aligned;
reg               s1_c_sticky;

wire [151:0] s1_c_wide = {s1_c_sig, 52'd0, 76'd0};
reg  [151:0] s1_c_shifted;
wire [6:0]   s1_sh_u = s1_sh_diff[6:0];

always @(*) begin
    s1_grid_exp  = s1_p_exp + 11'sd27;
    s1_p_aligned = 76'd0;
    s1_c_aligned = 76'd0;
    s1_c_sticky  = 1'b0;
    s1_c_shifted = 152'd0;

    if (s1_p_zero) begin
        // Product is zero -> pass c cleanly at top of grid
        s1_grid_exp  = s1_c_exp;
        s1_p_aligned = 76'd0;
        s1_c_aligned = {s1_c_sig, 52'd0};
        s1_c_sticky  = 1'b0;
    end else if (s1_c_zero) begin
        // Addend is zero -> pass exact 48b product at bits [49:2]
        s1_grid_exp  = s1_p_exp + 11'sd27;
        s1_p_aligned = {26'd0, s1_p_sig, 2'd0};
        s1_c_aligned = 76'd0;
        s1_c_sticky  = 1'b0;
    end else if (s1_sh_diff < 11'sd0) begin
        // c_exp >= p_exp + 28: p is strictly < 0.25 ULP of (c/2) -> pure sticky bit!
        s1_grid_exp  = s1_c_exp;
        s1_c_aligned = {s1_c_sig, 52'd0};
        s1_p_aligned = 76'd1; // exact sticky representation (p_nonzero guaranteed!)
        s1_c_sticky  = 1'b0;
    end else if (s1_sh_diff >= 11'sd76) begin
        // c falls completely below bit 0 (23+ bits below p's round bit) -> pure c_sticky!
        s1_grid_exp  = s1_p_exp + 11'sd27;
        s1_p_aligned = {26'd0, s1_p_sig, 2'd0};
        s1_c_aligned = 76'd0;
        s1_c_sticky  = 1'b1; // c_nonzero guaranteed!
    end else begin
        // Overlap regime: 0 <= sh <= 75
        s1_grid_exp  = s1_p_exp + 11'sd27;
        s1_p_aligned = {26'd0, s1_p_sig, 2'd0};
        s1_c_shifted = s1_c_wide >> s1_sh_u;
        s1_c_aligned = s1_c_shifted[151:76];
        s1_c_sticky  = |s1_c_shifted[75:0];
    end
end

// ============================================================================
// Stage 2 Registers: Aligned 76-Bit Operands
// ============================================================================
reg               s2_valid;
reg               s2_sp_valid;
reg [31:0]        s2_sp_res;
reg [4:0]         s2_sp_flags;
reg [2:0]         s2_rm;
reg               s2_p_sign, s2_c_sign;
reg signed [10:0] s2_grid_exp;
reg [75:0]        s2_p_aligned;
reg [75:0]        s2_c_aligned;
reg               s2_c_sticky;
reg [TAG_W-1:0]   s2_tag;
reg [31:0]        s2_seq;

// ============================================================================
// Combinational S2 -> S3: Exact 77-Bit Sign-Magnitude Add/Sub & 77-Bit CLZ
// ============================================================================
wire s2_eff_sub = (s2_p_sign != s2_c_sign);
wire s2_p_gt_c  = (s2_p_aligned > s2_c_aligned);
wire s2_p_eq_c  = (s2_p_aligned == s2_c_aligned);

reg [76:0] s2_sum_mag;
reg        s2_res_sign;
reg        s2_res_sticky;
reg        s2_exact_zero;

always @(*) begin
    s2_sum_mag    = 77'd0;
    s2_res_sign   = s2_p_sign;
    s2_res_sticky = s2_c_sticky;
    s2_exact_zero = 1'b0;

    if (!s2_eff_sub) begin
        s2_sum_mag    = {1'b0, s2_p_aligned} + {1'b0, s2_c_aligned};
        s2_res_sign   = s2_p_sign;
        s2_res_sticky = s2_c_sticky;
        s2_exact_zero = (s2_sum_mag == 77'd0) && !s2_c_sticky;
    end else begin
        // Note: whenever s2_c_sticky==1, s2_p_aligned >= 2^48 > 2^23 > s2_c_aligned!
        if (s2_p_gt_c) begin
            s2_sum_mag    = {1'b0, s2_p_aligned} - {1'b0, s2_c_aligned} - {76'd0, s2_c_sticky};
            s2_res_sign   = s2_p_sign;
            s2_res_sticky = s2_c_sticky;
        end else if (s2_p_eq_c && !s2_c_sticky) begin
            s2_sum_mag    = 77'd0;
            s2_res_sign   = (s2_rm == FPU_RM_RDN);
            s2_res_sticky = 1'b0;
            s2_exact_zero = 1'b1;
        end else begin
            s2_sum_mag    = {1'b0, s2_c_aligned} - {1'b0, s2_p_aligned};
            s2_res_sign   = s2_c_sign;
            s2_res_sticky = s2_c_sticky;
        end
    end
end

wire [6:0] s2_lzc = clz77(s2_sum_mag);

// ============================================================================
// Stage 3 Registers: Sum Magnitude & Leading-Zero Count
// ============================================================================
reg               s3_valid;
reg               s3_sp_valid;
reg [31:0]        s3_sp_res;
reg [4:0]         s3_sp_flags;
reg [2:0]         s3_rm;
reg               s3_res_sign;
reg               s3_res_sticky;
reg               s3_exact_zero;
reg signed [10:0] s3_grid_exp;
reg [76:0]        s3_sum_mag;
reg [6:0]         s3_lzc;
reg [TAG_W-1:0]   s3_tag;
reg [31:0]        s3_seq;

// ============================================================================
// Combinational S3 -> S4: Normalize, Exact Tininess-After-Rounding, Round & Pack
// ============================================================================
// Bit 75 of s3_sum_mag has weight 2^(s3_grid_exp); bit 76 has weight 2^(s3_grid_exp + 1).
// After shifting left by s3_lzc, MSB is at bit 76 with weight 2^(s3_grid_exp + 1 - s3_lzc)!
wire [76:0]       s3_norm_mag = s3_sum_mag << s3_lzc;
wire signed [10:0] s3_norm_exp = s3_grid_exp + 11'sd1 - $signed({4'd0, s3_lzc});

// Unbounded-exponent 24-bit significand + round/sticky (for exact tininess-after-rounding!)
wire [23:0] s3_unb_sig   = s3_norm_mag[76:53];
wire        s3_unb_rb    = s3_norm_mag[52];
wire        s3_unb_sb    = (|s3_norm_mag[51:0]) || s3_res_sticky;
wire        s3_unb_inc   = round_inc(s3_res_sign, s3_rm, s3_unb_sig[0], s3_unb_rb, s3_unb_sb);

// IEEE-754-2008 / RISC-V Tininess AFTER Rounding:
// True iff unbounded-exponent rounded magnitude is strictly < 2^-126!
wire s3_tiny_after = (s3_norm_exp < -11'sd126) &&
                     !((s3_norm_exp == -11'sd127) && (s3_unb_sig == 24'hFF_FFFF) && s3_unb_inc);

// Subnormal right-shift when s3_norm_exp < -126
wire        s3_is_sub_in = (s3_norm_exp < -11'sd126);
wire signed [10:0] s3_sub_sh_s = -11'sd126 - s3_norm_exp; // >= 1 when subnormal

reg [23:0] s3_pre_sig;
reg        s3_pre_rb;
reg        s3_pre_sb;
reg signed [10:0] s3_pre_exp;

// 52-bit shift buffer: {s3_unb_sig[23:0], s3_unb_rb, s3_unb_sb, 26'd0}
wire [51:0] s3_sub_buf = {s3_unb_sig, s3_unb_rb, s3_unb_sb, 26'd0};
reg  [51:0] s3_sub_shifted;

always @(*) begin
    s3_sub_shifted = 52'd0;
    if (!s3_is_sub_in) begin
        s3_pre_sig = s3_unb_sig;
        s3_pre_rb  = s3_unb_rb;
        s3_pre_sb  = s3_unb_sb;
        s3_pre_exp = s3_norm_exp;
    end else if (s3_sub_sh_s >= 11'sd26) begin
        // Everything shifts into sticky (magnitude < 2^-151 < 0.25 * MIN_SUBNORM)
        s3_pre_sig = 24'd0;
        s3_pre_rb  = 1'b0;
        s3_pre_sb  = 1'b1; // sum_mag is non-zero whenever !exact_zero
        s3_pre_exp = -11'sd126;
    end else begin
        s3_sub_shifted = s3_sub_buf >> s3_sub_sh_s[4:0];
        s3_pre_sig = s3_sub_shifted[51:28];
        s3_pre_rb  = s3_sub_shifted[27];
        s3_pre_sb  = |s3_sub_shifted[26:0];
        s3_pre_exp = -11'sd126;
    end
end

wire        s3_fin_inexact = s3_pre_rb || s3_pre_sb;
wire        s3_fin_inc     = round_inc(s3_res_sign, s3_rm, s3_pre_sig[0], s3_pre_rb, s3_pre_sb);
wire [24:0] s3_sig_rounded = {1'b0, s3_pre_sig} + {24'd0, s3_fin_inc};

// Final exponent & mantissa assembly + Overflow / Underflow handling
reg [31:0] s3_final_res;
reg [4:0]  s3_final_flags;

reg [22:0]        s3_pack_frac;
reg signed [10:0] s3_pack_exp;
reg               s3_ovf_to_inf;

always @(*) begin
    s3_final_res   = 32'd0;
    s3_final_flags = 5'd0;
    s3_pack_frac   = 23'd0;
    s3_pack_exp    = 11'sd0;
    s3_ovf_to_inf  = 1'b0;

    if (s3_sp_valid) begin
        s3_final_res   = s3_sp_res;
        s3_final_flags = s3_sp_flags;
    end else if (s3_exact_zero) begin
        s3_final_res   = {s3_res_sign, 31'd0};
        s3_final_flags = 5'd0;
    end else begin
        if (s3_is_sub_in) begin
            // In subnormal grid, bit 23 == 1 means it rounded UP into smallest normal (exp_field = 1)!
            if (s3_sig_rounded[23]) begin
                s3_final_res = {s3_res_sign, 8'd1, 23'd0};
            end else begin
                s3_final_res = {s3_res_sign, 8'd0, s3_sig_rounded[22:0]};
            end
            // Exact RISC-V IEEE-754-2008: UF = tiny_after && inexact!
            s3_final_flags = {3'b000, (s3_tiny_after && s3_fin_inexact), s3_fin_inexact};
        end else begin
            // Normal range
            if (s3_sig_rounded[24]) begin
                s3_pack_exp  = s3_pre_exp + 11'sd1;
                s3_pack_frac = s3_sig_rounded[23:1];
            end else begin
                s3_pack_exp  = s3_pre_exp;
                s3_pack_frac = s3_sig_rounded[22:0];
            end

            if (s3_pack_exp >= 11'sd128) begin
                // Overflow!
                case (s3_rm)
                    FPU_RM_RNE,
                    FPU_RM_RMM: s3_ovf_to_inf = 1'b1;
                    FPU_RM_RTZ: s3_ovf_to_inf = 1'b0;
                    FPU_RM_RDN: s3_ovf_to_inf = s3_res_sign;
                    FPU_RM_RUP: s3_ovf_to_inf = !s3_res_sign;
                    default:    s3_ovf_to_inf = 1'b1;
                endcase
                if (s3_ovf_to_inf)
                    s3_final_res = {s3_res_sign, 8'hFF, 23'd0};
                else
                    s3_final_res = {s3_res_sign, 8'hFE, 23'h7FFFFF};
                s3_final_flags = 5'b00101; // OF | NX
            end else begin
                s3_final_res   = {s3_res_sign, (s3_pack_exp[7:0] + 8'd127), s3_pack_frac};
                s3_final_flags = {4'b0000, s3_fin_inexact};
            end
        end
    end
end

// ============================================================================
// Selective Age-Based Branch Squash (Kills Only Younger Entries!)
// ============================================================================
wire [TAG_W-1:0] sq_br_age = squash_tag - rob_head;
wire kill_in = squash_valid && ((in_tag  - rob_head) > sq_br_age);
wire kill_s1 = squash_valid && ((s1_tag  - rob_head) > sq_br_age);
wire kill_s2 = squash_valid && ((s2_tag  - rob_head) > sq_br_age);
wire kill_s3 = squash_valid && ((s3_tag  - rob_head) > sq_br_age);
wire kill_s4 = squash_valid && ((out_tag - rob_head) > sq_br_age);

// ============================================================================
// Sequential Pipeline Advance
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        s1_valid      <= 1'b0;
        s1_sp_valid   <= 1'b0;
        s1_sp_res     <= 32'd0;
        s1_sp_flags   <= 5'd0;
        s1_rm         <= 3'd0;
        s1_p_sign     <= 1'b0;
        s1_c_sign     <= 1'b0;
        s1_p_zero     <= 1'b0;
        s1_c_zero     <= 1'b0;
        s1_a_sig      <= 24'd0;
        s1_b_sig      <= 24'd0;
        s1_c_sig      <= 24'd0;
        s1_p_exp      <= 11'sd0;
        s1_c_exp      <= 11'sd0;
        s1_tag        <= {TAG_W{1'b0}};
        s1_seq        <= 32'd0;

        s2_valid      <= 1'b0;
        s2_sp_valid   <= 1'b0;
        s2_sp_res     <= 32'd0;
        s2_sp_flags   <= 5'd0;
        s2_rm         <= 3'd0;
        s2_p_sign     <= 1'b0;
        s2_c_sign     <= 1'b0;
        s2_grid_exp   <= 11'sd0;
        s2_p_aligned  <= 76'd0;
        s2_c_aligned  <= 76'd0;
        s2_c_sticky   <= 1'b0;
        s2_tag        <= {TAG_W{1'b0}};
        s2_seq        <= 32'd0;

        s3_valid      <= 1'b0;
        s3_sp_valid   <= 1'b0;
        s3_sp_res     <= 32'd0;
        s3_sp_flags   <= 5'd0;
        s3_rm         <= 3'd0;
        s3_res_sign   <= 1'b0;
        s3_res_sticky <= 1'b0;
        s3_exact_zero <= 1'b0;
        s3_grid_exp   <= 11'sd0;
        s3_sum_mag    <= 77'd0;
        s3_lzc        <= 7'd0;
        s3_tag        <= {TAG_W{1'b0}};
        s3_seq        <= 32'd0;

        s4_valid      <= 1'b0;
        out_result    <= 32'd0;
        out_fflags    <= 5'd0;
        out_tag       <= {TAG_W{1'b0}};
        out_seq       <= 32'd0;
    end else if (flush) begin
        s1_valid      <= 1'b0;
        s2_valid      <= 1'b0;
        s3_valid      <= 1'b0;
        s4_valid      <= 1'b0;
    end else if (pipe_en) begin
        // Stage 1
        s1_valid      <= in_valid && !kill_in;
        s1_sp_valid   <= sp_valid_in;
        s1_sp_res     <= sp_res_in;
        s1_sp_flags   <= sp_flags_in;
        s1_rm         <= in_rm;
        s1_p_sign     <= p_sign_in;
        s1_c_sign     <= c_sign_in;
        s1_p_zero     <= p_zero_in;
        s1_c_zero     <= c_zero;
        s1_a_sig      <= a_sig_in;
        s1_b_sig      <= b_sig_in;
        s1_c_sig      <= c_sig_in;
        s1_p_exp      <= p_exp_in;
        s1_c_exp      <= c_exp_in;
        s1_tag        <= in_tag;
        s1_seq        <= in_seq;

        // Stage 2
        s2_valid      <= s1_valid && !kill_s1;
        s2_sp_valid   <= s1_sp_valid;
        s2_sp_res     <= s1_sp_res;
        s2_sp_flags   <= s1_sp_flags;
        s2_rm         <= s1_rm;
        s2_p_sign     <= s1_p_sign;
        s2_c_sign     <= s1_c_sign;
        s2_grid_exp   <= s1_grid_exp;
        s2_p_aligned  <= s1_p_aligned;
        s2_c_aligned  <= s1_c_aligned;
        s2_c_sticky   <= s1_c_sticky;
        s2_tag        <= s1_tag;
        s2_seq        <= s1_seq;

        // Stage 3
        s3_valid      <= s2_valid && !kill_s2;
        s3_sp_valid   <= s2_sp_valid;
        s3_sp_res     <= s2_sp_res;
        s3_sp_flags   <= s2_sp_flags;
        s3_rm         <= s2_rm;
        s3_res_sign   <= s2_res_sign;
        s3_res_sticky <= s2_res_sticky;
        s3_exact_zero <= s2_exact_zero;
        s3_grid_exp   <= s2_grid_exp;
        s3_sum_mag    <= s2_sum_mag;
        s3_lzc        <= s2_lzc;
        s3_tag        <= s2_tag;
        s3_seq        <= s2_seq;

        // Stage 4
        s4_valid      <= s3_valid && !kill_s3;
        out_result    <= s3_final_res;
        out_fflags    <= s3_final_flags;
        out_tag       <= s3_tag;
        out_seq       <= s3_seq;
    end else begin
        // Pipeline frozen on CDB backpressure: still squash younger speculative entries!
        if (kill_s1) s1_valid <= 1'b0;
        if (kill_s2) s2_valid <= 1'b0;
        if (kill_s3) s3_valid <= 1'b0;
        if (kill_s4) s4_valid <= 1'b0;
    end
end

endmodule
