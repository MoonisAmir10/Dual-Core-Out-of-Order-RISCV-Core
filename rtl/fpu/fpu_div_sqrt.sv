`timescale 1ns / 1ps

// ============================================================================
// Non-Blocking Iterative IEEE-754-2008 / RISC-V RV32F FDIV.S & FSQRT.S Unit
//
// Latency (normal)  : 27 cycles (1 setup + 25 radix-2 iters + 1 round/pack)
// Latency (special) : 1 cycle fast-path (NaN, Inf, Zero, Div-by-Zero, Neg Sqrt)
// Compliance:
//   - Full subnormal input & output support
//   - Exact restoring radix-2 division & binary digit-by-digit square root
//     with exact round & sticky bits
//   - All 5 rounding modes (RNE, RTZ, RDN, RUP, RMM)
//   - Exact RISC-V F flags: NV, DZ, OF, UF (tininess AFTER rounding!), NX
//   - Clean backpressure (out_ready) + flush + selective ROB-tag branch squash
// ============================================================================
module fpu_div_sqrt #(
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
    input  wire             in_op,      // 0: FDIV.S, 1: FSQRT.S
    input  wire [2:0]       in_rm,
    input  wire [31:0]      in_rs1,
    input  wire [31:0]      in_rs2,
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

localparam [1:0] S_IDLE  = 2'd0;
localparam [1:0] S_RUN   = 2'd1;
localparam [1:0] S_ROUND = 2'd2;
localparam [1:0] S_DONE  = 2'd3;

reg [1:0]         state;
reg               op_r;      // 0: FDIV, 1: FSQRT
reg [2:0]         rm_r;
reg               sign_r;
reg signed [10:0] exp_r;
reg [4:0]         iter_r;    // 24 down to 0 (25 bits computed)

// Shared iterative datapath registers
reg [24:0]        q_r;       // Quotient / Root bits [24:0] (q[24] == 1!)
reg [27:0]        rem_r;     // Remainder (div uses [25:0], sqrt uses [27:0])
reg [24:0]        div_r;     // Normalized divisor (for FDIV)
reg [49:0]        rad_r;     // Shifting 50-bit radicand (for FSQRT)

assign in_ready  = (state == S_IDLE);
assign out_valid = (state == S_DONE);

// ============================================================================
// Input Unpack, Subnormal Normalization & Special-Case Fast Path
// ============================================================================
wire        a_s = in_rs1[31]; wire [7:0] a_e = in_rs1[30:23]; wire [22:0] a_f = in_rs1[22:0];
wire        b_s = in_rs2[31]; wire [7:0] b_e = in_rs2[30:23]; wire [22:0] b_f = in_rs2[22:0];

wire a_exp_0 = (a_e == 8'h00); wire a_exp_ff = (a_e == 8'hFF);
wire b_exp_0 = (b_e == 8'h00); wire b_exp_ff = (b_e == 8'hFF);
wire a_f0 = (a_f == 23'd0);    wire b_f0 = (b_f == 23'd0);

wire a_zero = a_exp_0 &&  a_f0; wire a_sub = a_exp_0 && !a_f0;
wire b_zero = b_exp_0 &&  b_f0; wire b_sub = b_exp_0 && !b_f0;
wire a_inf  = a_exp_ff &&  a_f0; wire a_nan = a_exp_ff && !a_f0; wire a_snan = a_nan && !a_f[22];
wire b_inf  = b_exp_ff &&  b_f0; wire b_nan = b_exp_ff && !b_f0; wire b_snan = b_nan && !b_f[22];

wire div_sign_in = a_s ^ b_s;

reg        sp_hit;
reg [31:0] sp_res;
reg [4:0]  sp_flags;

always @(*) begin
    sp_hit   = 1'b0;
    sp_res   = 32'd0;
    sp_flags = 5'd0;

    if (in_op == FDIV_OP_SQRT) begin
        if (a_nan) begin
            sp_hit   = 1'b1;
            sp_res   = FPU_CANON_QNAN;
            sp_flags = {a_snan, 4'b0000};
        end else if (a_zero) begin
            // IEEE-754: sqrt(-0.0) == -0.0, sqrt(+0.0) == +0.0
            sp_hit   = 1'b1;
            sp_res   = {a_s, 31'd0};
            sp_flags = 5'd0;
        end else if (a_s) begin
            // Negative non-zero (incl. -Inf) -> qNaN + NV
            sp_hit   = 1'b1;
            sp_res   = FPU_CANON_QNAN;
            sp_flags = 5'b10000;
        end else if (a_inf) begin
            sp_hit   = 1'b1;
            sp_res   = {1'b0, 8'hFF, 23'd0};
            sp_flags = 5'd0;
        end
    end else begin
        // FDIV.S
        if (a_nan || b_nan) begin
            sp_hit   = 1'b1;
            sp_res   = FPU_CANON_QNAN;
            sp_flags = {(a_snan || b_snan), 4'b0000};
        end else if ((a_inf && b_inf) || (a_zero && b_zero)) begin
            sp_hit   = 1'b1;
            sp_res   = FPU_CANON_QNAN;
            sp_flags = 5'b10000; // NV
        end else if (b_zero) begin
            sp_hit   = 1'b1;
            sp_res   = {div_sign_in, 8'hFF, 23'd0};
            sp_flags = 5'b01000; // DZ
        end else if (a_inf) begin
            sp_hit   = 1'b1;
            sp_res   = {div_sign_in, 8'hFF, 23'd0};
            sp_flags = 5'd0;
        end else if (a_zero || b_inf) begin
            sp_hit   = 1'b1;
            sp_res   = {div_sign_in, 31'd0};
            sp_flags = 5'd0;
        end
    end
end

wire [4:0]  a_lz  = clz24({1'b0, a_f});
wire [4:0]  b_lz  = clz24({1'b0, b_f});
wire [23:0] a_sig = a_sub ? ({1'b0, a_f} << a_lz) : {1'b1, a_f};
wire [23:0] b_sig = b_sub ? ({1'b0, b_f} << b_lz) : {1'b1, b_f};

wire signed [10:0] a_exp = a_sub ? (-11'sd126 - $signed({6'd0, a_lz})) : ($signed({3'd0, a_e}) - 11'sd127);
wire signed [10:0] b_exp = b_sub ? (-11'sd126 - $signed({6'd0, b_lz})) : ($signed({3'd0, b_e}) - 11'sd127);

// FDIV initial alignment: ensure rem0 >= b_sig so q[24] == 1!
wire               div_lt    = (a_sig < b_sig);
wire signed [10:0] div_exp0  = div_lt ? (a_exp - b_exp - 11'sd1) : (a_exp - b_exp);
wire [27:0]        div_rem0  = div_lt ? {3'd0, a_sig, 1'b0} : {4'd0, a_sig};

// FSQRT initial alignment: make exponent even so radicand M in [1, 4) -> sqrt(M) in [1, 2)!
wire               sqrt_odd  = a_exp[0];
wire signed [10:0] sqrt_exp0 = sqrt_odd ? ((a_exp - 11'sd1) >>> 1) : (a_exp >>> 1);
wire [49:0]        sqrt_rad0 = sqrt_odd ? {a_sig, 26'd0} : {1'b0, a_sig, 25'd0};

// ============================================================================
// Iterative Step Logic (FDIV restoring radix-2 & FSQRT digit-by-digit)
// ============================================================================
// FDIV step
wire [27:0] div_d_ext = {3'd0, div_r};
wire        div_ge    = (rem_r >= div_d_ext);
wire [27:0] div_diff  = rem_r - div_d_ext;
wire [27:0] div_rem_n = div_ge ? {div_diff[26:0], 1'b0} : {rem_r[26:0], 1'b0};

// FSQRT step
wire [27:0] sq_rem_in = {rem_r[25:0], rad_r[49:48]};
wire [27:0] sq_trial  = {1'b0, q_r, 2'b01};
wire        sq_ge     = (sq_rem_in >= sq_trial);
wire [27:0] sq_rem_n  = sq_ge ? (sq_rem_in - sq_trial) : sq_rem_in;

// ============================================================================
// Final S_ROUND Logic: Exact Subnormal Shift, Tininess-After-Rounding & Pack
// ============================================================================
wire [23:0] r_unb_sig   = q_r[24:1];
wire        r_unb_rb    = q_r[0];
wire        r_unb_sb    = (rem_r != 28'd0);
wire        r_unb_inc   = round_inc(sign_r, rm_r, r_unb_sig[0], r_unb_rb, r_unb_sb);

wire r_tiny_after = (exp_r < -11'sd126) &&
                    !((exp_r == -11'sd127) && (r_unb_sig == 24'hFF_FFFF) && r_unb_inc);

wire               r_is_sub   = (exp_r < -11'sd126);
wire signed [10:0] r_sub_sh_s = -11'sd126 - exp_r;

wire [51:0] r_sub_buf = {r_unb_sig, r_unb_rb, r_unb_sb, 26'd0};
reg  [51:0] r_sub_shifted;
reg  [23:0] r_pre_sig;
reg         r_pre_rb;
reg         r_pre_sb;

always @(*) begin
    r_sub_shifted = 52'd0;
    if (!r_is_sub) begin
        r_pre_sig = r_unb_sig;
        r_pre_rb  = r_unb_rb;
        r_pre_sb  = r_unb_sb;
    end else if (r_sub_sh_s >= 11'sd26) begin
        r_pre_sig = 24'd0;
        r_pre_rb  = 1'b0;
        r_pre_sb  = 1'b1; // Non-zero exact quotient/root
    end else begin
        r_sub_shifted = r_sub_buf >> r_sub_sh_s[4:0];
        r_pre_sig = r_sub_shifted[51:28];
        r_pre_rb  = r_sub_shifted[27];
        r_pre_sb  = |r_sub_shifted[26:0];
    end
end

wire        r_fin_inexact = r_pre_rb || r_pre_sb;
wire        r_fin_inc     = round_inc(sign_r, rm_r, r_pre_sig[0], r_pre_rb, r_pre_sb);
wire [24:0] r_sig_rounded = {1'b0, r_pre_sig} + {24'd0, r_fin_inc};

reg [31:0]        r_final_res;
reg [4:0]         r_final_flags;
reg [22:0]        r_pack_frac;
reg signed [10:0] r_pack_exp;
reg               r_ovf_to_inf;

always @(*) begin
    r_final_res   = 32'd0;
    r_final_flags = 5'd0;
    r_pack_frac   = 23'd0;
    r_pack_exp    = 11'sd0;
    r_ovf_to_inf  = 1'b0;

    if (r_is_sub) begin
        if (r_sig_rounded[23])
            r_final_res = {sign_r, 8'd1, 23'd0};
        else
            r_final_res = {sign_r, 8'd0, r_sig_rounded[22:0]};
        r_final_flags = {3'b000, (r_tiny_after && r_fin_inexact), r_fin_inexact};
    end else begin
        if (r_sig_rounded[24]) begin
            r_pack_exp  = exp_r + 11'sd1;
            r_pack_frac = r_sig_rounded[23:1];
        end else begin
            r_pack_exp  = exp_r;
            r_pack_frac = r_sig_rounded[22:0];
        end

        if (r_pack_exp >= 11'sd128) begin
            case (rm_r)
                FPU_RM_RNE,
                FPU_RM_RMM: r_ovf_to_inf = 1'b1;
                FPU_RM_RTZ: r_ovf_to_inf = 1'b0;
                FPU_RM_RDN: r_ovf_to_inf = sign_r;
                FPU_RM_RUP: r_ovf_to_inf = !sign_r;
                default:    r_ovf_to_inf = 1'b1;
            endcase
            if (r_ovf_to_inf)
                r_final_res = {sign_r, 8'hFF, 23'd0};
            else
                r_final_res = {sign_r, 8'hFE, 23'h7FFFFF};
            r_final_flags = 5'b00101; // OF | NX
        end else begin
            r_final_res   = {sign_r, (r_pack_exp[7:0] + 8'd127), r_pack_frac};
            r_final_flags = {4'b0000, r_fin_inexact};
        end
    end
end

// ============================================================================
// Selective Age-Based Branch Squash
// ============================================================================
wire [TAG_W-1:0] sq_br_age = squash_tag - rob_head;
wire kill_cur = squash_valid && (state != S_IDLE) && ((out_tag - rob_head) > sq_br_age);
wire kill_in  = squash_valid && ((in_tag - rob_head) > sq_br_age);

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state      <= S_IDLE;
        op_r       <= 1'b0;
        rm_r       <= 3'd0;
        sign_r     <= 1'b0;
        exp_r      <= 11'sd0;
        iter_r     <= 5'd0;
        q_r        <= 25'd0;
        rem_r      <= 28'd0;
        div_r      <= 25'd0;
        rad_r      <= 50'd0;
        out_result <= 32'd0;
        out_fflags <= 5'd0;
        out_tag    <= {TAG_W{1'b0}};
        out_seq    <= 32'd0;
    end else if (flush || kill_cur) begin
        state      <= S_IDLE;
        iter_r     <= 5'd0;
    end else begin
        case (state)
            S_IDLE: begin
                if (in_valid && !kill_in) begin
                    out_tag <= in_tag;
                    out_seq <= in_seq;
                    op_r    <= in_op;
                    rm_r    <= in_rm;
                    if (sp_hit) begin
                        out_result <= sp_res;
                        out_fflags <= sp_flags;
                        state      <= S_DONE;
                    end else if (in_op == FDIV_OP_SQRT) begin
                        sign_r <= 1'b0;
                        exp_r  <= sqrt_exp0;
                        q_r    <= 25'd0;
                        rem_r  <= 28'd0;
                        rad_r  <= sqrt_rad0;
                        iter_r <= 5'd24;
                        state  <= S_RUN;
                    end else begin
                        sign_r <= div_sign_in;
                        exp_r  <= div_exp0;
                        q_r    <= 25'd0;
                        rem_r  <= div_rem0;
                        div_r  <= {1'b0, b_sig};
                        iter_r <= 5'd24;
                        state  <= S_RUN;
                    end
                end
            end

            S_RUN: begin
                if (op_r == FDIV_OP_SQRT) begin
                    rem_r <= sq_rem_n;
                    q_r   <= {q_r[23:0], sq_ge};
                    rad_r <= {rad_r[47:0], 2'b00};
                end else begin
                    // On final iteration (iter_r==0), keep unshifted remainder for exact sticky check!
                    rem_r <= (iter_r == 5'd0) ? (div_ge ? div_diff : rem_r) : div_rem_n;
                    q_r   <= {q_r[23:0], div_ge};
                end

                if (iter_r == 5'd0)
                    state <= S_ROUND;
                else
                    iter_r <= iter_r - 5'd1;
            end

            S_ROUND: begin
                out_result <= r_final_res;
                out_fflags <= r_final_flags;
                state      <= S_DONE;
            end

            S_DONE: begin
                if (out_ready)
                    state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
