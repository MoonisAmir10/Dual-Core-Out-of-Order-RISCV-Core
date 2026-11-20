`timescale 1ns / 1ps

// ============================================================================
// 4-Cycle Pipelined RV32M Multiplier (MUL, MULH, MULHSU, MULHU)
//
// Latency    : 4 clock cycles from input accept to out_valid
// Throughput : 1 multiply/cycle (fully pipelined, II=1) when out_ready == 1
// Backpressure: Freezes all 4 stages cleanly if out_valid && !out_ready
// Flush      : Synchronous 1-cycle clear of all in-flight pipeline stages
// ============================================================================
module mul_unit #(
    parameter integer TAG_W = 4
) (
    input  wire             clk,
    input  wire             reset_n,
    input  wire             flush,
    input  wire             squash_valid,
    input  wire [TAG_W-1:0] squash_tag,
    input  wire [TAG_W-1:0] rob_head,

    input  wire             in_valid,
    output wire             in_ready,
    input  wire [1:0]       in_op,
    input  wire [31:0]      in_rs1,
    input  wire [31:0]      in_rs2,
    input  wire [TAG_W-1:0] in_tag,
    input  wire [31:0]      in_seq,

    output wire             out_valid,
    input  wire             out_ready,
    output reg  [31:0]      out_result,
    output reg  [TAG_W-1:0] out_tag,
    output reg  [31:0]      out_seq
);

`include "rv32ima_defs.vh"

// Stage 4 valid register declared before use for strict Verilog ordering
reg s4_valid;

// Pipeline advance enable: advance unless stage 4 is holding a valid unaccepted output
wire pipe_en = !s4_valid || out_ready;
assign in_ready  = pipe_en;
assign out_valid = s4_valid;

// Input sign-magnitude preparation (combinational ahead of Stage 1 latch)
wire a_signed = (in_op == MUL_OP_MULH) || (in_op == MUL_OP_MULHSU);
wire b_signed = (in_op == MUL_OP_MULH);
wire a_neg    = a_signed && in_rs1[31];
wire b_neg    = b_signed && in_rs2[31];
wire [31:0] a_mag_in  = a_neg ? (~in_rs1 + 32'd1) : in_rs1;
wire [31:0] b_mag_in  = b_neg ? (~in_rs2 + 32'd1) : in_rs2;
wire        res_neg_in = a_neg ^ b_neg;
wire        take_hi_in = (in_op != MUL_OP_MUL);

// Stage 1: Sign-magnitude operands registered
reg             s1_valid;
reg [31:0]      s1_a_mag;
reg [31:0]      s1_b_mag;
reg             s1_res_neg;
reg             s1_take_hi;
reg [TAG_W-1:0] s1_tag;
reg [31:0]      s1_seq;

// Stage 2: Four 16x16 unsigned partial products
reg             s2_valid;
reg [31:0]      s2_pp_ll;
reg [31:0]      s2_pp_lh;
reg [31:0]      s2_pp_hl;
reg [31:0]      s2_pp_hh;
reg             s2_res_neg;
reg             s2_take_hi;
reg [TAG_W-1:0] s2_tag;
reg [31:0]      s2_seq;

// Stage 3: 64-bit unsigned magnitude product
reg             s3_valid;
reg [63:0]      s3_prod_mag;
reg             s3_res_neg;
reg             s3_take_hi;
reg [TAG_W-1:0] s3_tag;
reg [31:0]      s3_seq;

// Stage 4 combinational intermediates
wire [32:0] s2_mid_sum = {1'b0, s2_pp_lh} + {1'b0, s2_pp_hl};
wire [63:0] s2_prod_64 = {s2_pp_hh, s2_pp_ll} + {15'd0, s2_mid_sum, 16'd0};
wire [63:0] s3_prod_signed = s3_res_neg ? (~s3_prod_mag + 64'd1) : s3_prod_mag;

// Selective age-based squash (Phase 4b: kills only entries strictly younger than squash_tag)
wire [TAG_W-1:0] sq_br_age = squash_tag - rob_head;
wire kill_in = squash_valid && ((in_tag  - rob_head) > sq_br_age);
wire kill_s1 = squash_valid && ((s1_tag  - rob_head) > sq_br_age);
wire kill_s2 = squash_valid && ((s2_tag  - rob_head) > sq_br_age);
wire kill_s3 = squash_valid && ((s3_tag  - rob_head) > sq_br_age);
wire kill_s4 = squash_valid && ((out_tag - rob_head) > sq_br_age);

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        s1_valid    <= 1'b0;
        s1_a_mag    <= 32'd0;
        s1_b_mag    <= 32'd0;
        s1_res_neg  <= 1'b0;
        s1_take_hi  <= 1'b0;
        s1_tag      <= {TAG_W{1'b0}};
        s1_seq      <= 32'd0;

        s2_valid    <= 1'b0;
        s2_pp_ll    <= 32'd0;
        s2_pp_lh    <= 32'd0;
        s2_pp_hl    <= 32'd0;
        s2_pp_hh    <= 32'd0;
        s2_res_neg  <= 1'b0;
        s2_take_hi  <= 1'b0;
        s2_tag      <= {TAG_W{1'b0}};
        s2_seq      <= 32'd0;

        s3_valid    <= 1'b0;
        s3_prod_mag <= 64'd0;
        s3_res_neg  <= 1'b0;
        s3_take_hi  <= 1'b0;
        s3_tag      <= {TAG_W{1'b0}};
        s3_seq      <= 32'd0;

        s4_valid    <= 1'b0;
        out_result  <= 32'd0;
        out_tag     <= {TAG_W{1'b0}};
        out_seq     <= 32'd0;
    end else if (flush) begin
        s1_valid    <= 1'b0;
        s2_valid    <= 1'b0;
        s3_valid    <= 1'b0;
        s4_valid    <= 1'b0;
    end else if (pipe_en) begin
        // Stage 1
        s1_valid    <= in_valid && !kill_in;
        s1_a_mag    <= a_mag_in;
        s1_b_mag    <= b_mag_in;
        s1_res_neg  <= res_neg_in;
        s1_take_hi  <= take_hi_in;
        s1_tag      <= in_tag;
        s1_seq      <= in_seq;

        // Stage 2: 16x16 partial products
        s2_valid    <= s1_valid && !kill_s1;
        s2_pp_ll    <= {16'd0, s1_a_mag[15:0]}  * {16'd0, s1_b_mag[15:0]};
        s2_pp_lh    <= {16'd0, s1_a_mag[15:0]}  * {16'd0, s1_b_mag[31:16]};
        s2_pp_hl    <= {16'd0, s1_a_mag[31:16]} * {16'd0, s1_b_mag[15:0]};
        s2_pp_hh    <= {16'd0, s1_a_mag[31:16]} * {16'd0, s1_b_mag[31:16]};
        s2_res_neg  <= s1_res_neg;
        s2_take_hi  <= s1_take_hi;
        s2_tag      <= s1_tag;
        s2_seq      <= s1_seq;

        // Stage 3: 64-bit partial product reduction
        s3_valid    <= s2_valid && !kill_s2;
        s3_prod_mag <= s2_prod_64;
        s3_res_neg  <= s2_res_neg;
        s3_take_hi  <= s2_take_hi;
        s3_tag      <= s2_tag;
        s3_seq      <= s2_seq;

        // Stage 4: Final sign restoration & output mux
        s4_valid    <= s3_valid && !kill_s3;
        out_result  <= s3_take_hi ? s3_prod_signed[63:32] : s3_prod_signed[31:0];
        out_tag     <= s3_tag;
        out_seq     <= s3_seq;
    end else begin
        // Pipeline held on CDB backpressure: still kill any selectively squashed stages
        if (kill_s1) s1_valid <= 1'b0;
        if (kill_s2) s2_valid <= 1'b0;
        if (kill_s3) s3_valid <= 1'b0;
        if (kill_s4) s4_valid <= 1'b0;
    end
end

endmodule
