`timescale 1ns / 1ps

// ============================================================================
// 18-Cycle Radix-4 Iterative RV32M Divider (DIV, DIVU, REM, REMU)
//
// Latency (normal)  : 18 cycles (1 setup + 16 radix-4 iters + 1 sign fixup)
// Latency (special) : 1 cycle fast-path for divide-by-zero & INT_MIN / -1
// Exact RISC-V M-Ext:
//   - DIV/DIVU by 0  -> 32'hFFFF_FFFF (-1)
//   - REM/REMU by 0  -> rs1 (dividend)
//   - DIV INT_MIN/-1 -> 32'h8000_0000 (-2^31)
//   - REM INT_MIN/-1 -> 32'h0000_0000 (0)
//   - Truncation toward zero; remainder sign matches dividend sign
// Flush             : Synchronous 1-cycle abort from any state
// ============================================================================
module div_unit #(
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

localparam [1:0] S_IDLE  = 2'd0;
localparam [1:0] S_RUN   = 2'd1;
localparam [1:0] S_FIXUP = 2'd2;
localparam [1:0] S_DONE  = 2'd3;

reg [1:0]       state;
reg [3:0]       iter;
reg [33:0]      rem_r;
reg [31:0]      quot_r;
reg [31:0]      div_r;
reg             is_rem_r;
reg             q_neg_r;
reg             r_neg_r;

assign in_ready  = (state == S_IDLE);
assign out_valid = (state == S_DONE);

// Input decode
wire in_signed   = (in_op == DIV_OP_DIV) || (in_op == DIV_OP_REM);
wire in_is_rem   = (in_op == DIV_OP_REM) || (in_op == DIV_OP_REMU);
wire div_by_zero = (in_rs2 == 32'd0);
wire signed_ovf  = in_signed && (in_rs1 == 32'h8000_0000) && (in_rs2 == 32'hFFFF_FFFF);

wire a_neg_in    = in_signed && in_rs1[31];
wire b_neg_in    = in_signed && in_rs2[31];
wire [31:0] a_mag_in = a_neg_in ? (~in_rs1 + 32'd1) : in_rs1;
wire [31:0] b_mag_in = b_neg_in ? (~in_rs2 + 32'd1) : in_rs2;

// Radix-4 restoring step combinational logic
wire [33:0] rem_shift = {rem_r[31:0], quot_r[31:30]};
wire [33:0] d1        = {2'b00, div_r};
wire [33:0] d2        = {1'b0, div_r, 1'b0};
wire [33:0] d3        = d2 + d1;

reg  [33:0] rem_next;
reg  [1:0]  q_bits;

always @(*) begin
    if (rem_shift >= d3) begin
        rem_next = rem_shift - d3;
        q_bits   = 2'd3;
    end else if (rem_shift >= d2) begin
        rem_next = rem_shift - d2;
        q_bits   = 2'd2;
    end else if (rem_shift >= d1) begin
        rem_next = rem_shift - d1;
        q_bits   = 2'd1;
    end else begin
        rem_next = rem_shift;
        q_bits   = 2'd0;
    end
end

// Sign restoration in S_FIXUP
wire [31:0] q_signed = q_neg_r ? (~quot_r + 32'd1) : quot_r;
wire [31:0] r_signed = r_neg_r ? (~rem_r[31:0] + 32'd1) : rem_r[31:0];

// Selective age-based squash (Phase 4b: aborts younger DIV, preserves older DIV!)
wire [TAG_W-1:0] sq_br_age = squash_tag - rob_head;
wire kill_cur = squash_valid && (state != S_IDLE) && ((out_tag - rob_head) > sq_br_age);
wire kill_in  = squash_valid && ((in_tag - rob_head) > sq_br_age);

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state      <= S_IDLE;
        iter       <= 4'd0;
        rem_r      <= 34'd0;
        quot_r     <= 32'd0;
        div_r      <= 32'd0;
        is_rem_r   <= 1'b0;
        q_neg_r    <= 1'b0;
        r_neg_r    <= 1'b0;
        out_result <= 32'd0;
        out_tag    <= {TAG_W{1'b0}};
        out_seq    <= 32'd0;
    end else if (flush || kill_cur) begin
        state      <= S_IDLE;
        iter       <= 4'd0;
    end else begin
        case (state)
            S_IDLE: begin
                if (in_valid && !kill_in) begin
                    out_tag  <= in_tag;
                    out_seq  <= in_seq;
                    is_rem_r <= in_is_rem;
                    if (div_by_zero) begin
                        out_result <= in_is_rem ? in_rs1 : 32'hFFFF_FFFF;
                        state      <= S_DONE;
                    end else if (signed_ovf) begin
                        out_result <= in_is_rem ? 32'd0 : 32'h8000_0000;
                        state      <= S_DONE;
                    end else begin
                        rem_r   <= 34'd0;
                        quot_r  <= a_mag_in;
                        div_r   <= b_mag_in;
                        q_neg_r <= a_neg_in ^ b_neg_in;
                        r_neg_r <= a_neg_in;
                        iter    <= 4'd15;
                        state   <= S_RUN;
                    end
                end
            end

            S_RUN: begin
                rem_r  <= rem_next;
                quot_r <= {quot_r[29:0], q_bits};
                if (iter == 4'd0)
                    state <= S_FIXUP;
                else
                    iter  <= iter - 4'd1;
            end

            S_FIXUP: begin
                out_result <= is_rem_r ? r_signed : q_signed;
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
