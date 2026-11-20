`timescale 1ns / 1ps

// ============================================================================
// 2-Wide Superscalar Front-End Fetch Unit with 8-Entry Circular Fetch Queue
// (Upgrade U2: passes pre-decoded instructions & enqueue fires to GSHARE+RAS
//  predictor and carries per-instruction pred_ghr & pred_used_ras metadata!)
// ============================================================================
module fetch2w (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        halted,

    // Mispredict redirect & flush
    input  wire        flush,
    input  wire [31:0] flush_target_pc,

    // Dual-port instruction memory interface
    output wire [31:0] imem_pc0,
    input  wire [31:0] imem_instr0,
    output wire [31:0] imem_pc1,
    input  wire [31:0] imem_instr1,

    // Dual-lookup branch predictor interface (GSHARE + 8-entry RAS + BTB)
    output wire [31:0] pred_req_pc0,
    output wire [31:0] pred_req_instr0,
    input  wire        pred_taken0,
    input  wire [31:0] pred_target0,
    input  wire [7:0]  pred_ghr0,
    input  wire        pred_used_ras0,
    input  wire        pred_bim0,
    input  wire        pred_gsh0,

    output wire [31:0] pred_req_pc1,
    output wire [31:0] pred_req_instr1,
    input  wire        pred_taken1,
    input  wire [31:0] pred_target1,
    input  wire [7:0]  pred_ghr1,
    input  wire        pred_used_ras1,
    input  wire        pred_bim1,
    input  wire        pred_gsh1,

    output wire        enq0_fire,
    output wire        enq1_fire,

    // 2-Wide output to Decode/Dispatch
    output wire        out0_valid,
    input  wire        out0_ready,
    output wire [31:0] out0_pc,
    output wire [31:0] out0_instr,
    output wire        out0_pred_taken,
    output wire [31:0] out0_pred_target,
    output wire [7:0]  out0_pred_ghr,
    output wire        out0_pred_used_ras,
    output wire        out0_pred_bim,
    output wire        out0_pred_gsh,

    output wire        out1_valid,
    input  wire        out1_ready, // Must only be 1 when out0_ready == 1
    output wire [31:0] out1_pc,
    output wire [31:0] out1_instr,
    output wire        out1_pred_taken,
    output wire [31:0] out1_pred_target,
    output wire [7:0]  out1_pred_ghr,
    output wire        out1_pred_used_ras,
    output wire        out1_pred_bim,
    output wire        out1_pred_gsh
);

`include "rv32ima_defs.vh"

localparam integer FQ_DEPTH = 8;

reg [31:0] pc_r;
reg        ebreak_seen;

reg [31:0] fq_pc     [0:FQ_DEPTH-1];
reg [31:0] fq_instr  [0:FQ_DEPTH-1];
reg        fq_ptaken [0:FQ_DEPTH-1];
reg [31:0] fq_ptgt   [0:FQ_DEPTH-1];
reg [7:0]  fq_pghr   [0:FQ_DEPTH-1];
reg        fq_puras  [0:FQ_DEPTH-1];
reg        fq_pbim   [0:FQ_DEPTH-1];
reg        fq_pgsh   [0:FQ_DEPTH-1];
reg [2:0]  fq_head;
reg [2:0]  fq_tail;
reg [3:0]  fq_cnt;

wire [31:0] pc_plus4 = pc_r + 32'd4;
wire [2:0]  fq_head1 = fq_head + 3'd1;
wire [2:0]  fq_tail1 = fq_tail + 3'd1;

assign imem_pc0        = pc_r;
assign imem_pc1        = pc_plus4;
assign pred_req_pc0    = pc_r;
assign pred_req_instr0 = imem_instr0;
assign pred_req_pc1    = pc_plus4;
assign pred_req_instr1 = imem_instr1;

assign out0_valid         = (fq_cnt >= 4'd1) && !flush && !halted;
assign out0_pc            = fq_pc[fq_head];
assign out0_instr         = fq_instr[fq_head];
assign out0_pred_taken    = fq_ptaken[fq_head];
assign out0_pred_target   = fq_ptgt[fq_head];
assign out0_pred_ghr      = fq_pghr[fq_head];
assign out0_pred_used_ras = fq_puras[fq_head];
assign out0_pred_bim      = fq_pbim[fq_head];
assign out0_pred_gsh      = fq_pgsh[fq_head];

assign out1_valid         = (fq_cnt >= 4'd2) && !flush && !halted;
assign out1_pc            = fq_pc[fq_head1];
assign out1_instr         = fq_instr[fq_head1];
assign out1_pred_taken    = fq_ptaken[fq_head1];
assign out1_pred_target   = fq_ptgt[fq_head1];
assign out1_pred_ghr      = fq_pghr[fq_head1];
assign out1_pred_used_ras = fq_puras[fq_head1];
assign out1_pred_bim      = fq_pbim[fq_head1];
assign out1_pred_gsh      = fq_pgsh[fq_head1];

wire can_fetch   = !flush && !halted && !ebreak_seen;
wire slot0_ebrk  = (imem_instr0 == INSTR_EBREAK);
wire slot1_ebrk  = (imem_instr1 == INSTR_EBREAK);
wire can_enq2    = can_fetch && (fq_cnt <= 4'd6) && !slot0_ebrk && !pred_taken0;
wire can_enq1    = can_fetch && (fq_cnt <= 4'd7) && !can_enq2;

assign enq0_fire = can_enq1 || can_enq2;
assign enq1_fire = can_enq2;

wire [3:0] enq_cnt = can_enq2 ? 4'd2 : (can_enq1 ? 4'd1 : 4'd0);
wire do_deq0       = out0_valid && out0_ready;
wire do_deq1       = do_deq0 && out1_valid && out1_ready;
wire [3:0] deq_cnt = do_deq1 ? 4'd2 : (do_deq0 ? 4'd1 : 4'd0);

integer i;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        pc_r        <= 32'd0;
        ebreak_seen <= 1'b0;
        fq_head     <= 3'd0;
        fq_tail     <= 3'd0;
        fq_cnt      <= 4'd0;
        for (i = 0; i < FQ_DEPTH; i = i + 1) begin
            fq_pc[i]     <= 32'd0;
            fq_instr[i]  <= INSTR_NOP;
            fq_ptaken[i] <= 1'b0;
            fq_ptgt[i]   <= 32'd0;
            fq_pghr[i]   <= 8'd0;
            fq_puras[i]  <= 1'b0;
            fq_pbim[i]   <= 1'b0;
            fq_pgsh[i]   <= 1'b0;
        end
    end else if (flush) begin
        pc_r        <= flush_target_pc;
        ebreak_seen <= 1'b0;
        fq_head     <= 3'd0;
        fq_tail     <= 3'd0;
        fq_cnt      <= 4'd0;
    end else begin
        if (can_enq2) begin
            fq_pc[fq_tail]      <= pc_r;
            fq_instr[fq_tail]   <= imem_instr0;
            fq_ptaken[fq_tail]  <= 1'b0;
            fq_ptgt[fq_tail]    <= pc_plus4;
            fq_pghr[fq_tail]    <= pred_ghr0;
            fq_puras[fq_tail]   <= 1'b0;
            fq_pbim[fq_tail]    <= pred_bim0;
            fq_pgsh[fq_tail]    <= pred_gsh0;

            fq_pc[fq_tail1]     <= pc_plus4;
            fq_instr[fq_tail1]  <= imem_instr1;
            fq_ptaken[fq_tail1] <= pred_taken1;
            fq_ptgt[fq_tail1]   <= pred_taken1 ? pred_target1 : (pc_r + 32'd8);
            fq_pghr[fq_tail1]   <= pred_ghr1;
            fq_puras[fq_tail1]  <= pred_used_ras1;
            fq_pbim[fq_tail1]   <= pred_bim1;
            fq_pgsh[fq_tail1]   <= pred_gsh1;

            fq_tail <= fq_tail + 3'd2;
            if (slot1_ebrk) begin
                ebreak_seen <= 1'b1;
            end else begin
                pc_r <= pred_taken1 ? pred_target1 : (pc_r + 32'd8);
            end
        end else if (can_enq1) begin
            fq_pc[fq_tail]     <= pc_r;
            fq_instr[fq_tail]  <= imem_instr0;
            fq_ptaken[fq_tail] <= pred_taken0;
            fq_ptgt[fq_tail]   <= pred_taken0 ? pred_target0 : pc_plus4;
            fq_pghr[fq_tail]   <= pred_ghr0;
            fq_puras[fq_tail]  <= pred_used_ras0;
            fq_pbim[fq_tail]   <= pred_bim0;
            fq_pgsh[fq_tail]   <= pred_gsh0;

            fq_tail <= fq_tail + 3'd1;
            if (slot0_ebrk) begin
                ebreak_seen <= 1'b1;
            end else begin
                pc_r <= pred_taken0 ? pred_target0 : pc_plus4;
            end
        end

        if (do_deq1)
            fq_head <= fq_head + 3'd2;
        else if (do_deq0)
            fq_head <= fq_head + 3'd1;

        fq_cnt <= fq_cnt + enq_cnt - deq_cnt;
    end
end

endmodule
