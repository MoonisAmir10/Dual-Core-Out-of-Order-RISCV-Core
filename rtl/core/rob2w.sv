`timescale 1ns / 1ps

// ============================================================================
// 32-Entry 2-Wide Superscalar Circular Reorder Buffer (ROB2W - Phase 1)
//
// Phase 1 Additions:
//   - Full ROB-head serialization metadata for CSR, FENCE, FENCE.I, and RV32A AMO
//   - Oldest in-flight Ordering Barrier tracker (barrier_valid / barrier_tag)
//     so younger speculative loads never bypass an older FENCE / FENCE.I / AMO
//   - Speculative Load Memory-Order Violation Replay Flush (lsq_head0/1_replay)
//   - FENCE.I post-commit full_flush (after SQ drain + D$ clean_all_done)
//   - Exposes commit0/1_br_taken for exact Architectural GHR recovery in btb_bht2w
// ============================================================================
module rob2w #(
    parameter integer DEPTH = 32,
    parameter integer TAG_W = 5
) (
    input  wire             clk,
    input  wire             reset_n,

    // 2-Wide Dispatch allocation interface
    input  wire             disp0_valid,
    input  wire             disp1_valid,
    output wire             disp_free_ge1,
    output wire             disp_free_ge2,
    output wire [TAG_W-1:0] disp0_tag,
    output wire [TAG_W-1:0] disp1_tag,

    input  wire [31:0]      disp0_seq_id,
    input  wire [31:0]      disp0_pc,
    input  wire [31:0]      disp0_instr,
    input  wire [2:0]       disp0_fu_type,
    input  wire             disp0_rd_we,
    input  wire [4:0]       disp0_rd_addr,
    input  wire             disp0_pred_taken,
    input  wire [31:0]      disp0_pred_target,
    input  wire [7:0]       disp0_pred_ghr,
    input  wire             disp0_pred_used_ras,
    input  wire             disp0_pred_bim,
    input  wire             disp0_pred_gsh,
    input  wire             disp0_is_ebreak,
    input  wire             disp0_is_fence,
    input  wire             disp0_is_fence_i,
    input  wire             disp0_is_csr,
    input  wire             disp0_is_amo,
    input  wire             disp0_ready_init,

    input  wire [31:0]      disp1_seq_id,
    input  wire [31:0]      disp1_pc,
    input  wire [31:0]      disp1_instr,
    input  wire [2:0]       disp1_fu_type,
    input  wire             disp1_rd_we,
    input  wire [4:0]       disp1_rd_addr,
    input  wire             disp1_pred_taken,
    input  wire [31:0]      disp1_pred_target,
    input  wire [7:0]       disp1_pred_ghr,
    input  wire             disp1_pred_used_ras,
    input  wire             disp1_pred_bim,
    input  wire             disp1_pred_gsh,
    input  wire             disp1_is_ebreak,
    input  wire             disp1_is_fence,
    input  wire             disp1_is_fence_i,
    input  wire             disp1_is_csr,
    input  wire             disp1_is_amo,
    input  wire             disp1_ready_init,

    // 4 Combinational operand value read ports
    input  wire [TAG_W-1:0] rd0_tag,
    output wire             rd0_ready,
    output wire [31:0]      rd0_value,
    input  wire [TAG_W-1:0] rd1_tag,
    output wire             rd1_ready,
    output wire [31:0]      rd1_value,
    input  wire [TAG_W-1:0] rd2_tag,
    output wire             rd2_ready,
    output wire [31:0]      rd2_value,
    input  wire [TAG_W-1:0] rd3_tag,
    output wire             rd3_ready,
    output wire [31:0]      rd3_value,

    // Dual-CDB out-of-order completion ports
    input  wire             cdb0_valid,
    input  wire [TAG_W-1:0] cdb0_tag,
    input  wire [31:0]      cdb0_val,
    input  wire             cdb0_is_br,
    input  wire             cdb0_br_taken,
    input  wire [31:0]      cdb0_br_target,
    input  wire             cdb0_is_mem,
    input  wire             cdb0_mem_we,
    input  wire [31:0]      cdb0_mem_addr,
    input  wire [3:0]       cdb0_mem_wstrb,
    input  wire [31:0]      cdb0_mem_wdata,

    input  wire             cdb1_valid,
    input  wire [TAG_W-1:0] cdb1_tag,
    input  wire [31:0]      cdb1_val,

    // External commit readiness & replay gates
    input  wire             halted_in,
    input  wire             lsq_sq_empty,      // sq_empty && dc_writes_idle
    input  wire             fence_i_ready,     // D$ dirty clean-all finished for FENCE.I
    input  wire             lsq_head0_replay,  // Speculative load at h0 was snoop-invalidated!
    input  wire             lsq_head1_replay,  // Speculative load at h1 was snoop-invalidated!

    // ROB-Head Non-Speculative Execution Observability (CSR / FENCE.I / AMO)
    output wire             head_valid,
    output wire             head_ready,
    output wire [TAG_W-1:0] head_tag,
    output wire [31:0]      head_seq_id,
    output wire [31:0]      head_pc,
    output wire [31:0]      head_instr,
    output wire             head_is_csr,
    output wire             head_is_fence,
    output wire             head_is_fence_i,
    output wire             head_is_amo,

    // Oldest in-flight Ordering Barrier (blocks younger speculative loads in LSQ)
    output reg              barrier_valid,
    output reg  [TAG_W-1:0] barrier_tag,

    // Commit Port 0 (older: head_ptr)
    output wire             commit0_valid,
    output wire [TAG_W-1:0] commit0_tag,
    output wire [31:0]      commit0_seq_id,
    output wire [31:0]      commit0_pc,
    output wire [31:0]      commit0_instr,
    output wire [2:0]       commit0_fu_type,
    output wire             commit0_rd_we,
    output wire [4:0]       commit0_rd_addr,
    output wire [31:0]      commit0_rd_wdata,
    output wire             commit0_mem_valid,
    output wire             commit0_mem_we,
    output wire [31:0]      commit0_mem_addr,
    output wire [3:0]       commit0_mem_wstrb,
    output wire [31:0]      commit0_mem_wdata,
    output wire             commit0_is_ebreak,
    output wire             commit0_is_fence_i,
    output wire             commit0_is_br,
    output wire             commit0_br_taken,
    output wire             commit0_mispred,
    output wire             commit0_used_ras,

    // Commit Port 1 (younger: head_ptr + 1)
    output wire             commit1_valid,
    output wire [TAG_W-1:0] commit1_tag,
    output wire [31:0]      commit1_seq_id,
    output wire [31:0]      commit1_pc,
    output wire [31:0]      commit1_instr,
    output wire [2:0]       commit1_fu_type,
    output wire             commit1_rd_we,
    output wire [4:0]       commit1_rd_addr,
    output wire [31:0]      commit1_rd_wdata,
    output wire             commit1_mem_valid,
    output wire             commit1_mem_we,
    output wire [31:0]      commit1_mem_addr,
    output wire [3:0]       commit1_mem_wstrb,
    output wire [31:0]      commit1_mem_wdata,
    output wire             commit1_is_br,
    output wire             commit1_br_taken,
    output wire             commit1_mispred,
    output wire             commit1_used_ras,

    // Fast Checkpoint Selective Branch Squash & ROB-Head Full Replay/FENCE.I Flush
    output wire             squash_out,
    output wire [TAG_W-1:0] squash_tag_out,
    output wire [31:0]      squash_seq_out,
    output wire [31:0]      squash_redirect_pc,
    output wire             squash_has_older,
    output wire             full_flush_out,        // FENCE.I post-commit OR LQ snoop-replay!
    output wire             replay_flush_out,      // Specifically speculative load replay pulse (PMU)
    output wire [31:0]      full_flush_redirect_pc,
    output wire [31:0]      full_flush_after_seq,

    output wire             cdb0_br_ok,
    output wire [31:0]      cdb0_br_pc,
    output wire             cdb0_br_is_cond,
    output wire             cdb0_br_is_jalr,
    output wire [7:0]       cdb0_br_ghr,
    output wire             cdb0_br_pred_bim,
    output wire             cdb0_br_pred_gsh,
    output wire [TAG_W-1:0] head_ptr_out,
    output wire             rob_full,
    output wire             rob_empty
);

`include "rv32ima_defs.vh"

reg             e_valid      [0:DEPTH-1];
reg             e_ready      [0:DEPTH-1];
reg [31:0]      e_seq_id     [0:DEPTH-1];
reg [31:0]      e_pc         [0:DEPTH-1];
reg [31:0]      e_instr      [0:DEPTH-1];
reg [2:0]       e_fu_type    [0:DEPTH-1];
reg             e_rd_we      [0:DEPTH-1];
reg [4:0]       e_rd_addr    [0:DEPTH-1];
reg [31:0]      e_value      [0:DEPTH-1];
reg             e_pred_taken [0:DEPTH-1];
reg [31:0]      e_pred_tgt   [0:DEPTH-1];
reg [7:0]       e_pred_ghr   [0:DEPTH-1];
reg             e_pred_uras  [0:DEPTH-1];
reg             e_pred_bim   [0:DEPTH-1];
reg             e_pred_gsh   [0:DEPTH-1];
reg             e_act_taken  [0:DEPTH-1];
reg [31:0]      e_act_tgt    [0:DEPTH-1];
reg             e_mispred    [0:DEPTH-1];
reg             e_mem_valid  [0:DEPTH-1];
reg             e_mem_we     [0:DEPTH-1];
reg [31:0]      e_mem_addr   [0:DEPTH-1];
reg [3:0]       e_mem_wstrb  [0:DEPTH-1];
reg [31:0]      e_mem_wdata  [0:DEPTH-1];
reg             e_is_ebreak  [0:DEPTH-1];
reg             e_is_fence   [0:DEPTH-1];
reg             e_is_fence_i [0:DEPTH-1];
reg             e_is_csr     [0:DEPTH-1];
reg             e_is_amo     [0:DEPTH-1];

reg [TAG_W-1:0] head_ptr;
reg [TAG_W-1:0] tail_ptr;
reg [TAG_W:0]   count;

wire [TAG_W-1:0] h0 = head_ptr;
wire [TAG_W-1:0] h1 = head_ptr + {{(TAG_W-1){1'b0}}, 1'b1};
wire [TAG_W-1:0] t0 = tail_ptr;
wire [TAG_W-1:0] t1 = tail_ptr + {{(TAG_W-1){1'b0}}, 1'b1};

assign head_ptr_out = head_ptr;
assign disp0_tag    = t0;
assign disp1_tag    = t1;
assign rob_full     = (count == DEPTH[TAG_W:0]);
assign rob_empty    = (count == {(TAG_W+1){1'b0}});

// ROB-Head observability for non-speculative CSR / FENCE.I / AMO execution
assign head_valid      = (count >= {{TAG_W{1'b0}}, 1'b1}) && e_valid[h0] && !halted_in;
assign head_ready      = e_ready[h0];
assign head_tag        = h0;
assign head_seq_id     = e_seq_id[h0];
assign head_pc         = e_pc[h0];
assign head_instr      = e_instr[h0];
assign head_is_csr     = head_valid && e_is_csr[h0];
assign head_is_fence   = head_valid && e_is_fence[h0];
assign head_is_fence_i = head_valid && e_is_fence_i[h0];
assign head_is_amo     = head_valid && e_is_amo[h0];

// Oldest active ordering barrier scan (FENCE, FENCE.I, or AMO)
integer bi;
reg [TAG_W-1:0] bar_best_age;
always @(*) begin
    bi            = 0;
    barrier_valid = 1'b0;
    barrier_tag   = {TAG_W{1'b0}};
    bar_best_age  = {TAG_W{1'b1}};
    for (bi = 0; bi < DEPTH; bi = bi + 1) begin
        if (e_valid[bi] && (e_is_fence[bi] || e_is_fence_i[bi] || e_is_amo[bi])) begin
            if (!barrier_valid || ((bi[TAG_W-1:0] - head_ptr) < bar_best_age)) begin
                barrier_valid = 1'b1;
                barrier_tag   = bi[TAG_W-1:0];
                bar_best_age  = bi[TAG_W-1:0] - head_ptr;
            end
        end
    end
end

// Speculative Load Memory-Order Replay at ROB head (fires before commit0!)
wire h0_replay_fire = head_valid && (e_fu_type[h0] == FU_LOAD) && lsq_head0_replay;

// Commit Port 0 (older: h0)
wire h0_needs_sq_drain = e_is_ebreak[h0] || e_is_fence[h0] || e_is_fence_i[h0];
wire c0_allow          = !halted_in && !h0_replay_fire &&
                         (!h0_needs_sq_drain || lsq_sq_empty) &&
                         (!e_is_fence_i[h0] || fence_i_ready);

assign commit0_valid      = (count >= {{TAG_W{1'b0}}, 1'b1}) && e_valid[h0] && e_ready[h0] && c0_allow;
assign commit0_tag        = h0;
assign commit0_seq_id     = e_seq_id[h0];
assign commit0_pc         = e_pc[h0];
assign commit0_instr      = e_instr[h0];
assign commit0_fu_type    = e_fu_type[h0];
assign commit0_rd_we      = e_rd_we[h0];
assign commit0_rd_addr    = e_rd_we[h0] ? e_rd_addr[h0] : 5'd0;
assign commit0_rd_wdata   = e_rd_we[h0] ? e_value[h0]   : 32'd0;
assign commit0_mem_valid  = e_mem_valid[h0];
assign commit0_mem_we     = e_mem_we[h0];
assign commit0_mem_addr   = e_mem_valid[h0] ? e_mem_addr[h0]  : 32'd0;
assign commit0_mem_wstrb  = e_mem_we[h0]    ? e_mem_wstrb[h0] : 4'd0;
assign commit0_mem_wdata  = e_mem_we[h0]    ? e_mem_wdata[h0] : 32'd0;
assign commit0_is_ebreak  = e_is_ebreak[h0];
assign commit0_is_fence_i = e_is_fence_i[h0];
assign commit0_is_br      = (e_fu_type[h0] == FU_BR);
assign commit0_br_taken   = e_act_taken[h0];
assign commit0_mispred    = commit0_is_br && e_mispred[h0];
assign commit0_used_ras   = commit0_is_br && e_pred_uras[h0];

// ROB-Head Full Flush: either Speculative Load Replay (re-fetch h0) OR FENCE.I (re-fetch h0+4)
wire fence_i_flush_fire        = commit0_valid && e_is_fence_i[h0];
assign replay_flush_out        = h0_replay_fire;
assign full_flush_out          = h0_replay_fire || fence_i_flush_fire;
assign full_flush_redirect_pc  = h0_replay_fire ? e_pc[h0] : (e_pc[h0] + 32'd4);
assign full_flush_after_seq    = h0_replay_fire ? (e_seq_id[h0] - 32'd1) : e_seq_id[h0];

// Commit Port 1 (younger: h1) — never crosses special serializing ops, replay, or 2nd store
wire h0_serial   = e_is_ebreak[h0] || e_is_fence[h0] || e_is_fence_i[h0] || e_is_csr[h0] || e_is_amo[h0];
wire h1_serial   = e_is_ebreak[h1] || e_is_fence[h1] || e_is_fence_i[h1] || e_is_csr[h1] || e_is_amo[h1];
wire both_stores = (e_fu_type[h0] == FU_STORE) && (e_fu_type[h1] == FU_STORE);
wire h1_replay_b = (e_fu_type[h1] == FU_LOAD) && lsq_head1_replay;

assign commit1_valid     = commit0_valid &&
                           (count >= {{(TAG_W-1){1'b0}}, 2'd2}) &&
                           e_valid[h1] && e_ready[h1] &&
                           !h0_serial && !h1_serial && !both_stores && !h1_replay_b;
assign commit1_tag       = h1;
assign commit1_seq_id    = e_seq_id[h1];
assign commit1_pc        = e_pc[h1];
assign commit1_instr     = e_instr[h1];
assign commit1_fu_type   = e_fu_type[h1];
assign commit1_rd_we     = e_rd_we[h1];
assign commit1_rd_addr   = e_rd_we[h1] ? e_rd_addr[h1] : 5'd0;
assign commit1_rd_wdata  = e_rd_we[h1] ? e_value[h1]   : 32'd0;
assign commit1_mem_valid = e_mem_valid[h1];
assign commit1_mem_we    = e_mem_we[h1];
assign commit1_mem_addr  = e_mem_valid[h1] ? e_mem_addr[h1]  : 32'd0;
assign commit1_mem_wstrb = e_mem_we[h1]    ? e_mem_wstrb[h1] : 4'd0;
assign commit1_mem_wdata = e_mem_we[h1]    ? e_mem_wdata[h1] : 32'd0;
assign commit1_is_br     = (e_fu_type[h1] == FU_BR);
assign commit1_br_taken  = e_act_taken[h1];
assign commit1_mispred   = commit1_is_br && e_mispred[h1];
assign commit1_used_ras  = commit1_is_br && e_pred_uras[h1];

// Branch misprediction detection on CDB0 (suppressed if full_flush takes precedence!)
wire cdb0_mispred_calc = (cdb0_br_taken != e_pred_taken[cdb0_tag]) ||
                         (cdb0_br_taken && (cdb0_br_target != e_pred_tgt[cdb0_tag]));
wire cdb0_br_fire      = cdb0_valid && cdb0_is_br && e_valid[cdb0_tag] && !full_flush_out;

assign cdb0_br_ok         = cdb0_br_fire && !cdb0_mispred_calc;
assign cdb0_br_pc         = e_pc[cdb0_tag];
assign cdb0_br_is_cond    = (e_instr[cdb0_tag][6:0] == OP_BRANCH);
assign cdb0_br_is_jalr    = (e_instr[cdb0_tag][6:0] == OP_JALR);
assign cdb0_br_ghr        = e_pred_ghr[cdb0_tag];
assign cdb0_br_pred_bim   = e_pred_bim[cdb0_tag];
assign cdb0_br_pred_gsh   = e_pred_gsh[cdb0_tag];
assign squash_out         = cdb0_br_fire && cdb0_mispred_calc;
assign squash_tag_out     = cdb0_tag;
assign squash_seq_out     = e_seq_id[cdb0_tag];
assign squash_redirect_pc = cdb0_br_taken ? cdb0_br_target : (e_pc[cdb0_tag] + 32'd4);
assign squash_has_older   = squash_out && (cdb0_tag != head_ptr);

assign disp_free_ge1 = (count <= (DEPTH[TAG_W:0] - {{TAG_W{1'b0}}, 1'b1})) && !squash_out && !full_flush_out;
assign disp_free_ge2 = (count <= (DEPTH[TAG_W:0] - {{(TAG_W-1){1'b0}}, 2'd2})) && !squash_out && !full_flush_out;

// 4 Combinational operand value read ports
assign rd0_ready = e_valid[rd0_tag] && e_ready[rd0_tag];
assign rd0_value = e_value[rd0_tag];
assign rd1_ready = e_valid[rd1_tag] && e_ready[rd1_tag];
assign rd1_value = e_value[rd1_tag];
assign rd2_ready = e_valid[rd2_tag] && e_ready[rd2_tag];
assign rd2_value = e_value[rd2_tag];
assign rd3_ready = e_valid[rd3_tag] && e_ready[rd3_tag];
assign rd3_value = e_value[rd3_tag];

wire [TAG_W-1:0] sq_br_age = cdb0_tag - head_ptr;
wire [TAG_W:0]   n_comm    = commit1_valid ? {{(TAG_W-1){1'b0}}, 2'd2} :
                             commit0_valid ? {{TAG_W{1'b0}}, 1'b1}     : {(TAG_W+1){1'b0}};
wire             do_disp0  = disp0_valid && disp_free_ge1;
wire             do_disp1  = do_disp0 && disp1_valid && disp_free_ge2;
wire [TAG_W:0]   n_disp    = do_disp1 ? {{(TAG_W-1){1'b0}}, 2'd2} :
                             do_disp0 ? {{TAG_W{1'b0}}, 1'b1}     : {(TAG_W+1){1'b0}};

integer i;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        head_ptr <= {TAG_W{1'b0}};
        tail_ptr <= {TAG_W{1'b0}};
        count    <= {(TAG_W+1){1'b0}};
        for (i = 0; i < DEPTH; i = i + 1) begin
            e_valid[i]      <= 1'b0;
            e_ready[i]      <= 1'b0;
            e_seq_id[i]     <= 32'd0;
            e_pc[i]         <= 32'd0;
            e_instr[i]      <= INSTR_NOP;
            e_fu_type[i]    <= FU_ALU;
            e_rd_we[i]      <= 1'b0;
            e_rd_addr[i]    <= 5'd0;
            e_value[i]      <= 32'd0;
            e_pred_taken[i] <= 1'b0;
            e_pred_tgt[i]   <= 32'd0;
            e_pred_ghr[i]   <= 8'd0;
            e_pred_uras[i]  <= 1'b0;
            e_pred_bim[i]   <= 1'b0;
            e_pred_gsh[i]   <= 1'b0;
            e_act_taken[i]  <= 1'b0;
            e_act_tgt[i]    <= 32'd0;
            e_mispred[i]    <= 1'b0;
            e_mem_valid[i]  <= 1'b0;
            e_mem_we[i]     <= 1'b0;
            e_mem_addr[i]   <= 32'd0;
            e_mem_wstrb[i]  <= 4'd0;
            e_mem_wdata[i]  <= 32'd0;
            e_is_ebreak[i]  <= 1'b0;
            e_is_fence[i]   <= 1'b0;
            e_is_fence_i[i] <= 1'b0;
            e_is_csr[i]     <= 1'b0;
            e_is_amo[i]     <= 1'b0;
        end
    end else if (full_flush_out) begin
        // Full flush at ROB head (FENCE.I already committed on commit0_valid; Replay did not)
        head_ptr <= {TAG_W{1'b0}};
        tail_ptr <= {TAG_W{1'b0}};
        count    <= {(TAG_W+1){1'b0}};
        for (i = 0; i < DEPTH; i = i + 1) begin
            e_valid[i] <= 1'b0;
            e_ready[i] <= 1'b0;
        end
    end else if (squash_out) begin
        e_ready[cdb0_tag]     <= 1'b1;
        e_value[cdb0_tag]     <= cdb0_val;
        e_act_taken[cdb0_tag] <= cdb0_br_taken;
        e_act_tgt[cdb0_tag]   <= cdb0_br_target;
        e_mispred[cdb0_tag]   <= 1'b1;

        if (cdb1_valid && e_valid[cdb1_tag] && ((cdb1_tag - head_ptr) <= sq_br_age)) begin
            e_ready[cdb1_tag] <= 1'b1;
            e_value[cdb1_tag] <= cdb1_val;
        end

        if (commit0_valid) begin
            e_valid[h0] <= 1'b0;
            e_ready[h0] <= 1'b0;
        end
        if (commit1_valid) begin
            e_valid[h1] <= 1'b0;
            e_ready[h1] <= 1'b0;
        end
        head_ptr <= head_ptr + n_comm[TAG_W-1:0];

        for (i = 0; i < DEPTH; i = i + 1) begin
            if (e_valid[i] && ((i[TAG_W-1:0] - head_ptr) > sq_br_age)) begin
                e_valid[i] <= 1'b0;
                e_ready[i] <= 1'b0;
            end
        end

        tail_ptr <= cdb0_tag + {{(TAG_W-1){1'b0}}, 1'b1};
        count    <= ({1'b0, sq_br_age} + {{TAG_W{1'b0}}, 1'b1}) - n_comm;
    end else begin
        // 1. Dual-CDB out-of-order writebacks
        if (cdb0_valid && e_valid[cdb0_tag]) begin
            e_ready[cdb0_tag] <= 1'b1;
            e_value[cdb0_tag] <= cdb0_val;
            if (cdb0_is_br) begin
                e_act_taken[cdb0_tag] <= cdb0_br_taken;
                e_act_tgt[cdb0_tag]   <= cdb0_br_target;
                e_mispred[cdb0_tag]   <= cdb0_mispred_calc;
            end
            if (cdb0_is_mem) begin
                e_mem_valid[cdb0_tag] <= 1'b1;
                e_mem_we[cdb0_tag]    <= cdb0_mem_we;
                e_mem_addr[cdb0_tag]  <= cdb0_mem_addr;
                e_mem_wstrb[cdb0_tag] <= cdb0_mem_wstrb;
                e_mem_wdata[cdb0_tag] <= cdb0_mem_wdata;
            end
        end
        if (cdb1_valid && e_valid[cdb1_tag]) begin
            e_ready[cdb1_tag] <= 1'b1;
            e_value[cdb1_tag] <= cdb1_val;
        end

        // 2. 2-Wide strict in-order head retirement
        if (commit0_valid) begin
            e_valid[h0] <= 1'b0;
            e_ready[h0] <= 1'b0;
        end
        if (commit1_valid) begin
            e_valid[h1] <= 1'b0;
            e_ready[h1] <= 1'b0;
        end
        head_ptr <= head_ptr + n_comm[TAG_W-1:0];

        // 3. 2-Wide in-order tail allocation
        if (do_disp0) begin
            e_valid[t0]      <= 1'b1;
            e_ready[t0]      <= disp0_ready_init;
            e_seq_id[t0]     <= disp0_seq_id;
            e_pc[t0]         <= disp0_pc;
            e_instr[t0]      <= disp0_instr;
            e_fu_type[t0]    <= disp0_fu_type;
            e_rd_we[t0]      <= disp0_rd_we;
            e_rd_addr[t0]    <= disp0_rd_addr;
            e_value[t0]      <= 32'd0;
            e_pred_taken[t0] <= disp0_pred_taken;
            e_pred_tgt[t0]   <= disp0_pred_target;
            e_pred_ghr[t0]   <= disp0_pred_ghr;
            e_pred_uras[t0]  <= disp0_pred_used_ras;
            e_pred_bim[t0]   <= disp0_pred_bim;
            e_pred_gsh[t0]   <= disp0_pred_gsh;
            e_act_taken[t0]  <= 1'b0;
            e_act_tgt[t0]    <= 32'd0;
            e_mispred[t0]    <= 1'b0;
            e_mem_valid[t0]  <= 1'b0;
            e_mem_we[t0]     <= 1'b0;
            e_mem_addr[t0]   <= 32'd0;
            e_mem_wstrb[t0]  <= 4'd0;
            e_mem_wdata[t0]  <= 32'd0;
            e_is_ebreak[t0]  <= disp0_is_ebreak;
            e_is_fence[t0]   <= disp0_is_fence;
            e_is_fence_i[t0] <= disp0_is_fence_i;
            e_is_csr[t0]     <= disp0_is_csr;
            e_is_amo[t0]     <= disp0_is_amo;
        end
        if (do_disp1) begin
            e_valid[t1]      <= 1'b1;
            e_ready[t1]      <= disp1_ready_init;
            e_seq_id[t1]     <= disp1_seq_id;
            e_pc[t1]         <= disp1_pc;
            e_instr[t1]      <= disp1_instr;
            e_fu_type[t1]    <= disp1_fu_type;
            e_rd_we[t1]      <= disp1_rd_we;
            e_rd_addr[t1]    <= disp1_rd_addr;
            e_value[t1]      <= 32'd0;
            e_pred_taken[t1] <= disp1_pred_taken;
            e_pred_tgt[t1]   <= disp1_pred_target;
            e_pred_ghr[t1]   <= disp1_pred_ghr;
            e_pred_uras[t1]  <= disp1_pred_used_ras;
            e_pred_bim[t1]   <= disp1_pred_bim;
            e_pred_gsh[t1]   <= disp1_pred_gsh;
            e_act_taken[t1]  <= 1'b0;
            e_act_tgt[t1]    <= 32'd0;
            e_mispred[t1]    <= 1'b0;
            e_mem_valid[t1]  <= 1'b0;
            e_mem_we[t1]     <= 1'b0;
            e_mem_addr[t1]   <= 32'd0;
            e_mem_wstrb[t1]  <= 4'd0;
            e_mem_wdata[t1]  <= 32'd0;
            e_is_ebreak[t1]  <= disp1_is_ebreak;
            e_is_fence[t1]   <= disp1_is_fence;
            e_is_fence_i[t1] <= disp1_is_fence_i;
            e_is_csr[t1]     <= disp1_is_csr;
            e_is_amo[t1]     <= disp1_is_amo;
        end
        tail_ptr <= tail_ptr + n_disp[TAG_W-1:0];
        count    <= count + n_disp - n_comm;
    end
end

endmodule
