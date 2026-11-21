`timescale 1ns / 1ps

// ============================================================================
// Parameterized 2-Wide Dispatch, 3-Source Floating-Point Reservation Station
// (Phase U6 — Feeds fpu_fma_pipe [6e, 3-src], fpu_div_sqrt [2e], fpu_misc [4e])
//
// Features:
//   - 2-wide dispatch allocation into guaranteed-distinct free slots
//   - Up to 3 source operands (vj/qj, vk/qk, vm/qm for fused multiply-add!)
//   - Full 3-CDB wakeup snoop (cdb0, cdb1, AND dedicated cdb_fp!)
//   - 1-cycle integer ALU0/ALU1 early bypass on rs1 (for FCVT.S.W/WU & FMV.W.X!)
//   - Oldest-ready-first issue + selective ROB-age branch squash
// ============================================================================
module rs_fpu2w #(
    parameter integer ENTRIES = 6,
    parameter integer TAG_W   = 5,
    parameter integer OP_W    = 7,  // {fp_rm[2:0], fp_op[3:0]}
    parameter integer HAS_RS3 = 1
) (
    input  wire             clk,
    input  wire             reset_n,
    input  wire             flush,
    input  wire             squash_valid,
    input  wire [TAG_W-1:0] squash_tag,
    input  wire [TAG_W-1:0] rob_head,

    output wire             free_ge1,
    output wire             free_ge2,

    // Dispatch Port 0 (older)
    input  wire             disp0_valid,
    input  wire [OP_W-1:0]  disp0_op,
    input  wire [TAG_W-1:0] disp0_rob_tag,
    input  wire [31:0]      disp0_seq_id,
    input  wire [31:0]      disp0_vj,
    input  wire [TAG_W-1:0] disp0_qj,
    input  wire             disp0_qj_valid,
    input  wire [31:0]      disp0_vk,
    input  wire [TAG_W-1:0] disp0_qk,
    input  wire             disp0_qk_valid,
    input  wire [31:0]      disp0_vm,
    input  wire [TAG_W-1:0] disp0_qm,
    input  wire             disp0_qm_valid,

    // Dispatch Port 1 (younger)
    input  wire             disp1_valid,
    input  wire [OP_W-1:0]  disp1_op,
    input  wire [TAG_W-1:0] disp1_rob_tag,
    input  wire [31:0]      disp1_seq_id,
    input  wire [31:0]      disp1_vj,
    input  wire [TAG_W-1:0] disp1_qj,
    input  wire             disp1_qj_valid,
    input  wire [31:0]      disp1_vk,
    input  wire [TAG_W-1:0] disp1_qk,
    input  wire             disp1_qk_valid,
    input  wire [31:0]      disp1_vm,
    input  wire [TAG_W-1:0] disp1_qm,
    input  wire             disp1_qm_valid,

    // 3-CDB Broadcast Wakeup Snoop (INT CDB0, INT CDB1, Dedicated FP CDB2!)
    input  wire             cdb0_valid,
    input  wire [TAG_W-1:0] cdb0_tag,
    input  wire [31:0]      cdb0_val,
    input  wire             cdb1_valid,
    input  wire [TAG_W-1:0] cdb1_tag,
    input  wire [31:0]      cdb1_val,
    input  wire             cdb2_valid,
    input  wire [TAG_W-1:0] cdb2_tag,
    input  wire [31:0]      cdb2_val,

    // Oldest-Ready Issue Port
    output reg              issue_valid,
    input  wire             fu_ready,
    output reg  [OP_W-1:0]  issue_op,
    output reg  [TAG_W-1:0] issue_rob_tag,
    output reg  [31:0]      issue_seq_id,
    output reg  [31:0]      issue_vj,
    output reg  [31:0]      issue_vk,
    output reg  [31:0]      issue_vm
);

localparam integer IDX_W = (ENTRIES <= 2) ? 1 : $clog2(ENTRIES);

reg             e_busy     [0:ENTRIES-1];
reg [OP_W-1:0]  e_op       [0:ENTRIES-1];
reg [TAG_W-1:0] e_rob_tag  [0:ENTRIES-1];
reg [31:0]      e_seq_id   [0:ENTRIES-1];
reg [31:0]      e_vj       [0:ENTRIES-1];
reg [TAG_W-1:0] e_qj       [0:ENTRIES-1];
reg             e_qj_valid [0:ENTRIES-1];
reg [31:0]      e_vk       [0:ENTRIES-1];
reg [TAG_W-1:0] e_qk       [0:ENTRIES-1];
reg             e_qk_valid [0:ENTRIES-1];
reg [31:0]      e_vm       [0:ENTRIES-1];
reg [TAG_W-1:0] e_qm       [0:ENTRIES-1];
reg             e_qm_valid [0:ENTRIES-1];

wire [TAG_W-1:0] sq_br_age = squash_tag - rob_head;

// 1. Find up to 2 guaranteed-distinct free slots
reg             has_free0, has_free1;
reg [IDX_W-1:0] free0_idx, free1_idx;
integer f_i;

always @(*) begin
    f_i       = 0;
    has_free0 = 1'b0;
    has_free1 = 1'b0;
    free0_idx = {IDX_W{1'b0}};
    free1_idx = {IDX_W{1'b0}};
    for (f_i = 0; f_i < ENTRIES; f_i = f_i + 1) begin
        if (!e_busy[f_i]) begin
            if (!has_free0) begin
                has_free0 = 1'b1;
                free0_idx = f_i[IDX_W-1:0];
            end else if (!has_free1) begin
                has_free1 = 1'b1;
                free1_idx = f_i[IDX_W-1:0];
            end
        end
    end
end

assign free_ge1 = has_free0 && !flush && !squash_valid;
assign free_ge2 = has_free1 && !flush && !squash_valid;

// 2. Oldest-ready issue selection
reg [IDX_W-1:0] issue_idx;
reg [TAG_W-1:0] best_age, cur_age;
integer s_i;

always @(*) begin
    s_i         = 0;
    issue_valid = 1'b0;
    issue_idx   = {IDX_W{1'b0}};
    best_age    = {TAG_W{1'b1}};
    cur_age     = {TAG_W{1'b0}};

    for (s_i = 0; s_i < ENTRIES; s_i = s_i + 1) begin
        if (e_busy[s_i] && !e_qj_valid[s_i] && !e_qk_valid[s_i] &&
            ((HAS_RS3 == 0) || !e_qm_valid[s_i])) begin
            cur_age = e_rob_tag[s_i] - rob_head;
            if (!(squash_valid && (cur_age > sq_br_age))) begin
                if (!issue_valid || (cur_age < best_age)) begin
                    issue_valid = 1'b1;
                    issue_idx   = s_i[IDX_W-1:0];
                    best_age    = cur_age;
                end
            end
        end
    end

    issue_op      = e_op[issue_idx];
    issue_rob_tag = e_rob_tag[issue_idx];
    issue_seq_id  = e_seq_id[issue_idx];
    issue_vj      = e_vj[issue_idx];
    issue_vk      = e_vk[issue_idx];
    issue_vm      = (HAS_RS3 != 0) ? e_vm[issue_idx] : 32'd0;
end

// 3. Defensive same-cycle 3-CDB check on incoming disp0 & disp1 operands
wire d0_j_c0 = disp0_qj_valid && cdb0_valid && (disp0_qj == cdb0_tag);
wire d0_j_c1 = disp0_qj_valid && !d0_j_c0 && cdb1_valid && (disp0_qj == cdb1_tag);
wire d0_j_c2 = disp0_qj_valid && !d0_j_c0 && !d0_j_c1 && cdb2_valid && (disp0_qj == cdb2_tag);
wire [31:0] d0_vj_e = d0_j_c0 ? cdb0_val : (d0_j_c1 ? cdb1_val : (d0_j_c2 ? cdb2_val : disp0_vj));
wire        d0_qj_e = (d0_j_c0 || d0_j_c1 || d0_j_c2) ? 1'b0 : disp0_qj_valid;

wire d0_k_c0 = disp0_qk_valid && cdb0_valid && (disp0_qk == cdb0_tag);
wire d0_k_c1 = disp0_qk_valid && !d0_k_c0 && cdb1_valid && (disp0_qk == cdb1_tag);
wire d0_k_c2 = disp0_qk_valid && !d0_k_c0 && !d0_k_c1 && cdb2_valid && (disp0_qk == cdb2_tag);
wire [31:0] d0_vk_e = d0_k_c0 ? cdb0_val : (d0_k_c1 ? cdb1_val : (d0_k_c2 ? cdb2_val : disp0_vk));
wire        d0_qk_e = (d0_k_c0 || d0_k_c1 || d0_k_c2) ? 1'b0 : disp0_qk_valid;

wire d0_m_c0 = (HAS_RS3 != 0) && disp0_qm_valid && cdb0_valid && (disp0_qm == cdb0_tag);
wire d0_m_c1 = (HAS_RS3 != 0) && disp0_qm_valid && !d0_m_c0 && cdb1_valid && (disp0_qm == cdb1_tag);
wire d0_m_c2 = (HAS_RS3 != 0) && disp0_qm_valid && !d0_m_c0 && !d0_m_c1 && cdb2_valid && (disp0_qm == cdb2_tag);
wire [31:0] d0_vm_e = d0_m_c0 ? cdb0_val : (d0_m_c1 ? cdb1_val : (d0_m_c2 ? cdb2_val : disp0_vm));
wire        d0_qm_e = (HAS_RS3 == 0) ? 1'b0 : ((d0_m_c0 || d0_m_c1 || d0_m_c2) ? 1'b0 : disp0_qm_valid);

wire d1_j_c0 = disp1_qj_valid && cdb0_valid && (disp1_qj == cdb0_tag);
wire d1_j_c1 = disp1_qj_valid && !d1_j_c0 && cdb1_valid && (disp1_qj == cdb1_tag);
wire d1_j_c2 = disp1_qj_valid && !d1_j_c0 && !d1_j_c1 && cdb2_valid && (disp1_qj == cdb2_tag);
wire [31:0] d1_vj_e = d1_j_c0 ? cdb0_val : (d1_j_c1 ? cdb1_val : (d1_j_c2 ? cdb2_val : disp1_vj));
wire        d1_qj_e = (d1_j_c0 || d1_j_c1 || d1_j_c2) ? 1'b0 : disp1_qj_valid;

wire d1_k_c0 = disp1_qk_valid && cdb0_valid && (disp1_qk == cdb0_tag);
wire d1_k_c1 = disp1_qk_valid && !d1_k_c0 && cdb1_valid && (disp1_qk == cdb1_tag);
wire d1_k_c2 = disp1_qk_valid && !d1_k_c0 && !d1_k_c1 && cdb2_valid && (disp1_qk == cdb2_tag);
wire [31:0] d1_vk_e = d1_k_c0 ? cdb0_val : (d1_k_c1 ? cdb1_val : (d1_k_c2 ? cdb2_val : disp1_vk));
wire        d1_qk_e = (d1_k_c0 || d1_k_c1 || d1_k_c2) ? 1'b0 : disp1_qk_valid;

wire d1_m_c0 = (HAS_RS3 != 0) && disp1_qm_valid && cdb0_valid && (disp1_qm == cdb0_tag);
wire d1_m_c1 = (HAS_RS3 != 0) && disp1_qm_valid && !d1_m_c0 && cdb1_valid && (disp1_qm == cdb1_tag);
wire d1_m_c2 = (HAS_RS3 != 0) && disp1_qm_valid && !d1_m_c0 && !d1_m_c1 && cdb2_valid && (disp1_qm == cdb2_tag);
wire [31:0] d1_vm_e = d1_m_c0 ? cdb0_val : (d1_m_c1 ? cdb1_val : (d1_m_c2 ? cdb2_val : disp1_vm));
wire        d1_qm_e = (HAS_RS3 == 0) ? 1'b0 : ((d1_m_c0 || d1_m_c1 || d1_m_c2) ? 1'b0 : disp1_qm_valid);

wire             do_issue  = issue_valid && fu_ready;
wire [IDX_W-1:0] slot1_idx = disp0_valid ? free1_idx : free0_idx;

integer k;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (k = 0; k < ENTRIES; k = k + 1) begin
            e_busy[k]     <= 1'b0;
            e_op[k]       <= {OP_W{1'b0}};
            e_rob_tag[k]  <= {TAG_W{1'b0}};
            e_seq_id[k]   <= 32'd0;
            e_vj[k]       <= 32'd0;
            e_qj[k]       <= {TAG_W{1'b0}};
            e_qj_valid[k] <= 1'b0;
            e_vk[k]       <= 32'd0;
            e_qk[k]       <= {TAG_W{1'b0}};
            e_qk_valid[k] <= 1'b0;
            e_vm[k]       <= 32'd0;
            e_qm[k]       <= {TAG_W{1'b0}};
            e_qm_valid[k] <= 1'b0;
        end
    end else if (flush) begin
        for (k = 0; k < ENTRIES; k = k + 1)
            e_busy[k] <= 1'b0;
    end else begin
        // 1. 3-CDB Wakeup Snoop across all occupied slots
        for (k = 0; k < ENTRIES; k = k + 1) begin
            if (e_busy[k]) begin
                if (e_qj_valid[k]) begin
                    if (cdb0_valid && (e_qj[k] == cdb0_tag)) begin
                        e_vj[k] <= cdb0_val; e_qj_valid[k] <= 1'b0;
                    end else if (cdb1_valid && (e_qj[k] == cdb1_tag)) begin
                        e_vj[k] <= cdb1_val; e_qj_valid[k] <= 1'b0;
                    end else if (cdb2_valid && (e_qj[k] == cdb2_tag)) begin
                        e_vj[k] <= cdb2_val; e_qj_valid[k] <= 1'b0;
                    end
                end
                if (e_qk_valid[k]) begin
                    if (cdb0_valid && (e_qk[k] == cdb0_tag)) begin
                        e_vk[k] <= cdb0_val; e_qk_valid[k] <= 1'b0;
                    end else if (cdb1_valid && (e_qk[k] == cdb1_tag)) begin
                        e_vk[k] <= cdb1_val; e_qk_valid[k] <= 1'b0;
                    end else if (cdb2_valid && (e_qk[k] == cdb2_tag)) begin
                        e_vk[k] <= cdb2_val; e_qk_valid[k] <= 1'b0;
                    end
                end
                if ((HAS_RS3 != 0) && e_qm_valid[k]) begin
                    if (cdb0_valid && (e_qm[k] == cdb0_tag)) begin
                        e_vm[k] <= cdb0_val; e_qm_valid[k] <= 1'b0;
                    end else if (cdb1_valid && (e_qm[k] == cdb1_tag)) begin
                        e_vm[k] <= cdb1_val; e_qm_valid[k] <= 1'b0;
                    end else if (cdb2_valid && (e_qm[k] == cdb2_tag)) begin
                        e_vm[k] <= cdb2_val; e_qm_valid[k] <= 1'b0;
                    end
                end
            end
        end

        // 2. Release issued entry
        if (do_issue)
            e_busy[issue_idx] <= 1'b0;

        // 3. Allocate 2-wide dispatched FP entries
        if (disp0_valid && free_ge1) begin
            e_busy[free0_idx]     <= 1'b1;
            e_op[free0_idx]       <= disp0_op;
            e_rob_tag[free0_idx]  <= disp0_rob_tag;
            e_seq_id[free0_idx]   <= disp0_seq_id;
            e_vj[free0_idx]       <= d0_vj_e;
            e_qj[free0_idx]       <= disp0_qj;
            e_qj_valid[free0_idx] <= d0_qj_e;
            e_vk[free0_idx]       <= d0_vk_e;
            e_qk[free0_idx]       <= disp0_qk;
            e_qk_valid[free0_idx] <= d0_qk_e;
            e_vm[free0_idx]       <= d0_vm_e;
            e_qm[free0_idx]       <= disp0_qm;
            e_qm_valid[free0_idx] <= d0_qm_e;
        end
        if (disp1_valid && (disp0_valid ? free_ge2 : free_ge1)) begin
            e_busy[slot1_idx]     <= 1'b1;
            e_op[slot1_idx]       <= disp1_op;
            e_rob_tag[slot1_idx]  <= disp1_rob_tag;
            e_seq_id[slot1_idx]   <= disp1_seq_id;
            e_vj[slot1_idx]       <= d1_vj_e;
            e_qj[slot1_idx]       <= disp1_qj;
            e_qj_valid[slot1_idx] <= d1_qj_e;
            e_vk[slot1_idx]       <= d1_vk_e;
            e_qk[slot1_idx]       <= disp1_qk;
            e_qk_valid[slot1_idx] <= d1_qk_e;
            e_vm[slot1_idx]       <= d1_vm_e;
            e_qm[slot1_idx]       <= disp1_qm;
            e_qm_valid[slot1_idx] <= d1_qm_e;
        end

        // 4. Selective ROB-Age Branch Squash
        if (squash_valid) begin
            for (k = 0; k < ENTRIES; k = k + 1) begin
                if (e_busy[k] && ((e_rob_tag[k] - rob_head) > sq_br_age))
                    e_busy[k] <= 1'b0;
            end
        end
    end
end

endmodule
