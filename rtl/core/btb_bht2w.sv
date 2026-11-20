`timescale 1ns / 1ps

// ============================================================================
// Dual-Lookup Hybrid Tournament Dynamic Branch Predictor (Upgrade U2):
//   - 256-Entry Bimodal 2-bit BHT (instant 1-iter warm-up on simple loops!)
//   - 256-Entry GSHARE 2-bit BHT (8-bit speculative GHR + exact ROB recovery,
//     learns correlated & alternating T-N-T-N branches!)
//   - 256-Entry 2-bit Tournament Choice Table (selects Bimodal vs. GSHARE!)
//   - 8-Entry Speculative Return Address Stack (RAS) with Dispatch-Order RAS
//     & 4 Coherent Branch Checkpoints (100% exact recovery across nested
//     wrong-path calls/returns!)
//   - 32-Entry BTB (indirect JALR jumps/calls) + 0-cycle Fetch JAL/BRANCH target
// ============================================================================
module btb_bht2w #(
    parameter integer BTB_ENTRIES = 32,
    parameter integer BHT_ENTRIES = 256,
    parameter integer RAS_DEPTH   = 8,
    parameter integer TAG_W       = 5,
    parameter integer NUM_CP      = 4
) (
    input  wire             clk,
    input  wire             reset_n,

    // Dual combinational fetch lookup ports (with pre-decoded instructions!)
    input  wire [31:0]      req_pc0,
    input  wire [31:0]      req_instr0,
    output reg              pred_taken0,
    output reg  [31:0]      pred_target0,
    output wire [7:0]       pred_ghr0,
    output reg              pred_used_ras0,
    output wire             pred_bim0,
    output wire             pred_gsh0,

    input  wire [31:0]      req_pc1,
    input  wire [31:0]      req_instr1,
    output reg              pred_taken1,
    output reg  [31:0]      pred_target1,
    output wire [7:0]       pred_ghr1,
    output reg              pred_used_ras1,
    output wire             pred_bim1,
    output wire             pred_gsh1,

    // Fetch enqueue notifications (advance speculative GHR & Fetch RAS)
    input  wire             enq0_fire,
    input  wire             enq1_fire, // Only 1 when enq0_fire == 1 && !pred_taken0

    // Dispatch-order CALL/RET notifications + 4-CP lockstep with rat2w
    input  wire             disp0_fire,
    input  wire [31:0]      disp0_pc,
    input  wire [31:0]      disp0_instr,
    input  wire             disp1_fire,
    input  wire [31:0]      disp1_pc,
    input  wire [31:0]      disp1_instr,
    input  wire             cp_take,
    input  wire             cp_is_lane1,
    input  wire [TAG_W-1:0] cp_take_tag,

    // Branch resolution & checkpoint recovery (from ROB2W / CDB0)
    input  wire             upd_valid,
    input  wire [31:0]      upd_pc,
    input  wire             upd_is_cond,
    input  wire             upd_is_jalr,
    input  wire             upd_taken,
    input  wire [31:0]      upd_target,
    input  wire [7:0]       upd_ghr,
    input  wire             upd_pred_bim,
    input  wire             upd_pred_gsh,
    input  wire             br_resolve_ok,
    input  wire [TAG_W-1:0] br_resolve_tag,
    input  wire             squash_valid,
    input  wire [TAG_W-1:0] squash_tag,
    input  wire [TAG_W-1:0] rob_head
);

localparam [6:0] OP_JAL    = 7'b1101111;
localparam [6:0] OP_JALR   = 7'b1100111;
localparam [6:0] OP_BRANCH = 7'b1100011;

localparam integer BTB_IDX_W = $clog2(BTB_ENTRIES); // 5
localparam integer BHT_IDX_W = $clog2(BHT_ENTRIES); // 8
localparam integer BTB_TLSB  = 2 + BTB_IDX_W;       // 7
localparam integer BTB_TAG_W = 32 - BTB_TLSB;       // 25

// ----------------------------------------------------------------------------
// 1. Pre-Decode Helpers (RISC-V Unprivileged Spec §2.5 CALL/RET Hints)
// ----------------------------------------------------------------------------
function automatic is_link_reg(input [4:0] r);
    begin
        is_link_reg = (r == 5'd1) || (r == 5'd5);
    end
endfunction

function automatic [31:0] dec_imm_j(input [31:0] ins);
    begin
        dec_imm_j = {{12{ins[31]}}, ins[19:12], ins[20], ins[30:21], 1'b0};
    end
endfunction

function automatic [31:0] dec_imm_b(input [31:0] ins);
    begin
        dec_imm_b = {{20{ins[31]}}, ins[7], ins[30:25], ins[11:8], 1'b0};
    end
endfunction

// Fetch Slot 0 pre-decode
wire [6:0]  f0_op    = req_instr0[6:0];
wire        f0_rd_l  = is_link_reg(req_instr0[11:7]);
wire        f0_rs1_l = is_link_reg(req_instr0[19:15]);
wire        f0_br    = (f0_op == OP_BRANCH);
wire        f0_jal   = (f0_op == OP_JAL);
wire        f0_jalr  = (f0_op == OP_JALR);
wire        f0_call  = (f0_jal && f0_rd_l) || (f0_jalr && f0_rd_l);
wire        f0_ret   = f0_jalr && f0_rs1_l && (!f0_rd_l || (req_instr0[11:7] != req_instr0[19:15]));
wire [31:0] f0_tgt_j = req_pc0 + dec_imm_j(req_instr0);
wire [31:0] f0_tgt_b = req_pc0 + dec_imm_b(req_instr0);

// Fetch Slot 1 pre-decode
wire [6:0]  f1_op    = req_instr1[6:0];
wire        f1_rd_l  = is_link_reg(req_instr1[11:7]);
wire        f1_rs1_l = is_link_reg(req_instr1[19:15]);
wire        f1_br    = (f1_op == OP_BRANCH);
wire        f1_jal   = (f1_op == OP_JAL);
wire        f1_jalr  = (f1_op == OP_JALR);
wire        f1_call  = (f1_jal && f1_rd_l) || (f1_jalr && f1_rd_l);
wire        f1_ret   = f1_jalr && f1_rs1_l && (!f1_rd_l || (req_instr1[11:7] != req_instr1[19:15]));
wire [31:0] f1_tgt_j = req_pc1 + dec_imm_j(req_instr1);
wire [31:0] f1_tgt_b = req_pc1 + dec_imm_b(req_instr1);

// Dispatch Lane 0 / Lane 1 pre-decode (for exact in-order RAS + 4-CP snapshots)
wire [6:0]  d0_op    = disp0_instr[6:0];
wire        d0_rd_l  = is_link_reg(disp0_instr[11:7]);
wire        d0_rs1_l = is_link_reg(disp0_instr[19:15]);
wire        d0_call  = disp0_fire && (((d0_op == OP_JAL) && d0_rd_l) || ((d0_op == OP_JALR) && d0_rd_l));
wire        d0_ret   = disp0_fire && (d0_op == OP_JALR) && d0_rs1_l &&
                       (!d0_rd_l || (disp0_instr[11:7] != disp0_instr[19:15]));

wire [6:0]  d1_op    = disp1_instr[6:0];
wire        d1_rd_l  = is_link_reg(disp1_instr[11:7]);
wire        d1_rs1_l = is_link_reg(disp1_instr[19:15]);
wire        d1_call  = disp1_fire && (((d1_op == OP_JAL) && d1_rd_l) || ((d1_op == OP_JALR) && d1_rd_l));
wire        d1_ret   = disp1_fire && (d1_op == OP_JALR) && d1_rs1_l &&
                       (!d1_rd_l || (disp1_instr[11:7] != disp1_instr[19:15]));

// ----------------------------------------------------------------------------
// 2. Predictor Storage: 32-BTB, 256-Bimodal, 256-GSHARE, 256-Choice, GHR, RAS
// ----------------------------------------------------------------------------
reg                 btb_valid  [0:BTB_ENTRIES-1];
reg [BTB_TAG_W-1:0] btb_tag    [0:BTB_ENTRIES-1];
reg [31:0]          btb_target [0:BTB_ENTRIES-1];

reg [1:0]           bim_bht    [0:BHT_ENTRIES-1]; // PC-indexed bimodal (fast loop warm-up)
reg [1:0]           gsh_bht    [0:BHT_ENTRIES-1]; // (PC ^ GHR)-indexed GSHARE
reg [1:0]           choice     [0:BHT_ENTRIES-1]; // 00/01=Bimodal, 10/11=GSHARE
reg [7:0]           spec_ghr;

// Speculative Fetch RAS (8 entries, circular stack; tos = next push slot)
reg [31:0]          f_ras      [0:RAS_DEPTH-1];
reg [2:0]           f_tos;
reg [3:0]           f_cnt;

// Dispatch-Order RAS + 4 Coherent Branch Checkpoints (lockstep with rat2w)
reg [31:0]          d_ras      [0:RAS_DEPTH-1];
reg [2:0]           d_tos;
reg [3:0]           d_cnt;

reg                 cp_valid   [0:NUM_CP-1];
reg [TAG_W-1:0]     cp_tag     [0:NUM_CP-1];
reg [31:0]          cp_ras     [0:NUM_CP-1][0:RAS_DEPTH-1];
reg [2:0]           cp_tos     [0:NUM_CP-1];
reg [3:0]           cp_cnt     [0:NUM_CP-1];

// ----------------------------------------------------------------------------
// 3. Dual Combinational Fetch Lookup (Slot 0 & Slot 1)
// ----------------------------------------------------------------------------
wire [BTB_IDX_W-1:0] idx0_btb = req_pc0[2 +: BTB_IDX_W];
wire [BTB_TAG_W-1:0] tag0_btb = req_pc0[31:BTB_TLSB];
wire                 hit0_btb = btb_valid[idx0_btb] && (btb_tag[idx0_btb] == tag0_btb);

assign pred_ghr0 = spec_ghr;
wire [BHT_IDX_W-1:0] idx0_bim = req_pc0[2 +: BHT_IDX_W];
wire [BHT_IDX_W-1:0] idx0_gsh = req_pc0[2 +: BHT_IDX_W] ^ pred_ghr0;
assign pred_bim0 = bim_bht[idx0_bim][1];
assign pred_gsh0 = gsh_bht[idx0_gsh][1];
wire                 dir0_tkn = choice[idx0_bim][1] ? pred_gsh0 : pred_bim0;
wire [2:0]           f_top0   = f_tos - 3'd1;

always @(*) begin
    pred_taken0    = 1'b0;
    pred_target0   = req_pc0 + 32'd4;
    pred_used_ras0 = 1'b0;
    if (f0_jal) begin
        pred_taken0  = 1'b1;
        pred_target0 = f0_tgt_j;
    end else if (f0_ret && (f_cnt != 4'd0)) begin
        pred_taken0    = 1'b1;
        pred_target0   = f_ras[f_top0];
        pred_used_ras0 = 1'b1;
    end else if (f0_jalr) begin
        pred_taken0  = hit0_btb;
        pred_target0 = hit0_btb ? btb_target[idx0_btb] : (req_pc0 + 32'd4);
    end else if (f0_br) begin
        pred_taken0  = dir0_tkn;
        pred_target0 = dir0_tkn ? f0_tgt_b : (req_pc0 + 32'd4);
    end
end

// Compute Fetch RAS state after Slot 0 (needed for Slot 1 lookup & enqueue update)
reg [31:0] f_ras_a0 [0:RAS_DEPTH-1];
reg [2:0]  f_tos_a0, f_tos_p0;
reg [3:0]  f_cnt_a0, f_cnt_p0;
integer    r0;

always @(*) begin
    r0 = 0;
    for (r0 = 0; r0 < RAS_DEPTH; r0 = r0 + 1)
        f_ras_a0[r0] = f_ras[r0];
    if (f0_ret && (f_cnt != 4'd0)) begin
        f_tos_p0 = f_tos - 3'd1;
        f_cnt_p0 = f_cnt - 4'd1;
    end else begin
        f_tos_p0 = f_tos;
        f_cnt_p0 = f_cnt;
    end
    if (f0_call) begin
        f_ras_a0[f_tos_p0] = req_pc0 + 32'd4;
        f_tos_a0           = f_tos_p0 + 3'd1;
        f_cnt_a0           = (f_cnt_p0 < 4'd8) ? (f_cnt_p0 + 4'd1) : 4'd8;
    end else begin
        f_tos_a0 = f_tos_p0;
        f_cnt_a0 = f_cnt_p0;
    end
end

// Compute Fetch RAS state after Slot 1 (used when enq1_fire == 1)
reg [31:0] f_ras_a1 [0:RAS_DEPTH-1];
reg [2:0]  f_tos_a1, f_tos_p1;
reg [3:0]  f_cnt_a1, f_cnt_p1;
integer    r1;

always @(*) begin
    r1 = 0;
    for (r1 = 0; r1 < RAS_DEPTH; r1 = r1 + 1)
        f_ras_a1[r1] = f_ras_a0[r1];
    if (f1_ret && (f_cnt_a0 != 4'd0)) begin
        f_tos_p1 = f_tos_a0 - 3'd1;
        f_cnt_p1 = f_cnt_a0 - 4'd1;
    end else begin
        f_tos_p1 = f_tos_a0;
        f_cnt_p1 = f_cnt_a0;
    end
    if (f1_call) begin
        f_ras_a1[f_tos_p1] = req_pc1 + 32'd4;
        f_tos_a1           = f_tos_p1 + 3'd1;
        f_cnt_a1           = (f_cnt_p1 < 4'd8) ? (f_cnt_p1 + 4'd1) : 4'd8;
    end else begin
        f_tos_a1 = f_tos_p1;
        f_cnt_a1 = f_cnt_p1;
    end
end

// Slot 1 lookup (assumes Slot 0 was not taken -> if f0_br, shift 0 into GHR!)
wire [BTB_IDX_W-1:0] idx1_btb = req_pc1[2 +: BTB_IDX_W];
wire [BTB_TAG_W-1:0] tag1_btb = req_pc1[31:BTB_TLSB];
wire                 hit1_btb = btb_valid[idx1_btb] && (btb_tag[idx1_btb] == tag1_btb);

assign pred_ghr1 = f0_br ? {spec_ghr[6:0], 1'b0} : spec_ghr;
wire [BHT_IDX_W-1:0] idx1_bim = req_pc1[2 +: BHT_IDX_W];
wire [BHT_IDX_W-1:0] idx1_gsh = req_pc1[2 +: BHT_IDX_W] ^ pred_ghr1;
assign pred_bim1 = bim_bht[idx1_bim][1];
assign pred_gsh1 = gsh_bht[idx1_gsh][1];
wire                 dir1_tkn = choice[idx1_bim][1] ? pred_gsh1 : pred_bim1;
wire [2:0]           f_top1   = f_tos_a0 - 3'd1;

always @(*) begin
    pred_taken1    = 1'b0;
    pred_target1   = req_pc1 + 32'd4;
    pred_used_ras1 = 1'b0;
    if (f1_jal) begin
        pred_taken1  = 1'b1;
        pred_target1 = f1_tgt_j;
    end else if (f1_ret && (f_cnt_a0 != 4'd0)) begin
        pred_taken1    = 1'b1;
        pred_target1   = f_ras_a0[f_top1];
        pred_used_ras1 = 1'b1;
    end else if (f1_jalr) begin
        pred_taken1  = hit1_btb;
        pred_target1 = hit1_btb ? btb_target[idx1_btb] : (req_pc1 + 32'd4);
    end else if (f1_br) begin
        pred_taken1  = dir1_tkn;
        pred_target1 = dir1_tkn ? f1_tgt_b : (req_pc1 + 32'd4);
    end
end

// ----------------------------------------------------------------------------
// 4. Dispatch-Order RAS Transitions & 4-Checkpoint Slot Management
// ----------------------------------------------------------------------------
reg [31:0] d_ras_a0 [0:RAS_DEPTH-1];
reg [2:0]  d_tos_a0, d_tos_p0;
reg [3:0]  d_cnt_a0, d_cnt_p0;

reg [31:0] d_ras_a1 [0:RAS_DEPTH-1];
reg [2:0]  d_tos_a1, d_tos_p1;
reg [3:0]  d_cnt_a1, d_cnt_p1;
integer    dr;

always @(*) begin
    dr = 0;
    for (dr = 0; dr < RAS_DEPTH; dr = dr + 1)
        d_ras_a0[dr] = d_ras[dr];
    if (d0_ret && (d_cnt != 4'd0)) begin
        d_tos_p0 = d_tos - 3'd1;
        d_cnt_p0 = d_cnt - 4'd1;
    end else begin
        d_tos_p0 = d_tos;
        d_cnt_p0 = d_cnt;
    end
    if (d0_call) begin
        d_ras_a0[d_tos_p0] = disp0_pc + 32'd4;
        d_tos_a0           = d_tos_p0 + 3'd1;
        d_cnt_a0           = (d_cnt_p0 < 4'd8) ? (d_cnt_p0 + 4'd1) : 4'd8;
    end else begin
        d_tos_a0 = d_tos_p0;
        d_cnt_a0 = d_cnt_p0;
    end

    for (dr = 0; dr < RAS_DEPTH; dr = dr + 1)
        d_ras_a1[dr] = d_ras_a0[dr];
    if (d1_ret && (d_cnt_a0 != 4'd0)) begin
        d_tos_p1 = d_tos_a0 - 3'd1;
        d_cnt_p1 = d_cnt_a0 - 4'd1;
    end else begin
        d_tos_p1 = d_tos_a0;
        d_cnt_p1 = d_cnt_a0;
    end
    if (d1_call) begin
        d_ras_a1[d_tos_p1] = disp1_pc + 32'd4;
        d_tos_a1           = d_tos_p1 + 3'd1;
        d_cnt_a1           = (d_cnt_p1 < 4'd8) ? (d_cnt_p1 + 4'd1) : 4'd8;
    end else begin
        d_tos_a1 = d_tos_p1;
        d_cnt_a1 = d_cnt_p1;
    end
end

reg       cp_has_free, cp_sq_found;
reg [1:0] cp_free_idx, cp_sq_idx;
integer   ci;

always @(*) begin
    ci          = 0;
    cp_has_free = 1'b0;
    cp_free_idx = 2'd0;
    cp_sq_found = 1'b0;
    cp_sq_idx   = 2'd0;
    for (ci = NUM_CP - 1; ci >= 0; ci = ci - 1) begin
        if (!cp_valid[ci] || (br_resolve_ok && (cp_tag[ci] == br_resolve_tag))) begin
            cp_has_free = 1'b1;
            cp_free_idx = ci[1:0];
        end
        if (cp_valid[ci] && (cp_tag[ci] == squash_tag)) begin
            cp_sq_found = 1'b1;
            cp_sq_idx   = ci[1:0];
        end
    end
end

// Training indices & tournament agreement checks
wire [BTB_IDX_W-1:0] upd_btb_idx = upd_pc[2 +: BTB_IDX_W];
wire [BTB_TAG_W-1:0] upd_btb_tag = upd_pc[31:BTB_TLSB];
wire [BHT_IDX_W-1:0] upd_bim_idx = upd_pc[2 +: BHT_IDX_W];
wire [BHT_IDX_W-1:0] upd_gsh_idx = upd_pc[2 +: BHT_IDX_W] ^ upd_ghr;
wire                 upd_bim_ok  = (upd_pred_bim == upd_taken);
wire                 upd_gsh_ok  = (upd_pred_gsh == upd_taken);
wire [TAG_W-1:0]     sq_br_age   = squash_tag - rob_head;

// ----------------------------------------------------------------------------
// 5. Sequential State Updates
// ----------------------------------------------------------------------------
integer i, j;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (i = 0; i < BTB_ENTRIES; i = i + 1) begin
            btb_valid[i]  <= 1'b0;
            btb_tag[i]    <= {BTB_TAG_W{1'b0}};
            btb_target[i] <= 32'd0;
        end
        for (i = 0; i < BHT_ENTRIES; i = i + 1) begin
            bim_bht[i] <= 2'b01; // Weakly Not-Taken
            gsh_bht[i] <= 2'b01; // Weakly Not-Taken
            choice[i]  <= 2'b01; // Weakly Bimodal (fast loop warm-up, flips to GSHARE on correlated!)
        end
        spec_ghr <= 8'd0;
        f_tos    <= 3'd0;
        f_cnt    <= 4'd0;
        d_tos    <= 3'd0;
        d_cnt    <= 4'd0;
        for (i = 0; i < RAS_DEPTH; i = i + 1) begin
            f_ras[i] <= 32'd0;
            d_ras[i] <= 32'd0;
        end
        for (i = 0; i < NUM_CP; i = i + 1) begin
            cp_valid[i] <= 1'b0;
            cp_tag[i]   <= {TAG_W{1'b0}};
            cp_tos[i]   <= 3'd0;
            cp_cnt[i]   <= 4'd0;
            for (j = 0; j < RAS_DEPTH; j = j + 1)
                cp_ras[i][j] <= 32'd0;
        end
    end else begin
        // A. BTB + Hybrid Bimodal/GSHARE + Tournament Choice Training (from CDB0)
        if (upd_valid) begin
            if (upd_is_jalr) begin
                btb_valid[upd_btb_idx]  <= 1'b1;
                btb_tag[upd_btb_idx]    <= upd_btb_tag;
                btb_target[upd_btb_idx] <= upd_target;
            end
            if (upd_is_cond) begin
                // Train Bimodal
                if (upd_taken) begin
                    if (bim_bht[upd_bim_idx] != 2'b11)
                        bim_bht[upd_bim_idx] <= bim_bht[upd_bim_idx] + 2'b01;
                end else begin
                    if (bim_bht[upd_bim_idx] != 2'b00)
                        bim_bht[upd_bim_idx] <= bim_bht[upd_bim_idx] - 2'b01;
                end
                // Train GSHARE
                if (upd_taken) begin
                    if (gsh_bht[upd_gsh_idx] != 2'b11)
                        gsh_bht[upd_gsh_idx] <= gsh_bht[upd_gsh_idx] + 2'b01;
                end else begin
                    if (gsh_bht[upd_gsh_idx] != 2'b00)
                        gsh_bht[upd_gsh_idx] <= gsh_bht[upd_gsh_idx] - 2'b01;
                end
                // Train Tournament Choice (when Bimodal & GSHARE disagree)
                if (upd_gsh_ok && !upd_bim_ok) begin
                    if (choice[upd_bim_idx] != 2'b11)
                        choice[upd_bim_idx] <= choice[upd_bim_idx] + 2'b01;
                end else if (upd_bim_ok && !upd_gsh_ok) begin
                    if (choice[upd_bim_idx] != 2'b00)
                        choice[upd_bim_idx] <= choice[upd_bim_idx] - 2'b01;
                end
            end
        end

        // B. Free resolved-correct checkpoint slot
        if (br_resolve_ok) begin
            for (i = 0; i < NUM_CP; i = i + 1) begin
                if (cp_valid[i] && (cp_tag[i] == br_resolve_tag))
                    cp_valid[i] <= 1'b0;
            end
        end

        // C. Speculative GHR & RAS Update vs. Exact Checkpoint Squash Recovery
        if (squash_valid) begin
            spec_ghr <= upd_is_cond ? {upd_ghr[6:0], upd_taken} : upd_ghr;

            if (cp_sq_found) begin
                for (i = 0; i < RAS_DEPTH; i = i + 1) begin
                    f_ras[i] <= cp_ras[cp_sq_idx][i];
                    d_ras[i] <= cp_ras[cp_sq_idx][i];
                end
                f_tos <= cp_tos[cp_sq_idx];
                f_cnt <= cp_cnt[cp_sq_idx];
                d_tos <= cp_tos[cp_sq_idx];
                d_cnt <= cp_cnt[cp_sq_idx];
            end

            for (i = 0; i < NUM_CP; i = i + 1) begin
                if (cp_valid[i] && ((cp_tag[i] - rob_head) >= sq_br_age))
                    cp_valid[i] <= 1'b0;
            end
        end else begin
            // 1. Advance speculative Fetch GHR & Fetch RAS on FQ enqueue
            if (enq1_fire) begin
                if (f1_br)
                    spec_ghr <= {pred_ghr1[6:0], pred_taken1};
                else if (f0_br)
                    spec_ghr <= pred_ghr1;

                for (i = 0; i < RAS_DEPTH; i = i + 1)
                    f_ras[i] <= f_ras_a1[i];
                f_tos <= f_tos_a1;
                f_cnt <= f_cnt_a1;
            end else if (enq0_fire) begin
                if (f0_br)
                    spec_ghr <= {spec_ghr[6:0], pred_taken0};

                for (i = 0; i < RAS_DEPTH; i = i + 1)
                    f_ras[i] <= f_ras_a0[i];
                f_tos <= f_tos_a0;
                f_cnt <= f_cnt_a0;
            end

            // 2. Advance Dispatch-Order RAS strictly in program order
            if (disp1_fire) begin
                for (i = 0; i < RAS_DEPTH; i = i + 1)
                    d_ras[i] <= d_ras_a1[i];
                d_tos <= d_tos_a1;
                d_cnt <= d_cnt_a1;
            end else if (disp0_fire) begin
                for (i = 0; i < RAS_DEPTH; i = i + 1)
                    d_ras[i] <= d_ras_a0[i];
                d_tos <= d_tos_a0;
                d_cnt <= d_cnt_a0;
            end

            // 3. Snapshot Dispatch-Order RAS at Branch Checkpoint (Lane 0 vs Lane 1!)
            if (cp_take && cp_has_free) begin
                cp_valid[cp_free_idx] <= 1'b1;
                cp_tag[cp_free_idx]   <= cp_take_tag;
                if (cp_is_lane1) begin
                    for (i = 0; i < RAS_DEPTH; i = i + 1)
                        cp_ras[cp_free_idx][i] <= d_ras_a1[i];
                    cp_tos[cp_free_idx] <= d_tos_a1;
                    cp_cnt[cp_free_idx] <= d_cnt_a1;
                end else begin
                    for (i = 0; i < RAS_DEPTH; i = i + 1)
                        cp_ras[cp_free_idx][i] <= d_ras_a0[i];
                    cp_tos[cp_free_idx] <= d_tos_a0;
                    cp_cnt[cp_free_idx] <= d_cnt_a0;
                end
            end
        end
    end
end

endmodule
