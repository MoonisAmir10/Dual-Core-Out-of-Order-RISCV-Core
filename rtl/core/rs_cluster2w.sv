`timescale 1ns / 1ps

// ============================================================================
// Parameterized 2-Wide Reservation Station Bank (Dual-Alloc, Dual-CDB,
// Dual-Issue + 1-Cycle Speculative Issue-Stage Wakeup & EX->EX Bypass!)
//
// Features:
//   - Optional 2-wide dispatch allocation into guaranteed-distinct free slots
//   - 1-cycle speculative Issue-stage tag wakeup (wk0/wk1 from ALU0/ALU1 issue)
//     + combinational EX->EX bypass from registered alu0_out_val/alu1_out_val
//   - Automatic register capture at end of bypass cycle if woken consumer does
//     not issue immediately (zero lost values, zero replay needed)
//   - Dual-CDB wakeup snoop (cdb0 & cdb1) across all occupied & incoming ops
//   - Optional 2-wide oldest-ready-first issue (e.g. RS_ALU -> FU_ALU0 + FU_ALU1)
//   - Selective circular-ROB-age squash (kills only entries younger than squash_tag)
// ============================================================================
module rs_cluster2w #(
    parameter integer ENTRIES    = 8,
    parameter integer TAG_W      = 5,
    parameter integer OP_W       = 4,
    parameter integer DUAL_ISSUE = 1
) (
    input  wire             clk,
    input  wire             reset_n,
    input  wire             flush,
    input  wire             squash_valid,
    input  wire [TAG_W-1:0] squash_tag,
    input  wire [TAG_W-1:0] rob_head,

    // Free slot availability
    output wire             free_ge1,
    output wire             free_ge2,

    // Dispatch Port 0 (older)
    input  wire             disp0_valid,
    input  wire [OP_W-1:0]  disp0_op,
    input  wire [TAG_W-1:0] disp0_rob_tag,
    input  wire [31:0]      disp0_seq_id,
    input  wire [31:0]      disp0_pc,
    input  wire [31:0]      disp0_imm,
    input  wire             disp0_asel,
    input  wire             disp0_bsel,
    input  wire [31:0]      disp0_vj,
    input  wire [TAG_W-1:0] disp0_qj,
    input  wire             disp0_qj_valid,
    input  wire [31:0]      disp0_vk,
    input  wire [TAG_W-1:0] disp0_qk,
    input  wire             disp0_qk_valid,

    // Dispatch Port 1 (younger; can be asserted simultaneously with disp0_valid or alone!)
    input  wire             disp1_valid,
    input  wire [OP_W-1:0]  disp1_op,
    input  wire [TAG_W-1:0] disp1_rob_tag,
    input  wire [31:0]      disp1_seq_id,
    input  wire [31:0]      disp1_pc,
    input  wire [31:0]      disp1_imm,
    input  wire             disp1_asel,
    input  wire             disp1_bsel,
    input  wire [31:0]      disp1_vj,
    input  wire [TAG_W-1:0] disp1_qj,
    input  wire             disp1_qj_valid,
    input  wire [31:0]      disp1_vk,
    input  wire [TAG_W-1:0] disp1_qk,
    input  wire             disp1_qk_valid,

    // Dual-CDB wakeup broadcast snoop
    input  wire             cdb0_valid,
    input  wire [TAG_W-1:0] cdb0_tag,
    input  wire [31:0]      cdb0_val,
    input  wire             cdb1_valid,
    input  wire [TAG_W-1:0] cdb1_tag,
    input  wire [31:0]      cdb1_val,

    // 1-Cycle Speculative Issue-Stage Tag Wakeup + Registered EX->EX Bypass Data
    input  wire             wk0_valid,
    input  wire [TAG_W-1:0] wk0_tag,
    input  wire             wk1_valid,
    input  wire [TAG_W-1:0] wk1_tag,
    input  wire [31:0]      byp_alu0_val,
    input  wire [31:0]      byp_alu1_val,

    // Issue Port 0 (oldest ready)
    output reg              issue0_valid,
    input  wire             fu0_ready,
    output reg  [OP_W-1:0]  issue0_op,
    output reg  [TAG_W-1:0] issue0_rob_tag,
    output reg  [31:0]      issue0_seq_id,
    output reg  [31:0]      issue0_pc,
    output reg  [31:0]      issue0_imm,
    output reg              issue0_asel,
    output reg              issue0_bsel,
    output reg  [31:0]      issue0_vj,
    output reg  [31:0]      issue0_vk,

    // Issue Port 1 (second-oldest ready, active when DUAL_ISSUE != 0)
    output reg              issue1_valid,
    input  wire             fu1_ready,
    output reg  [OP_W-1:0]  issue1_op,
    output reg  [TAG_W-1:0] issue1_rob_tag,
    output reg  [31:0]      issue1_seq_id,
    output reg  [31:0]      issue1_pc,
    output reg  [31:0]      issue1_imm,
    output reg              issue1_asel,
    output reg              issue1_bsel,
    output reg  [31:0]      issue1_vj,
    output reg  [31:0]      issue1_vk,

    // PMU telemetry: number of operand reads served via EX->EX bypass this cycle (0..4)
    output wire [2:0]       stat_ex_byp_cnt
);

localparam integer IDX_W = (ENTRIES <= 2) ? 1 : $clog2(ENTRIES);
localparam [1:0] BYP_NONE = 2'b00;
localparam [1:0] BYP_ALU0 = 2'b01;
localparam [1:0] BYP_ALU1 = 2'b10;

reg             e_busy     [0:ENTRIES-1];
reg [OP_W-1:0]  e_op       [0:ENTRIES-1];
reg [TAG_W-1:0] e_rob_tag  [0:ENTRIES-1];
reg [31:0]      e_seq_id   [0:ENTRIES-1];
reg [31:0]      e_pc       [0:ENTRIES-1];
reg [31:0]      e_imm      [0:ENTRIES-1];
reg             e_asel     [0:ENTRIES-1];
reg             e_bsel     [0:ENTRIES-1];
reg [31:0]      e_vj       [0:ENTRIES-1];
reg [TAG_W-1:0] e_qj       [0:ENTRIES-1];
reg             e_qj_valid [0:ENTRIES-1];
reg [1:0]       e_qj_byp   [0:ENTRIES-1];
reg [31:0]      e_vk       [0:ENTRIES-1];
reg [TAG_W-1:0] e_qk       [0:ENTRIES-1];
reg             e_qk_valid [0:ENTRIES-1];
reg [1:0]       e_qk_byp   [0:ENTRIES-1];

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

// 2. Oldest-ready and second-oldest-ready issue selection + EX->EX Bypass Mux
reg [IDX_W-1:0] issue0_idx, issue1_idx;
reg [TAG_W-1:0] best0_age, best1_age, cur_age0, cur_age1;
integer s0_i, s1_i;

always @(*) begin
    s0_i           = 0;
    s1_i           = 0;
    issue0_valid   = 1'b0;
    issue0_idx     = {IDX_W{1'b0}};
    best0_age      = {TAG_W{1'b1}};
    cur_age0       = {TAG_W{1'b0}};

    for (s0_i = 0; s0_i < ENTRIES; s0_i = s0_i + 1) begin
        if (e_busy[s0_i] && !e_qj_valid[s0_i] && !e_qk_valid[s0_i]) begin
            cur_age0 = e_rob_tag[s0_i] - rob_head;
            if (!(squash_valid && (cur_age0 > sq_br_age))) begin
                if (!issue0_valid || (cur_age0 < best0_age)) begin
                    issue0_valid = 1'b1;
                    issue0_idx   = s0_i[IDX_W-1:0];
                    best0_age    = cur_age0;
                end
            end
        end
    end

    issue1_valid   = 1'b0;
    issue1_idx     = {IDX_W{1'b0}};
    best1_age      = {TAG_W{1'b1}};
    cur_age1       = {TAG_W{1'b0}};

    if (DUAL_ISSUE != 0) begin
        for (s1_i = 0; s1_i < ENTRIES; s1_i = s1_i + 1) begin
            if (e_busy[s1_i] && !e_qj_valid[s1_i] && !e_qk_valid[s1_i] &&
                (!issue0_valid || (s1_i[IDX_W-1:0] != issue0_idx))) begin
                cur_age1 = e_rob_tag[s1_i] - rob_head;
                if (!(squash_valid && (cur_age1 > sq_br_age))) begin
                    if (!issue1_valid || (cur_age1 < best1_age)) begin
                        issue1_valid = 1'b1;
                        issue1_idx   = s1_i[IDX_W-1:0];
                        best1_age    = cur_age1;
                    end
                end
            end
        end
    end

    issue0_op      = e_op[issue0_idx];
    issue0_rob_tag = e_rob_tag[issue0_idx];
    issue0_seq_id  = e_seq_id[issue0_idx];
    issue0_pc      = e_pc[issue0_idx];
    issue0_imm     = e_imm[issue0_idx];
    issue0_asel    = e_asel[issue0_idx];
    issue0_bsel    = e_bsel[issue0_idx];
    issue0_vj      = (e_qj_byp[issue0_idx] == BYP_ALU0) ? byp_alu0_val :
                     (e_qj_byp[issue0_idx] == BYP_ALU1) ? byp_alu1_val :
                                                          e_vj[issue0_idx];
    issue0_vk      = (e_qk_byp[issue0_idx] == BYP_ALU0) ? byp_alu0_val :
                     (e_qk_byp[issue0_idx] == BYP_ALU1) ? byp_alu1_val :
                                                          e_vk[issue0_idx];

    issue1_op      = e_op[issue1_idx];
    issue1_rob_tag = e_rob_tag[issue1_idx];
    issue1_seq_id  = e_seq_id[issue1_idx];
    issue1_pc      = e_pc[issue1_idx];
    issue1_imm     = e_imm[issue1_idx];
    issue1_asel    = e_asel[issue1_idx];
    issue1_bsel    = e_bsel[issue1_idx];
    issue1_vj      = (e_qj_byp[issue1_idx] == BYP_ALU0) ? byp_alu0_val :
                     (e_qj_byp[issue1_idx] == BYP_ALU1) ? byp_alu1_val :
                                                          e_vj[issue1_idx];
    issue1_vk      = (e_qk_byp[issue1_idx] == BYP_ALU0) ? byp_alu0_val :
                     (e_qk_byp[issue1_idx] == BYP_ALU1) ? byp_alu1_val :
                                                          e_vk[issue1_idx];
end

// Defensive same-cycle Dual-CDB & Early Issue-Wakeup check on incoming disp0 & disp1
wire        d0_j_c0    = disp0_qj_valid && cdb0_valid && (disp0_qj == cdb0_tag);
wire        d0_j_c1    = disp0_qj_valid && !d0_j_c0 && cdb1_valid && (disp0_qj == cdb1_tag);
wire        d0_j_w0    = disp0_qj_valid && !d0_j_c0 && !d0_j_c1 && wk0_valid && (disp0_qj == wk0_tag);
wire        d0_j_w1    = disp0_qj_valid && !d0_j_c0 && !d0_j_c1 && !d0_j_w0 && wk1_valid && (disp0_qj == wk1_tag);
wire [31:0] d0_vj_e    = d0_j_c0 ? cdb0_val : (d0_j_c1 ? cdb1_val : disp0_vj);
wire        d0_qj_e    = (d0_j_c0 || d0_j_c1 || d0_j_w0 || d0_j_w1) ? 1'b0 : disp0_qj_valid;
wire [1:0]  d0_qjbyp_e = d0_j_w0 ? BYP_ALU0 : (d0_j_w1 ? BYP_ALU1 : BYP_NONE);

wire        d0_k_c0    = disp0_qk_valid && cdb0_valid && (disp0_qk == cdb0_tag);
wire        d0_k_c1    = disp0_qk_valid && !d0_k_c0 && cdb1_valid && (disp0_qk == cdb1_tag);
wire        d0_k_w0    = disp0_qk_valid && !d0_k_c0 && !d0_k_c1 && wk0_valid && (disp0_qk == wk0_tag);
wire        d0_k_w1    = disp0_qk_valid && !d0_k_c0 && !d0_k_c1 && !d0_k_w0 && wk1_valid && (disp0_qk == wk1_tag);
wire [31:0] d0_vk_e    = d0_k_c0 ? cdb0_val : (d0_k_c1 ? cdb1_val : disp0_vk);
wire        d0_qk_e    = (d0_k_c0 || d0_k_c1 || d0_k_w0 || d0_k_w1) ? 1'b0 : disp0_qk_valid;
wire [1:0]  d0_qkbyp_e = d0_k_w0 ? BYP_ALU0 : (d0_k_w1 ? BYP_ALU1 : BYP_NONE);

wire        d1_j_c0    = disp1_qj_valid && cdb0_valid && (disp1_qj == cdb0_tag);
wire        d1_j_c1    = disp1_qj_valid && !d1_j_c0 && cdb1_valid && (disp1_qj == cdb1_tag);
wire        d1_j_w0    = disp1_qj_valid && !d1_j_c0 && !d1_j_c1 && wk0_valid && (disp1_qj == wk0_tag);
wire        d1_j_w1    = disp1_qj_valid && !d1_j_c0 && !d1_j_c1 && !d1_j_w0 && wk1_valid && (disp1_qj == wk1_tag);
wire [31:0] d1_vj_e    = d1_j_c0 ? cdb0_val : (d1_j_c1 ? cdb1_val : disp1_vj);
wire        d1_qj_e    = (d1_j_c0 || d1_j_c1 || d1_j_w0 || d1_j_w1) ? 1'b0 : disp1_qj_valid;
wire [1:0]  d1_qjbyp_e = d1_j_w0 ? BYP_ALU0 : (d1_j_w1 ? BYP_ALU1 : BYP_NONE);

wire        d1_k_c0    = disp1_qk_valid && cdb0_valid && (disp1_qk == cdb0_tag);
wire        d1_k_c1    = disp1_qk_valid && !d1_k_c0 && cdb1_valid && (disp1_qk == cdb1_tag);
wire        d1_k_w0    = disp1_qk_valid && !d1_k_c0 && !d1_k_c1 && wk0_valid && (disp1_qk == wk0_tag);
wire        d1_k_w1    = disp1_qk_valid && !d1_k_c0 && !d1_k_c1 && !d1_k_w0 && wk1_valid && (disp1_qk == wk1_tag);
wire [31:0] d1_vk_e    = d1_k_c0 ? cdb0_val : (d1_k_c1 ? cdb1_val : disp1_vk);
wire        d1_qk_e    = (d1_k_c0 || d1_k_c1 || d1_k_w0 || d1_k_w1) ? 1'b0 : disp1_qk_valid;
wire [1:0]  d1_qkbyp_e = d1_k_w0 ? BYP_ALU0 : (d1_k_w1 ? BYP_ALU1 : BYP_NONE);

wire do_issue0 = issue0_valid && fu0_ready;
wire do_issue1 = (DUAL_ISSUE != 0) && issue1_valid && fu1_ready;
wire [IDX_W-1:0] slot1_idx = disp0_valid ? free1_idx : free0_idx;

assign stat_ex_byp_cnt =
    {2'b00, (do_issue0 && (e_qj_byp[issue0_idx] != BYP_NONE))} +
    {2'b00, (do_issue0 && (e_qk_byp[issue0_idx] != BYP_NONE))} +
    {2'b00, (do_issue1 && (e_qj_byp[issue1_idx] != BYP_NONE))} +
    {2'b00, (do_issue1 && (e_qk_byp[issue1_idx] != BYP_NONE))};

integer k;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (k = 0; k < ENTRIES; k = k + 1) begin
            e_busy[k]     <= 1'b0;
            e_op[k]       <= {OP_W{1'b0}};
            e_rob_tag[k]  <= {TAG_W{1'b0}};
            e_seq_id[k]   <= 32'd0;
            e_pc[k]       <= 32'd0;
            e_imm[k]      <= 32'd0;
            e_asel[k]     <= 1'b0;
            e_bsel[k]     <= 1'b0;
            e_vj[k]       <= 32'd0;
            e_qj[k]       <= {TAG_W{1'b0}};
            e_qj_valid[k] <= 1'b0;
            e_qj_byp[k]   <= BYP_NONE;
            e_vk[k]       <= 32'd0;
            e_qk[k]       <= {TAG_W{1'b0}};
            e_qk_valid[k] <= 1'b0;
            e_qk_byp[k]   <= BYP_NONE;
        end
    end else if (flush) begin
        for (k = 0; k < ENTRIES; k = k + 1) begin
            e_busy[k]   <= 1'b0;
            e_qj_byp[k] <= BYP_NONE;
            e_qk_byp[k] <= BYP_NONE;
        end
    end else begin
        // 1. Capture EX->EX bypass data into RS registers for non-issued entries,
        //    and snoop Dual-CDB + 1-Cycle Speculative Issue-Stage Wakeup (wk0/wk1)
        for (k = 0; k < ENTRIES; k = k + 1) begin
            if (e_busy[k]) begin
                // Source J
                if (e_qj_byp[k] == BYP_ALU0) begin
                    e_vj[k]     <= byp_alu0_val;
                    e_qj_byp[k] <= BYP_NONE;
                end else if (e_qj_byp[k] == BYP_ALU1) begin
                    e_vj[k]     <= byp_alu1_val;
                    e_qj_byp[k] <= BYP_NONE;
                end else if (e_qj_valid[k]) begin
                    if (cdb0_valid && (e_qj[k] == cdb0_tag)) begin
                        e_vj[k]       <= cdb0_val;
                        e_qj_valid[k] <= 1'b0;
                        e_qj_byp[k]   <= BYP_NONE;
                    end else if (cdb1_valid && (e_qj[k] == cdb1_tag)) begin
                        e_vj[k]       <= cdb1_val;
                        e_qj_valid[k] <= 1'b0;
                        e_qj_byp[k]   <= BYP_NONE;
                    end else if (wk0_valid && (e_qj[k] == wk0_tag)) begin
                        e_qj_valid[k] <= 1'b0;
                        e_qj_byp[k]   <= BYP_ALU0;
                    end else if (wk1_valid && (e_qj[k] == wk1_tag)) begin
                        e_qj_valid[k] <= 1'b0;
                        e_qj_byp[k]   <= BYP_ALU1;
                    end
                end

                // Source K
                if (e_qk_byp[k] == BYP_ALU0) begin
                    e_vk[k]     <= byp_alu0_val;
                    e_qk_byp[k] <= BYP_NONE;
                end else if (e_qk_byp[k] == BYP_ALU1) begin
                    e_vk[k]     <= byp_alu1_val;
                    e_qk_byp[k] <= BYP_NONE;
                end else if (e_qk_valid[k]) begin
                    if (cdb0_valid && (e_qk[k] == cdb0_tag)) begin
                        e_vk[k]       <= cdb0_val;
                        e_qk_valid[k] <= 1'b0;
                        e_qk_byp[k]   <= BYP_NONE;
                    end else if (cdb1_valid && (e_qk[k] == cdb1_tag)) begin
                        e_vk[k]       <= cdb1_val;
                        e_qk_valid[k] <= 1'b0;
                        e_qk_byp[k]   <= BYP_NONE;
                    end else if (wk0_valid && (e_qk[k] == wk0_tag)) begin
                        e_qk_valid[k] <= 1'b0;
                        e_qk_byp[k]   <= BYP_ALU0;
                    end else if (wk1_valid && (e_qk[k] == wk1_tag)) begin
                        e_qk_valid[k] <= 1'b0;
                        e_qk_byp[k]   <= BYP_ALU1;
                    end
                end
            end
        end

        // 2. Release issued entries
        if (do_issue0) begin
            e_busy[issue0_idx]   <= 1'b0;
            e_qj_byp[issue0_idx] <= BYP_NONE;
            e_qk_byp[issue0_idx] <= BYP_NONE;
        end
        if (do_issue1) begin
            e_busy[issue1_idx]   <= 1'b0;
            e_qj_byp[issue1_idx] <= BYP_NONE;
            e_qk_byp[issue1_idx] <= BYP_NONE;
        end

        // 3. Allocate dispatched entries into guaranteed-distinct free slots
        if (disp0_valid && free_ge1) begin
            e_busy[free0_idx]     <= 1'b1;
            e_op[free0_idx]       <= disp0_op;
            e_rob_tag[free0_idx]  <= disp0_rob_tag;
            e_seq_id[free0_idx]   <= disp0_seq_id;
            e_pc[free0_idx]       <= disp0_pc;
            e_imm[free0_idx]      <= disp0_imm;
            e_asel[free0_idx]     <= disp0_asel;
            e_bsel[free0_idx]     <= disp0_bsel;
            e_vj[free0_idx]       <= d0_vj_e;
            e_qj[free0_idx]       <= disp0_qj;
            e_qj_valid[free0_idx] <= d0_qj_e;
            e_qj_byp[free0_idx]   <= d0_qjbyp_e;
            e_vk[free0_idx]       <= d0_vk_e;
            e_qk[free0_idx]       <= disp0_qk;
            e_qk_valid[free0_idx] <= d0_qk_e;
            e_qk_byp[free0_idx]   <= d0_qkbyp_e;
        end
        if (disp1_valid && (disp0_valid ? free_ge2 : free_ge1)) begin
            e_busy[slot1_idx]     <= 1'b1;
            e_op[slot1_idx]       <= disp1_op;
            e_rob_tag[slot1_idx]  <= disp1_rob_tag;
            e_seq_id[slot1_idx]   <= disp1_seq_id;
            e_pc[slot1_idx]       <= disp1_pc;
            e_imm[slot1_idx]      <= disp1_imm;
            e_asel[slot1_idx]     <= disp1_asel;
            e_bsel[slot1_idx]     <= disp1_bsel;
            e_vj[slot1_idx]       <= d1_vj_e;
            e_qj[slot1_idx]       <= disp1_qj;
            e_qj_valid[slot1_idx] <= d1_qj_e;
            e_qj_byp[slot1_idx]   <= d1_qjbyp_e;
            e_vk[slot1_idx]       <= d1_vk_e;
            e_qk[slot1_idx]       <= disp1_qk;
            e_qk_valid[slot1_idx] <= d1_qk_e;
            e_qk_byp[slot1_idx]   <= d1_qkbyp_e;
        end

        // 4. Selective Age-Based Squash: kill ONLY entries strictly younger than squash_tag
        if (squash_valid) begin
            for (k = 0; k < ENTRIES; k = k + 1) begin
                if (e_busy[k] && ((e_rob_tag[k] - rob_head) > sq_br_age)) begin
                    e_busy[k]   <= 1'b0;
                    e_qj_byp[k] <= BYP_NONE;
                    e_qk_byp[k] <= BYP_NONE;
                end
            end
        end
    end
end

endmodule
