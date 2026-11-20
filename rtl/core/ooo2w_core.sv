`timescale 1ns / 1ps

// ============================================================================
// 2-Wide Superscalar Tomasulo Out-of-Order RV32IM Processor Core (Phase 6)
//
// Microarchitecture:
//   - 2-Wide Fetch (8-entry FQ) + Dual-Lookup BTB(32)/BHT(64)
//   - 2-Wide Decode + Rename (rat2w: 32-reg RAT + 4 Coherent Branch Checkpoints)
//     with intra-pair RAW forwarding & WAW ordering
//   - 32-Entry 2-Wide Circular ROB (rob2w, TAG_W=5) with Fast Selective Squash
//   - Partitioned RS: RS_ALU(8, 2W-alloc/2W-issue), RS_MUL(4, 2W-alloc/1W-issue),
//     RS_DIV(2), RS_BR(4), LSQ(8 LQ + 8 SQ + 1-cyc STLF)
//   - 6 Functional Units: ALU0(1c), ALU1(1c), MUL(4c II=1), DIV(18c Radix-4),
//     BR(1c), LSU(2-way D-Cache + STLF)
//   - Dual-CDB Arbiter (cdb0 full sideband + cdb1 value) -> up to 2 completions/cyc
//   - 2-Wide In-Order Commit (regfile2w 4R2W ARF) -> breaks scalar IPC > 1.0!
// ============================================================================
module ooo2w_core (
    input  wire        clk,
    input  wire        reset_n,

    // Dual-port instruction memory interface
    output wire [31:0] imem_pc0,
    input  wire [31:0] imem_instr0,
    output wire [31:0] imem_pc1,
    input  wire [31:0] imem_instr1,

    // Split-transaction non-blocking D-Cache interface (Upgrade U3)
    output wire        dc_req_valid,
    input  wire        dc_req_ready,
    output wire        dc_req_is_write,
    output wire [15:0] dc_req_addr,
    output wire [31:0] dc_req_wdata,
    output wire [3:0]  dc_req_wstrb,
    output wire [5:0]  dc_req_id,
    output wire [4:0]  dc_req_rob_tag,

    input  wire        dc_hit_valid,
    input  wire [5:0]  dc_hit_id,
    input  wire [31:0] dc_hit_rdata,

    input  wire        dc_fill_valid,
    input  wire [5:0]  dc_fill_id,
    input  wire [31:0] dc_fill_rdata,

    output wire        dc_squash_valid,
    output wire [4:0]  dc_squash_tag,
    output wire [4:0]  dc_rob_head,
    input  wire        dc_writes_idle,
    input  wire        dc_any_miss,

    // 2-Wide RVFI Commit Trace Ports (strict in-order: commit0 older, commit1 younger)
    output wire        commit0_valid,
    output wire [31:0] commit0_seq_id,
    output wire [31:0] commit0_pc,
    output wire [31:0] commit0_instr,
    output wire        commit0_rd_we,
    output wire [4:0]  commit0_rd_addr,
    output wire [31:0] commit0_rd_wdata,
    output wire        commit0_mem_valid,
    output wire        commit0_mem_we,
    output wire [31:0] commit0_mem_addr,
    output wire [3:0]  commit0_mem_wstrb,
    output wire [31:0] commit0_mem_wdata,

    output wire        commit1_valid,
    output wire [31:0] commit1_seq_id,
    output wire [31:0] commit1_pc,
    output wire [31:0] commit1_instr,
    output wire        commit1_rd_we,
    output wire [4:0]  commit1_rd_addr,
    output wire [31:0] commit1_rd_wdata,
    output wire        commit1_mem_valid,
    output wire        commit1_mem_we,
    output wire [31:0] commit1_mem_addr,
    output wire [3:0]  commit1_mem_wstrb,
    output wire [31:0] commit1_mem_wdata,

    // 2-Wide Out-of-Order lifecycle trace hooks (6 issue ports, 2 CDB ports)
    output wire        trace_disp0_valid,
    output wire [31:0] trace_disp0_seq,
    output wire [31:0] trace_disp0_pc,
    output wire [31:0] trace_disp0_instr,
    output wire        trace_disp1_valid,
    output wire [31:0] trace_disp1_seq,
    output wire [31:0] trace_disp1_pc,
    output wire [31:0] trace_disp1_instr,
    output wire [5:0]  trace_issue_valid,
    output wire [191:0] trace_issue_seq,
    output wire [1:0]  trace_cdb_valid,
    output wire [63:0] trace_cdb_seq,
    output wire        trace_flush_valid,
    output wire [31:0] trace_flush_after_seq,

    // Status & PMU telemetry pulses
    output reg         halted,
    output wire [31:0] x14,
    output wire        stat_rob_full_stall,
    output wire        stat_rs_full_stall,
    output wire        stat_lsq_full_stall,
    output wire        stat_cp_full_stall,
    output wire        stat_stlf_hit,
    output wire [1:0]  stat_br_total_cnt,
    output wire [1:0]  stat_br_mispred_cnt,
    output wire        stat_br_fast_squash,
    output wire [1:0]  stat_war_waw_cnt,
    output wire        stat_div_overlap_cyc,
    output wire        stat_dcache_miss_overlap_cyc,
    output wire        stat_dual_disp,
    output wire        stat_dual_cdb,
    output wire        stat_dual_commit,
    output wire [3:0]  stat_ex_byp_cnt,
    output wire [1:0]  stat_ras_pred_cnt,
    output wire [1:0]  stat_ras_mispred_cnt
);

`include "rv32ima_defs.vh"

localparam integer ROB2W_DEPTH = 32;
localparam integer TAG_W       = 5;

// ============================================================================
// 1. 2-Wide Front-End: GSHARE+8-RAS BTB, Fetch2W & Dual Decoder (Upgrade U2)
// ============================================================================
wire        rob_squash;
wire [4:0]  rob_squash_tag;
wire [31:0] rob_squash_seq;
wire [31:0] rob_squash_redirect_pc;
wire        rob_squash_has_older;
wire        rob_cdb0_br_ok;
wire [31:0] rob_cdb0_br_pc;
wire        rob_cdb0_br_is_cond;
wire        rob_cdb0_br_is_jalr;
wire [7:0]  rob_cdb0_br_ghr;
wire        rob_cdb0_br_pbim, rob_cdb0_br_pgsh;

// Forward declarations for 2W BP checkpoint lockstep & operand bypass
wire        disp0_fire, disp1_fire;
wire [4:0]  rob_disp0_tag, rob_disp1_tag, rob_head_ptr;
wire        cp_take_l0, cp_take_l1;

// Dual-CDB & Early Issue-Wakeup / EX-Output Bypass wires (declared early)
wire        cdb0_valid, cdb1_valid;
wire [4:0]  cdb0_tag, cdb1_tag;
wire [31:0] cdb0_seq, cdb0_val, cdb1_seq, cdb1_val;
wire        cdb0_is_br, cdb0_br_taken;
wire [31:0] cdb0_br_target;
wire        cdb0_is_mem, cdb0_mem_we;
wire [31:0] cdb0_mem_addr, cdb0_mem_wdata;
wire [3:0]  cdb0_mem_wstrb;

reg         alu0_out_v, alu1_out_v;
reg  [4:0]  alu0_out_tag, alu1_out_tag;
reg  [31:0] alu0_out_seq, alu0_out_val, alu1_out_seq, alu1_out_val;
wire        wk0_valid, wk1_valid;
wire [4:0]  wk0_tag, wk1_tag;

wire [31:0] pred_req_pc0, pred_req_instr0, pred_req_pc1, pred_req_instr1;
wire        bp_pred_taken0, bp_pred_taken1, bp_pred_uras0, bp_pred_uras1;
wire        bp_pred_bim0, bp_pred_gsh0, bp_pred_bim1, bp_pred_gsh1;
wire [31:0] bp_pred_target0, bp_pred_target1;
wire [7:0]  bp_pred_ghr0, bp_pred_ghr1;
wire        fq_enq0_fire, fq_enq1_fire;

wire        fq0_valid, fq1_valid;
wire [31:0] fq0_pc, fq0_instr, fq0_ptgt;
wire [31:0] fq1_pc, fq1_instr, fq1_ptgt;
wire        fq0_ptaken, fq1_ptaken, fq0_puras, fq1_puras;
wire        fq0_pbim, fq0_pgsh, fq1_pbim, fq1_pgsh;
wire [7:0]  fq0_pghr, fq1_pghr;

btb_bht2w #(
    .BTB_ENTRIES(BTB_ENTRIES),
    .BHT_ENTRIES(256),
    .RAS_DEPTH(8),
    .TAG_W(TAG_W),
    .NUM_CP(4)
) u_btb_bht (
    .clk(clk),
    .reset_n(reset_n),
    .req_pc0(pred_req_pc0),
    .req_instr0(pred_req_instr0),
    .pred_taken0(bp_pred_taken0),
    .pred_target0(bp_pred_target0),
    .pred_ghr0(bp_pred_ghr0),
    .pred_used_ras0(bp_pred_uras0),
    .pred_bim0(bp_pred_bim0),
    .pred_gsh0(bp_pred_gsh0),
    .req_pc1(pred_req_pc1),
    .req_instr1(pred_req_instr1),
    .pred_taken1(bp_pred_taken1),
    .pred_target1(bp_pred_target1),
    .pred_ghr1(bp_pred_ghr1),
    .pred_used_ras1(bp_pred_uras1),
    .pred_bim1(bp_pred_bim1),
    .pred_gsh1(bp_pred_gsh1),
    .enq0_fire(fq_enq0_fire),
    .enq1_fire(fq_enq1_fire),
    .disp0_fire(disp0_fire),
    .disp0_pc(fq0_pc),
    .disp0_instr(fq0_instr),
    .disp1_fire(disp1_fire),
    .disp1_pc(fq1_pc),
    .disp1_instr(fq1_instr),
    .cp_take(cp_take_l0 || cp_take_l1),
    .cp_is_lane1(cp_take_l1),
    .cp_take_tag(cp_take_l1 ? rob_disp1_tag : rob_disp0_tag),
    .upd_valid(cdb0_valid && cdb0_is_br),
    .upd_pc(rob_cdb0_br_pc),
    .upd_is_cond(rob_cdb0_br_is_cond),
    .upd_is_jalr(rob_cdb0_br_is_jalr),
    .upd_taken(cdb0_br_taken),
    .upd_target(cdb0_br_target),
    .upd_ghr(rob_cdb0_br_ghr),
    .upd_pred_bim(rob_cdb0_br_pbim),
    .upd_pred_gsh(rob_cdb0_br_pgsh),
    .br_resolve_ok(rob_cdb0_br_ok),
    .br_resolve_tag(cdb0_tag),
    .squash_valid(rob_squash),
    .squash_tag(rob_squash_tag),
    .rob_head(rob_head_ptr)
);

fetch2w u_fetch (
    .clk(clk),
    .reset_n(reset_n),
    .halted(halted),
    .flush(rob_squash),
    .flush_target_pc(rob_squash_redirect_pc),
    .imem_pc0(imem_pc0),
    .imem_instr0(imem_instr0),
    .imem_pc1(imem_pc1),
    .imem_instr1(imem_instr1),
    .pred_req_pc0(pred_req_pc0),
    .pred_req_instr0(pred_req_instr0),
    .pred_taken0(bp_pred_taken0),
    .pred_target0(bp_pred_target0),
    .pred_ghr0(bp_pred_ghr0),
    .pred_used_ras0(bp_pred_uras0),
    .pred_bim0(bp_pred_bim0),
    .pred_gsh0(bp_pred_gsh0),
    .pred_req_pc1(pred_req_pc1),
    .pred_req_instr1(pred_req_instr1),
    .pred_taken1(bp_pred_taken1),
    .pred_target1(bp_pred_target1),
    .pred_ghr1(bp_pred_ghr1),
    .pred_used_ras1(bp_pred_uras1),
    .pred_bim1(bp_pred_bim1),
    .pred_gsh1(bp_pred_gsh1),
    .enq0_fire(fq_enq0_fire),
    .enq1_fire(fq_enq1_fire),
    .out0_valid(fq0_valid),
    .out0_ready(disp0_fire),
    .out0_pc(fq0_pc),
    .out0_instr(fq0_instr),
    .out0_pred_taken(fq0_ptaken),
    .out0_pred_target(fq0_ptgt),
    .out0_pred_ghr(fq0_pghr),
    .out0_pred_used_ras(fq0_puras),
    .out0_pred_bim(fq0_pbim),
    .out0_pred_gsh(fq0_pgsh),
    .out1_valid(fq1_valid),
    .out1_ready(disp1_fire),
    .out1_pc(fq1_pc),
    .out1_instr(fq1_instr),
    .out1_pred_taken(fq1_ptaken),
    .out1_pred_target(fq1_ptgt),
    .out1_pred_ghr(fq1_pghr),
    .out1_pred_used_ras(fq1_puras),
    .out1_pred_bim(fq1_pbim),
    .out1_pred_gsh(fq1_pgsh)
);

// Lane 0 Decoder & ImmGen
wire [4:0]  d0_rs1, d0_rs2, d0_rd;
wire        d0_uses_rs1, d0_uses_rs2, d0_rd_we;
wire [2:0]  d0_imm_sel, d0_fu;
wire [3:0]  d0_alu_sel;
wire        d0_asel, d0_bsel;
wire [1:0]  d0_mul_op, d0_div_op, d0_mem_sz;
wire [2:0]  d0_br_f3;
wire        d0_is_br, d0_is_jal, d0_is_jalr, d0_mem_rd, d0_mem_wr, d0_mem_uns, d0_ebrk, d0_ill;
wire        d0_is_fence, d0_is_fence_i, d0_is_csr, d0_csr_uimm;
wire [2:0]  d0_csr_op;
wire [11:0] d0_csr_addr;
wire        d0_is_amo, d0_is_lr, d0_is_sc, d0_amo_aq, d0_amo_rl;
wire [3:0]  d0_amo_op;
wire [31:0] d0_imm;

decoder u_dec0 (
    .instr(fq0_instr), .rs1(d0_rs1), .rs2(d0_rs2), .rd(d0_rd),
    .uses_rs1(d0_uses_rs1), .uses_rs2(d0_uses_rs2), .rd_we(d0_rd_we),
    .imm_sel(d0_imm_sel), .fu_type(d0_fu), .alu_select(d0_alu_sel),
    .asel(d0_asel), .bsel(d0_bsel), .mul_op(d0_mul_op), .div_op(d0_div_op),
    .br_funct3(d0_br_f3), .is_branch(d0_is_br), .is_jal(d0_is_jal), .is_jalr(d0_is_jalr),
    .mem_read(d0_mem_rd), .mem_write(d0_mem_wr), .mem_size(d0_mem_sz), .mem_unsigned(d0_mem_uns),
    .is_ebreak(d0_ebrk),
    .is_fence(d0_is_fence), .is_fence_i(d0_is_fence_i),
    .is_csr(d0_is_csr), .csr_op(d0_csr_op), .csr_addr(d0_csr_addr), .csr_use_imm(d0_csr_uimm),
    .is_amo(d0_is_amo), .is_lr(d0_is_lr), .is_sc(d0_is_sc), .amo_op(d0_amo_op),
    .amo_aq(d0_amo_aq), .amo_rl(d0_amo_rl),
    .illegal(d0_ill)
);
imm_gen u_imm0 (.instr(fq0_instr), .imm_sel(d0_imm_sel), .imm_out(d0_imm));

// Lane 1 Decoder & ImmGen
wire [4:0]  d1_rs1, d1_rs2, d1_rd;
wire        d1_uses_rs1, d1_uses_rs2, d1_rd_we;
wire [2:0]  d1_imm_sel, d1_fu;
wire [3:0]  d1_alu_sel;
wire        d1_asel, d1_bsel;
wire [1:0]  d1_mul_op, d1_div_op, d1_mem_sz;
wire [2:0]  d1_br_f3;
wire        d1_is_br, d1_is_jal, d1_is_jalr, d1_mem_rd, d1_mem_wr, d1_mem_uns, d1_ebrk, d1_ill;
wire        d1_is_fence, d1_is_fence_i, d1_is_csr, d1_csr_uimm;
wire [2:0]  d1_csr_op;
wire [11:0] d1_csr_addr;
wire        d1_is_amo, d1_is_lr, d1_is_sc, d1_amo_aq, d1_amo_rl;
wire [3:0]  d1_amo_op;
wire [31:0] d1_imm;

decoder u_dec1 (
    .instr(fq1_instr), .rs1(d1_rs1), .rs2(d1_rs2), .rd(d1_rd),
    .uses_rs1(d1_uses_rs1), .uses_rs2(d1_uses_rs2), .rd_we(d1_rd_we),
    .imm_sel(d1_imm_sel), .fu_type(d1_fu), .alu_select(d1_alu_sel),
    .asel(d1_asel), .bsel(d1_bsel), .mul_op(d1_mul_op), .div_op(d1_div_op),
    .br_funct3(d1_br_f3), .is_branch(d1_is_br), .is_jal(d1_is_jal), .is_jalr(d1_is_jalr),
    .mem_read(d1_mem_rd), .mem_write(d1_mem_wr), .mem_size(d1_mem_sz), .mem_unsigned(d1_mem_uns),
    .is_ebreak(d1_ebrk),
    .is_fence(d1_is_fence), .is_fence_i(d1_is_fence_i),
    .is_csr(d1_is_csr), .csr_op(d1_csr_op), .csr_addr(d1_csr_addr), .csr_use_imm(d1_csr_uimm),
    .is_amo(d1_is_amo), .is_lr(d1_is_lr), .is_sc(d1_is_sc), .amo_op(d1_amo_op),
    .amo_aq(d1_amo_aq), .amo_rl(d1_amo_rl),
    .illegal(d1_ill)
);
imm_gen u_imm1 (.instr(fq1_instr), .imm_sel(d1_imm_sel), .imm_out(d1_imm));

// ============================================================================
// 2. 4R2W ARF, 2-Wide Coherent-CP RAT & 32-Entry 2-Wide ROB
// ============================================================================
wire [31:0] arf_l0_rs1, arf_l0_rs2, arf_l1_rs1, arf_l1_rs2;
wire        rob_c0_valid, rob_c1_valid;
wire [4:0]  rob_c0_tag, rob_c1_tag;
wire [2:0]  rob_c0_fu, rob_c1_fu;
wire        rob_c0_rd_we, rob_c1_rd_we;
wire [4:0]  rob_c0_rd_addr, rob_c1_rd_addr;
wire [31:0] rob_c0_rd_wdata, rob_c1_rd_wdata;
wire        rob_c0_ebrk, rob_c0_is_br, rob_c0_mispred, rob_c0_uras;
wire        rob_c1_is_br, rob_c1_mispred, rob_c1_uras;
wire        rob_free_ge1, rob_free_ge2, rob_full, rob_empty;

regfile2w u_arf (
    .clk(clk), .reset_n(reset_n),
    .we0(rob_c0_valid && rob_c0_rd_we), .waddr0(rob_c0_rd_addr), .wdata0(rob_c0_rd_wdata),
    .we1(rob_c1_valid && rob_c1_rd_we), .waddr1(rob_c1_rd_addr), .wdata1(rob_c1_rd_wdata),
    .raddr0(d0_rs1), .rdata0(arf_l0_rs1),
    .raddr1(d0_rs2), .rdata1(arf_l0_rs2),
    .raddr2(d1_rs1), .rdata2(arf_l1_rs1),
    .raddr3(d1_rs2), .rdata3(arf_l1_rs2),
    .x14(x14)
);

wire       rat_l0_rs1_b, rat_l0_rs2_b, rat_l1_rs1_b, rat_l1_rs2_b;
wire [4:0] rat_l0_rs1_t, rat_l0_rs2_t, rat_l1_rs1_t, rat_l1_rs2_t;
wire       rat_d0_was_b, rat_d1_was_b, rat_cp_ready;

assign cp_take_l0 = disp0_fire && (d0_fu == FU_BR);
assign cp_take_l1 = disp1_fire && (d1_fu == FU_BR);

rat2w #(.TAG_W(TAG_W), .NUM_CP(4)) u_rat (
    .clk(clk), .reset_n(reset_n), .flush(1'b0), .rob_head(rob_head_ptr),
    .l0_rs1_addr(d0_rs1), .l0_rs1_busy(rat_l0_rs1_b), .l0_rs1_tag(rat_l0_rs1_t),
    .l0_rs2_addr(d0_rs2), .l0_rs2_busy(rat_l0_rs2_b), .l0_rs2_tag(rat_l0_rs2_t),
    .l1_rs1_addr(d1_rs1), .l1_rs1_busy(rat_l1_rs1_b), .l1_rs1_tag(rat_l1_rs1_t),
    .l1_rs2_addr(d1_rs2), .l1_rs2_busy(rat_l1_rs2_b), .l1_rs2_tag(rat_l1_rs2_t),
    .disp0_we(disp0_fire && d0_rd_we), .disp0_rd(d0_rd), .disp0_tag(rob_disp0_tag), .disp0_rd_was_busy(rat_d0_was_b),
    .disp1_we(disp1_fire && d1_rd_we), .disp1_rd(d1_rd), .disp1_tag(rob_disp1_tag), .disp1_rd_was_busy(rat_d1_was_b),
    .commit0_we(rob_c0_valid && rob_c0_rd_we), .commit0_rd(rob_c0_rd_addr), .commit0_tag(rob_c0_tag),
    .commit1_we(rob_c1_valid && rob_c1_rd_we), .commit1_rd(rob_c1_rd_addr), .commit1_tag(rob_c1_tag),
    .cp_take(cp_take_l0 || cp_take_l1),
    .cp_is_lane1(cp_take_l1),
    .cp_take_tag(cp_take_l1 ? rob_disp1_tag : rob_disp0_tag),
    .cp_ready(rat_cp_ready),
    .br_resolve_ok(rob_cdb0_br_ok),
    .br_resolve_tag(cdb0_tag),
    .squash_valid(rob_squash),
    .squash_tag(rob_squash_tag)
);

// Monotonic instruction sequence IDs (never rewind on squash)
reg [31:0] next_seq_id;
wire [31:0] seq0_id = next_seq_id;
wire [31:0] seq1_id = next_seq_id + 32'd1;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n)
        next_seq_id <= 32'd1;
    else if (disp1_fire)
        next_seq_id <= next_seq_id + 32'd2;
    else if (disp0_fire)
        next_seq_id <= next_seq_id + 32'd1;
end

wire        rob_rd0_rdy, rob_rd1_rdy, rob_rd2_rdy, rob_rd3_rdy;
wire [31:0] rob_rd0_val, rob_rd1_val, rob_rd2_val, rob_rd3_val;
wire        lsq_sq_empty;

rob2w #(.DEPTH(ROB2W_DEPTH), .TAG_W(TAG_W)) u_rob (
    .clk(clk), .reset_n(reset_n),
    .disp0_valid(disp0_fire), .disp1_valid(disp1_fire),
    .disp_free_ge1(rob_free_ge1), .disp_free_ge2(rob_free_ge2),
    .disp0_tag(rob_disp0_tag), .disp1_tag(rob_disp1_tag),
    .disp0_seq_id(seq0_id), .disp0_pc(fq0_pc), .disp0_instr(fq0_instr),
    .disp0_fu_type(d0_fu), .disp0_rd_we(d0_rd_we), .disp0_rd_addr(d0_rd),
    .disp0_pred_taken(fq0_ptaken), .disp0_pred_target(fq0_ptgt),
    .disp0_pred_ghr(fq0_pghr), .disp0_pred_used_ras(fq0_puras),
    .disp0_pred_bim(fq0_pbim), .disp0_pred_gsh(fq0_pgsh),
    .disp0_is_ebreak(d0_ebrk), .disp0_ready_init(d0_ebrk),
    .disp1_seq_id(seq1_id), .disp1_pc(fq1_pc), .disp1_instr(fq1_instr),
    .disp1_fu_type(d1_fu), .disp1_rd_we(d1_rd_we), .disp1_rd_addr(d1_rd),
    .disp1_pred_taken(fq1_ptaken), .disp1_pred_target(fq1_ptgt),
    .disp1_pred_ghr(fq1_pghr), .disp1_pred_used_ras(fq1_puras),
    .disp1_pred_bim(fq1_pbim), .disp1_pred_gsh(fq1_pgsh),
    .disp1_is_ebreak(d1_ebrk), .disp1_ready_init(d1_ebrk),
    .rd0_tag(rat_l0_rs1_t), .rd0_ready(rob_rd0_rdy), .rd0_value(rob_rd0_val),
    .rd1_tag(rat_l0_rs2_t), .rd1_ready(rob_rd1_rdy), .rd1_value(rob_rd1_val),
    .rd2_tag(rat_l1_rs1_t), .rd2_ready(rob_rd2_rdy), .rd2_value(rob_rd2_val),
    .rd3_tag(rat_l1_rs2_t), .rd3_ready(rob_rd3_rdy), .rd3_value(rob_rd3_val),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb0_is_br(cdb0_is_br), .cdb0_br_taken(cdb0_br_taken), .cdb0_br_target(cdb0_br_target),
    .cdb0_is_mem(cdb0_is_mem), .cdb0_mem_we(cdb0_mem_we),
    .cdb0_mem_addr(cdb0_mem_addr), .cdb0_mem_wstrb(cdb0_mem_wstrb), .cdb0_mem_wdata(cdb0_mem_wdata),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .halted_in(halted), .lsq_sq_empty(lsq_sq_empty && dc_writes_idle),
    .commit0_valid(rob_c0_valid), .commit0_tag(rob_c0_tag), .commit0_seq_id(commit0_seq_id),
    .commit0_pc(commit0_pc), .commit0_instr(commit0_instr), .commit0_fu_type(rob_c0_fu),
    .commit0_rd_we(rob_c0_rd_we), .commit0_rd_addr(rob_c0_rd_addr), .commit0_rd_wdata(rob_c0_rd_wdata),
    .commit0_mem_valid(commit0_mem_valid), .commit0_mem_we(commit0_mem_we),
    .commit0_mem_addr(commit0_mem_addr), .commit0_mem_wstrb(commit0_mem_wstrb), .commit0_mem_wdata(commit0_mem_wdata),
    .commit0_is_ebreak(rob_c0_ebrk), .commit0_is_br(rob_c0_is_br),
    .commit0_mispred(rob_c0_mispred), .commit0_used_ras(rob_c0_uras),
    .commit1_valid(rob_c1_valid), .commit1_tag(rob_c1_tag), .commit1_seq_id(commit1_seq_id),
    .commit1_pc(commit1_pc), .commit1_instr(commit1_instr), .commit1_fu_type(rob_c1_fu),
    .commit1_rd_we(rob_c1_rd_we), .commit1_rd_addr(rob_c1_rd_addr), .commit1_rd_wdata(rob_c1_rd_wdata),
    .commit1_mem_valid(commit1_mem_valid), .commit1_mem_we(commit1_mem_we),
    .commit1_mem_addr(commit1_mem_addr), .commit1_mem_wstrb(commit1_mem_wstrb), .commit1_mem_wdata(commit1_mem_wdata),
    .commit1_is_br(rob_c1_is_br), .commit1_mispred(rob_c1_mispred), .commit1_used_ras(rob_c1_uras),
    .squash_out(rob_squash), .squash_tag_out(rob_squash_tag), .squash_seq_out(rob_squash_seq),
    .squash_redirect_pc(rob_squash_redirect_pc), .squash_has_older(rob_squash_has_older),
    .cdb0_br_ok(rob_cdb0_br_ok), .cdb0_br_pc(rob_cdb0_br_pc),
    .cdb0_br_is_cond(rob_cdb0_br_is_cond), .cdb0_br_is_jalr(rob_cdb0_br_is_jalr),
    .cdb0_br_ghr(rob_cdb0_br_ghr),
    .cdb0_br_pred_bim(rob_cdb0_br_pbim), .cdb0_br_pred_gsh(rob_cdb0_br_pgsh),
    .head_ptr_out(rob_head_ptr), .rob_full(rob_full), .rob_empty(rob_empty)
);

always @(posedge clk or negedge reset_n) begin
    if (!reset_n)
        halted <= 1'b0;
    else if (rob_c0_valid && rob_c0_ebrk)
        halted <= 1'b1;
end

// ============================================================================
// 3. 2-Wide Operand Resolution (9-Way Bypass + Intra-Pair RAW Forwarding!)
//    Checks: x0 -> Intra-Pair -> ARF -> ROB -> ALU0/ALU1 EX-out -> CDB0/CDB1
// ============================================================================
reg [31:0] d0_vj, d0_vk, d1_vj, d1_vk;
reg [4:0]  d0_qj, d0_qk, d1_qj, d1_qk;
reg        d0_qj_v, d0_qk_v, d1_qj_v, d1_qk_v;

wire l1_rs1_from_l0 = d0_rd_we && (d0_rd != 5'd0) && (d0_rd == d1_rs1);
wire l1_rs2_from_l0 = d0_rd_we && (d0_rd != 5'd0) && (d0_rd == d1_rs2);

always @(*) begin
    // --- Lane 0 Source 1 ---
    if (!d0_uses_rs1 || (d0_rs1 == 5'd0)) begin
        d0_vj = 32'd0; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (!rat_l0_rs1_b) begin
        d0_vj = arf_l0_rs1; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (rob_rd0_rdy) begin
        d0_vj = rob_rd0_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (alu0_out_v && (alu0_out_tag == rat_l0_rs1_t)) begin
        d0_vj = alu0_out_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (alu1_out_v && (alu1_out_tag == rat_l0_rs1_t)) begin
        d0_vj = alu1_out_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (cdb0_valid && (cdb0_tag == rat_l0_rs1_t)) begin
        d0_vj = cdb0_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (cdb1_valid && (cdb1_tag == rat_l0_rs1_t)) begin
        d0_vj = cdb1_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else begin
        d0_vj = 32'd0; d0_qj = rat_l0_rs1_t; d0_qj_v = 1'b1;
    end

    // --- Lane 0 Source 2 ---
    if (!d0_uses_rs2 || (d0_rs2 == 5'd0)) begin
        d0_vk = 32'd0; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (!rat_l0_rs2_b) begin
        d0_vk = arf_l0_rs2; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (rob_rd1_rdy) begin
        d0_vk = rob_rd1_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (alu0_out_v && (alu0_out_tag == rat_l0_rs2_t)) begin
        d0_vk = alu0_out_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (alu1_out_v && (alu1_out_tag == rat_l0_rs2_t)) begin
        d0_vk = alu1_out_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (cdb0_valid && (cdb0_tag == rat_l0_rs2_t)) begin
        d0_vk = cdb0_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (cdb1_valid && (cdb1_tag == rat_l0_rs2_t)) begin
        d0_vk = cdb1_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else begin
        d0_vk = 32'd0; d0_qk = rat_l0_rs2_t; d0_qk_v = 1'b1;
    end

    // --- Lane 1 Source 1 (with Intra-Pair RAW check against Lane 0 rd!) ---
    if (!d1_uses_rs1 || (d1_rs1 == 5'd0)) begin
        d1_vj = 32'd0; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (l1_rs1_from_l0) begin
        d1_vj = 32'd0; d1_qj = rob_disp0_tag; d1_qj_v = 1'b1;
    end else if (!rat_l1_rs1_b) begin
        d1_vj = arf_l1_rs1; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (rob_rd2_rdy) begin
        d1_vj = rob_rd2_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (alu0_out_v && (alu0_out_tag == rat_l1_rs1_t)) begin
        d1_vj = alu0_out_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (alu1_out_v && (alu1_out_tag == rat_l1_rs1_t)) begin
        d1_vj = alu1_out_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (cdb0_valid && (cdb0_tag == rat_l1_rs1_t)) begin
        d1_vj = cdb0_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (cdb1_valid && (cdb1_tag == rat_l1_rs1_t)) begin
        d1_vj = cdb1_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else begin
        d1_vj = 32'd0; d1_qj = rat_l1_rs1_t; d1_qj_v = 1'b1;
    end

    // --- Lane 1 Source 2 (with Intra-Pair RAW check against Lane 0 rd!) ---
    if (!d1_uses_rs2 || (d1_rs2 == 5'd0)) begin
        d1_vk = 32'd0; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (l1_rs2_from_l0) begin
        d1_vk = 32'd0; d1_qk = rob_disp0_tag; d1_qk_v = 1'b1;
    end else if (!rat_l1_rs2_b) begin
        d1_vk = arf_l1_rs2; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (rob_rd3_rdy) begin
        d1_vk = rob_rd3_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (alu0_out_v && (alu0_out_tag == rat_l1_rs2_t)) begin
        d1_vk = alu0_out_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (alu1_out_v && (alu1_out_tag == rat_l1_rs2_t)) begin
        d1_vk = alu1_out_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (cdb0_valid && (cdb0_tag == rat_l1_rs2_t)) begin
        d1_vk = cdb0_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (cdb1_valid && (cdb1_tag == rat_l1_rs2_t)) begin
        d1_vk = cdb1_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else begin
        d1_vk = 32'd0; d1_qk = rat_l1_rs2_t; d1_qk_v = 1'b1;
    end
end

// ============================================================================
// 4. 2-Wide Structural Dispatch Pairing & Partitioned RS / LSQ
// ============================================================================
wire alu_free1, alu_free2, mul_free1, mul_free2;
wire div_free1, div_free2_u, br_free1, br_free2_u;
wire lsq_rdy_ld, lsq_rdy_st;

wire d0_is_mem = (d0_fu == FU_LOAD || d0_fu == FU_STORE);
wire d1_is_mem = (d1_fu == FU_LOAD || d1_fu == FU_STORE);

reg lane0_rs_ready, lane1_rs_ready;

always @(*) begin
    case (d0_fu)
        FU_ALU:    lane0_rs_ready = alu_free1;
        FU_MUL:    lane0_rs_ready = mul_free1;
        FU_DIV:    lane0_rs_ready = div_free1;
        FU_BR:     lane0_rs_ready = br_free1 && rat_cp_ready;
        FU_LOAD:   lane0_rs_ready = lsq_rdy_ld;
        FU_STORE:  lane0_rs_ready = lsq_rdy_st;
        FU_SYSTEM: lane0_rs_ready = 1'b1;
        default:   lane0_rs_ready = 1'b0;
    endcase

    case (d1_fu)
        FU_ALU:    lane1_rs_ready = (d0_fu == FU_ALU) ? alu_free2 : alu_free1;
        FU_MUL:    lane1_rs_ready = (d0_fu == FU_MUL) ? mul_free2 : mul_free1;
        FU_DIV:    lane1_rs_ready = (d0_fu != FU_DIV) && div_free1;
        FU_BR:     lane1_rs_ready = (d0_fu != FU_BR)  && br_free1 && rat_cp_ready;
        FU_LOAD:   lane1_rs_ready = !d0_is_mem && lsq_rdy_ld;
        FU_STORE:  lane1_rs_ready = !d0_is_mem && lsq_rdy_st;
        FU_SYSTEM: lane1_rs_ready = 1'b1;
        default:   lane1_rs_ready = 1'b0;
    endcase
end

assign disp0_fire = fq0_valid && !halted && !rob_squash && rob_free_ge1 && lane0_rs_ready;
assign disp1_fire = disp0_fire && fq1_valid && !d0_ebrk && rob_free_ge2 && lane1_rs_ready;

// --- RS_ALU (8 entries, 2-wide alloc, 2-wide issue -> ALU0 + ALU1!) ---
wire        alu0_iss_v, alu1_iss_v, alu0_fu_rdy, alu1_fu_rdy;
wire [3:0]  alu0_iss_op, alu1_iss_op;
wire [4:0]  alu0_iss_tag, alu1_iss_tag;
wire [31:0] alu0_iss_seq, alu0_iss_pc, alu0_iss_imm, alu0_iss_vj, alu0_iss_vk;
wire [31:0] alu1_iss_seq, alu1_iss_pc, alu1_iss_imm, alu1_iss_vj, alu1_iss_vk;
wire        alu0_iss_asel, alu0_iss_bsel, alu1_iss_asel, alu1_iss_bsel;
wire [2:0]  alu_byp_cnt, mul_byp_cnt, div_byp_cnt, br_byp_cnt;
wire [1:0]  lsq_byp_cnt;

assign wk0_valid = alu0_iss_v && alu0_fu_rdy;
assign wk0_tag   = alu0_iss_tag;
assign wk1_valid = alu1_iss_v && alu1_fu_rdy;
assign wk1_tag   = alu1_iss_tag;

rs_cluster2w #(.ENTRIES(8), .TAG_W(TAG_W), .OP_W(4), .DUAL_ISSUE(1)) u_rs_alu (
    .clk(clk), .reset_n(reset_n), .flush(1'b0),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .free_ge1(alu_free1), .free_ge2(alu_free2),
    .disp0_valid(disp0_fire && (d0_fu == FU_ALU)), .disp0_op(d0_alu_sel),
    .disp0_rob_tag(rob_disp0_tag), .disp0_seq_id(seq0_id), .disp0_pc(fq0_pc), .disp0_imm(d0_imm),
    .disp0_asel(d0_asel), .disp0_bsel(d0_bsel),
    .disp0_vj(d0_vj), .disp0_qj(d0_qj), .disp0_qj_valid(d0_qj_v),
    .disp0_vk(d0_vk), .disp0_qk(d0_qk), .disp0_qk_valid(d0_qk_v),
    .disp1_valid(disp1_fire && (d1_fu == FU_ALU)), .disp1_op(d1_alu_sel),
    .disp1_rob_tag(rob_disp1_tag), .disp1_seq_id(seq1_id), .disp1_pc(fq1_pc), .disp1_imm(d1_imm),
    .disp1_asel(d1_asel), .disp1_bsel(d1_bsel),
    .disp1_vj(d1_vj), .disp1_qj(d1_qj), .disp1_qj_valid(d1_qj_v),
    .disp1_vk(d1_vk), .disp1_qk(d1_qk), .disp1_qk_valid(d1_qk_v),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .wk0_valid(wk0_valid), .wk0_tag(wk0_tag), .wk1_valid(wk1_valid), .wk1_tag(wk1_tag),
    .byp_alu0_val(alu0_out_val), .byp_alu1_val(alu1_out_val),
    .issue0_valid(alu0_iss_v), .fu0_ready(alu0_fu_rdy), .issue0_op(alu0_iss_op),
    .issue0_rob_tag(alu0_iss_tag), .issue0_seq_id(alu0_iss_seq), .issue0_pc(alu0_iss_pc),
    .issue0_imm(alu0_iss_imm), .issue0_asel(alu0_iss_asel), .issue0_bsel(alu0_iss_bsel),
    .issue0_vj(alu0_iss_vj), .issue0_vk(alu0_iss_vk),
    .issue1_valid(alu1_iss_v), .fu1_ready(alu1_fu_rdy), .issue1_op(alu1_iss_op),
    .issue1_rob_tag(alu1_iss_tag), .issue1_seq_id(alu1_iss_seq), .issue1_pc(alu1_iss_pc),
    .issue1_imm(alu1_iss_imm), .issue1_asel(alu1_iss_asel), .issue1_bsel(alu1_iss_bsel),
    .issue1_vj(alu1_iss_vj), .issue1_vk(alu1_iss_vk),
    .stat_ex_byp_cnt(alu_byp_cnt)
);

// --- RS_MUL (4 entries, 2-wide alloc, 1-wide issue -> 4-cyc pipelined MUL) ---
wire        mul_iss_v, mul_fu_rdy, mul_i1_v_u;
wire [1:0]  mul_iss_op, mul_i1_op_u;
wire [4:0]  mul_iss_tag, mul_i1_tag_u;
wire [31:0] mul_iss_seq, mul_iss_vj, mul_iss_vk;
wire [31:0] mul_u0, mul_u1, mul_u2, mul_u3, mul_u4, mul_u5, mul_u6;
wire        mul_b0, mul_b1, mul_b2, mul_b3;

rs_cluster2w #(.ENTRIES(4), .TAG_W(TAG_W), .OP_W(2), .DUAL_ISSUE(0)) u_rs_mul (
    .clk(clk), .reset_n(reset_n), .flush(1'b0),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .free_ge1(mul_free1), .free_ge2(mul_free2),
    .disp0_valid(disp0_fire && (d0_fu == FU_MUL)), .disp0_op(d0_mul_op),
    .disp0_rob_tag(rob_disp0_tag), .disp0_seq_id(seq0_id), .disp0_pc(fq0_pc), .disp0_imm(32'd0),
    .disp0_asel(1'b0), .disp0_bsel(1'b0),
    .disp0_vj(d0_vj), .disp0_qj(d0_qj), .disp0_qj_valid(d0_qj_v),
    .disp0_vk(d0_vk), .disp0_qk(d0_qk), .disp0_qk_valid(d0_qk_v),
    .disp1_valid(disp1_fire && (d1_fu == FU_MUL)), .disp1_op(d1_mul_op),
    .disp1_rob_tag(rob_disp1_tag), .disp1_seq_id(seq1_id), .disp1_pc(fq1_pc), .disp1_imm(32'd0),
    .disp1_asel(1'b0), .disp1_bsel(1'b0),
    .disp1_vj(d1_vj), .disp1_qj(d1_qj), .disp1_qj_valid(d1_qj_v),
    .disp1_vk(d1_vk), .disp1_qk(d1_qk), .disp1_qk_valid(d1_qk_v),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .wk0_valid(wk0_valid), .wk0_tag(wk0_tag), .wk1_valid(wk1_valid), .wk1_tag(wk1_tag),
    .byp_alu0_val(alu0_out_val), .byp_alu1_val(alu1_out_val),
    .issue0_valid(mul_iss_v), .fu0_ready(mul_fu_rdy), .issue0_op(mul_iss_op),
    .issue0_rob_tag(mul_iss_tag), .issue0_seq_id(mul_iss_seq), .issue0_pc(mul_u0),
    .issue0_imm(mul_u1), .issue0_asel(mul_b0), .issue0_bsel(mul_b1),
    .issue0_vj(mul_iss_vj), .issue0_vk(mul_iss_vk),
    .issue1_valid(mul_i1_v_u), .fu1_ready(1'b0), .issue1_op(mul_i1_op_u),
    .issue1_rob_tag(mul_i1_tag_u), .issue1_seq_id(mul_u2), .issue1_pc(mul_u3),
    .issue1_imm(mul_u4), .issue1_asel(mul_b2), .issue1_bsel(mul_b3),
    .issue1_vj(mul_u5), .issue1_vk(mul_u6),
    .stat_ex_byp_cnt(mul_byp_cnt)
);

// --- RS_DIV (2 entries, 1-wide alloc from either lane) ---
wire        div_iss_v, div_fu_rdy, div_i1_v_u;
wire [1:0]  div_iss_op, div_i1_op_u;
wire [4:0]  div_iss_tag, div_i1_tag_u;
wire [31:0] div_iss_seq, div_iss_vj, div_iss_vk;
wire [31:0] div_u0, div_u1, div_u2, div_u3, div_u4, div_u5, div_u6;
wire        div_b0, div_b1, div_b2, div_b3;

rs_cluster2w #(.ENTRIES(2), .TAG_W(TAG_W), .OP_W(2), .DUAL_ISSUE(0)) u_rs_div (
    .clk(clk), .reset_n(reset_n), .flush(1'b0),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .free_ge1(div_free1), .free_ge2(div_free2_u),
    .disp0_valid(disp0_fire && (d0_fu == FU_DIV)), .disp0_op(d0_div_op),
    .disp0_rob_tag(rob_disp0_tag), .disp0_seq_id(seq0_id), .disp0_pc(fq0_pc), .disp0_imm(32'd0),
    .disp0_asel(1'b0), .disp0_bsel(1'b0),
    .disp0_vj(d0_vj), .disp0_qj(d0_qj), .disp0_qj_valid(d0_qj_v),
    .disp0_vk(d0_vk), .disp0_qk(d0_qk), .disp0_qk_valid(d0_qk_v),
    .disp1_valid(disp1_fire && (d1_fu == FU_DIV)), .disp1_op(d1_div_op),
    .disp1_rob_tag(rob_disp1_tag), .disp1_seq_id(seq1_id), .disp1_pc(fq1_pc), .disp1_imm(32'd0),
    .disp1_asel(1'b0), .disp1_bsel(1'b0),
    .disp1_vj(d1_vj), .disp1_qj(d1_qj), .disp1_qj_valid(d1_qj_v),
    .disp1_vk(d1_vk), .disp1_qk(d1_qk), .disp1_qk_valid(d1_qk_v),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .wk0_valid(wk0_valid), .wk0_tag(wk0_tag), .wk1_valid(wk1_valid), .wk1_tag(wk1_tag),
    .byp_alu0_val(alu0_out_val), .byp_alu1_val(alu1_out_val),
    .issue0_valid(div_iss_v), .fu0_ready(div_fu_rdy), .issue0_op(div_iss_op),
    .issue0_rob_tag(div_iss_tag), .issue0_seq_id(div_iss_seq), .issue0_pc(div_u0),
    .issue0_imm(div_u1), .issue0_asel(div_b0), .issue0_bsel(div_b1),
    .issue0_vj(div_iss_vj), .issue0_vk(div_iss_vk),
    .issue1_valid(div_i1_v_u), .fu1_ready(1'b0), .issue1_op(div_i1_op_u),
    .issue1_rob_tag(div_i1_tag_u), .issue1_seq_id(div_u2), .issue1_pc(div_u3),
    .issue1_imm(div_u4), .issue1_asel(div_b2), .issue1_bsel(div_b3),
    .issue1_vj(div_u5), .issue1_vk(div_u6),
    .stat_ex_byp_cnt(div_byp_cnt)
);

// --- RS_BR (4 entries, 1-wide alloc from either lane) ---
wire        br_iss_v, br_fu_rdy, br_i1_v_u;
wire [4:0]  br_iss_op, br_i1_op_u;
wire [4:0]  br_iss_tag, br_i1_tag_u;
wire [31:0] br_iss_seq, br_iss_pc, br_iss_imm, br_iss_vj, br_iss_vk;
wire [31:0] br_u2, br_u3, br_u4, br_u5, br_u6;
wire        br_b0, br_b1, br_b2, br_b3;

rs_cluster2w #(.ENTRIES(4), .TAG_W(TAG_W), .OP_W(5), .DUAL_ISSUE(0)) u_rs_br (
    .clk(clk), .reset_n(reset_n), .flush(1'b0),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .free_ge1(br_free1), .free_ge2(br_free2_u),
    .disp0_valid(disp0_fire && (d0_fu == FU_BR)), .disp0_op({d0_is_jal, d0_is_jalr, d0_br_f3}),
    .disp0_rob_tag(rob_disp0_tag), .disp0_seq_id(seq0_id), .disp0_pc(fq0_pc), .disp0_imm(d0_imm),
    .disp0_asel(1'b0), .disp0_bsel(1'b0),
    .disp0_vj(d0_vj), .disp0_qj(d0_qj), .disp0_qj_valid(d0_qj_v),
    .disp0_vk(d0_vk), .disp0_qk(d0_qk), .disp0_qk_valid(d0_qk_v),
    .disp1_valid(disp1_fire && (d1_fu == FU_BR)), .disp1_op({d1_is_jal, d1_is_jalr, d1_br_f3}),
    .disp1_rob_tag(rob_disp1_tag), .disp1_seq_id(seq1_id), .disp1_pc(fq1_pc), .disp1_imm(d1_imm),
    .disp1_asel(1'b0), .disp1_bsel(1'b0),
    .disp1_vj(d1_vj), .disp1_qj(d1_qj), .disp1_qj_valid(d1_qj_v),
    .disp1_vk(d1_vk), .disp1_qk(d1_qk), .disp1_qk_valid(d1_qk_v),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .wk0_valid(wk0_valid), .wk0_tag(wk0_tag), .wk1_valid(wk1_valid), .wk1_tag(wk1_tag),
    .byp_alu0_val(alu0_out_val), .byp_alu1_val(alu1_out_val),
    .issue0_valid(br_iss_v), .fu0_ready(br_fu_rdy), .issue0_op(br_iss_op),
    .issue0_rob_tag(br_iss_tag), .issue0_seq_id(br_iss_seq), .issue0_pc(br_iss_pc),
    .issue0_imm(br_iss_imm), .issue0_asel(br_b0), .issue0_bsel(br_b1),
    .issue0_vj(br_iss_vj), .issue0_vk(br_iss_vk),
    .issue1_valid(br_i1_v_u), .fu1_ready(1'b0), .issue1_op(br_i1_op_u),
    .issue1_rob_tag(br_i1_tag_u), .issue1_seq_id(br_u2), .issue1_pc(br_u3),
    .issue1_imm(br_u4), .issue1_asel(br_b2), .issue1_bsel(br_b3),
    .issue1_vj(br_u5), .issue1_vk(br_u6),
    .stat_ex_byp_cnt(br_byp_cnt)
);

// ============================================================================
// 5. Six Parallel Functional Units (ALU0, ALU1, MUL, DIV, BR, LSU)
// ============================================================================
wire cdb_div_grant, cdb_lsu_grant, cdb_mul_grant, cdb_br_grant, cdb_alu0_grant, cdb_alu1_grant;
wire [4:0] sq_br_age_core = rob_squash_tag - rob_head_ptr;

// --- FU_ALU0 ---
wire [31:0] alu0_comb_res;
alu u_fu_alu0 (
    .inp1(alu0_iss_asel ? alu0_iss_pc  : alu0_iss_vj),
    .inp2(alu0_iss_bsel ? alu0_iss_imm : alu0_iss_vk),
    .alu_select(alu0_iss_op),
    .alu_out(alu0_comb_res)
);

assign alu0_fu_rdy = !alu0_out_v || cdb_alu0_grant;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        alu0_out_v <= 1'b0; alu0_out_tag <= 5'd0; alu0_out_seq <= 32'd0; alu0_out_val <= 32'd0;
    end else if (alu0_fu_rdy) begin
        alu0_out_v   <= alu0_iss_v;
        alu0_out_tag <= alu0_iss_tag;
        alu0_out_seq <= alu0_iss_seq;
        alu0_out_val <= alu0_comb_res;
    end else if (rob_squash && ((alu0_out_tag - rob_head_ptr) > sq_br_age_core)) begin
        alu0_out_v <= 1'b0;
    end
end

// --- FU_ALU1 ---
wire [31:0] alu1_comb_res;
alu u_fu_alu1 (
    .inp1(alu1_iss_asel ? alu1_iss_pc  : alu1_iss_vj),
    .inp2(alu1_iss_bsel ? alu1_iss_imm : alu1_iss_vk),
    .alu_select(alu1_iss_op),
    .alu_out(alu1_comb_res)
);

assign alu1_fu_rdy = !alu1_out_v || cdb_alu1_grant;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        alu1_out_v <= 1'b0; alu1_out_tag <= 5'd0; alu1_out_seq <= 32'd0; alu1_out_val <= 32'd0;
    end else if (alu1_fu_rdy) begin
        alu1_out_v   <= alu1_iss_v;
        alu1_out_tag <= alu1_iss_tag;
        alu1_out_seq <= alu1_iss_seq;
        alu1_out_val <= alu1_comb_res;
    end else if (rob_squash && ((alu1_out_tag - rob_head_ptr) > sq_br_age_core)) begin
        alu1_out_v <= 1'b0;
    end
end

// --- FU_MUL (4-cycle pipelined, TAG_W=5) ---
wire        mul_out_v;
wire [4:0]  mul_out_tag;
wire [31:0] mul_out_seq, mul_out_val;

mul_unit #(.TAG_W(TAG_W)) u_fu_mul (
    .clk(clk), .reset_n(reset_n), .flush(1'b0),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .in_valid(mul_iss_v), .in_ready(mul_fu_rdy), .in_op(mul_iss_op),
    .in_rs1(mul_iss_vj), .in_rs2(mul_iss_vk), .in_tag(mul_iss_tag), .in_seq(mul_iss_seq),
    .out_valid(mul_out_v), .out_ready(cdb_mul_grant),
    .out_result(mul_out_val), .out_tag(mul_out_tag), .out_seq(mul_out_seq)
);

// --- FU_DIV (18-cycle radix-4, TAG_W=5) ---
wire        div_out_v;
wire [4:0]  div_out_tag;
wire [31:0] div_out_seq, div_out_val;

div_unit #(.TAG_W(TAG_W)) u_fu_div (
    .clk(clk), .reset_n(reset_n), .flush(1'b0),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .in_valid(div_iss_v), .in_ready(div_fu_rdy), .in_op(div_iss_op),
    .in_rs1(div_iss_vj), .in_rs2(div_iss_vk), .in_tag(div_iss_tag), .in_seq(div_iss_seq),
    .out_valid(div_out_v), .out_ready(cdb_div_grant),
    .out_result(div_out_val), .out_tag(div_out_tag), .out_seq(div_out_seq)
);

// --- FU_BR ---
wire       br_op_is_jal  = br_iss_op[4];
wire       br_op_is_jalr = br_iss_op[3];
wire [2:0] br_op_f3      = br_iss_op[2:0];
wire       br_cmp_eq, br_cmp_lt, br_cmp_ge;

branch_comp u_fu_brcmp (
    .inpA(br_iss_vj), .inpB(br_iss_vk), .brun_en(br_op_f3[2:1] == 2'b11),
    .breq_flag(br_cmp_eq), .brlt_flag(br_cmp_lt), .bge_flag(br_cmp_ge)
);

reg br_comb_taken;
always @(*) begin
    if (br_op_is_jal || br_op_is_jalr) begin
        br_comb_taken = 1'b1;
    end else begin
        case (br_op_f3)
            3'b000: br_comb_taken = br_cmp_eq;
            3'b001: br_comb_taken = ~br_cmp_eq;
            3'b100: br_comb_taken = br_cmp_lt;
            3'b101: br_comb_taken = br_cmp_ge;
            3'b110: br_comb_taken = br_cmp_lt;
            3'b111: br_comb_taken = br_cmp_ge;
            default: br_comb_taken = 1'b0;
        endcase
    end
end

wire [31:0] br_comb_target = br_op_is_jalr ? ((br_iss_vj + br_iss_imm) & 32'hFFFF_FFFE)
                                           : (br_iss_pc + br_iss_imm);
wire [31:0] br_comb_link   = br_iss_pc + 32'd4;

reg        br_out_v, br_out_taken;
reg [4:0]  br_out_tag;
reg [31:0] br_out_seq, br_out_val, br_out_target;
assign br_fu_rdy = !br_out_v || cdb_br_grant;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        br_out_v <= 1'b0; br_out_tag <= 5'd0; br_out_seq <= 32'd0;
        br_out_val <= 32'd0; br_out_taken <= 1'b0; br_out_target <= 32'd0;
    end else if (br_fu_rdy) begin
        br_out_v      <= br_iss_v;
        br_out_tag    <= br_iss_tag;
        br_out_seq    <= br_iss_seq;
        br_out_val    <= br_comb_link;
        br_out_taken  <= br_comb_taken;
        br_out_target <= br_comb_target;
    end else if (rob_squash && ((br_out_tag - rob_head_ptr) > sq_br_age_core)) begin
        br_out_v <= 1'b0;
    end
end

// --- FU_LSU (8 LQ + 8 SQ, Dual-CDB wakeup, max 1 mem dispatch/cycle) ---
wire lsq_disp_l0 = disp0_fire && d0_is_mem;
wire lsq_disp_l1 = disp1_fire && d1_is_mem;

wire        lsu_out_v, lsu_out_mem_we;
wire [4:0]  lsu_out_tag;
wire [3:0]  lsu_out_mem_wstrb;
wire [31:0] lsu_out_seq, lsu_out_val, lsu_out_mem_addr, lsu_out_mem_wdata;
wire        lsq_trace_issue_v;
wire [31:0] lsq_trace_issue_seq;

wire rob_commit_store = (rob_c0_valid && (rob_c0_fu == FU_STORE)) ||
                        (rob_c1_valid && (rob_c1_fu == FU_STORE));

lsq2w #(.LQ_DEPTH(8), .SQ_DEPTH(8), .TAG_W(TAG_W), .ID_W(6)) u_lsq (
    .clk(clk), .reset_n(reset_n), .flush(1'b0),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .disp_valid(lsq_disp_l0 || lsq_disp_l1),
    .disp_is_store(lsq_disp_l0 ? (d0_fu == FU_STORE) : (d1_fu == FU_STORE)),
    .disp_ready_load(lsq_rdy_ld),
    .disp_ready_store(lsq_rdy_st),
    .disp_rob_tag(lsq_disp_l0 ? rob_disp0_tag : rob_disp1_tag),
    .disp_seq_id(lsq_disp_l0 ? seq0_id : seq1_id),
    .disp_mem_size(lsq_disp_l0 ? d0_mem_sz : d1_mem_sz),
    .disp_mem_unsigned(lsq_disp_l0 ? d0_mem_uns : d1_mem_uns),
    .disp_imm(lsq_disp_l0 ? d0_imm : d1_imm),
    .disp_vj(lsq_disp_l0 ? d0_vj : d1_vj),
    .disp_qj(lsq_disp_l0 ? d0_qj : d1_qj),
    .disp_qj_valid(lsq_disp_l0 ? d0_qj_v : d1_qj_v),
    .disp_vk(lsq_disp_l0 ? d0_vk : d1_vk),
    .disp_qk(lsq_disp_l0 ? d0_qk : d1_qk),
    .disp_qk_valid(lsq_disp_l0 ? d0_qk_v : d1_qk_v),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .wk0_valid(wk0_valid), .wk0_tag(wk0_tag), .wk1_valid(wk1_valid), .wk1_tag(wk1_tag),
    .byp_alu0_val(alu0_out_val), .byp_alu1_val(alu1_out_val),
    .rob_commit_store(rob_commit_store),
    .lsu_cdb_valid(lsu_out_v), .lsu_cdb_grant(cdb_lsu_grant),
    .lsu_cdb_tag(lsu_out_tag), .lsu_cdb_seq(lsu_out_seq), .lsu_cdb_val(lsu_out_val),
    .lsu_cdb_mem_we(lsu_out_mem_we), .lsu_cdb_mem_addr(lsu_out_mem_addr),
    .lsu_cdb_mem_wstrb(lsu_out_mem_wstrb), .lsu_cdb_mem_wdata(lsu_out_mem_wdata),
    .dc_req_valid(dc_req_valid), .dc_req_ready(dc_req_ready), .dc_req_is_write(dc_req_is_write),
    .dc_req_addr(dc_req_addr), .dc_req_wdata(dc_req_wdata), .dc_req_wstrb(dc_req_wstrb),
    .dc_req_id(dc_req_id), .dc_req_rob_tag(dc_req_rob_tag),
    .dc_hit_valid(dc_hit_valid), .dc_hit_id(dc_hit_id), .dc_hit_rdata(dc_hit_rdata),
    .dc_fill_valid(dc_fill_valid), .dc_fill_id(dc_fill_id), .dc_fill_rdata(dc_fill_rdata),
    .sq_empty(lsq_sq_empty),
    .trace_issue_valid(lsq_trace_issue_v), .trace_issue_seq(lsq_trace_issue_seq),
    .stat_stlf_hit(stat_stlf_hit),
    .stat_ex_byp_cnt(lsq_byp_cnt)
);

assign dc_squash_valid = rob_squash;
assign dc_squash_tag   = rob_squash_tag;
assign dc_rob_head     = rob_head_ptr;

// ============================================================================
// 6. Dual-CDB Arbiter (up to 2 completions/cycle!)
// ============================================================================
cdb_arbiter2w #(.TAG_W(TAG_W)) u_cdb (
    .div_valid(div_out_v), .div_grant(cdb_div_grant), .div_tag(div_out_tag), .div_seq(div_out_seq), .div_val(div_out_val),
    .lsu_valid(lsu_out_v), .lsu_grant(cdb_lsu_grant), .lsu_tag(lsu_out_tag), .lsu_seq(lsu_out_seq), .lsu_val(lsu_out_val),
    .lsu_mem_we(lsu_out_mem_we), .lsu_mem_addr(lsu_out_mem_addr), .lsu_mem_wstrb(lsu_out_mem_wstrb), .lsu_mem_wdata(lsu_out_mem_wdata),
    .mul_valid(mul_out_v), .mul_grant(cdb_mul_grant), .mul_tag(mul_out_tag), .mul_seq(mul_out_seq), .mul_val(mul_out_val),
    .br_valid(br_out_v), .br_grant(cdb_br_grant), .br_tag(br_out_tag), .br_seq(br_out_seq), .br_val(br_out_val),
    .br_taken(br_out_taken), .br_target(br_out_target),
    .alu0_valid(alu0_out_v), .alu0_grant(cdb_alu0_grant), .alu0_tag(alu0_out_tag), .alu0_seq(alu0_out_seq), .alu0_val(alu0_out_val),
    .alu1_valid(alu1_out_v), .alu1_grant(cdb_alu1_grant), .alu1_tag(alu1_out_tag), .alu1_seq(alu1_out_seq), .alu1_val(alu1_out_val),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_seq(cdb0_seq), .cdb0_val(cdb0_val),
    .cdb0_is_br(cdb0_is_br), .cdb0_br_taken(cdb0_br_taken), .cdb0_br_target(cdb0_br_target),
    .cdb0_is_mem(cdb0_is_mem), .cdb0_mem_we(cdb0_mem_we), .cdb0_mem_addr(cdb0_mem_addr),
    .cdb0_mem_wstrb(cdb0_mem_wstrb), .cdb0_mem_wdata(cdb0_mem_wdata),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_seq(cdb1_seq), .cdb1_val(cdb1_val)
);

// ============================================================================
// 7. RVFI Outputs, 2-Wide Trace Hooks & PMU Telemetry
// ============================================================================
assign commit0_valid    = rob_c0_valid;
assign commit0_rd_we    = rob_c0_rd_we;
assign commit0_rd_addr  = rob_c0_rd_addr;
assign commit0_rd_wdata = rob_c0_rd_wdata;

assign commit1_valid    = rob_c1_valid;
assign commit1_rd_we    = rob_c1_rd_we;
assign commit1_rd_addr  = rob_c1_rd_addr;
assign commit1_rd_wdata = rob_c1_rd_wdata;

assign trace_disp0_valid = disp0_fire;
assign trace_disp0_seq   = seq0_id;
assign trace_disp0_pc    = fq0_pc;
assign trace_disp0_instr = fq0_instr;

assign trace_disp1_valid = disp1_fire;
assign trace_disp1_seq   = seq1_id;
assign trace_disp1_pc    = fq1_pc;
assign trace_disp1_instr = fq1_instr;

assign trace_issue_valid[0] = alu0_iss_v && alu0_fu_rdy;
assign trace_issue_valid[1] = alu1_iss_v && alu1_fu_rdy;
assign trace_issue_valid[2] = mul_iss_v  && mul_fu_rdy;
assign trace_issue_valid[3] = div_iss_v  && div_fu_rdy;
assign trace_issue_valid[4] = br_iss_v   && br_fu_rdy;
assign trace_issue_valid[5] = lsq_trace_issue_v;

assign trace_issue_seq[0*32 +: 32] = alu0_iss_seq;
assign trace_issue_seq[1*32 +: 32] = alu1_iss_seq;
assign trace_issue_seq[2*32 +: 32] = mul_iss_seq;
assign trace_issue_seq[3*32 +: 32] = div_iss_seq;
assign trace_issue_seq[4*32 +: 32] = br_iss_seq;
assign trace_issue_seq[5*32 +: 32] = lsq_trace_issue_seq;

assign trace_cdb_valid          = {cdb1_valid, cdb0_valid};
assign trace_cdb_seq[31:0]      = cdb0_seq;
assign trace_cdb_seq[63:32]     = cdb1_seq;
assign trace_flush_valid        = rob_squash;
assign trace_flush_after_seq    = rob_squash_seq;

assign stat_rob_full_stall = fq0_valid && !halted && !rob_squash && !rob_free_ge1;
assign stat_cp_full_stall  = fq0_valid && !halted && !rob_squash && rob_free_ge1 &&
                             (d0_fu == FU_BR) && br_free1 && !rat_cp_ready;
assign stat_rs_full_stall  = fq0_valid && !halted && !rob_squash && rob_free_ge1 &&
                             !d0_is_mem && !lane0_rs_ready && !stat_cp_full_stall;
assign stat_lsq_full_stall = fq0_valid && !halted && !rob_squash && rob_free_ge1 &&
                             d0_is_mem && !lane0_rs_ready;

assign stat_br_total_cnt   = {1'b0, (rob_c0_valid && rob_c0_is_br)} +
                             {1'b0, (rob_c1_valid && rob_c1_is_br)};
assign stat_br_mispred_cnt = {1'b0, (rob_c0_valid && rob_c0_mispred)} +
                             {1'b0, (rob_c1_valid && rob_c1_mispred)};
assign stat_br_fast_squash = rob_squash_has_older;
assign stat_war_waw_cnt    = {1'b0, (disp0_fire && d0_rd_we && (d0_rd != 5'd0) && rat_d0_was_b)} +
                             {1'b0, (disp1_fire && d1_rd_we && (d1_rd != 5'd0) && rat_d1_was_b)};

wire other_fu_no_div = trace_issue_valid[0] || trace_issue_valid[1] || trace_issue_valid[2] ||
                       trace_issue_valid[4] || trace_issue_valid[5] ||
                       (cdb0_valid && !cdb_div_grant) || cdb1_valid;
wire other_fu_no_dc  = trace_issue_valid[0] || trace_issue_valid[1] || trace_issue_valid[2] ||
                       trace_issue_valid[3] || trace_issue_valid[4] ||
                       stat_stlf_hit || cdb0_valid || cdb1_valid;

assign stat_div_overlap_cyc         = (!div_fu_rdy) && other_fu_no_div;
assign stat_dcache_miss_overlap_cyc = dc_any_miss   && other_fu_no_dc;
assign stat_dual_disp               = disp1_fire;
assign stat_dual_cdb                = cdb0_valid && cdb1_valid;
assign stat_dual_commit             = rob_c1_valid;
assign stat_ex_byp_cnt              = {1'b0, alu_byp_cnt} + {1'b0, mul_byp_cnt} +
                                      {1'b0, div_byp_cnt} + {1'b0, br_byp_cnt} +
                                      {2'b00, lsq_byp_cnt};
assign stat_ras_pred_cnt            = {1'b0, (rob_c0_valid && rob_c0_uras)} +
                                      {1'b0, (rob_c1_valid && rob_c1_uras)};
assign stat_ras_mispred_cnt         = {1'b0, (rob_c0_valid && rob_c0_uras && rob_c0_mispred)} +
                                      {1'b0, (rob_c1_valid && rob_c1_uras && rob_c1_mispred)};

endmodule
