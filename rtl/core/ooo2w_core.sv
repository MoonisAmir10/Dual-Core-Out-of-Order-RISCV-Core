`timescale 1ns / 1ps

// ============================================================================
// 2-Wide Superscalar Tomasulo Out-of-Order RV32IMAF_Zicsr_Zifencei Core (v4)
//
// Phase U6 Non-Blocking RV32F Integration:
//   - Full 26-instruction RV32F decode w/ mstatus.FS & frm legality checks
//   - Separate 6R2W FP Register File (fregfile2w, f0 writable) & 2-wide
//     4-checkpoint FP RAT (frat2w) synchronized with integer RAT
//   - 3 Dedicated FP RS Banks (rs_fpu2w): 6e 3-src FMA, 2e FDIV/FSQRT, 4e FMISC
//   - 3 Non-Blocking FP Execution Units: 4-stage II=1 fpu_fma_pipe, iterative
//     fpu_div_sqrt, 1-cycle fpu_misc
//   - Dedicated 3rd CDB (cdb_fp: FDIV > FMA > FMISC) -> ZERO INT<->FP contention!
//   - Seamless FLW/FSW via unified 8LQ+8SQ (STLF, misaligned trap, snoop-replay)
//   - In-order non-blocking fflags accumulation & mstatus.FS=Dirty at ROB commit
//   - Post-commit CSR sync flush on writes to mstatus/frm/fcsr
// ============================================================================
module ooo2w_core #(
    parameter [31:0]  CORE_ID         = 32'd0,
    parameter [31:0]  RESET_PC        = 32'd0,
    parameter integer AWIDTH          = 20,
    parameter integer ID_W            = 7,
    parameter integer ENABLE_MMIO_PMA = 1,
    parameter [31:0]  RAM_SIZE_BYTES  = 32'h0004_0000
) (
    input  wire                clk,
    input  wire                reset_n,

    // Stall-Capable L1 Instruction Cache Interface
    output wire                imem_req_valid,
    output wire [31:0]         imem_pc0,
    input  wire                imem_valid0,
    input  wire [31:0]         imem_instr0,
    output wire [31:0]         imem_pc1,
    input  wire                imem_valid1,
    input  wire [31:0]         imem_instr1,
    output wire                ic_flush,
    output wire                ic_invalidate_all,

    // Split-Transaction Non-Blocking D-Cache Interface (with RV32A Atomics + FENCE.I + MMIO)
    output wire                dc_req_valid,
    input  wire                dc_req_ready,
    output wire                dc_req_is_write,
    output wire                dc_req_is_mmio,
    output wire [3:0]          dc_req_amo_op,
    output wire [AWIDTH-1:0]   dc_req_addr,
    output wire [31:0]         dc_req_paddr,
    output wire [31:0]         dc_req_wdata,
    output wire [3:0]          dc_req_wstrb,
    output wire [ID_W-1:0]     dc_req_id,
    output wire [4:0]          dc_req_rob_tag,

    input  wire                dc_hit_valid,
    input  wire [ID_W-1:0]     dc_hit_id,
    input  wire [31:0]         dc_hit_rdata,
    input  wire                dc_hit_err,

    input  wire                dc_fill_valid,
    input  wire [ID_W-1:0]     dc_fill_id,
    input  wire [31:0]         dc_fill_rdata,
    input  wire                dc_fill_err,

    output wire                dc_flush,
    output wire                dc_squash_valid,
    output wire [4:0]          dc_squash_tag,
    output wire [4:0]          dc_rob_head,
    output wire                dc_clean_all_req,
    input  wire                dc_clean_all_done,
    input  wire                dc_writes_idle,
    input  wire                dc_any_miss,

    // External Coherence Snoop-Invalidation Port (triggers LQ Memory-Order Replay!)
    input  wire                coh_snoop_inv_valid,
    input  wire [AWIDTH-5:0]   coh_snoop_inv_line,

    // External M-Mode Hardware Interrupt Lines (CLINT + PLIC)
    input  wire                irq_msip,
    input  wire                irq_mtip,
    input  wire                irq_meip,

    // 2-Wide RVFI Commit Trace Ports (strict in-order: commit0 older, commit1 younger)
    output wire                commit0_valid,
    output wire [31:0]         commit0_seq_id,
    output wire [31:0]         commit0_pc,
    output wire [31:0]         commit0_instr,
    output wire                commit0_rd_we,
    output wire                commit0_rd_is_fp,
    output wire [4:0]          commit0_rd_addr,
    output wire [31:0]         commit0_rd_wdata,
    output wire [4:0]          commit0_fflags,
    output wire                commit0_mem_valid,
    output wire                commit0_mem_we,
    output wire [31:0]         commit0_mem_addr,
    output wire [3:0]          commit0_mem_wstrb,
    output wire [31:0]         commit0_mem_wdata,

    output wire                commit1_valid,
    output wire [31:0]         commit1_seq_id,
    output wire [31:0]         commit1_pc,
    output wire [31:0]         commit1_instr,
    output wire                commit1_rd_we,
    output wire                commit1_rd_is_fp,
    output wire [4:0]          commit1_rd_addr,
    output wire [31:0]         commit1_rd_wdata,
    output wire [4:0]          commit1_fflags,
    output wire                commit1_mem_valid,
    output wire                commit1_mem_we,
    output wire [31:0]         commit1_mem_addr,
    output wire [3:0]          commit1_mem_wstrb,
    output wire [31:0]         commit1_mem_wdata,

    // 2-Wide Out-of-Order lifecycle trace hooks
    output wire                trace_disp0_valid,
    output wire [31:0]         trace_disp0_seq,
    output wire [31:0]         trace_disp0_pc,
    output wire [31:0]         trace_disp0_instr,
    output wire                trace_disp1_valid,
    output wire [31:0]         trace_disp1_seq,
    output wire [31:0]         trace_disp1_pc,
    output wire [31:0]         trace_disp1_instr,
    output wire [5:0]          trace_issue_valid,
    output wire [191:0]        trace_issue_seq,
    output wire [1:0]          trace_cdb_valid,
    output wire [63:0]         trace_cdb_seq,
    output wire                trace_flush_valid,
    output wire [31:0]         trace_flush_after_seq,

    // Status & PMU telemetry pulses
    output reg                 halted,
    output wire [31:0]         x14,
    output wire                stat_rob_full_stall,
    output wire                stat_rs_full_stall,
    output wire                stat_lsq_full_stall,
    output wire                stat_cp_full_stall,
    output wire                stat_stlf_hit,
    output wire [1:0]          stat_br_total_cnt,
    output wire [1:0]          stat_br_mispred_cnt,
    output wire                stat_br_fast_squash,
    output wire [1:0]          stat_war_waw_cnt,
    output wire                stat_div_overlap_cyc,
    output wire                stat_dcache_miss_overlap_cyc,
    output wire                stat_dual_disp,
    output wire                stat_dual_cdb,
    output wire                stat_dual_commit,
    output wire [3:0]          stat_ex_byp_cnt,
    output wire [1:0]          stat_ras_pred_cnt,
    output wire [1:0]          stat_ras_mispred_cnt,
    output wire                stat_replay_flush,
    output wire                stat_amo_commit,
    output wire                stat_fence_commit,
    output wire                stat_trap_taken
);

`include "rv32ima_defs.vh"
`include "fpu_defs.vh"

localparam integer ROB2W_DEPTH = 32;
localparam integer TAG_W       = 5;

// ============================================================================
// 1. 2-Wide Front-End: Tournament BP + 8-RAS, Stallable Fetch2W & Dual Decoder
// ============================================================================
wire        rob_squash;
wire [4:0]  rob_squash_tag;
wire [31:0] rob_squash_seq;
wire [31:0] rob_squash_redirect_pc;
wire        rob_squash_has_older;

wire        rob_full_flush;
wire        rob_replay_flush;
wire [31:0] rob_full_flush_pc;
wire [31:0] rob_full_flush_after_seq;

wire        rob_cdb0_br_ok;
wire [31:0] rob_cdb0_br_pc;
wire        rob_cdb0_br_is_cond;
wire        rob_cdb0_br_is_jalr;
wire [7:0]  rob_cdb0_br_ghr;
wire        rob_cdb0_br_pbim, rob_cdb0_br_pgsh;

wire        disp0_fire, disp1_fire;
wire [4:0]  rob_disp0_tag, rob_disp1_tag, rob_head_ptr;
wire        cp_take_l0, cp_take_l1;

wire        cdb0_valid, cdb1_valid;
wire [4:0]  cdb0_tag, cdb1_tag;
wire [31:0] cdb0_seq, cdb0_val, cdb1_seq, cdb1_val;
wire        cdb0_is_br, cdb0_br_taken;
wire [31:0] cdb0_br_target;
wire        cdb0_is_mem, cdb0_mem_we;
wire [31:0] cdb0_mem_addr, cdb0_mem_wdata;
wire [3:0]  cdb0_mem_wstrb;
wire        cdb0_exc_valid;
wire [3:0]  cdb0_exc_cause;
wire [31:0] cdb0_exc_tval;

// Dedicated 3rd CDB (FP CDB — zero contention with integer CDB0/CDB1!)
reg         cdb_fp_valid;
reg  [4:0]  cdb_fp_tag;
reg  [31:0] cdb_fp_seq;
reg  [31:0] cdb_fp_val;
reg  [4:0]  cdb_fp_fflags;

reg         alu0_out_v, alu1_out_v;
reg  [4:0]  alu0_out_tag, alu1_out_tag;
reg  [31:0] alu0_out_seq, alu0_out_val, alu1_out_seq, alu1_out_val;
wire        wk0_valid, wk1_valid;
wire [4:0]  wk0_tag, wk1_tag;

wire [1:0]  csr_mstatus_fs;
wire [2:0]  csr_frm;
wire [4:0]  csr_fflags_u;

wire [31:0] pred_req_pc0, pred_req_instr0, pred_req_pc1, pred_req_instr1;
wire        bp_pred_taken0, bp_pred_taken1, bp_pred_uras0, bp_pred_uras1;
wire        bp_pred_bim0, bp_pred_gsh0, bp_pred_bim1, bp_pred_gsh1;
wire [31:0] bp_pred_target0, bp_pred_target1;
wire [7:0]  bp_pred_ghr0, bp_pred_ghr1;
wire        fq_enq0_fire, fq_enq1_fire;

wire        fq0_valid, fq1_valid;
wire [31:0] fq0_pc, fq0_instr, fq0_ptgt, fq1_pc, fq1_instr, fq1_ptgt;
wire        fq0_ptaken, fq1_ptaken, fq0_puras, fq1_puras;
wire        fq0_pbim, fq0_pgsh, fq1_pbim, fq1_pgsh;
wire [7:0]  fq0_pghr, fq1_pghr;

wire        rob_c0_valid, rob_c1_valid;
wire        rob_c0_br_taken, rob_c1_br_taken;
wire        rob_c0_is_fence_i;

wire        fe_flush     = rob_full_flush || rob_squash;
wire [31:0] fe_flush_pc  = rob_full_flush ? rob_full_flush_pc : rob_squash_redirect_pc;

assign ic_flush          = fe_flush;
assign ic_invalidate_all = rob_c0_valid && rob_c0_is_fence_i;

btb_bht2w #(
    .BTB_ENTRIES(BTB_ENTRIES),
    .BHT_ENTRIES(256),
    .RAS_DEPTH(8),
    .TAG_W(TAG_W),
    .NUM_CP(4)
) u_btb_bht (
    .clk(clk), .reset_n(reset_n),
    .req_pc0(pred_req_pc0), .req_instr0(pred_req_instr0),
    .pred_taken0(bp_pred_taken0), .pred_target0(bp_pred_target0),
    .pred_ghr0(bp_pred_ghr0), .pred_used_ras0(bp_pred_uras0),
    .pred_bim0(bp_pred_bim0), .pred_gsh0(bp_pred_gsh0),
    .req_pc1(pred_req_pc1), .req_instr1(pred_req_instr1),
    .pred_taken1(bp_pred_taken1), .pred_target1(bp_pred_target1),
    .pred_ghr1(bp_pred_ghr1), .pred_used_ras1(bp_pred_uras1),
    .pred_bim1(bp_pred_bim1), .pred_gsh1(bp_pred_gsh1),
    .enq0_fire(fq_enq0_fire), .enq1_fire(fq_enq1_fire),
    .disp0_fire(disp0_fire), .disp0_pc(fq0_pc), .disp0_instr(fq0_instr),
    .disp1_fire(disp1_fire), .disp1_pc(fq1_pc), .disp1_instr(fq1_instr),
    .cp_take(cp_take_l0 || cp_take_l1),
    .cp_is_lane1(cp_take_l1),
    .cp_take_tag(cp_take_l1 ? rob_disp1_tag : rob_disp0_tag),
    .commit0_valid(rob_c0_valid), .commit0_pc(commit0_pc), .commit0_instr(commit0_instr), .commit0_br_taken(rob_c0_br_taken),
    .commit1_valid(rob_c1_valid), .commit1_pc(commit1_pc), .commit1_instr(commit1_instr), .commit1_br_taken(rob_c1_br_taken),
    .full_flush(rob_full_flush),
    .upd_valid(cdb0_valid && cdb0_is_br && !cdb0_exc_valid && !rob_full_flush),
    .upd_pc(rob_cdb0_br_pc), .upd_is_cond(rob_cdb0_br_is_cond), .upd_is_jalr(rob_cdb0_br_is_jalr),
    .upd_taken(cdb0_br_taken), .upd_target(cdb0_br_target), .upd_ghr(rob_cdb0_br_ghr),
    .upd_pred_bim(rob_cdb0_br_pbim), .upd_pred_gsh(rob_cdb0_br_pgsh),
    .br_resolve_ok(rob_cdb0_br_ok), .br_resolve_tag(cdb0_tag),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr)
);

fetch2w #(.RESET_PC(RESET_PC)) u_fetch (
    .clk(clk), .reset_n(reset_n), .halted(halted),
    .flush(fe_flush), .flush_target_pc(fe_flush_pc),
    .imem_req_valid(imem_req_valid),
    .imem_pc0(imem_pc0), .imem_valid0(imem_valid0), .imem_instr0(imem_instr0),
    .imem_pc1(imem_pc1), .imem_valid1(imem_valid1), .imem_instr1(imem_instr1),
    .pred_req_pc0(pred_req_pc0), .pred_req_instr0(pred_req_instr0),
    .pred_taken0(bp_pred_taken0), .pred_target0(bp_pred_target0),
    .pred_ghr0(bp_pred_ghr0), .pred_used_ras0(bp_pred_uras0),
    .pred_bim0(bp_pred_bim0), .pred_gsh0(bp_pred_gsh0),
    .pred_req_pc1(pred_req_pc1), .pred_req_instr1(pred_req_instr1),
    .pred_taken1(bp_pred_taken1), .pred_target1(bp_pred_target1),
    .pred_ghr1(bp_pred_ghr1), .pred_used_ras1(bp_pred_uras1),
    .pred_bim1(bp_pred_bim1), .pred_gsh1(bp_pred_gsh1),
    .enq0_fire(fq_enq0_fire), .enq1_fire(fq_enq1_fire),
    .out0_valid(fq0_valid), .out0_ready(disp0_fire), .out0_pc(fq0_pc), .out0_instr(fq0_instr),
    .out0_pred_taken(fq0_ptaken), .out0_pred_target(fq0_ptgt), .out0_pred_ghr(fq0_pghr),
    .out0_pred_used_ras(fq0_puras), .out0_pred_bim(fq0_pbim), .out0_pred_gsh(fq0_pgsh),
    .out1_valid(fq1_valid), .out1_ready(disp1_fire), .out1_pc(fq1_pc), .out1_instr(fq1_instr),
    .out1_pred_taken(fq1_ptaken), .out1_pred_target(fq1_ptgt), .out1_pred_ghr(fq1_pghr),
    .out1_pred_used_ras(fq1_puras), .out1_pred_bim(fq1_pbim), .out1_pred_gsh(fq1_pgsh)
);

// Lane 0 Decoder & ImmGen
wire [4:0]  d0_rs1, d0_rs2, d0_rs3, d0_rd;
wire        d0_uses_rs1, d0_uses_rs2, d0_uses_rs3;
wire        d0_rs1_fp, d0_rs2_fp, d0_rd_fp, d0_rd_we;
wire [2:0]  d0_imm_sel, d0_br_f3, d0_csr_op, d0_fp_rm;
wire [3:0]  d0_fu, d0_alu_sel, d0_amo_op, d0_fp_op;
wire        d0_asel, d0_bsel;
wire [1:0]  d0_mul_op, d0_div_op, d0_mem_sz;
wire        d0_is_br, d0_is_jal, d0_is_jalr, d0_mem_rd, d0_mem_wr, d0_mem_uns;
wire        d0_ebrk, d0_ecall, d0_mret, d0_ill, d0_is_fp;
wire        d0_is_fence, d0_is_fence_i, d0_is_csr, d0_csr_uimm;
wire [11:0] d0_csr_addr;
wire        d0_is_amo, d0_is_lr, d0_is_sc, d0_amo_aq, d0_amo_rl;
wire [31:0] d0_imm;

decoder u_dec0 (
    .instr(fq0_instr), .mstatus_fs(csr_mstatus_fs), .csr_frm(csr_frm),
    .rs1(d0_rs1), .rs2(d0_rs2), .rs3(d0_rs3), .rd(d0_rd),
    .uses_rs1(d0_uses_rs1), .uses_rs2(d0_uses_rs2), .uses_rs3(d0_uses_rs3),
    .rs1_is_fp(d0_rs1_fp), .rs2_is_fp(d0_rs2_fp), .rd_is_fp(d0_rd_fp), .rd_we(d0_rd_we),
    .imm_sel(d0_imm_sel), .fu_type(d0_fu), .alu_select(d0_alu_sel),
    .asel(d0_asel), .bsel(d0_bsel), .mul_op(d0_mul_op), .div_op(d0_div_op),
    .br_funct3(d0_br_f3), .is_branch(d0_is_br), .is_jal(d0_is_jal), .is_jalr(d0_is_jalr),
    .mem_read(d0_mem_rd), .mem_write(d0_mem_wr), .mem_size(d0_mem_sz), .mem_unsigned(d0_mem_uns),
    .is_ebreak(d0_ebrk), .is_ecall(d0_ecall), .is_mret(d0_mret),
    .is_fence(d0_is_fence), .is_fence_i(d0_is_fence_i),
    .is_csr(d0_is_csr), .csr_op(d0_csr_op), .csr_addr(d0_csr_addr), .csr_use_imm(d0_csr_uimm),
    .is_amo(d0_is_amo), .is_lr(d0_is_lr), .is_sc(d0_is_sc), .amo_op(d0_amo_op),
    .amo_aq(d0_amo_aq), .amo_rl(d0_amo_rl),
    .is_fp(d0_is_fp), .fp_op(d0_fp_op), .fp_rm(d0_fp_rm),
    .illegal(d0_ill)
);
imm_gen u_imm0 (.instr(fq0_instr), .imm_sel(d0_imm_sel), .imm_out(d0_imm));

// Lane 1 Decoder & ImmGen
wire [4:0]  d1_rs1, d1_rs2, d1_rs3, d1_rd;
wire        d1_uses_rs1, d1_uses_rs2, d1_uses_rs3;
wire        d1_rs1_fp, d1_rs2_fp, d1_rd_fp, d1_rd_we;
wire [2:0]  d1_imm_sel, d1_br_f3, d1_csr_op, d1_fp_rm;
wire [3:0]  d1_fu, d1_alu_sel, d1_amo_op, d1_fp_op;
wire        d1_asel, d1_bsel;
wire [1:0]  d1_mul_op, d1_div_op, d1_mem_sz;
wire        d1_is_br, d1_is_jal, d1_is_jalr, d1_mem_rd, d1_mem_wr, d1_mem_uns;
wire        d1_ebrk, d1_ecall, d1_mret, d1_ill, d1_is_fp;
wire        d1_is_fence, d1_is_fence_i, d1_is_csr, d1_csr_uimm;
wire [11:0] d1_csr_addr;
wire        d1_is_amo, d1_is_lr, d1_is_sc, d1_amo_aq, d1_amo_rl;
wire [31:0] d1_imm;

decoder u_dec1 (
    .instr(fq1_instr), .mstatus_fs(csr_mstatus_fs), .csr_frm(csr_frm),
    .rs1(d1_rs1), .rs2(d1_rs2), .rs3(d1_rs3), .rd(d1_rd),
    .uses_rs1(d1_uses_rs1), .uses_rs2(d1_uses_rs2), .uses_rs3(d1_uses_rs3),
    .rs1_is_fp(d1_rs1_fp), .rs2_is_fp(d1_rs2_fp), .rd_is_fp(d1_rd_fp), .rd_we(d1_rd_we),
    .imm_sel(d1_imm_sel), .fu_type(d1_fu), .alu_select(d1_alu_sel),
    .asel(d1_asel), .bsel(d1_bsel), .mul_op(d1_mul_op), .div_op(d1_div_op),
    .br_funct3(d1_br_f3), .is_branch(d1_is_br), .is_jal(d1_is_jal), .is_jalr(d1_is_jalr),
    .mem_read(d1_mem_rd), .mem_write(d1_mem_wr), .mem_size(d1_mem_sz), .mem_unsigned(d1_mem_uns),
    .is_ebreak(d1_ebrk), .is_ecall(d1_ecall), .is_mret(d1_mret),
    .is_fence(d1_is_fence), .is_fence_i(d1_is_fence_i),
    .is_csr(d1_is_csr), .csr_op(d1_csr_op), .csr_addr(d1_csr_addr), .csr_use_imm(d1_csr_uimm),
    .is_amo(d1_is_amo), .is_lr(d1_is_lr), .is_sc(d1_is_sc), .amo_op(d1_amo_op),
    .amo_aq(d1_amo_aq), .amo_rl(d1_amo_rl),
    .is_fp(d1_is_fp), .fp_op(d1_fp_op), .fp_rm(d1_fp_rm),
    .illegal(d1_ill)
);
imm_gen u_imm1 (.instr(fq1_instr), .imm_sel(d1_imm_sel), .imm_out(d1_imm));

// ============================================================================
// 2. INT + FP Register Files, Dual Coherent-CP RATs, 32-Entry ROB & CSR Unit
// ============================================================================
wire [31:0] arf_l0_rs1, arf_l0_rs2, arf_l1_rs1, arf_l1_rs2;
wire [31:0] arf_head_rs1, arf_head_rs2;
wire [31:0] farf_l0_rs1, farf_l0_rs2, farf_l0_rs3, farf_l1_rs1, farf_l1_rs2, farf_l1_rs3;

wire [4:0]  rob_c0_tag, rob_c1_tag;
wire [3:0]  rob_c0_fu, rob_c1_fu;
wire        rob_c0_rd_we, rob_c1_rd_we;
wire        rob_c0_rd_fp, rob_c1_rd_fp;
wire [4:0]  rob_c0_rd_addr, rob_c1_rd_addr;
wire [31:0] rob_c0_rd_wdata, rob_c1_rd_wdata;
wire [4:0]  rob_c0_fflags, rob_c1_fflags;
wire        rob_c0_ebrk, rob_c0_is_br, rob_c0_mispred, rob_c0_uras;
wire        rob_c1_is_br, rob_c1_mispred, rob_c1_uras;
wire        rob_free_ge1, rob_free_ge2, rob_full, rob_empty;

wire        rob_head_valid, rob_head_ready;
wire [4:0]  rob_head_tag;
wire [31:0] rob_head_seq, rob_head_pc, rob_head_instr;
wire        rob_head_is_csr, rob_head_is_fence, rob_head_is_fence_i, rob_head_is_amo;
wire        rob_barrier_valid;
wire [4:0]  rob_barrier_tag;
wire        lsq_head0_replay, lsq_head1_replay;

// Decode ROB-head instruction for non-speculative CSR & RV32A AMO execution
wire [4:0]  hd_rs1, hd_rs2, hd_rs3_u, hd_rd_u;
wire        hd_u0, hd_u1, hd_u2, hd_u3, hd_u4, hd_u5, hd_u6, hd_u7, hd_u8, hd_u9;
wire        hd_u10, hd_u11, hd_u12, hd_u13, hd_u14, hd_u15, hd_u16, hd_u17, hd_u18, hd_u19;
wire        hd_u20, hd_u21, hd_u22, hd_u23, hd_u24, hd_u25, hd_u26, hd_u27, hd_u28;
wire [2:0]  hd_imm_u, hd_br_u, hd_csr_op, hd_fprm_u;
wire [3:0]  hd_fu_u, hd_alu_u, hd_amo_op, hd_fpop_u;
wire [1:0]  hd_mul_u, hd_div_u, hd_msz_u;
wire [11:0] hd_csr_addr;

decoder u_dec_head (
    .instr(rob_head_instr), .mstatus_fs(csr_mstatus_fs), .csr_frm(csr_frm),
    .rs1(hd_rs1), .rs2(hd_rs2), .rs3(hd_rs3_u), .rd(hd_rd_u),
    .uses_rs1(hd_u0), .uses_rs2(hd_u1), .uses_rs3(hd_u24),
    .rs1_is_fp(hd_u25), .rs2_is_fp(hd_u26), .rd_is_fp(hd_u27), .rd_we(hd_u2),
    .imm_sel(hd_imm_u), .fu_type(hd_fu_u), .alu_select(hd_alu_u),
    .asel(hd_u3), .bsel(hd_u4), .mul_op(hd_mul_u), .div_op(hd_div_u),
    .br_funct3(hd_br_u), .is_branch(hd_u5), .is_jal(hd_u6), .is_jalr(hd_u7),
    .mem_read(hd_u8), .mem_write(hd_u9), .mem_size(hd_msz_u), .mem_unsigned(hd_u10),
    .is_ebreak(hd_u11), .is_ecall(hd_u22), .is_mret(hd_u23),
    .is_fence(hd_u12), .is_fence_i(hd_u13),
    .is_csr(hd_u14), .csr_op(hd_csr_op), .csr_addr(hd_csr_addr), .csr_use_imm(hd_u15),
    .is_amo(hd_u16), .is_lr(hd_u17), .is_sc(hd_u18), .amo_op(hd_amo_op),
    .amo_aq(hd_u19), .amo_rl(hd_u20),
    .is_fp(hd_u28), .fp_op(hd_fpop_u), .fp_rm(hd_fprm_u),
    .illegal(hd_u21)
);

regfile2w u_arf (
    .clk(clk), .reset_n(reset_n),
    .we0(rob_c0_valid && rob_c0_rd_we && !rob_c0_rd_fp), .waddr0(rob_c0_rd_addr), .wdata0(rob_c0_rd_wdata),
    .we1(rob_c1_valid && rob_c1_rd_we && !rob_c1_rd_fp), .waddr1(rob_c1_rd_addr), .wdata1(rob_c1_rd_wdata),
    .raddr0(d0_rs1), .rdata0(arf_l0_rs1),
    .raddr1(d0_rs2), .rdata1(arf_l0_rs2),
    .raddr2(d1_rs1), .rdata2(arf_l1_rs1),
    .raddr3(d1_rs2), .rdata3(arf_l1_rs2),
    .raddr4(hd_rs1), .rdata4(arf_head_rs1),
    .raddr5(hd_rs2), .rdata5(arf_head_rs2),
    .x14(x14)
);

fregfile2w u_farf (
    .clk(clk), .reset_n(reset_n),
    .we0(rob_c0_valid && rob_c0_rd_we && rob_c0_rd_fp), .waddr0(rob_c0_rd_addr), .wdata0(rob_c0_rd_wdata),
    .we1(rob_c1_valid && rob_c1_rd_we && rob_c1_rd_fp), .waddr1(rob_c1_rd_addr), .wdata1(rob_c1_rd_wdata),
    .raddr0(d0_rs1), .rdata0(farf_l0_rs1),
    .raddr1(d0_rs2), .rdata1(farf_l0_rs2),
    .raddr2(d0_rs3), .rdata2(farf_l0_rs3),
    .raddr3(d1_rs1), .rdata3(farf_l1_rs1),
    .raddr4(d1_rs2), .rdata4(farf_l1_rs2),
    .raddr5(d1_rs3), .rdata5(farf_l1_rs3)
);

// Non-speculative M-Mode Zicsr, FCSR & Trap Control Unit
wire        cdb_p1_grant;
wire        csr_exec_req  = rob_head_valid && !rob_head_ready && rob_head_is_csr && !rob_full_flush;
wire        csr_exec_fire = csr_exec_req && cdb_p1_grant;
wire [31:0] csr_rdata;
wire        csr_access_illegal, csr_illegal_u, csr_sync_req;
wire [31:0] csr_mtvec_val, csr_mepc_val;
wire        csr_irq_pending;
wire [3:0]  csr_irq_cause;
wire        rob_trap_enter, rob_trap_is_irq, rob_mret_exec;
wire [3:0]  rob_trap_cause;
wire [31:0] rob_trap_pc, rob_trap_tval;
wire [63:0] mcycle_u, minstret_u;
wire [1:0]  retire_cnt = {1'b0, rob_c0_valid} + {1'b0, rob_c1_valid};

wire [4:0]  fp_commit_fflags = (rob_c0_valid ? rob_c0_fflags : 5'd0) |
                               (rob_c1_valid ? rob_c1_fflags : 5'd0);
wire        fp_commit_dirty  = (rob_c0_valid && rob_c0_rd_we && rob_c0_rd_fp) ||
                               (rob_c1_valid && rob_c1_rd_we && rob_c1_rd_fp) ||
                               (fp_commit_fflags != 5'd0);

csr_unit #(.CORE_ID(CORE_ID), .MISA_EXT_F(1'b1)) u_csr (
    .clk(clk), .reset_n(reset_n), .halted(halted),
    .retire_cnt(retire_cnt),
    .fp_commit_dirty(fp_commit_dirty),
    .fp_commit_fflags(fp_commit_fflags),
    .csr_req_valid(csr_exec_fire),
    .csr_op(hd_csr_op),
    .csr_addr(hd_csr_addr),
    .csr_rs1_uimm(hd_rs1),
    .csr_rs1_val(arf_head_rs1),
    .csr_rdata(csr_rdata),
    .csr_access_illegal(csr_access_illegal),
    .csr_illegal(csr_illegal_u),
    .csr_sync_required(csr_sync_req),
    .irq_msip(irq_msip),
    .irq_mtip(irq_mtip),
    .irq_meip(irq_meip),
    .trap_enter(rob_trap_enter),
    .trap_is_irq(rob_trap_is_irq),
    .trap_cause(rob_trap_cause),
    .trap_pc(rob_trap_pc),
    .trap_tval(rob_trap_tval),
    .mret_exec(rob_mret_exec),
    .mtvec_out(csr_mtvec_val),
    .mepc_out(csr_mepc_val),
    .irq_pending(csr_irq_pending),
    .irq_cause(csr_irq_cause),
    .mstatus_fs_out(csr_mstatus_fs),
    .frm_out(csr_frm),
    .fflags_out(csr_fflags_u),
    .mcycle_out(mcycle_u),
    .minstret_out(minstret_u)
);

// FENCE.I Dirty D-Cache Clean-All Handshake
wire lsq_sq_empty, lsq_mmio_busy;
wire sq_and_dc_drained = lsq_sq_empty && dc_writes_idle;

reg  fence_i_cleaned;
reg  fence_i_clean_sent;
assign dc_clean_all_req = rob_head_valid && rob_head_is_fence_i &&
                          sq_and_dc_drained && !fence_i_cleaned && !fence_i_clean_sent;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        fence_i_cleaned    <= 1'b0;
        fence_i_clean_sent <= 1'b0;
    end else if (rob_full_flush || !rob_head_is_fence_i) begin
        fence_i_cleaned    <= 1'b0;
        fence_i_clean_sent <= 1'b0;
    end else begin
        if (dc_clean_all_req)
            fence_i_clean_sent <= 1'b1;
        if (dc_clean_all_done)
            fence_i_cleaned <= 1'b1;
    end
end

// Integer RAT + Synchronized FP RAT (both take identical branch checkpoints!)
wire       rat_l0_rs1_b, rat_l0_rs2_b, rat_l1_rs1_b, rat_l1_rs2_b;
wire [4:0] rat_l0_rs1_t, rat_l0_rs2_t, rat_l1_rs1_t, rat_l1_rs2_t;
wire       rat_d0_was_b, rat_d1_was_b, int_cp_ready;

wire       frat_l0_rs1_b, frat_l0_rs2_b, frat_l0_rs3_b, frat_l1_rs1_b, frat_l1_rs2_b, frat_l1_rs3_b;
wire [4:0] frat_l0_rs1_t, frat_l0_rs2_t, frat_l0_rs3_t, frat_l1_rs1_t, frat_l1_rs2_t, frat_l1_rs3_t;
wire       frat_d0_was_b, frat_d1_was_b, fp_cp_ready;
wire       rat_cp_ready = int_cp_ready && fp_cp_ready;

assign cp_take_l0 = disp0_fire && (d0_fu == FU_BR);
assign cp_take_l1 = disp1_fire && (d1_fu == FU_BR);

rat2w #(.TAG_W(TAG_W), .NUM_CP(4)) u_rat (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush), .rob_head(rob_head_ptr),
    .l0_rs1_addr(d0_rs1), .l0_rs1_busy(rat_l0_rs1_b), .l0_rs1_tag(rat_l0_rs1_t),
    .l0_rs2_addr(d0_rs2), .l0_rs2_busy(rat_l0_rs2_b), .l0_rs2_tag(rat_l0_rs2_t),
    .l1_rs1_addr(d1_rs1), .l1_rs1_busy(rat_l1_rs1_b), .l1_rs1_tag(rat_l1_rs1_t),
    .l1_rs2_addr(d1_rs2), .l1_rs2_busy(rat_l1_rs2_b), .l1_rs2_tag(rat_l1_rs2_t),
    .disp0_we(disp0_fire && d0_rd_we && !d0_rd_fp), .disp0_rd(d0_rd), .disp0_tag(rob_disp0_tag), .disp0_rd_was_busy(rat_d0_was_b),
    .disp1_we(disp1_fire && d1_rd_we && !d1_rd_fp), .disp1_rd(d1_rd), .disp1_tag(rob_disp1_tag), .disp1_rd_was_busy(rat_d1_was_b),
    .commit0_we(rob_c0_valid && rob_c0_rd_we && !rob_c0_rd_fp), .commit0_rd(rob_c0_rd_addr), .commit0_tag(rob_c0_tag),
    .commit1_we(rob_c1_valid && rob_c1_rd_we && !rob_c1_rd_fp), .commit1_rd(rob_c1_rd_addr), .commit1_tag(rob_c1_tag),
    .cp_take(cp_take_l0 || cp_take_l1), .cp_is_lane1(cp_take_l1),
    .cp_take_tag(cp_take_l1 ? rob_disp1_tag : rob_disp0_tag),
    .cp_ready(int_cp_ready),
    .br_resolve_ok(rob_cdb0_br_ok), .br_resolve_tag(cdb0_tag),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag)
);

frat2w #(.TAG_W(TAG_W), .NUM_CP(4)) u_frat (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush), .rob_head(rob_head_ptr),
    .l0_rs1_addr(d0_rs1), .l0_rs1_busy(frat_l0_rs1_b), .l0_rs1_tag(frat_l0_rs1_t),
    .l0_rs2_addr(d0_rs2), .l0_rs2_busy(frat_l0_rs2_b), .l0_rs2_tag(frat_l0_rs2_t),
    .l0_rs3_addr(d0_rs3), .l0_rs3_busy(frat_l0_rs3_b), .l0_rs3_tag(frat_l0_rs3_t),
    .l1_rs1_addr(d1_rs1), .l1_rs1_busy(frat_l1_rs1_b), .l1_rs1_tag(frat_l1_rs1_t),
    .l1_rs2_addr(d1_rs2), .l1_rs2_busy(frat_l1_rs2_b), .l1_rs2_tag(frat_l1_rs2_t),
    .l1_rs3_addr(d1_rs3), .l1_rs3_busy(frat_l1_rs3_b), .l1_rs3_tag(frat_l1_rs3_t),
    .disp0_we(disp0_fire && d0_rd_we && d0_rd_fp), .disp0_rd(d0_rd), .disp0_tag(rob_disp0_tag), .disp0_rd_was_busy(frat_d0_was_b),
    .disp1_we(disp1_fire && d1_rd_we && d1_rd_fp), .disp1_rd(d1_rd), .disp1_tag(rob_disp1_tag), .disp1_rd_was_busy(frat_d1_was_b),
    .commit0_we(rob_c0_valid && rob_c0_rd_we && rob_c0_rd_fp), .commit0_rd(rob_c0_rd_addr), .commit0_tag(rob_c0_tag),
    .commit1_we(rob_c1_valid && rob_c1_rd_we && rob_c1_rd_fp), .commit1_rd(rob_c1_rd_addr), .commit1_tag(rob_c1_tag),
    .cp_take(cp_take_l0 || cp_take_l1), .cp_is_lane1(cp_take_l1),
    .cp_take_tag(cp_take_l1 ? rob_disp1_tag : rob_disp0_tag),
    .cp_ready(fp_cp_ready),
    .br_resolve_ok(rob_cdb0_br_ok), .br_resolve_tag(cdb0_tag),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag)
);

// Monotonic instruction sequence IDs (never rewind on squash/replay)
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

// Mux INT vs FP RAT tags for unified 6-port ROB operand lookup
wire       l0_rs1_busy = d0_rs1_fp ? frat_l0_rs1_b : rat_l0_rs1_b;
wire [4:0] l0_rs1_tag  = d0_rs1_fp ? frat_l0_rs1_t : rat_l0_rs1_t;
wire [31:0] l0_rs1_arf = d0_rs1_fp ? farf_l0_rs1   : arf_l0_rs1;

wire       l0_rs2_busy = d0_rs2_fp ? frat_l0_rs2_b : rat_l0_rs2_b;
wire [4:0] l0_rs2_tag  = d0_rs2_fp ? frat_l0_rs2_t : rat_l0_rs2_t;
wire [31:0] l0_rs2_arf = d0_rs2_fp ? farf_l0_rs2   : arf_l0_rs2;

wire       l1_rs1_busy = d1_rs1_fp ? frat_l1_rs1_b : rat_l1_rs1_b;
wire [4:0] l1_rs1_tag  = d1_rs1_fp ? frat_l1_rs1_t : rat_l1_rs1_t;
wire [31:0] l1_rs1_arf = d1_rs1_fp ? farf_l1_rs1   : arf_l1_rs1;

wire       l1_rs2_busy = d1_rs2_fp ? frat_l1_rs2_b : rat_l1_rs2_b;
wire [4:0] l1_rs2_tag  = d1_rs2_fp ? frat_l1_rs2_t : rat_l1_rs2_t;
wire [31:0] l1_rs2_arf = d1_rs2_fp ? farf_l1_rs2   : arf_l1_rs2;

wire        rob_rd0_rdy, rob_rd1_rdy, rob_rd2_rdy, rob_rd3_rdy, rob_rd4_rdy, rob_rd5_rdy;
wire [31:0] rob_rd0_val, rob_rd1_val, rob_rd2_val, rob_rd3_val, rob_rd4_val, rob_rd5_val;

wire d0_ready_init = d0_ebrk || d0_ecall || d0_mret || d0_ill || d0_is_fence || d0_is_fence_i;
wire d1_ready_init = d1_ebrk || d1_ecall || d1_mret || d1_ill || d1_is_fence || d1_is_fence_i;

rob2w #(.DEPTH(ROB2W_DEPTH), .TAG_W(TAG_W)) u_rob (
    .clk(clk), .reset_n(reset_n),
    .disp0_valid(disp0_fire), .disp1_valid(disp1_fire),
    .disp_free_ge1(rob_free_ge1), .disp_free_ge2(rob_free_ge2),
    .disp0_tag(rob_disp0_tag), .disp1_tag(rob_disp1_tag),
    .disp0_seq_id(seq0_id), .disp0_pc(fq0_pc), .disp0_instr(fq0_instr),
    .disp0_fu_type(d0_fu), .disp0_rd_we(d0_rd_we), .disp0_rd_is_fp(d0_rd_fp), .disp0_rd_addr(d0_rd),
    .disp0_pred_taken(fq0_ptaken), .disp0_pred_target(fq0_ptgt),
    .disp0_pred_ghr(fq0_pghr), .disp0_pred_used_ras(fq0_puras),
    .disp0_pred_bim(fq0_pbim), .disp0_pred_gsh(fq0_pgsh),
    .disp0_is_ebreak(d0_ebrk), .disp0_is_ecall(d0_ecall), .disp0_is_mret(d0_mret), .disp0_illegal(d0_ill),
    .disp0_is_fence(d0_is_fence), .disp0_is_fence_i(d0_is_fence_i),
    .disp0_is_csr(d0_is_csr), .disp0_is_amo(d0_is_amo), .disp0_ready_init(d0_ready_init),
    .disp1_seq_id(seq1_id), .disp1_pc(fq1_pc), .disp1_instr(fq1_instr),
    .disp1_fu_type(d1_fu), .disp1_rd_we(d1_rd_we), .disp1_rd_is_fp(d1_rd_fp), .disp1_rd_addr(d1_rd),
    .disp1_pred_taken(fq1_ptaken), .disp1_pred_target(fq1_ptgt),
    .disp1_pred_ghr(fq1_pghr), .disp1_pred_used_ras(fq1_puras),
    .disp1_pred_bim(fq1_pbim), .disp1_pred_gsh(fq1_pgsh),
    .disp1_is_ebreak(d1_ebrk), .disp1_is_ecall(d1_ecall), .disp1_is_mret(d1_mret), .disp1_illegal(d1_ill),
    .disp1_is_fence(d1_is_fence), .disp1_is_fence_i(d1_is_fence_i),
    .disp1_is_csr(d1_is_csr), .disp1_is_amo(d1_is_amo), .disp1_ready_init(d1_ready_init),
    .rd0_tag(l0_rs1_tag),    .rd0_ready(rob_rd0_rdy), .rd0_value(rob_rd0_val),
    .rd1_tag(l0_rs2_tag),    .rd1_ready(rob_rd1_rdy), .rd1_value(rob_rd1_val),
    .rd2_tag(l1_rs1_tag),    .rd2_ready(rob_rd2_rdy), .rd2_value(rob_rd2_val),
    .rd3_tag(l1_rs2_tag),    .rd3_ready(rob_rd3_rdy), .rd3_value(rob_rd3_val),
    .rd4_tag(frat_l0_rs3_t), .rd4_ready(rob_rd4_rdy), .rd4_value(rob_rd4_val),
    .rd5_tag(frat_l1_rs3_t), .rd5_ready(rob_rd5_rdy), .rd5_value(rob_rd5_val),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb0_is_br(cdb0_is_br), .cdb0_br_taken(cdb0_br_taken), .cdb0_br_target(cdb0_br_target),
    .cdb0_is_mem(cdb0_is_mem && !csr_exec_fire), .cdb0_mem_we(cdb0_mem_we),
    .cdb0_mem_addr(cdb0_mem_addr), .cdb0_mem_wstrb(cdb0_mem_wstrb), .cdb0_mem_wdata(cdb0_mem_wdata),
    .cdb0_exc_valid(cdb0_exc_valid), .cdb0_exc_cause(cdb0_exc_cause), .cdb0_exc_tval(cdb0_exc_tval),
    .cdb0_csr_sync(csr_exec_fire && csr_sync_req),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .cdb2_valid(cdb_fp_valid), .cdb2_tag(cdb_fp_tag), .cdb2_val(cdb_fp_val), .cdb2_fflags(cdb_fp_fflags),
    .halted_in(halted), .lsq_sq_empty(sq_and_dc_drained), .lsq_mmio_busy(lsq_mmio_busy),
    .fence_i_ready(fence_i_cleaned),
    .lsq_head0_replay(lsq_head0_replay), .lsq_head1_replay(lsq_head1_replay),
    .irq_pending(csr_irq_pending), .irq_cause(csr_irq_cause),
    .csr_mtvec_in(csr_mtvec_val), .csr_mepc_in(csr_mepc_val),
    .head_valid(rob_head_valid), .head_ready(rob_head_ready), .head_tag(rob_head_tag),
    .head_seq_id(rob_head_seq), .head_pc(rob_head_pc), .head_instr(rob_head_instr),
    .head_is_csr(rob_head_is_csr), .head_is_fence(rob_head_is_fence),
    .head_is_fence_i(rob_head_is_fence_i), .head_is_amo(rob_head_is_amo),
    .barrier_valid(rob_barrier_valid), .barrier_tag(rob_barrier_tag),
    .commit0_valid(rob_c0_valid), .commit0_tag(rob_c0_tag), .commit0_seq_id(commit0_seq_id),
    .commit0_pc(commit0_pc), .commit0_instr(commit0_instr), .commit0_fu_type(rob_c0_fu),
    .commit0_rd_we(rob_c0_rd_we), .commit0_rd_is_fp(rob_c0_rd_fp),
    .commit0_rd_addr(rob_c0_rd_addr), .commit0_rd_wdata(rob_c0_rd_wdata), .commit0_fflags(rob_c0_fflags),
    .commit0_mem_valid(commit0_mem_valid), .commit0_mem_we(commit0_mem_we),
    .commit0_mem_addr(commit0_mem_addr), .commit0_mem_wstrb(commit0_mem_wstrb), .commit0_mem_wdata(commit0_mem_wdata),
    .commit0_is_ebreak(rob_c0_ebrk), .commit0_is_fence_i(rob_c0_is_fence_i),
    .commit0_is_br(rob_c0_is_br), .commit0_br_taken(rob_c0_br_taken),
    .commit0_mispred(rob_c0_mispred), .commit0_used_ras(rob_c0_uras),
    .commit1_valid(rob_c1_valid), .commit1_tag(rob_c1_tag), .commit1_seq_id(commit1_seq_id),
    .commit1_pc(commit1_pc), .commit1_instr(commit1_instr), .commit1_fu_type(rob_c1_fu),
    .commit1_rd_we(rob_c1_rd_we), .commit1_rd_is_fp(rob_c1_rd_fp),
    .commit1_rd_addr(rob_c1_rd_addr), .commit1_rd_wdata(rob_c1_rd_wdata), .commit1_fflags(rob_c1_fflags),
    .commit1_mem_valid(commit1_mem_valid), .commit1_mem_we(commit1_mem_we),
    .commit1_mem_addr(commit1_mem_addr), .commit1_mem_wstrb(commit1_mem_wstrb), .commit1_mem_wdata(commit1_mem_wdata),
    .commit1_is_br(rob_c1_is_br), .commit1_br_taken(rob_c1_br_taken),
    .commit1_mispred(rob_c1_mispred), .commit1_used_ras(rob_c1_uras),
    .squash_out(rob_squash), .squash_tag_out(rob_squash_tag), .squash_seq_out(rob_squash_seq),
    .squash_redirect_pc(rob_squash_redirect_pc), .squash_has_older(rob_squash_has_older),
    .full_flush_out(rob_full_flush), .replay_flush_out(rob_replay_flush),
    .full_flush_redirect_pc(rob_full_flush_pc), .full_flush_after_seq(rob_full_flush_after_seq),
    .trap_enter_out(rob_trap_enter), .trap_is_irq_out(rob_trap_is_irq),
    .trap_cause_out(rob_trap_cause), .trap_pc_out(rob_trap_pc), .trap_tval_out(rob_trap_tval),
    .mret_exec_out(rob_mret_exec),
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
// 3. 2-Wide INT + FP Operand Resolution (6 Sources + Triple-CDB Bypass!)
// ============================================================================
reg [31:0] d0_vj, d0_vk, d0_vm, d1_vj, d1_vk, d1_vm;
reg [4:0]  d0_qj, d0_qk, d0_qm, d1_qj, d1_qk, d1_qm;
reg        d0_qj_v, d0_qk_v, d0_qm_v, d1_qj_v, d1_qk_v, d1_qm_v;

wire l0_rs1_x0 = !d0_rs1_fp && (d0_rs1 == 5'd0);
wire l0_rs2_x0 = !d0_rs2_fp && (d0_rs2 == 5'd0);
wire l1_rs1_x0 = !d1_rs1_fp && (d1_rs1 == 5'd0);
wire l1_rs2_x0 = !d1_rs2_fp && (d1_rs2 == 5'd0);

wire l1_rs1_from_l0 = d0_rd_we && (d0_rd_fp == d1_rs1_fp) && (d0_rd_fp || (d0_rd != 5'd0)) && (d0_rd == d1_rs1);
wire l1_rs2_from_l0 = d0_rd_we && (d0_rd_fp == d1_rs2_fp) && (d0_rd_fp || (d0_rd != 5'd0)) && (d0_rd == d1_rs2);
wire l1_rs3_from_l0 = d0_rd_we && d0_rd_fp && (d0_rd == d1_rs3);

always @(*) begin
    // Lane 0 rs1 (INT or FP)
    if (!d0_uses_rs1 || l0_rs1_x0) begin
        d0_vj = 32'd0; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (!l0_rs1_busy) begin
        d0_vj = l0_rs1_arf; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (rob_rd0_rdy) begin
        d0_vj = rob_rd0_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (alu0_out_v && (alu0_out_tag == l0_rs1_tag)) begin
        d0_vj = alu0_out_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (alu1_out_v && (alu1_out_tag == l0_rs1_tag)) begin
        d0_vj = alu1_out_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (cdb0_valid && (cdb0_tag == l0_rs1_tag)) begin
        d0_vj = cdb0_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (cdb1_valid && (cdb1_tag == l0_rs1_tag)) begin
        d0_vj = cdb1_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else if (cdb_fp_valid && (cdb_fp_tag == l0_rs1_tag)) begin
        d0_vj = cdb_fp_val; d0_qj = 5'd0; d0_qj_v = 1'b0;
    end else begin
        d0_vj = 32'd0; d0_qj = l0_rs1_tag; d0_qj_v = 1'b1;
    end

    // Lane 0 rs2 (INT or FP)
    if (!d0_uses_rs2 || l0_rs2_x0) begin
        d0_vk = 32'd0; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (!l0_rs2_busy) begin
        d0_vk = l0_rs2_arf; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (rob_rd1_rdy) begin
        d0_vk = rob_rd1_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (alu0_out_v && (alu0_out_tag == l0_rs2_tag)) begin
        d0_vk = alu0_out_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (alu1_out_v && (alu1_out_tag == l0_rs2_tag)) begin
        d0_vk = alu1_out_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (cdb0_valid && (cdb0_tag == l0_rs2_tag)) begin
        d0_vk = cdb0_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (cdb1_valid && (cdb1_tag == l0_rs2_tag)) begin
        d0_vk = cdb1_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else if (cdb_fp_valid && (cdb_fp_tag == l0_rs2_tag)) begin
        d0_vk = cdb_fp_val; d0_qk = 5'd0; d0_qk_v = 1'b0;
    end else begin
        d0_vk = 32'd0; d0_qk = l0_rs2_tag; d0_qk_v = 1'b1;
    end

    // Lane 0 rs3 (always FP FMA addend)
    if (!d0_uses_rs3) begin
        d0_vm = 32'd0; d0_qm = 5'd0; d0_qm_v = 1'b0;
    end else if (!frat_l0_rs3_b) begin
        d0_vm = farf_l0_rs3; d0_qm = 5'd0; d0_qm_v = 1'b0;
    end else if (rob_rd4_rdy) begin
        d0_vm = rob_rd4_val; d0_qm = 5'd0; d0_qm_v = 1'b0;
    end else if (cdb0_valid && (cdb0_tag == frat_l0_rs3_t)) begin
        d0_vm = cdb0_val; d0_qm = 5'd0; d0_qm_v = 1'b0;
    end else if (cdb1_valid && (cdb1_tag == frat_l0_rs3_t)) begin
        d0_vm = cdb1_val; d0_qm = 5'd0; d0_qm_v = 1'b0;
    end else if (cdb_fp_valid && (cdb_fp_tag == frat_l0_rs3_t)) begin
        d0_vm = cdb_fp_val; d0_qm = 5'd0; d0_qm_v = 1'b0;
    end else begin
        d0_vm = 32'd0; d0_qm = frat_l0_rs3_t; d0_qm_v = 1'b1;
    end

    // Lane 1 rs1 (INT or FP)
    if (!d1_uses_rs1 || l1_rs1_x0) begin
        d1_vj = 32'd0; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (l1_rs1_from_l0) begin
        d1_vj = 32'd0; d1_qj = rob_disp0_tag; d1_qj_v = 1'b1;
    end else if (!l1_rs1_busy) begin
        d1_vj = l1_rs1_arf; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (rob_rd2_rdy) begin
        d1_vj = rob_rd2_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (alu0_out_v && (alu0_out_tag == l1_rs1_tag)) begin
        d1_vj = alu0_out_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (alu1_out_v && (alu1_out_tag == l1_rs1_tag)) begin
        d1_vj = alu1_out_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (cdb0_valid && (cdb0_tag == l1_rs1_tag)) begin
        d1_vj = cdb0_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (cdb1_valid && (cdb1_tag == l1_rs1_tag)) begin
        d1_vj = cdb1_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else if (cdb_fp_valid && (cdb_fp_tag == l1_rs1_tag)) begin
        d1_vj = cdb_fp_val; d1_qj = 5'd0; d1_qj_v = 1'b0;
    end else begin
        d1_vj = 32'd0; d1_qj = l1_rs1_tag; d1_qj_v = 1'b1;
    end

    // Lane 1 rs2 (INT or FP)
    if (!d1_uses_rs2 || l1_rs2_x0) begin
        d1_vk = 32'd0; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (l1_rs2_from_l0) begin
        d1_vk = 32'd0; d1_qk = rob_disp0_tag; d1_qk_v = 1'b1;
    end else if (!l1_rs2_busy) begin
        d1_vk = l1_rs2_arf; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (rob_rd3_rdy) begin
        d1_vk = rob_rd3_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (alu0_out_v && (alu0_out_tag == l1_rs2_tag)) begin
        d1_vk = alu0_out_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (alu1_out_v && (alu1_out_tag == l1_rs2_tag)) begin
        d1_vk = alu1_out_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (cdb0_valid && (cdb0_tag == l1_rs2_tag)) begin
        d1_vk = cdb0_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (cdb1_valid && (cdb1_tag == l1_rs2_tag)) begin
        d1_vk = cdb1_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else if (cdb_fp_valid && (cdb_fp_tag == l1_rs2_tag)) begin
        d1_vk = cdb_fp_val; d1_qk = 5'd0; d1_qk_v = 1'b0;
    end else begin
        d1_vk = 32'd0; d1_qk = l1_rs2_tag; d1_qk_v = 1'b1;
    end

    // Lane 1 rs3 (always FP FMA addend)
    if (!d1_uses_rs3) begin
        d1_vm = 32'd0; d1_qm = 5'd0; d1_qm_v = 1'b0;
    end else if (l1_rs3_from_l0) begin
        d1_vm = 32'd0; d1_qm = rob_disp0_tag; d1_qm_v = 1'b1;
    end else if (!frat_l1_rs3_b) begin
        d1_vm = farf_l1_rs3; d1_qm = 5'd0; d1_qm_v = 1'b0;
    end else if (rob_rd5_rdy) begin
        d1_vm = rob_rd5_val; d1_qm = 5'd0; d1_qm_v = 1'b0;
    end else if (cdb0_valid && (cdb0_tag == frat_l1_rs3_t)) begin
        d1_vm = cdb0_val; d1_qm = 5'd0; d1_qm_v = 1'b0;
    end else if (cdb1_valid && (cdb1_tag == frat_l1_rs3_t)) begin
        d1_vm = cdb1_val; d1_qm = 5'd0; d1_qm_v = 1'b0;
    end else if (cdb_fp_valid && (cdb_fp_tag == frat_l1_rs3_t)) begin
        d1_vm = cdb_fp_val; d1_qm = 5'd0; d1_qm_v = 1'b0;
    end else begin
        d1_vm = 32'd0; d1_qm = frat_l1_rs3_t; d1_qm_v = 1'b1;
    end
end

// ============================================================================
// 4. 2-Wide Structural Dispatch Pairing & Partitioned INT + FP RS / LSQ
// ============================================================================
wire alu_free1, alu_free2, mul_free1, mul_free2;
wire div_free1, div_free2_u, br_free1, br_free2_u;
wire fma_free1, fma_free2, fdiv_free1, fdiv_free2_u, fmisc_free1, fmisc_free2;
wire lsq_rdy_ld, lsq_rdy_ld2, lsq_rdy_st, lsq_rdy_st2;

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
        FU_AMO:    lane0_rs_ready = 1'b1;
        FU_FMA:    lane0_rs_ready = fma_free1;
        FU_FDIV:   lane0_rs_ready = fdiv_free1;
        FU_FMISC:  lane0_rs_ready = fmisc_free1;
        default:   lane0_rs_ready = 1'b0;
    endcase

    case (d1_fu)
        FU_ALU:    lane1_rs_ready = (d0_fu == FU_ALU)   ? alu_free2   : alu_free1;
        FU_MUL:    lane1_rs_ready = (d0_fu == FU_MUL)   ? mul_free2   : mul_free1;
        FU_DIV:    lane1_rs_ready = (d0_fu != FU_DIV)   && div_free1;
        FU_BR:     lane1_rs_ready = (d0_fu != FU_BR)    && br_free1   && rat_cp_ready;
        FU_LOAD:   lane1_rs_ready = (d0_fu == FU_LOAD)  ? lsq_rdy_ld2 : lsq_rdy_ld;
        FU_STORE:  lane1_rs_ready = (d0_fu == FU_STORE) ? lsq_rdy_st2 : lsq_rdy_st;
        FU_SYSTEM: lane1_rs_ready = 1'b1;
        FU_AMO:    lane1_rs_ready = 1'b1;
        FU_FMA:    lane1_rs_ready = (d0_fu == FU_FMA)   ? fma_free2   : fma_free1;
        FU_FDIV:   lane1_rs_ready = (d0_fu != FU_FDIV)  && fdiv_free1;
        FU_FMISC:  lane1_rs_ready = (d0_fu == FU_FMISC) ? fmisc_free2 : fmisc_free1;
        default:   lane1_rs_ready = 1'b0;
    endcase
end

assign disp0_fire = fq0_valid && !halted && !rob_squash && !rob_full_flush && rob_free_ge1 && lane0_rs_ready;
assign disp1_fire = disp0_fire && fq1_valid && !d0_ebrk && !d0_ecall && !d0_mret && !d0_ill && !d0_is_fence_i && rob_free_ge2 && lane1_rs_ready;

// --- RS_ALU ---
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
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
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
    .cdb2_valid(cdb_fp_valid), .cdb2_tag(cdb_fp_tag), .cdb2_val(cdb_fp_val),
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

// --- RS_MUL ---
wire        mul_iss_v, mul_fu_rdy, mul_i1_v_u;
wire [1:0]  mul_iss_op, mul_i1_op_u;
wire [4:0]  mul_iss_tag, mul_i1_tag_u;
wire [31:0] mul_iss_seq, mul_iss_vj, mul_iss_vk;
wire [31:0] mul_u0, mul_u1, mul_u2, mul_u3, mul_u4, mul_u5, mul_u6;
wire        mul_b0, mul_b1, mul_b2, mul_b3;

rs_cluster2w #(.ENTRIES(4), .TAG_W(TAG_W), .OP_W(2), .DUAL_ISSUE(0)) u_rs_mul (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
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
    .cdb2_valid(cdb_fp_valid), .cdb2_tag(cdb_fp_tag), .cdb2_val(cdb_fp_val),
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

// --- RS_DIV ---
wire        div_iss_v, div_fu_rdy, div_i1_v_u;
wire [1:0]  div_iss_op, div_i1_op_u;
wire [4:0]  div_iss_tag, div_i1_tag_u;
wire [31:0] div_iss_seq, div_iss_vj, div_iss_vk;
wire [31:0] div_u0, div_u1, div_u2, div_u3, div_u4, div_u5, div_u6;
wire        div_b0, div_b1, div_b2, div_b3;

rs_cluster2w #(.ENTRIES(2), .TAG_W(TAG_W), .OP_W(2), .DUAL_ISSUE(0)) u_rs_div (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
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
    .cdb2_valid(cdb_fp_valid), .cdb2_tag(cdb_fp_tag), .cdb2_val(cdb_fp_val),
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

// --- RS_BR ---
wire        br_iss_v, br_fu_rdy, br_i1_v_u;
wire [4:0]  br_iss_op, br_i1_op_u;
wire [4:0]  br_iss_tag, br_i1_tag_u;
wire [31:0] br_iss_seq, br_iss_pc, br_iss_imm, br_iss_vj, br_iss_vk;
wire [31:0] br_u2, br_u3, br_u4, br_u5, br_u6;
wire        br_b0, br_b1, br_b2, br_b3;

rs_cluster2w #(.ENTRIES(4), .TAG_W(TAG_W), .OP_W(5), .DUAL_ISSUE(0)) u_rs_br (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
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
    .cdb2_valid(cdb_fp_valid), .cdb2_tag(cdb_fp_tag), .cdb2_val(cdb_fp_val),
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

// --- Dedicated Non-Blocking FP Reservation Station Cluster (FMA, FDIV, FMISC) ---
wire        fma_iss_v, fma_fu_rdy;
wire [6:0]  fma_iss_op;
wire [4:0]  fma_iss_tag;
wire [31:0] fma_iss_seq, fma_iss_vj, fma_iss_vk, fma_iss_vm;

rs_fpu2w #(.ENTRIES(12), .TAG_W(TAG_W), .OP_W(7), .HAS_RS3(1)) u_rs_fma (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .free_ge1(fma_free1), .free_ge2(fma_free2),
    .disp0_valid(disp0_fire && (d0_fu == FU_FMA)), .disp0_op({d0_fp_rm, d0_fp_op}),
    .disp0_rob_tag(rob_disp0_tag), .disp0_seq_id(seq0_id),
    .disp0_vj(d0_vj), .disp0_qj(d0_qj), .disp0_qj_valid(d0_qj_v),
    .disp0_vk(d0_vk), .disp0_qk(d0_qk), .disp0_qk_valid(d0_qk_v),
    .disp0_vm(d0_vm), .disp0_qm(d0_qm), .disp0_qm_valid(d0_qm_v),
    .disp1_valid(disp1_fire && (d1_fu == FU_FMA)), .disp1_op({d1_fp_rm, d1_fp_op}),
    .disp1_rob_tag(rob_disp1_tag), .disp1_seq_id(seq1_id),
    .disp1_vj(d1_vj), .disp1_qj(d1_qj), .disp1_qj_valid(d1_qj_v),
    .disp1_vk(d1_vk), .disp1_qk(d1_qk), .disp1_qk_valid(d1_qk_v),
    .disp1_vm(d1_vm), .disp1_qm(d1_qm), .disp1_qm_valid(d1_qm_v),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .cdb2_valid(cdb_fp_valid), .cdb2_tag(cdb_fp_tag), .cdb2_val(cdb_fp_val),
    .issue_valid(fma_iss_v), .fu_ready(fma_fu_rdy), .issue_op(fma_iss_op),
    .issue_rob_tag(fma_iss_tag), .issue_seq_id(fma_iss_seq),
    .issue_vj(fma_iss_vj), .issue_vk(fma_iss_vk), .issue_vm(fma_iss_vm)
);

wire        fdiv_iss_v, fdiv_fu_rdy;
wire [6:0]  fdiv_iss_op;
wire [4:0]  fdiv_iss_tag;
wire [31:0] fdiv_iss_seq, fdiv_iss_vj, fdiv_iss_vk, fdiv_iss_vm_u;

rs_fpu2w #(.ENTRIES(2), .TAG_W(TAG_W), .OP_W(7), .HAS_RS3(0)) u_rs_fdiv (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .free_ge1(fdiv_free1), .free_ge2(fdiv_free2_u),
    .disp0_valid(disp0_fire && (d0_fu == FU_FDIV)), .disp0_op({d0_fp_rm, d0_fp_op}),
    .disp0_rob_tag(rob_disp0_tag), .disp0_seq_id(seq0_id),
    .disp0_vj(d0_vj), .disp0_qj(d0_qj), .disp0_qj_valid(d0_qj_v),
    .disp0_vk(d0_vk), .disp0_qk(d0_qk), .disp0_qk_valid(d0_qk_v),
    .disp0_vm(32'd0), .disp0_qm(5'd0), .disp0_qm_valid(1'b0),
    .disp1_valid(disp1_fire && (d1_fu == FU_FDIV)), .disp1_op({d1_fp_rm, d1_fp_op}),
    .disp1_rob_tag(rob_disp1_tag), .disp1_seq_id(seq1_id),
    .disp1_vj(d1_vj), .disp1_qj(d1_qj), .disp1_qj_valid(d1_qj_v),
    .disp1_vk(d1_vk), .disp1_qk(d1_qk), .disp1_qk_valid(d1_qk_v),
    .disp1_vm(32'd0), .disp1_qm(5'd0), .disp1_qm_valid(1'b0),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .cdb2_valid(cdb_fp_valid), .cdb2_tag(cdb_fp_tag), .cdb2_val(cdb_fp_val),
    .issue_valid(fdiv_iss_v), .fu_ready(fdiv_fu_rdy), .issue_op(fdiv_iss_op),
    .issue_rob_tag(fdiv_iss_tag), .issue_seq_id(fdiv_iss_seq),
    .issue_vj(fdiv_iss_vj), .issue_vk(fdiv_iss_vk), .issue_vm(fdiv_iss_vm_u)
);

wire        fmisc_iss_v, fmisc_fu_rdy;
wire [6:0]  fmisc_iss_op;
wire [4:0]  fmisc_iss_tag;
wire [31:0] fmisc_iss_seq, fmisc_iss_vj, fmisc_iss_vk, fmisc_iss_vm_u;

rs_fpu2w #(.ENTRIES(4), .TAG_W(TAG_W), .OP_W(7), .HAS_RS3(0)) u_rs_fmisc (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .free_ge1(fmisc_free1), .free_ge2(fmisc_free2),
    .disp0_valid(disp0_fire && (d0_fu == FU_FMISC)), .disp0_op({d0_fp_rm, d0_fp_op}),
    .disp0_rob_tag(rob_disp0_tag), .disp0_seq_id(seq0_id),
    .disp0_vj(d0_vj), .disp0_qj(d0_qj), .disp0_qj_valid(d0_qj_v),
    .disp0_vk(d0_vk), .disp0_qk(d0_qk), .disp0_qk_valid(d0_qk_v),
    .disp0_vm(32'd0), .disp0_qm(5'd0), .disp0_qm_valid(1'b0),
    .disp1_valid(disp1_fire && (d1_fu == FU_FMISC)), .disp1_op({d1_fp_rm, d1_fp_op}),
    .disp1_rob_tag(rob_disp1_tag), .disp1_seq_id(seq1_id),
    .disp1_vj(d1_vj), .disp1_qj(d1_qj), .disp1_qj_valid(d1_qj_v),
    .disp1_vk(d1_vk), .disp1_qk(d1_qk), .disp1_qk_valid(d1_qk_v),
    .disp1_vm(32'd0), .disp1_qm(5'd0), .disp1_qm_valid(1'b0),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .cdb2_valid(cdb_fp_valid), .cdb2_tag(cdb_fp_tag), .cdb2_val(cdb_fp_val),
    .issue_valid(fmisc_iss_v), .fu_ready(fmisc_fu_rdy), .issue_op(fmisc_iss_op),
    .issue_rob_tag(fmisc_iss_tag), .issue_seq_id(fmisc_iss_seq),
    .issue_vj(fmisc_iss_vj), .issue_vk(fmisc_iss_vk), .issue_vm(fmisc_iss_vm_u)
);

// ============================================================================
// 5. Nine Parallel Functional Units (6 INT/LSU + 3 Non-Blocking IEEE-754 FPU!)
// ============================================================================
wire cdb_div_grant, cdb_mul_grant, cdb_br_grant, cdb_alu0_grant, cdb_alu1_grant;
wire cdb_lsu_grant = cdb_p1_grant && !csr_exec_req;
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
    end else if (rob_full_flush) begin
        alu0_out_v <= 1'b0;
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
    end else if (rob_full_flush) begin
        alu1_out_v <= 1'b0;
    end else if (alu1_fu_rdy) begin
        alu1_out_v   <= alu1_iss_v;
        alu1_out_tag <= alu1_iss_tag;
        alu1_out_seq <= alu1_iss_seq;
        alu1_out_val <= alu1_comb_res;
    end else if (rob_squash && ((alu1_out_tag - rob_head_ptr) > sq_br_age_core)) begin
        alu1_out_v <= 1'b0;
    end
end

// --- FU_MUL ---
wire        mul_out_v;
wire [4:0]  mul_out_tag;
wire [31:0] mul_out_seq, mul_out_val;

mul_unit #(.TAG_W(TAG_W)) u_fu_mul (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .in_valid(mul_iss_v), .in_ready(mul_fu_rdy), .in_op(mul_iss_op),
    .in_rs1(mul_iss_vj), .in_rs2(mul_iss_vk), .in_tag(mul_iss_tag), .in_seq(mul_iss_seq),
    .out_valid(mul_out_v), .out_ready(cdb_mul_grant),
    .out_result(mul_out_val), .out_tag(mul_out_tag), .out_seq(mul_out_seq)
);

// --- FU_DIV ---
wire        div_out_v;
wire [4:0]  div_out_tag;
wire [31:0] div_out_seq, div_out_val;

div_unit #(.TAG_W(TAG_W)) u_fu_div (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
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
wire        br_comb_exc    = br_comb_taken && (br_comb_target[1:0] != 2'b00);

reg        br_out_v, br_out_taken, br_out_exc;
reg [4:0]  br_out_tag;
reg [31:0] br_out_seq, br_out_val, br_out_target;
assign br_fu_rdy = !br_out_v || cdb_br_grant;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        br_out_v <= 1'b0; br_out_tag <= 5'd0; br_out_seq <= 32'd0;
        br_out_val <= 32'd0; br_out_taken <= 1'b0; br_out_target <= 32'd0; br_out_exc <= 1'b0;
    end else if (rob_full_flush) begin
        br_out_v <= 1'b0;
    end else if (br_fu_rdy) begin
        br_out_v      <= br_iss_v;
        br_out_tag    <= br_iss_tag;
        br_out_seq    <= br_iss_seq;
        br_out_val    <= br_comb_link;
        br_out_taken  <= br_comb_taken;
        br_out_target <= br_comb_target;
        br_out_exc    <= br_comb_exc;
    end else if (rob_squash && ((br_out_tag - rob_head_ptr) > sq_br_age_core)) begin
        br_out_v <= 1'b0;
    end
end

// --- FU_FMA (4-Stage II=1 IEEE-754 Single-Precision FMA Pipeline) ---
wire        fma_out_v, cdb_fma_grant;
wire [4:0]  fma_out_tag, fma_out_flg;
wire [31:0] fma_out_seq, fma_out_val;

fpu_fma_pipe #(.TAG_W(TAG_W)) u_fu_fma (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .in_valid(fma_iss_v), .in_ready(fma_fu_rdy),
    .in_op(fma_iss_op[2:0]), .in_rm(fma_iss_op[6:4]),
    .in_rs1(fma_iss_vj), .in_rs2(fma_iss_vk), .in_rs3(fma_iss_vm),
    .in_tag(fma_iss_tag), .in_seq(fma_iss_seq),
    .out_valid(fma_out_v), .out_ready(cdb_fma_grant),
    .out_result(fma_out_val), .out_fflags(fma_out_flg),
    .out_tag(fma_out_tag), .out_seq(fma_out_seq)
);

// --- FU_FDIV (Non-Blocking Iterative IEEE-754 FDIV.S & FSQRT.S) ---
wire        fdiv_out_v, cdb_fdiv_grant;
wire [4:0]  fdiv_out_tag, fdiv_out_flg;
wire [31:0] fdiv_out_seq, fdiv_out_val;

fpu_div_sqrt #(.TAG_W(TAG_W)) u_fu_fdiv (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .in_valid(fdiv_iss_v), .in_ready(fdiv_fu_rdy),
    .in_op(fdiv_iss_op[0]), .in_rm(fdiv_iss_op[6:4]),
    .in_rs1(fdiv_iss_vj), .in_rs2(fdiv_iss_vk),
    .in_tag(fdiv_iss_tag), .in_seq(fdiv_iss_seq),
    .out_valid(fdiv_out_v), .out_ready(cdb_fdiv_grant),
    .out_result(fdiv_out_val), .out_fflags(fdiv_out_flg),
    .out_tag(fdiv_out_tag), .out_seq(fdiv_out_seq)
);

// --- FU_FMISC (1-Cycle IEEE-754 Sign/MinMax/Cmp/Class/Cvt/Move) ---
wire [31:0] fmisc_comb_res;
wire [4:0]  fmisc_comb_flg;
wire        cdb_fmisc_grant;

fpu_misc u_fu_fmisc (
    .in_op(fmisc_iss_op[3:0]), .in_rm(fmisc_iss_op[6:4]),
    .in_rs1(fmisc_iss_vj), .in_rs2(fmisc_iss_vk),
    .out_result(fmisc_comb_res), .out_fflags(fmisc_comb_flg)
);

reg        fmisc_out_v;
reg [4:0]  fmisc_out_tag, fmisc_out_flg;
reg [31:0] fmisc_out_seq, fmisc_out_val;
assign fmisc_fu_rdy = !fmisc_out_v || cdb_fmisc_grant;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        fmisc_out_v <= 1'b0; fmisc_out_tag <= 5'd0; fmisc_out_flg <= 5'd0;
        fmisc_out_seq <= 32'd0; fmisc_out_val <= 32'd0;
    end else if (rob_full_flush) begin
        fmisc_out_v <= 1'b0;
    end else if (fmisc_fu_rdy) begin
        fmisc_out_v   <= fmisc_iss_v;
        fmisc_out_tag <= fmisc_iss_tag;
        fmisc_out_seq <= fmisc_iss_seq;
        fmisc_out_val <= fmisc_comb_res;
        fmisc_out_flg <= fmisc_comb_flg;
    end else if (rob_squash && ((fmisc_out_tag - rob_head_ptr) > sq_br_age_core)) begin
        fmisc_out_v <= 1'b0;
    end
end

// --- Dedicated 3rd CDB Arbiter (cdb_fp: FDIV > FMA > FMISC) ---
reg cdb_fdiv_g_r, cdb_fma_g_r, cdb_fmisc_g_r;
assign cdb_fdiv_grant  = cdb_fdiv_g_r;
assign cdb_fma_grant   = cdb_fma_g_r;
assign cdb_fmisc_grant = cdb_fmisc_g_r;

always @(*) begin
    cdb_fdiv_g_r  = 1'b0;
    cdb_fma_g_r   = 1'b0;
    cdb_fmisc_g_r = 1'b0;
    cdb_fp_valid  = 1'b0;
    cdb_fp_tag    = 5'd0;
    cdb_fp_seq    = 32'd0;
    cdb_fp_val    = 32'd0;
    cdb_fp_fflags = 5'd0;

    if (fdiv_out_v) begin
        cdb_fdiv_g_r  = 1'b1;
        cdb_fp_valid  = 1'b1;
        cdb_fp_tag    = fdiv_out_tag;
        cdb_fp_seq    = fdiv_out_seq;
        cdb_fp_val    = fdiv_out_val;
        cdb_fp_fflags = fdiv_out_flg;
    end else if (fma_out_v) begin
        cdb_fma_g_r   = 1'b1;
        cdb_fp_valid  = 1'b1;
        cdb_fp_tag    = fma_out_tag;
        cdb_fp_seq    = fma_out_seq;
        cdb_fp_val    = fma_out_val;
        cdb_fp_fflags = fma_out_flg;
    end else if (fmisc_out_v) begin
        cdb_fmisc_g_r = 1'b1;
        cdb_fp_valid  = 1'b1;
        cdb_fp_tag    = fmisc_out_tag;
        cdb_fp_seq    = fmisc_out_seq;
        cdb_fp_val    = fmisc_out_val;
        cdb_fp_fflags = fmisc_out_flg;
    end
end

// --- FU_LSU (8 LQ Snoop-Replay + 8 SQ + ROB-Head RV32A Atomics + FLW/FSW!) ---
wire lsq_disp_l0 = disp0_fire && d0_is_mem;
wire lsq_disp_l1 = disp1_fire && d1_is_mem;

wire        lsu_out_v, lsu_out_mem_we, lsu_out_exc_valid;
wire [4:0]  lsu_out_tag;
wire [3:0]  lsu_out_mem_wstrb, lsu_out_exc_cause;
wire [31:0] lsu_out_seq, lsu_out_val, lsu_out_mem_addr, lsu_out_mem_wdata, lsu_out_exc_tval;
wire        lsq_trace_issue_v;
wire [31:0] lsq_trace_issue_seq;

wire rob_commit_store = (rob_c0_valid && (rob_c0_fu == FU_STORE)) ||
                        (rob_c1_valid && (rob_c1_fu == FU_STORE));
wire rob_c0_is_load   = rob_c0_valid && (rob_c0_fu == FU_LOAD);
wire rob_c1_is_load   = rob_c1_valid && (rob_c1_fu == FU_LOAD);

wire amo_exec_req     = rob_head_valid && !rob_head_ready && rob_head_is_amo && !rob_full_flush;

lsq2w #(
    .LQ_DEPTH(8), .SQ_DEPTH(8), .TAG_W(TAG_W), .ID_W(ID_W),
    .AWIDTH(AWIDTH), .ENABLE_MMIO_PMA(ENABLE_MMIO_PMA), .RAM_SIZE_BYTES(RAM_SIZE_BYTES)
) u_lsq (
    .clk(clk), .reset_n(reset_n), .flush(rob_full_flush),
    .squash_valid(rob_squash), .squash_tag(rob_squash_tag), .rob_head(rob_head_ptr),
    .barrier_valid(rob_barrier_valid), .barrier_tag(rob_barrier_tag),
    .disp_valid(lsq_disp_l0),
    .disp_is_store(d0_fu == FU_STORE),
    .disp_ready_load(lsq_rdy_ld),
    .disp_ready_load2(lsq_rdy_ld2),
    .disp_ready_store(lsq_rdy_st),
    .disp_ready_store2(lsq_rdy_st2),
    .disp_rob_tag(rob_disp0_tag),
    .disp_seq_id(seq0_id),
    .disp_mem_size(d0_mem_sz),
    .disp_mem_unsigned(d0_mem_uns),
    .disp_imm(d0_imm),
    .disp_vj(d0_vj),
    .disp_qj(d0_qj),
    .disp_qj_valid(d0_qj_v),
    .disp_vk(d0_vk),
    .disp_qk(d0_qk),
    .disp_qk_valid(d0_qk_v),
    .disp1_valid(lsq_disp_l1),
    .disp1_is_store(d1_fu == FU_STORE),
    .disp1_rob_tag(rob_disp1_tag),
    .disp1_seq_id(seq1_id),
    .disp1_mem_size(d1_mem_sz),
    .disp1_mem_unsigned(d1_mem_uns),
    .disp1_imm(d1_imm),
    .disp1_vj(d1_vj),
    .disp1_qj(d1_qj),
    .disp1_qj_valid(d1_qj_v),
    .disp1_vk(d1_vk),
    .disp1_qk(d1_qk),
    .disp1_qk_valid(d1_qk_v),
    .amo_req_valid(amo_exec_req),
    .amo_req_op(hd_amo_op),
    .amo_req_tag(rob_head_tag),
    .amo_req_seq(rob_head_seq),
    .amo_req_addr(arf_head_rs1),
    .amo_req_rs2(arf_head_rs2),
    .dc_writes_idle(dc_writes_idle),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_val(cdb0_val),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_val(cdb1_val),
    .cdb2_valid(cdb_fp_valid), .cdb2_tag(cdb_fp_tag), .cdb2_val(cdb_fp_val),
    .wk0_valid(wk0_valid), .wk0_tag(wk0_tag), .wk1_valid(wk1_valid), .wk1_tag(wk1_tag),
    .byp_alu0_val(alu0_out_val), .byp_alu1_val(alu1_out_val),
    .rob_commit_store(rob_commit_store),
    .rob_commit0_load(rob_c0_is_load), .rob_commit0_tag(rob_c0_tag),
    .rob_commit1_load(rob_c1_is_load), .rob_commit1_tag(rob_c1_tag),
    .coh_snoop_inv_valid(coh_snoop_inv_valid),
    .coh_snoop_inv_line(coh_snoop_inv_line),
    .lq_head0_replay(lsq_head0_replay),
    .lq_head1_replay(lsq_head1_replay),
    .lsu_cdb_valid(lsu_out_v), .lsu_cdb_grant(cdb_lsu_grant),
    .lsu_cdb_tag(lsu_out_tag), .lsu_cdb_seq(lsu_out_seq), .lsu_cdb_val(lsu_out_val),
    .lsu_cdb_mem_we(lsu_out_mem_we), .lsu_cdb_mem_addr(lsu_out_mem_addr),
    .lsu_cdb_mem_wstrb(lsu_out_mem_wstrb), .lsu_cdb_mem_wdata(lsu_out_mem_wdata),
    .lsu_cdb_exc_valid(lsu_out_exc_valid), .lsu_cdb_exc_cause(lsu_out_exc_cause), .lsu_cdb_exc_tval(lsu_out_exc_tval),
    .dc_req_valid(dc_req_valid), .dc_req_ready(dc_req_ready),
    .dc_req_is_write(dc_req_is_write), .dc_req_is_mmio(dc_req_is_mmio),
    .dc_req_amo_op(dc_req_amo_op),
    .dc_req_addr(dc_req_addr), .dc_req_paddr(dc_req_paddr), .dc_req_wdata(dc_req_wdata), .dc_req_wstrb(dc_req_wstrb),
    .dc_req_id(dc_req_id), .dc_req_rob_tag(dc_req_rob_tag),
    .dc_hit_valid(dc_hit_valid), .dc_hit_id(dc_hit_id), .dc_hit_rdata(dc_hit_rdata), .dc_hit_err(dc_hit_err),
    .dc_fill_valid(dc_fill_valid), .dc_fill_id(dc_fill_id), .dc_fill_rdata(dc_fill_rdata), .dc_fill_err(dc_fill_err),
    .sq_empty(lsq_sq_empty), .mmio_busy(lsq_mmio_busy),
    .trace_issue_valid(lsq_trace_issue_v), .trace_issue_seq(lsq_trace_issue_seq),
    .stat_stlf_hit(stat_stlf_hit),
    .stat_ex_byp_cnt(lsq_byp_cnt)
);

assign dc_flush        = rob_full_flush;
assign dc_squash_valid = rob_squash;
assign dc_squash_tag   = rob_squash_tag;
assign dc_rob_head     = rob_head_ptr;

// ============================================================================
// 6. Dual-CDB Integer/LSU Arbiter (Port 1 shared cleanly by Head CSR > LSU/AMO)
// ============================================================================
wire        p1_valid     = csr_exec_req || lsu_out_v;
wire [4:0]  p1_tag       = csr_exec_req ? rob_head_tag         : lsu_out_tag;
wire [31:0] p1_seq       = csr_exec_req ? rob_head_seq         : lsu_out_seq;
wire [31:0] p1_val       = csr_exec_req ? csr_rdata            : lsu_out_val;
wire        p1_exc_valid = csr_exec_req ? csr_access_illegal   : lsu_out_exc_valid;
wire [3:0]  p1_exc_cause = csr_exec_req ? MCAUSE_ILLEGAL_INSTR : lsu_out_exc_cause;
wire [31:0] p1_exc_tval  = csr_exec_req ? rob_head_instr       : lsu_out_exc_tval;

cdb_arbiter2w #(.TAG_W(TAG_W)) u_cdb (
    .div_valid(div_out_v), .div_grant(cdb_div_grant), .div_tag(div_out_tag), .div_seq(div_out_seq), .div_val(div_out_val),
    .lsu_valid(p1_valid), .lsu_grant(cdb_p1_grant), .lsu_tag(p1_tag), .lsu_seq(p1_seq), .lsu_val(p1_val),
    .lsu_mem_we(csr_exec_req ? 1'b0 : lsu_out_mem_we),
    .lsu_mem_addr(csr_exec_req ? 32'd0 : lsu_out_mem_addr),
    .lsu_mem_wstrb(csr_exec_req ? 4'd0 : lsu_out_mem_wstrb),
    .lsu_mem_wdata(csr_exec_req ? 32'd0 : lsu_out_mem_wdata),
    .lsu_exc_valid(p1_exc_valid), .lsu_exc_cause(p1_exc_cause), .lsu_exc_tval(p1_exc_tval),
    .mul_valid(mul_out_v), .mul_grant(cdb_mul_grant), .mul_tag(mul_out_tag), .mul_seq(mul_out_seq), .mul_val(mul_out_val),
    .br_valid(br_out_v), .br_grant(cdb_br_grant), .br_tag(br_out_tag), .br_seq(br_out_seq), .br_val(br_out_val),
    .br_taken(br_out_taken), .br_target(br_out_target),
    .br_exc_valid(br_out_exc), .br_exc_cause(MCAUSE_INSTR_MISALIGN), .br_exc_tval(br_out_target),
    .alu0_valid(alu0_out_v), .alu0_grant(cdb_alu0_grant), .alu0_tag(alu0_out_tag), .alu0_seq(alu0_out_seq), .alu0_val(alu0_out_val),
    .alu1_valid(alu1_out_v), .alu1_grant(cdb_alu1_grant), .alu1_tag(alu1_out_tag), .alu1_seq(alu1_out_seq), .alu1_val(alu1_out_val),
    .cdb0_valid(cdb0_valid), .cdb0_tag(cdb0_tag), .cdb0_seq(cdb0_seq), .cdb0_val(cdb0_val),
    .cdb0_is_br(cdb0_is_br), .cdb0_br_taken(cdb0_br_taken), .cdb0_br_target(cdb0_br_target),
    .cdb0_is_mem(cdb0_is_mem), .cdb0_mem_we(cdb0_mem_we), .cdb0_mem_addr(cdb0_mem_addr),
    .cdb0_mem_wstrb(cdb0_mem_wstrb), .cdb0_mem_wdata(cdb0_mem_wdata),
    .cdb0_exc_valid(cdb0_exc_valid), .cdb0_exc_cause(cdb0_exc_cause), .cdb0_exc_tval(cdb0_exc_tval),
    .cdb1_valid(cdb1_valid), .cdb1_tag(cdb1_tag), .cdb1_seq(cdb1_seq), .cdb1_val(cdb1_val)
);

// ============================================================================
// 7. RVFI Outputs, 2-Wide Trace Hooks & PMU Telemetry
// ============================================================================
assign commit0_valid    = rob_c0_valid;
assign commit0_rd_we    = rob_c0_rd_we;
assign commit0_rd_is_fp = rob_c0_rd_fp;
assign commit0_rd_addr  = rob_c0_rd_addr;
assign commit0_rd_wdata = rob_c0_rd_wdata;
assign commit0_fflags   = rob_c0_fflags;

assign commit1_valid    = rob_c1_valid;
assign commit1_rd_we    = rob_c1_rd_we;
assign commit1_rd_is_fp = rob_c1_rd_fp;
assign commit1_rd_addr  = rob_c1_rd_addr;
assign commit1_rd_wdata = rob_c1_rd_wdata;
assign commit1_fflags   = rob_c1_fflags;

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
assign trace_issue_valid[5] = lsq_trace_issue_v || csr_exec_fire;

assign trace_issue_seq[0*32 +: 32] = alu0_iss_seq;
assign trace_issue_seq[1*32 +: 32] = alu1_iss_seq;
assign trace_issue_seq[2*32 +: 32] = mul_iss_seq;
assign trace_issue_seq[3*32 +: 32] = div_iss_seq;
assign trace_issue_seq[4*32 +: 32] = br_iss_seq;
assign trace_issue_seq[5*32 +: 32] = csr_exec_fire ? rob_head_seq : lsq_trace_issue_seq;

assign trace_cdb_valid          = {cdb1_valid, cdb0_valid};
assign trace_cdb_seq[31:0]      = cdb0_seq;
assign trace_cdb_seq[63:32]     = cdb1_seq;
assign trace_flush_valid        = rob_squash || rob_full_flush;
assign trace_flush_after_seq    = rob_full_flush ? rob_full_flush_after_seq : rob_squash_seq;

assign stat_rob_full_stall = fq0_valid && !halted && !rob_squash && !rob_full_flush && !rob_free_ge1;
assign stat_cp_full_stall  = fq0_valid && !halted && !rob_squash && !rob_full_flush && rob_free_ge1 &&
                             (d0_fu == FU_BR) && br_free1 && !rat_cp_ready;
assign stat_rs_full_stall  = fq0_valid && !halted && !rob_squash && !rob_full_flush && rob_free_ge1 &&
                             !d0_is_mem && !lane0_rs_ready && !stat_cp_full_stall;
assign stat_lsq_full_stall = fq0_valid && !halted && !rob_squash && !rob_full_flush && rob_free_ge1 &&
                             d0_is_mem && !lane0_rs_ready;

assign stat_br_total_cnt   = {1'b0, (rob_c0_valid && rob_c0_is_br)} +
                             {1'b0, (rob_c1_valid && rob_c1_is_br)};
assign stat_br_mispred_cnt = {1'b0, (rob_c0_valid && rob_c0_mispred)} +
                             {1'b0, (rob_c1_valid && rob_c1_mispred)};
assign stat_br_fast_squash = rob_squash_has_older;
wire d0_was_b = d0_rd_fp ? frat_d0_was_b : ((d0_rd != 5'd0) && rat_d0_was_b);
wire d1_was_b = d1_rd_fp ? frat_d1_was_b : ((d1_rd != 5'd0) && rat_d1_was_b);
assign stat_war_waw_cnt    = {1'b0, (disp0_fire && d0_rd_we && d0_was_b)} +
                             {1'b0, (disp1_fire && d1_rd_we && d1_was_b)};

wire other_fu_no_div = trace_issue_valid[0] || trace_issue_valid[1] || trace_issue_valid[2] ||
                       trace_issue_valid[4] || trace_issue_valid[5] ||
                       (fma_iss_v && fma_fu_rdy) || (fmisc_iss_v && fmisc_fu_rdy);
wire other_fu_no_lsu = trace_issue_valid[0] || trace_issue_valid[1] || trace_issue_valid[2] ||
                       trace_issue_valid[3] || trace_issue_valid[4] ||
                       (fma_iss_v && fma_fu_rdy) || (fdiv_iss_v && fdiv_fu_rdy) || (fmisc_iss_v && fmisc_fu_rdy);

assign stat_div_overlap_cyc         = (!div_fu_rdy || !fdiv_fu_rdy) && other_fu_no_div;
assign stat_dcache_miss_overlap_cyc = dc_any_miss && other_fu_no_lsu;
assign stat_dual_disp               = disp1_fire;
assign stat_dual_cdb                = (cdb0_valid && cdb1_valid) || (cdb0_valid && cdb_fp_valid) || (cdb1_valid && cdb_fp_valid);
assign stat_dual_commit             = rob_c0_valid && rob_c1_valid;
assign stat_ex_byp_cnt              = {1'b0, alu_byp_cnt} + {1'b0, mul_byp_cnt} +
                                      {1'b0, div_byp_cnt} + {1'b0, br_byp_cnt} +
                                      {2'd0, lsq_byp_cnt};
assign stat_ras_pred_cnt            = {1'b0, (rob_c0_valid && rob_c0_uras)} +
                                      {1'b0, (rob_c1_valid && rob_c1_uras)};
assign stat_ras_mispred_cnt         = {1'b0, (rob_c0_valid && rob_c0_uras && rob_c0_mispred)} +
                                      {1'b0, (rob_c1_valid && rob_c1_uras && rob_c1_mispred)};
assign stat_replay_flush            = rob_replay_flush;
assign stat_amo_commit              = rob_c0_valid && (rob_c0_fu == FU_AMO);
assign stat_fence_commit            = rob_c0_valid && (rob_head_is_fence || rob_c0_is_fence_i);
assign stat_trap_taken              = rob_trap_enter;

endmodule
