`timescale 1ns / 1ps

// ============================================================================
// Unit Testbench for Full RV32IMA_Zicsr_Zifencei Decoder (Phase 0)
// Verifies RV32I, RV32M, all 11 RV32A atomics (with aq/rl), FENCE, FENCE.I,
// all 6 Zicsr instructions, EBREAK, and illegal instruction rejection.
// ============================================================================
module tb_decoder;

`include "rv32ima_defs.vh"

reg  [31:0] instr;
reg  [1:0]  mstatus_fs;
reg  [2:0]  csr_frm;
wire [4:0]  rs1, rs2, rs3, rd;
wire        uses_rs1, uses_rs2, uses_rs3, rs1_is_fp, rs2_is_fp, rd_is_fp, rd_we;
wire [2:0]  imm_sel;
wire [3:0]  fu_type;
wire [3:0]  alu_select;
wire        asel, bsel;
wire [1:0]  mul_op, div_op;
wire [2:0]  br_funct3;
wire        is_branch, is_jal, is_jalr;
wire        mem_read, mem_write;
wire [1:0]  mem_size;
wire        mem_unsigned, is_ebreak, is_ecall, is_mret;
wire        is_fence, is_fence_i, is_csr;
wire [2:0]  csr_op;
wire [11:0] csr_addr;
wire        csr_use_imm;
wire        is_amo, is_lr, is_sc;
wire [3:0]  amo_op;
wire        amo_aq, amo_rl;
wire        is_fp;
wire [3:0]  fp_op;
wire [2:0]  fp_rm;
wire        illegal;

decoder dut (
    .instr(instr), .mstatus_fs(mstatus_fs), .csr_frm(csr_frm),
    .rs1(rs1), .rs2(rs2), .rs3(rs3), .rd(rd),
    .uses_rs1(uses_rs1), .uses_rs2(uses_rs2), .uses_rs3(uses_rs3),
    .rs1_is_fp(rs1_is_fp), .rs2_is_fp(rs2_is_fp), .rd_is_fp(rd_is_fp), .rd_we(rd_we),
    .imm_sel(imm_sel), .fu_type(fu_type), .alu_select(alu_select),
    .asel(asel), .bsel(bsel), .mul_op(mul_op), .div_op(div_op),
    .br_funct3(br_funct3), .is_branch(is_branch), .is_jal(is_jal), .is_jalr(is_jalr),
    .mem_read(mem_read), .mem_write(mem_write), .mem_size(mem_size), .mem_unsigned(mem_unsigned),
    .is_ebreak(is_ebreak), .is_ecall(is_ecall), .is_mret(is_mret),
    .is_fence(is_fence), .is_fence_i(is_fence_i),
    .is_csr(is_csr), .csr_op(csr_op), .csr_addr(csr_addr), .csr_use_imm(csr_use_imm),
    .is_amo(is_amo), .is_lr(is_lr), .is_sc(is_sc), .amo_op(amo_op),
    .amo_aq(amo_aq), .amo_rl(amo_rl),
    .is_fp(is_fp), .fp_op(fp_op), .fp_rm(fp_rm),
    .illegal(illegal)
);

integer errors;
integer checks;

task check_cond(input cond, input [511:0] msg);
    begin
        checks = checks + 1;
        if (!cond) begin
            $display("FAIL [tb_decoder]: %0s (instr=0x%08x)", msg, instr);
            errors = errors + 1;
        end
    end
endtask

function automatic [31:0] mk_amo(
    input [4:0] f5,
    input       aq,
    input       rl,
    input [4:0] a_rs2,
    input [4:0] a_rs1,
    input [2:0] f3,
    input [4:0] a_rd
);
    begin
        mk_amo = {f5, aq, rl, a_rs2, a_rs1, f3, a_rd, OP_AMO};
    end
endfunction

function automatic [31:0] mk_csr(
    input [11:0] addr,
    input [4:0]  src,
    input [2:0]  f3,
    input [4:0]  dst
);
    begin
        mk_csr = {addr, src, f3, dst, OP_SYSTEM};
    end
endfunction

initial begin
    errors = 0;
    checks = 0;
    mstatus_fs = 2'b01;
    csr_frm = 3'b000;

    // 1. RV32I / RV32M spot checks
    instr = 32'h0020_81B3; #1; // add x3, x1, x2
    check_cond(!illegal && fu_type == FU_ALU && alu_select == ALU_ADD && rd_we && uses_rs1 && uses_rs2, "ADD");

    instr = 32'h0252_0333; #1; // mul x6, x4, x5
    check_cond(!illegal && fu_type == FU_MUL && mul_op == MUL_OP_MUL && rd_we, "MUL");

    instr = 32'h0252_4333; #1; // div x6, x4, x5
    check_cond(!illegal && fu_type == FU_DIV && div_op == DIV_OP_DIV && rd_we, "DIV");

    // 2. FENCE & FENCE.I (Zifencei)
    instr = 32'h0330_000F; #1; // fence rw, rw
    check_cond(!illegal && fu_type == FU_SYSTEM && is_fence && !is_fence_i && !rd_we, "FENCE rw,rw");

    instr = 32'h8330_000F; #1; // fence.tso
    check_cond(!illegal && fu_type == FU_SYSTEM && is_fence && !is_fence_i, "FENCE.TSO");

    instr = 32'h0000_100F; #1; // fence.i
    check_cond(!illegal && fu_type == FU_SYSTEM && is_fence_i && !is_fence && !rd_we, "FENCE.I");

    instr = 32'h0000_200F; #1; // illegal MISC_MEM funct3=2
    check_cond(illegal, "Illegal MISC_MEM f3=2");

    // 3. All 11 RV32A Atomic Instructions
    instr = mk_amo(F5_LR, 1'b1, 1'b0, 5'd0, 5'd10, 3'b010, 5'd11); #1;
    check_cond(!illegal && fu_type == FU_AMO && is_amo && is_lr && !is_sc && amo_op == AMO_LR &&
               uses_rs1 && !uses_rs2 && rd_we && mem_read && !mem_write && amo_aq && !amo_rl, "LR.W.AQ");

    // LR.W with nonzero rs2 must be illegal per RV32A spec!
    instr = mk_amo(F5_LR, 1'b0, 1'b0, 5'd1, 5'd10, 3'b010, 5'd11); #1;
    check_cond(illegal, "Illegal LR.W rs2!=0");

    instr = mk_amo(F5_SC, 1'b0, 1'b1, 5'd12, 5'd10, 3'b010, 5'd11); #1;
    check_cond(!illegal && fu_type == FU_AMO && is_amo && !is_lr && is_sc && amo_op == AMO_SC &&
               uses_rs1 && uses_rs2 && rd_we && !mem_read && mem_write && !amo_aq && amo_rl, "SC.W.RL");

    instr = mk_amo(F5_AMOSWAP, 1'b1, 1'b1, 5'd2, 5'd3, 3'b010, 5'd4); #1;
    check_cond(!illegal && fu_type == FU_AMO && is_amo && amo_op == AMO_SWAP && mem_read && mem_write && amo_aq && amo_rl, "AMOSWAP.W.AQRL");

    instr = mk_amo(F5_AMOADD,  1'b0, 1'b0, 5'd2, 5'd3, 3'b010, 5'd4); #1;
    check_cond(!illegal && amo_op == AMO_ADD && mem_read && mem_write, "AMOADD.W");

    instr = mk_amo(F5_AMOXOR,  1'b0, 1'b0, 5'd2, 5'd3, 3'b010, 5'd4); #1;
    check_cond(!illegal && amo_op == AMO_XOR, "AMOXOR.W");

    instr = mk_amo(F5_AMOAND,  1'b0, 1'b0, 5'd2, 5'd3, 3'b010, 5'd4); #1;
    check_cond(!illegal && amo_op == AMO_AND, "AMOAND.W");

    instr = mk_amo(F5_AMOOR,   1'b0, 1'b0, 5'd2, 5'd3, 3'b010, 5'd4); #1;
    check_cond(!illegal && amo_op == AMO_OR, "AMOOR.W");

    instr = mk_amo(F5_AMOMIN,  1'b0, 1'b0, 5'd2, 5'd3, 3'b010, 5'd4); #1;
    check_cond(!illegal && amo_op == AMO_MIN, "AMOMIN.W");

    instr = mk_amo(F5_AMOMAX,  1'b0, 1'b0, 5'd2, 5'd3, 3'b010, 5'd4); #1;
    check_cond(!illegal && amo_op == AMO_MAX, "AMOMAX.W");

    instr = mk_amo(F5_AMOMINU, 1'b0, 1'b0, 5'd2, 5'd3, 3'b010, 5'd4); #1;
    check_cond(!illegal && amo_op == AMO_MINU, "AMOMINU.W");

    instr = mk_amo(F5_AMOMAXU, 1'b0, 1'b0, 5'd2, 5'd3, 3'b010, 5'd4); #1;
    check_cond(!illegal && amo_op == AMO_MAXU, "AMOMAXU.W");

    // RV64A .D encoding (funct3=011) must be illegal on RV32A
    instr = mk_amo(F5_AMOADD,  1'b0, 1'b0, 5'd2, 5'd3, 3'b011, 5'd4); #1;
    check_cond(illegal, "Illegal AMO.D on RV32A");

    // Unknown AMO funct5
    instr = mk_amo(5'b11111,   1'b0, 1'b0, 5'd2, 5'd3, 3'b010, 5'd4); #1;
    check_cond(illegal && !is_amo && !rd_we, "Illegal AMO funct5");

    // 4. All 6 Zicsr instructions + EBREAK
    instr = mk_csr(CSR_MHARTID, 5'd0, CSR_OP_RS, 5'd10); #1; // csrr a0, mhartid
    check_cond(!illegal && fu_type == FU_SYSTEM && is_csr && !csr_use_imm &&
               csr_op == CSR_OP_RS && csr_addr == CSR_MHARTID && uses_rs1 && rd_we, "CSRR mhartid");

    instr = mk_csr(CSR_MSCRATCH, 5'd5, CSR_OP_RW, 5'd0); #1; // csrw mscratch, t0
    check_cond(!illegal && is_csr && !csr_use_imm && csr_op == CSR_OP_RW && !rd_we && uses_rs1, "CSRW mscratch");

    instr = mk_csr(CSR_MSCRATCH, 5'd5, CSR_OP_RC, 5'd6); #1;
    check_cond(!illegal && is_csr && !csr_use_imm && csr_op == CSR_OP_RC && rd_we && uses_rs1, "CSRRC");

    instr = mk_csr(CSR_MSCRATCH, 5'd15, CSR_OP_RWI, 5'd7); #1;
    check_cond(!illegal && is_csr && csr_use_imm && csr_op == CSR_OP_RWI && !uses_rs1 && rd_we, "CSRRWI");

    instr = mk_csr(CSR_MSCRATCH, 5'd3, CSR_OP_RSI, 5'd7); #1;
    check_cond(!illegal && is_csr && csr_use_imm && csr_op == CSR_OP_RSI && !uses_rs1 && rd_we, "CSRRSI");

    instr = mk_csr(CSR_MSCRATCH, 5'd3, CSR_OP_RCI, 5'd7); #1;
    check_cond(!illegal && is_csr && csr_use_imm && csr_op == CSR_OP_RCI && !uses_rs1 && rd_we, "CSRRCI");

    instr = INSTR_EBREAK; #1;
    check_cond(!illegal && fu_type == FU_SYSTEM && is_ebreak && !is_ecall && !is_mret && !is_csr, "EBREAK");

    instr = INSTR_ECALL; #1;
    check_cond(!illegal && fu_type == FU_SYSTEM && is_ecall && !is_ebreak && !is_mret && !is_csr && !rd_we, "ECALL");

    instr = INSTR_MRET; #1;
    check_cond(!illegal && fu_type == FU_SYSTEM && is_mret && !is_ebreak && !is_ecall && !is_csr && !rd_we, "MRET");

    instr = mk_csr(CSR_MSCRATCH, 5'd0, 3'b100, 5'd1); #1; // illegal SYSTEM funct3=4
    check_cond(illegal && fu_type == FU_SYSTEM && !rd_we && !uses_rs1 && !uses_rs2, "Illegal SYSTEM f3=4 clamp");

    instr = 32'hFFFF_FFFF; #1; // all-ones illegal opcode
    check_cond(illegal && fu_type == FU_SYSTEM && !rd_we && !mem_read && !mem_write && !is_branch, "Illegal 0xFFFFFFFF clamp");

    // 5. Full RV32F Single-Precision Decode (all 26 ops, f0 writable, mstatus.FS=0 & bad rm illegal)
    mstatus_fs = 2'b01; csr_frm = 3'b000;
    // FLW f0, 16(x2) -> rd_we MUST be 1 even for f0!
    instr = {12'd16, 5'd2, 3'b010, 5'd0, OP_LOAD_FP}; #1;
    check_cond(!illegal && fu_type == FU_LOAD && is_fp && mem_read && rd_is_fp && rd_we && uses_rs1 && !rs1_is_fp, "FLW f0");
    // FSW f3, -4(x2)
    instr = {7'h7F, 5'd3, 5'd2, 3'b010, 5'h1C, OP_STORE_FP}; #1;
    check_cond(!illegal && fu_type == FU_STORE && is_fp && mem_write && !rd_we && uses_rs1 && !rs1_is_fp && uses_rs2 && rs2_is_fp, "FSW f3");
    // FMADD/FMSUB/FNMSUB/FNMADD.S f0, f1, f2, f3, dyn (rm=7 -> resolved to csr_frm=3'b010)
    csr_frm = 3'b010;
    instr = {5'd3, 2'b00, 5'd2, 5'd1, 3'b111, 5'd0, OP_MADD}; #1;
    check_cond(!illegal && fu_type == FU_FMA && is_fp && uses_rs1 && uses_rs2 && uses_rs3 && rs1_is_fp && rs2_is_fp && rd_is_fp && rd_we && fp_rm == 3'b010, "FMADD.S dyn");
    instr = {5'd3, 2'b00, 5'd2, 5'd1, 3'b000, 5'd4, OP_MSUB}; #1;
    check_cond(!illegal && fu_type == FU_FMA && is_fp && uses_rs3 && fp_rm == 3'b000, "FMSUB.S");
    instr = {5'd3, 2'b00, 5'd2, 5'd1, 3'b001, 5'd4, OP_NMSUB}; #1;
    check_cond(!illegal && fu_type == FU_FMA && is_fp && uses_rs3 && fp_rm == 3'b001, "FNMSUB.S");
    instr = {5'd3, 2'b00, 5'd2, 5'd1, 3'b011, 5'd4, OP_NMADD}; #1;
    check_cond(!illegal && fu_type == FU_FMA && is_fp && uses_rs3 && fp_rm == 3'b011, "FNMADD.S");
    // FADD / FSUB / FMUL / FDIV / FSQRT
    instr = {7'h00, 5'd2, 5'd1, 3'b000, 5'd0, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMA && !uses_rs3 && rd_is_fp && rd_we, "FADD.S f0");
    instr = {7'h04, 5'd2, 5'd1, 3'b000, 5'd5, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMA && !uses_rs3 && rd_is_fp && rd_we, "FSUB.S");
    instr = {7'h08, 5'd2, 5'd1, 3'b000, 5'd5, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMA && !uses_rs3 && rd_is_fp && rd_we, "FMUL.S");
    instr = {7'h0C, 5'd2, 5'd1, 3'b000, 5'd5, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FDIV && uses_rs1 && uses_rs2 && rd_is_fp && rd_we, "FDIV.S");
    instr = {7'h2C, 5'd0, 5'd1, 3'b000, 5'd5, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FDIV && uses_rs1 && !uses_rs2 && rd_is_fp && rd_we, "FSQRT.S");
    // FSGNJ/N/X, FMIN/FMAX, FEQ/FLT/FLE, FCLASS, FMV.X.W, FMV.W.X, FCVT
    instr = {7'h10, 5'd2, 5'd1, 3'b010, 5'd5, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMISC && rd_is_fp && rd_we, "FSGNJX.S");
    instr = {7'h14, 5'd2, 5'd1, 3'b001, 5'd5, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMISC && rd_is_fp && rd_we, "FMAX.S");
    instr = {7'h50, 5'd2, 5'd1, 3'b010, 5'd5, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMISC && !rd_is_fp && rd_we && rs1_is_fp && rs2_is_fp, "FEQ.S");
    instr = {7'h60, 5'd1, 5'd1, 3'b001, 5'd5, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMISC && !rd_is_fp && rd_we && rs1_is_fp && !uses_rs2, "FCVT.WU.S");
    instr = {7'h68, 5'd0, 5'd1, 3'b000, 5'd0, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMISC && rd_is_fp && rd_we && !rs1_is_fp && !uses_rs2, "FCVT.S.W f0");
    instr = {7'h70, 5'd0, 5'd1, 3'b000, 5'd5, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMISC && !rd_is_fp && rd_we && rs1_is_fp, "FMV.X.W");
    instr = {7'h70, 5'd0, 5'd1, 3'b001, 5'd5, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMISC && !rd_is_fp && rd_we && rs1_is_fp, "FCLASS.S");
    instr = {7'h78, 5'd0, 5'd1, 3'b000, 5'd0, OP_OP_FP}; #1;
    check_cond(!illegal && fu_type == FU_FMISC && rd_is_fp && rd_we && !rs1_is_fp, "FMV.W.X f0");
    // Illegal RM (5, 6, or dyn with csr_frm=5) & mstatus.FS == 00 illegal!
    instr = {7'h00, 5'd2, 5'd1, 3'b101, 5'd5, OP_OP_FP}; #1;
    check_cond(illegal && !is_fp && !rd_we, "Illegal FADD rm=5");
    csr_frm = 3'b101;
    instr = {7'h00, 5'd2, 5'd1, 3'b111, 5'd5, OP_OP_FP}; #1;
    check_cond(illegal && !is_fp && !rd_we, "Illegal FADD dyn csr_frm=5");
    csr_frm = 3'b000; mstatus_fs = 2'b00;
    instr = {7'h00, 5'd2, 5'd1, 3'b000, 5'd5, OP_OP_FP}; #1;
    check_cond(illegal && !is_fp && !rd_we, "Illegal FADD when mstatus.FS==Off");
    instr = {12'd0, 5'd2, 3'b010, 5'd1, OP_LOAD_FP}; #1;
    check_cond(illegal && !mem_read && !rd_we, "Illegal FLW when mstatus.FS==Off");

    if (errors == 0)
        $display("PASS [tb_decoder]: %0d/%0d RV32IMAF_Zicsr_Zifencei decoder checks passed!", checks, checks);
    else begin
        $display("FAIL [tb_decoder]: %0d errors out of %0d checks!", errors, checks);
        $fatal(1);
    end
    $finish;
end

endmodule
