`timescale 1ns / 1ps

// ============================================================================
// Unit Testbench for Full RV32IMA_Zicsr_Zifencei Decoder (Phase 0)
// Verifies RV32I, RV32M, all 11 RV32A atomics (with aq/rl), FENCE, FENCE.I,
// all 6 Zicsr instructions, EBREAK, and illegal instruction rejection.
// ============================================================================
module tb_decoder;

`include "rv32ima_defs.vh"

reg  [31:0] instr;
wire [4:0]  rs1, rs2, rd;
wire        uses_rs1, uses_rs2, rd_we;
wire [2:0]  imm_sel, fu_type;
wire [3:0]  alu_select;
wire        asel, bsel;
wire [1:0]  mul_op, div_op;
wire [2:0]  br_funct3;
wire        is_branch, is_jal, is_jalr;
wire        mem_read, mem_write;
wire [1:0]  mem_size;
wire        mem_unsigned, is_ebreak;
wire        is_fence, is_fence_i, is_csr;
wire [2:0]  csr_op;
wire [11:0] csr_addr;
wire        csr_use_imm;
wire        is_amo, is_lr, is_sc;
wire [3:0]  amo_op;
wire        amo_aq, amo_rl;
wire        illegal;

decoder dut (
    .instr(instr),
    .rs1(rs1), .rs2(rs2), .rd(rd),
    .uses_rs1(uses_rs1), .uses_rs2(uses_rs2), .rd_we(rd_we),
    .imm_sel(imm_sel), .fu_type(fu_type), .alu_select(alu_select),
    .asel(asel), .bsel(bsel), .mul_op(mul_op), .div_op(div_op),
    .br_funct3(br_funct3), .is_branch(is_branch), .is_jal(is_jal), .is_jalr(is_jalr),
    .mem_read(mem_read), .mem_write(mem_write), .mem_size(mem_size), .mem_unsigned(mem_unsigned),
    .is_ebreak(is_ebreak),
    .is_fence(is_fence), .is_fence_i(is_fence_i),
    .is_csr(is_csr), .csr_op(csr_op), .csr_addr(csr_addr), .csr_use_imm(csr_use_imm),
    .is_amo(is_amo), .is_lr(is_lr), .is_sc(is_sc), .amo_op(amo_op),
    .amo_aq(amo_aq), .amo_rl(amo_rl),
    .illegal(illegal)
);

integer errors;
integer checks;

task check_cond(input cond, input [255:0] msg);
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
    check_cond(!illegal && fu_type == FU_SYSTEM && is_ebreak && !is_csr, "EBREAK");

    instr = mk_csr(CSR_MSCRATCH, 5'd0, 3'b100, 5'd1); #1; // illegal SYSTEM funct3=4
    check_cond(illegal, "Illegal SYSTEM f3=4");

    if (errors == 0)
        $display("PASS [tb_decoder]: %0d/%0d RV32IMA_Zicsr_Zifencei decoder checks passed!", checks, checks);
    else begin
        $display("FAIL [tb_decoder]: %0d errors out of %0d checks!", errors, checks);
        $fatal(1);
    end
    $finish;
end

endmodule
