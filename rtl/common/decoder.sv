`timescale 1ns / 1ps

// ============================================================================
// Full RV32IMAF_Zicsr_Zifencei Instruction Decoder (Phase U6)
// Supports:
//   - Base RV32I (all 37 user instructions + EBREAK, ECALL, MRET)
//   - RV32M (MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU)
//   - RV32A (LR.W, SC.W, AMOSWAP/ADD/XOR/AND/OR/MIN/MAX/MINU/MAXU.W + aq/rl)
//   - RV32F (All 26 Single-Precision IEEE-754 instructions + mstatus.FS & frm checks!)
//   - Zifencei & Base Ordering (FENCE, FENCE.I)
//   - Zicsr (CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI)
// ============================================================================
module decoder (
    input  wire [31:0] instr,
    input  wire [1:0]  mstatus_fs,
    input  wire [2:0]  csr_frm,

    output wire [4:0]  rs1,
    output wire [4:0]  rs2,
    output wire [4:0]  rs3,
    output wire [4:0]  rd,
    output reg         uses_rs1,
    output reg         uses_rs2,
    output reg         uses_rs3,
    output reg         rs1_is_fp,
    output reg         rs2_is_fp,
    output reg         rd_is_fp,
    output reg         rd_we,
    output reg  [2:0]  imm_sel,
    output reg  [3:0]  fu_type,
    output reg  [3:0]  alu_select,
    output reg         asel,         // 1: PC, 0: rs1
    output reg         bsel,         // 1: imm, 0: rs2
    output reg  [1:0]  mul_op,
    output reg  [1:0]  div_op,
    output wire [2:0]  br_funct3,
    output reg         is_branch,
    output reg         is_jal,
    output reg         is_jalr,
    output reg         mem_read,
    output reg         mem_write,
    output reg  [1:0]  mem_size,
    output reg         mem_unsigned,
    output reg         is_ebreak,
    output reg         is_ecall,
    output reg         is_mret,

    // FENCE / FENCE.I / CSR / RV32A Atomics
    output reg         is_fence,
    output reg         is_fence_i,
    output reg         is_csr,
    output reg  [2:0]  csr_op,
    output wire [11:0] csr_addr,
    output reg         csr_use_imm,
    output reg         is_amo,
    output reg         is_lr,
    output reg         is_sc,
    output reg  [3:0]  amo_op,
    output wire        amo_aq,
    output wire        amo_rl,

    // RV32F Floating-Point Decode Outputs
    output reg         is_fp,
    output reg  [3:0]  fp_op,
    output reg  [2:0]  fp_rm,

    output reg         illegal
);

`include "rv32ima_defs.vh"
`include "fpu_defs.vh"

wire [6:0] opcode = instr[6:0];
wire [2:0] funct3 = instr[14:12];
wire [6:0] funct7 = instr[31:25];
wire [4:0] funct5 = instr[31:27];
wire [1:0] fp_fmt = instr[26:25];

assign rs1       = instr[19:15];
assign rs2       = instr[24:20];
assign rs3       = instr[31:27];
assign rd        = instr[11:7];
assign br_funct3 = funct3;
assign csr_addr  = instr[31:20];
assign amo_aq    = instr[26];
assign amo_rl    = instr[25];

// Resolved rounding mode & legality for FP ops that consume rm[2:0]
wire [2:0] rm_resolved = (funct3 == FPU_RM_DYN) ? csr_frm : funct3;
wire       rm_illegal  = (funct3 == 3'b101) || (funct3 == 3'b110) || (rm_resolved > 3'd4);
wire       fp_disabled = (mstatus_fs == 2'b00);

always @(*) begin
    uses_rs1     = 1'b0;
    uses_rs2     = 1'b0;
    uses_rs3     = 1'b0;
    rs1_is_fp    = 1'b0;
    rs2_is_fp    = 1'b0;
    rd_is_fp     = 1'b0;
    rd_we        = 1'b0;
    imm_sel      = IMM_I;
    fu_type      = FU_ALU;
    alu_select   = ALU_ADD;
    asel         = 1'b0;
    bsel         = 1'b0;
    mul_op       = MUL_OP_MUL;
    div_op       = DIV_OP_DIV;
    is_branch    = 1'b0;
    is_jal       = 1'b0;
    is_jalr      = 1'b0;
    mem_read     = 1'b0;
    mem_write    = 1'b0;
    mem_size     = MEM_WORD;
    mem_unsigned = 1'b0;
    is_ebreak    = 1'b0;
    is_ecall     = 1'b0;
    is_mret      = 1'b0;
    is_fence     = 1'b0;
    is_fence_i   = 1'b0;
    is_csr       = 1'b0;
    csr_op       = 3'd0;
    csr_use_imm  = 1'b0;
    is_amo       = 1'b0;
    is_lr        = 1'b0;
    is_sc        = 1'b0;
    amo_op       = AMO_NONE;
    is_fp        = 1'b0;
    fp_op        = 4'd0;
    fp_rm        = rm_resolved;
    illegal      = 1'b0;

    case (opcode)
        OP_LUI: begin
            rd_we      = (rd != 5'd0);
            imm_sel    = IMM_U;
            fu_type    = FU_ALU;
            alu_select = ALU_PASS_B;
            bsel       = 1'b1;
        end

        OP_AUIPC: begin
            rd_we      = (rd != 5'd0);
            imm_sel    = IMM_U;
            fu_type    = FU_ALU;
            alu_select = ALU_ADD;
            asel       = 1'b1;
            bsel       = 1'b1;
        end

        OP_JAL: begin
            rd_we      = (rd != 5'd0);
            imm_sel    = IMM_J;
            fu_type    = FU_BR;
            alu_select = ALU_ADD;
            asel       = 1'b1;
            bsel       = 1'b1;
            is_jal     = 1'b1;
        end

        OP_JALR: begin
            if (funct3 == 3'b000) begin
                uses_rs1   = 1'b1;
                rd_we      = (rd != 5'd0);
                imm_sel    = IMM_I;
                fu_type    = FU_BR;
                alu_select = ALU_ADD;
                bsel       = 1'b1;
                is_jalr    = 1'b1;
            end else begin
                illegal = 1'b1;
            end
        end

        OP_BRANCH: begin
            uses_rs1   = 1'b1;
            uses_rs2   = 1'b1;
            imm_sel    = IMM_B;
            fu_type    = FU_BR;
            alu_select = ALU_ADD;
            asel       = 1'b1;
            bsel       = 1'b1;
            is_branch  = 1'b1;
            if (funct3 == 3'b010 || funct3 == 3'b011)
                illegal = 1'b1;
        end

        OP_LOAD: begin
            uses_rs1     = 1'b1;
            rd_we        = (rd != 5'd0);
            imm_sel      = IMM_I;
            fu_type      = FU_LOAD;
            alu_select   = ALU_ADD;
            bsel         = 1'b1;
            mem_read     = 1'b1;
            mem_size     = funct3[1:0];
            mem_unsigned = funct3[2];
            if (funct3 == 3'b011 || funct3 == 3'b110 || funct3 == 3'b111)
                illegal = 1'b1;
        end

        OP_LOAD_FP: begin
            is_fp = 1'b1;
            if (fp_disabled || (funct3 != 3'b010)) begin
                illegal = 1'b1;
            end else begin
                uses_rs1     = 1'b1;
                rd_we        = 1'b1; // f0 IS writable!
                rd_is_fp     = 1'b1;
                imm_sel      = IMM_I;
                fu_type      = FU_LOAD;
                bsel         = 1'b1;
                mem_read     = 1'b1;
                mem_size     = MEM_WORD;
                mem_unsigned = 1'b0;
            end
        end

        OP_STORE: begin
            uses_rs1   = 1'b1;
            uses_rs2   = 1'b1;
            imm_sel    = IMM_S;
            fu_type    = FU_STORE;
            alu_select = ALU_ADD;
            bsel       = 1'b1;
            mem_write  = 1'b1;
            mem_size   = funct3[1:0];
            if (funct3 != 3'b000 && funct3 != 3'b001 && funct3 != 3'b010)
                illegal = 1'b1;
        end

        OP_STORE_FP: begin
            is_fp = 1'b1;
            if (fp_disabled || (funct3 != 3'b010)) begin
                illegal = 1'b1;
            end else begin
                uses_rs1   = 1'b1;
                uses_rs2   = 1'b1;
                rs2_is_fp  = 1'b1;
                imm_sel    = IMM_S;
                fu_type    = FU_STORE;
                bsel       = 1'b1;
                mem_write  = 1'b1;
                mem_size   = MEM_WORD;
            end
        end

        OP_OP_IMM: begin
            uses_rs1 = 1'b1;
            rd_we    = (rd != 5'd0);
            imm_sel  = IMM_I;
            fu_type  = FU_ALU;
            bsel     = 1'b1;
            case (funct3)
                3'b000: alu_select = ALU_ADD;
                3'b001: begin
                    alu_select = ALU_SLL;
                    if (funct7 != 7'b0000000) illegal = 1'b1;
                end
                3'b010: alu_select = ALU_SLT;
                3'b011: alu_select = ALU_SLTU;
                3'b100: alu_select = ALU_XOR;
                3'b101: begin
                    if (funct7 == 7'b0100000)      alu_select = ALU_SRA;
                    else if (funct7 == 7'b0000000) alu_select = ALU_SRL;
                    else                           illegal    = 1'b1;
                end
                3'b110: alu_select = ALU_OR;
                3'b111: alu_select = ALU_AND;
                default: illegal = 1'b1;
            endcase
        end

        OP_OP: begin
            uses_rs1 = 1'b1;
            uses_rs2 = 1'b1;
            rd_we    = (rd != 5'd0);
            if (funct7 == 7'b0000001) begin
                if (!funct3[2]) begin
                    fu_type = FU_MUL;
                    mul_op  = funct3[1:0];
                end else begin
                    fu_type = FU_DIV;
                    div_op  = funct3[1:0];
                end
            end else begin
                fu_type = FU_ALU;
                case (funct3)
                    3'b000: begin
                        if (funct7 == 7'b0100000)      alu_select = ALU_SUB;
                        else if (funct7 == 7'b0000000) alu_select = ALU_ADD;
                        else                           illegal    = 1'b1;
                    end
                    3'b001: begin
                        alu_select = ALU_SLL;
                        if (funct7 != 7'b0000000) illegal = 1'b1;
                    end
                    3'b010: begin
                        alu_select = ALU_SLT;
                        if (funct7 != 7'b0000000) illegal = 1'b1;
                    end
                    3'b011: begin
                        alu_select = ALU_SLTU;
                        if (funct7 != 7'b0000000) illegal = 1'b1;
                    end
                    3'b100: begin
                        alu_select = ALU_XOR;
                        if (funct7 != 7'b0000000) illegal = 1'b1;
                    end
                    3'b101: begin
                        if (funct7 == 7'b0100000)      alu_select = ALU_SRA;
                        else if (funct7 == 7'b0000000) alu_select = ALU_SRL;
                        else                           illegal    = 1'b1;
                    end
                    3'b110: begin
                        alu_select = ALU_OR;
                        if (funct7 != 7'b0000000) illegal = 1'b1;
                    end
                    3'b111: begin
                        alu_select = ALU_AND;
                        if (funct7 != 7'b0000000) illegal = 1'b1;
                    end
                    default: illegal = 1'b1;
                endcase
            end
        end

        OP_MADD, OP_MSUB, OP_NMSUB, OP_NMADD: begin
            is_fp = 1'b1;
            if (fp_disabled || (fp_fmt != 2'b00) || rm_illegal) begin
                illegal = 1'b1;
            end else begin
                uses_rs1  = 1'b1;
                uses_rs2  = 1'b1;
                uses_rs3  = 1'b1;
                rs1_is_fp = 1'b1;
                rs2_is_fp = 1'b1;
                rd_is_fp  = 1'b1;
                rd_we     = 1'b1;
                fu_type   = FU_FMA;
                case (opcode)
                    OP_MADD:  fp_op = {1'b0, FMA_OP_FMADD};
                    OP_MSUB:  fp_op = {1'b0, FMA_OP_FMSUB};
                    OP_NMSUB: fp_op = {1'b0, FMA_OP_FNMSUB};
                    default:  fp_op = {1'b0, FMA_OP_FNMADD};
                endcase
            end
        end

        OP_OP_FP: begin
            is_fp = 1'b1;
            if (fp_disabled) begin
                illegal = 1'b1;
            end else begin
                case (funct7)
                    7'b0000000: begin // FADD.S
                        if (rm_illegal) illegal = 1'b1;
                        else begin
                            uses_rs1 = 1'b1; uses_rs2 = 1'b1; rs1_is_fp = 1'b1; rs2_is_fp = 1'b1;
                            rd_we = 1'b1; rd_is_fp = 1'b1; fu_type = FU_FMA; fp_op = {1'b0, FMA_OP_FADD};
                        end
                    end
                    7'b0000100: begin // FSUB.S
                        if (rm_illegal) illegal = 1'b1;
                        else begin
                            uses_rs1 = 1'b1; uses_rs2 = 1'b1; rs1_is_fp = 1'b1; rs2_is_fp = 1'b1;
                            rd_we = 1'b1; rd_is_fp = 1'b1; fu_type = FU_FMA; fp_op = {1'b0, FMA_OP_FSUB};
                        end
                    end
                    7'b0001000: begin // FMUL.S
                        if (rm_illegal) illegal = 1'b1;
                        else begin
                            uses_rs1 = 1'b1; uses_rs2 = 1'b1; rs1_is_fp = 1'b1; rs2_is_fp = 1'b1;
                            rd_we = 1'b1; rd_is_fp = 1'b1; fu_type = FU_FMA; fp_op = {1'b0, FMA_OP_FMUL};
                        end
                    end
                    7'b0001100: begin // FDIV.S
                        if (rm_illegal) illegal = 1'b1;
                        else begin
                            uses_rs1 = 1'b1; uses_rs2 = 1'b1; rs1_is_fp = 1'b1; rs2_is_fp = 1'b1;
                            rd_we = 1'b1; rd_is_fp = 1'b1; fu_type = FU_FDIV; fp_op = {3'd0, FDIV_OP_DIV};
                        end
                    end
                    7'b0101100: begin // FSQRT.S
                        if (rm_illegal || (rs2 != 5'd0)) illegal = 1'b1;
                        else begin
                            uses_rs1 = 1'b1; rs1_is_fp = 1'b1;
                            rd_we = 1'b1; rd_is_fp = 1'b1; fu_type = FU_FDIV; fp_op = {3'd0, FDIV_OP_SQRT};
                        end
                    end
                    7'b0010000: begin // FSGNJ.S / FSGNJN.S / FSGNJX.S
                        uses_rs1 = 1'b1; uses_rs2 = 1'b1; rs1_is_fp = 1'b1; rs2_is_fp = 1'b1;
                        rd_we = 1'b1; rd_is_fp = 1'b1; fu_type = FU_FMISC;
                        case (funct3)
                            3'b000:  fp_op = FMISC_OP_SGNJ;
                            3'b001:  fp_op = FMISC_OP_SGNJN;
                            3'b010:  fp_op = FMISC_OP_SGNJX;
                            default: illegal = 1'b1;
                        endcase
                    end
                    7'b0010100: begin // FMIN.S / FMAX.S
                        uses_rs1 = 1'b1; uses_rs2 = 1'b1; rs1_is_fp = 1'b1; rs2_is_fp = 1'b1;
                        rd_we = 1'b1; rd_is_fp = 1'b1; fu_type = FU_FMISC;
                        case (funct3)
                            3'b000:  fp_op = FMISC_OP_MIN;
                            3'b001:  fp_op = FMISC_OP_MAX;
                            default: illegal = 1'b1;
                        endcase
                    end
                    7'b1100000: begin // FCVT.W.S / FCVT.WU.S (FP -> INT)
                        if (rm_illegal || (rs2 > 5'd1)) illegal = 1'b1;
                        else begin
                            uses_rs1 = 1'b1; rs1_is_fp = 1'b1;
                            rd_we = (rd != 5'd0); rd_is_fp = 1'b0; fu_type = FU_FMISC;
                            fp_op = (rs2 == 5'd0) ? FMISC_OP_CVT_WS : FMISC_OP_CVT_WUS;
                        end
                    end
                    7'b1110000: begin // FMV.X.W / FCLASS.S (FP -> INT)
                        if (rs2 != 5'd0) illegal = 1'b1;
                        else begin
                            uses_rs1 = 1'b1; rs1_is_fp = 1'b1;
                            rd_we = (rd != 5'd0); rd_is_fp = 1'b0; fu_type = FU_FMISC;
                            case (funct3)
                                3'b000:  fp_op = FMISC_OP_MV_XW;
                                3'b001:  fp_op = FMISC_OP_CLASS;
                                default: illegal = 1'b1;
                            endcase
                        end
                    end
                    7'b1010000: begin // FLE.S / FLT.S / FEQ.S (FP -> INT)
                        uses_rs1 = 1'b1; uses_rs2 = 1'b1; rs1_is_fp = 1'b1; rs2_is_fp = 1'b1;
                        rd_we = (rd != 5'd0); rd_is_fp = 1'b0; fu_type = FU_FMISC;
                        case (funct3)
                            3'b000:  fp_op = FMISC_OP_LE;
                            3'b001:  fp_op = FMISC_OP_LT;
                            3'b010:  fp_op = FMISC_OP_EQ;
                            default: illegal = 1'b1;
                        endcase
                    end
                    7'b1101000: begin // FCVT.S.W / FCVT.S.WU (INT -> FP)
                        if (rm_illegal || (rs2 > 5'd1)) illegal = 1'b1;
                        else begin
                            uses_rs1 = 1'b1; rs1_is_fp = 1'b0;
                            rd_we = 1'b1; rd_is_fp = 1'b1; fu_type = FU_FMISC;
                            fp_op = (rs2 == 5'd0) ? FMISC_OP_CVT_SW : FMISC_OP_CVT_SWU;
                        end
                    end
                    7'b1111000: begin // FMV.W.X (INT -> FP)
                        if ((rs2 != 5'd0) || (funct3 != 3'b000)) illegal = 1'b1;
                        else begin
                            uses_rs1 = 1'b1; rs1_is_fp = 1'b0;
                            rd_we = 1'b1; rd_is_fp = 1'b1; fu_type = FU_FMISC;
                            fp_op = FMISC_OP_MV_WX;
                        end
                    end
                    default: illegal = 1'b1;
                endcase
            end
        end

        OP_MISC_MEM: begin
            fu_type = FU_SYSTEM;
            if (funct3 == 3'b000) begin
                is_fence = 1'b1;
            end else if (funct3 == 3'b001) begin
                is_fence_i = 1'b1;
            end else begin
                illegal = 1'b1;
            end
        end

        OP_AMO: begin
            if (funct3 != 3'b010) begin
                illegal = 1'b1;
            end else begin
                fu_type  = FU_AMO;
                is_amo   = 1'b1;
                uses_rs1 = 1'b1;
                rd_we    = (rd != 5'd0);
                mem_size = MEM_WORD;
                case (funct5)
                    F5_LR: begin
                        is_lr    = 1'b1;
                        amo_op   = AMO_LR;
                        mem_read = 1'b1;
                        if (rs2 != 5'd0)
                            illegal = 1'b1;
                    end
                    F5_SC: begin
                        is_sc     = 1'b1;
                        uses_rs2  = 1'b1;
                        amo_op    = AMO_SC;
                        mem_write = 1'b1;
                    end
                    F5_AMOSWAP: begin
                        uses_rs2  = 1'b1; amo_op = AMO_SWAP; mem_read = 1'b1; mem_write = 1'b1;
                    end
                    F5_AMOADD: begin
                        uses_rs2  = 1'b1; amo_op = AMO_ADD;  mem_read = 1'b1; mem_write = 1'b1;
                    end
                    F5_AMOXOR: begin
                        uses_rs2  = 1'b1; amo_op = AMO_XOR;  mem_read = 1'b1; mem_write = 1'b1;
                    end
                    F5_AMOAND: begin
                        uses_rs2  = 1'b1; amo_op = AMO_AND;  mem_read = 1'b1; mem_write = 1'b1;
                    end
                    F5_AMOOR: begin
                        uses_rs2  = 1'b1; amo_op = AMO_OR;   mem_read = 1'b1; mem_write = 1'b1;
                    end
                    F5_AMOMIN: begin
                        uses_rs2  = 1'b1; amo_op = AMO_MIN;  mem_read = 1'b1; mem_write = 1'b1;
                    end
                    F5_AMOMAX: begin
                        uses_rs2  = 1'b1; amo_op = AMO_MAX;  mem_read = 1'b1; mem_write = 1'b1;
                    end
                    F5_AMOMINU: begin
                        uses_rs2  = 1'b1; amo_op = AMO_MINU; mem_read = 1'b1; mem_write = 1'b1;
                    end
                    F5_AMOMAXU: begin
                        uses_rs2  = 1'b1; amo_op = AMO_MAXU; mem_read = 1'b1; mem_write = 1'b1;
                    end
                    default: begin
                        is_amo  = 1'b0;
                        rd_we   = 1'b0;
                        illegal = 1'b1;
                    end
                endcase
            end
        end

        OP_SYSTEM: begin
            fu_type = FU_SYSTEM;
            if (instr == INSTR_EBREAK) begin
                is_ebreak = 1'b1;
            end else if (instr == INSTR_ECALL) begin
                is_ecall = 1'b1;
            end else if (instr == INSTR_MRET) begin
                is_mret = 1'b1;
            end else begin
                case (funct3)
                    CSR_OP_RW, CSR_OP_RS, CSR_OP_RC: begin
                        is_csr      = 1'b1;
                        uses_rs1    = 1'b1;
                        rd_we       = (rd != 5'd0);
                        csr_op      = funct3;
                        csr_use_imm = 1'b0;
                    end
                    CSR_OP_RWI, CSR_OP_RSI, CSR_OP_RCI: begin
                        is_csr      = 1'b1;
                        uses_rs1    = 1'b0;
                        rd_we       = (rd != 5'd0);
                        csr_op      = funct3;
                        csr_use_imm = 1'b1;
                    end
                    default: begin
                        illegal = 1'b1;
                    end
                endcase
            end
        end

        default: begin
            illegal = 1'b1;
        end
    endcase

    // Safe non-mutating clamp on illegal instructions so speculative illegal ops
    // never allocate RS/LSQ/RAT checkpoints and squash cleanly with zero side effects.
    if (illegal) begin
        uses_rs1     = 1'b0;
        uses_rs2     = 1'b0;
        uses_rs3     = 1'b0;
        rs1_is_fp    = 1'b0;
        rs2_is_fp    = 1'b0;
        rd_is_fp     = 1'b0;
        rd_we        = 1'b0;
        fu_type      = FU_SYSTEM;
        is_branch    = 1'b0;
        is_jal       = 1'b0;
        is_jalr      = 1'b0;
        mem_read     = 1'b0;
        mem_write    = 1'b0;
        is_ebreak    = 1'b0;
        is_ecall     = 1'b0;
        is_mret      = 1'b0;
        is_fence     = 1'b0;
        is_fence_i   = 1'b0;
        is_csr       = 1'b0;
        is_amo       = 1'b0;
        is_lr        = 1'b0;
        is_sc        = 1'b0;
        amo_op       = AMO_NONE;
        is_fp        = 1'b0;
    end
end

endmodule
