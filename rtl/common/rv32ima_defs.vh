// ============================================================================
// RV32IMA_Zicsr_Zifencei Shared Architectural & Microarchitectural Definitions
// Included inside module scopes (no global include guard so every module gets
// its own localparams across Icarus Verilog, Verilator 5.054, and Yosys 0.71).
// ============================================================================

// ----------------------------------------------------------------------------
// RISC-V RV32IMA_Zicsr_Zifencei Major Opcodes (instr[6:0])
// ----------------------------------------------------------------------------
localparam [6:0] OP_LUI      = 7'b0110111;
localparam [6:0] OP_AUIPC    = 7'b0010111;
localparam [6:0] OP_JAL      = 7'b1101111;
localparam [6:0] OP_JALR     = 7'b1100111;
localparam [6:0] OP_BRANCH   = 7'b1100011;
localparam [6:0] OP_LOAD     = 7'b0000011;
localparam [6:0] OP_STORE    = 7'b0100011;
localparam [6:0] OP_OP_IMM   = 7'b0010011;
localparam [6:0] OP_OP       = 7'b0110011;
localparam [6:0] OP_MISC_MEM = 7'b0001111; // FENCE, FENCE.I
localparam [6:0] OP_AMO      = 7'b0101111; // RV32A: LR.W, SC.W, AMO*.W
localparam [6:0] OP_SYSTEM   = 7'b1110011; // EBREAK, CSRR*

localparam [31:0] INSTR_NOP    = 32'h0000_0013; // addi x0, x0, 0
localparam [31:0] INSTR_EBREAK = 32'h0010_0073; // ebreak (clean testbench halt)

// ----------------------------------------------------------------------------
// ALU Operation Codes (alu_select[3:0])
// ----------------------------------------------------------------------------
localparam [3:0] ALU_ADD    = 4'b0000;
localparam [3:0] ALU_SUB    = 4'b0001;
localparam [3:0] ALU_SLT    = 4'b0010;
localparam [3:0] ALU_SRA    = 4'b0011;
localparam [3:0] ALU_SLL    = 4'b0100;
localparam [3:0] ALU_SRL    = 4'b0101;
localparam [3:0] ALU_OR     = 4'b0110;
localparam [3:0] ALU_AND    = 4'b0111;
localparam [3:0] ALU_XOR    = 4'b1000;
localparam [3:0] ALU_SLTU   = 4'b1001;
localparam [3:0] ALU_PASS_B = 4'b1010; // LUI passthrough

// ----------------------------------------------------------------------------
// RV32M Sub-Operations (matches funct3[2:0] when funct7 == 7'b0000001)
// ----------------------------------------------------------------------------
localparam [1:0] MUL_OP_MUL    = 2'b00; // funct3 = 000
localparam [1:0] MUL_OP_MULH   = 2'b01; // funct3 = 001
localparam [1:0] MUL_OP_MULHSU = 2'b10; // funct3 = 010
localparam [1:0] MUL_OP_MULHU  = 2'b11; // funct3 = 011

localparam [1:0] DIV_OP_DIV    = 2'b00; // funct3 = 100
localparam [1:0] DIV_OP_DIVU   = 2'b01; // funct3 = 101
localparam [1:0] DIV_OP_REM    = 2'b10; // funct3 = 110
localparam [1:0] DIV_OP_REMU   = 2'b11; // funct3 = 111

// ----------------------------------------------------------------------------
// RV32A Atomic Operation Encodings (compact 4-bit internal amo_op[3:0])
// ----------------------------------------------------------------------------
localparam [3:0] AMO_NONE = 4'd0;
localparam [3:0] AMO_LR   = 4'd1;
localparam [3:0] AMO_SC   = 4'd2;
localparam [3:0] AMO_SWAP = 4'd3;
localparam [3:0] AMO_ADD  = 4'd4;
localparam [3:0] AMO_XOR  = 4'd5;
localparam [3:0] AMO_AND  = 4'd6;
localparam [3:0] AMO_OR   = 4'd7;
localparam [3:0] AMO_MIN  = 4'd8;
localparam [3:0] AMO_MAX  = 4'd9;
localparam [3:0] AMO_MINU = 4'd10;
localparam [3:0] AMO_MAXU = 4'd11;

// RV32A Official funct5 Encodings (instr[31:27])
localparam [4:0] F5_AMOADD  = 5'b00000;
localparam [4:0] F5_AMOSWAP = 5'b00001;
localparam [4:0] F5_LR      = 5'b00010;
localparam [4:0] F5_SC      = 5'b00011;
localparam [4:0] F5_AMOXOR  = 5'b00100;
localparam [4:0] F5_AMOOR   = 5'b01000;
localparam [4:0] F5_AMOAND  = 5'b01100;
localparam [4:0] F5_AMOMIN  = 5'b10000;
localparam [4:0] F5_AMOMAX  = 5'b10100;
localparam [4:0] F5_AMOMINU = 5'b11000;
localparam [4:0] F5_AMOMAXU = 5'b11100;

// ----------------------------------------------------------------------------
// Zicsr Operations (funct3[2:0] when opcode == OP_SYSTEM) & CSR Addresses
// ----------------------------------------------------------------------------
localparam [2:0] CSR_OP_RW  = 3'b001;
localparam [2:0] CSR_OP_RS  = 3'b010;
localparam [2:0] CSR_OP_RC  = 3'b011;
localparam [2:0] CSR_OP_RWI = 3'b101;
localparam [2:0] CSR_OP_RSI = 3'b110;
localparam [2:0] CSR_OP_RCI = 3'b111;

localparam [11:0] CSR_MSCRATCH  = 12'h340; // M-mode scratch (RW)
localparam [11:0] CSR_MCYCLE    = 12'hB00; // M-mode cycle counter low (RW)
localparam [11:0] CSR_MINSTRET  = 12'hB02; // M-mode instret counter low (RW)
localparam [11:0] CSR_MCYCLEH   = 12'hB80; // M-mode cycle counter high (RW)
localparam [11:0] CSR_MINSTRETH = 12'hB82; // M-mode instret counter high (RW)
localparam [11:0] CSR_CYCLE     = 12'hC00; // U-mode cycle shadow low (RO)
localparam [11:0] CSR_INSTRET   = 12'hC02; // U-mode instret shadow low (RO)
localparam [11:0] CSR_CYCLEH    = 12'hC80; // U-mode cycle shadow high (RO)
localparam [11:0] CSR_INSTRETH  = 12'hC82; // U-mode instret shadow high (RO)
localparam [11:0] CSR_MVENDORID = 12'hF11; // Vendor ID (RO)
localparam [11:0] CSR_MARCHID   = 12'hF12; // Architecture ID (RO)
localparam [11:0] CSR_MIMPID    = 12'hF13; // Implementation ID (RO)
localparam [11:0] CSR_MHARTID   = 12'hF14; // Hardware Thread / Core ID (RO)

// ----------------------------------------------------------------------------
// Memory Access Width (matches funct3[1:0] for LOAD/STORE)
// ----------------------------------------------------------------------------
localparam [1:0] MEM_BYTE = 2'b00;
localparam [1:0] MEM_HALF = 2'b01;
localparam [1:0] MEM_WORD = 2'b10;

// ----------------------------------------------------------------------------
// Immediate Format Select (imm_sel[2:0])
// ----------------------------------------------------------------------------
localparam [2:0] IMM_I = 3'd0;
localparam [2:0] IMM_S = 3'd1;
localparam [2:0] IMM_B = 3'd2;
localparam [2:0] IMM_U = 3'd3;
localparam [2:0] IMM_J = 3'd4;

// ----------------------------------------------------------------------------
// Functional Unit Routing Class (fu_type[2:0])
// ----------------------------------------------------------------------------
localparam [2:0] FU_ALU    = 3'd0;
localparam [2:0] FU_MUL    = 3'd1;
localparam [2:0] FU_DIV    = 3'd2;
localparam [2:0] FU_BR     = 3'd3; // Branches, JAL, JALR
localparam [2:0] FU_LOAD   = 3'd4;
localparam [2:0] FU_STORE  = 3'd5;
localparam [2:0] FU_SYSTEM = 3'd6; // EBREAK, CSRR*, FENCE, FENCE.I
localparam [2:0] FU_AMO    = 3'd7; // RV32A: LR.W, SC.W, AMO*.W

// ----------------------------------------------------------------------------
// Dual-Core 2-Wide OoO Default Sizing Parameters
// ----------------------------------------------------------------------------
localparam integer ROB_ENTRIES    = 32;
localparam integer ROB_IDX_W      = 5;  // $clog2(ROB_ENTRIES)
localparam integer RS_ALU_ENTRIES = 8;
localparam integer RS_MUL_ENTRIES = 4;
localparam integer RS_DIV_ENTRIES = 2;
localparam integer RS_BR_ENTRIES  = 4;
localparam integer LQ_ENTRIES     = 8;
localparam integer SQ_ENTRIES     = 8;
localparam integer BTB_ENTRIES    = 32;
localparam integer BHT_ENTRIES    = 64;
