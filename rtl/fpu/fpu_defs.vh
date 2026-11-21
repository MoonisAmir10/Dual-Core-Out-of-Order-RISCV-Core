// ============================================================================
// IEEE-754-2008 / RISC-V RV32F Single-Precision FPU Definitions (Phase U5)
// Included inside module scopes (no include guard so every module gets its
// own localparams across Icarus Verilog, Verilator 5.054, and Yosys 0.71).
// ============================================================================

// ----------------------------------------------------------------------------
// IEEE-754 / RISC-V Rounding Modes (frm[2:0] / instr[14:12])
// ----------------------------------------------------------------------------
localparam [2:0] FPU_RM_RNE = 3'b000; // Round to Nearest, ties to Even
localparam [2:0] FPU_RM_RTZ = 3'b001; // Round towards Zero
localparam [2:0] FPU_RM_RDN = 3'b010; // Round Down (towards -Inf)
localparam [2:0] FPU_RM_RUP = 3'b011; // Round Up   (towards +Inf)
localparam [2:0] FPU_RM_RMM = 3'b100; // Round to Nearest, ties to Max Magnitude
localparam [2:0] FPU_RM_DYN = 3'b111; // Dynamic (use fcsr.frm)

// ----------------------------------------------------------------------------
// IEEE-754 / RISC-V Exception Flag Bit Positions (fflags[4:0])
// ----------------------------------------------------------------------------
localparam integer FPU_FLAG_NX = 0; // Inexact
localparam integer FPU_FLAG_UF = 1; // Underflow (tininess AFTER rounding)
localparam integer FPU_FLAG_OF = 2; // Overflow
localparam integer FPU_FLAG_DZ = 3; // Divide by Zero
localparam integer FPU_FLAG_NV = 4; // Invalid Operation

// Canonical Quiet NaN mandated by RISC-V F Extension
localparam [31:0] FPU_CANON_QNAN = 32'h7FC0_0000;

// ----------------------------------------------------------------------------
// FPU FMA Pipeline Operations (fpu_fma_pipe.in_op[2:0])
// ----------------------------------------------------------------------------
localparam [2:0] FMA_OP_FMADD  = 3'd0; // +(a * b) + c
localparam [2:0] FMA_OP_FMSUB  = 3'd1; // +(a * b) - c
localparam [2:0] FMA_OP_FNMSUB = 3'd2; // -(a * b) + c
localparam [2:0] FMA_OP_FNMADD = 3'd3; // -(a * b) - c
localparam [2:0] FMA_OP_FADD   = 3'd4; // a + b  (steered as a * 1.0 + b)
localparam [2:0] FMA_OP_FSUB   = 3'd5; // a - b  (steered as a * 1.0 - b)
localparam [2:0] FMA_OP_FMUL   = 3'd6; // a * b  (steered as a * b + signed_0)

// ----------------------------------------------------------------------------
// FPU Iterative Div/Sqrt Operations (fpu_div_sqrt.in_op)
// ----------------------------------------------------------------------------
localparam       FDIV_OP_DIV   = 1'b0; // FDIV.S  (a / b)
localparam       FDIV_OP_SQRT  = 1'b1; // FSQRT.S (sqrt(a))

// ----------------------------------------------------------------------------
// FPU Misc / Compare / Convert Operations (fpu_misc.in_op[3:0])
// ----------------------------------------------------------------------------
localparam [3:0] FMISC_OP_SGNJ    = 4'd0;  // FSGNJ.S
localparam [3:0] FMISC_OP_SGNJN   = 4'd1;  // FSGNJN.S
localparam [3:0] FMISC_OP_SGNJX   = 4'd2;  // FSGNJX.S
localparam [3:0] FMISC_OP_MIN     = 4'd3;  // FMIN.S
localparam [3:0] FMISC_OP_MAX     = 4'd4;  // FMAX.S
localparam [3:0] FMISC_OP_EQ      = 4'd5;  // FEQ.S     (writes int rd)
localparam [3:0] FMISC_OP_LT      = 4'd6;  // FLT.S     (writes int rd)
localparam [3:0] FMISC_OP_LE      = 4'd7;  // FLE.S     (writes int rd)
localparam [3:0] FMISC_OP_CLASS   = 4'd8;  // FCLASS.S  (writes int rd)
localparam [3:0] FMISC_OP_CVT_WS  = 4'd9;  // FCVT.W.S  (float -> int32)
localparam [3:0] FMISC_OP_CVT_WUS = 4'd10; // FCVT.WU.S (float -> uint32)
localparam [3:0] FMISC_OP_CVT_SW  = 4'd11; // FCVT.S.W  (int32 -> float)
localparam [3:0] FMISC_OP_CVT_SWU = 4'd12; // FCVT.S.WU (uint32 -> float)
localparam [3:0] FMISC_OP_MV_XW   = 4'd13; // FMV.X.W   (raw f -> x)
localparam [3:0] FMISC_OP_MV_WX   = 4'd14; // FMV.W.X   (raw x -> f)
