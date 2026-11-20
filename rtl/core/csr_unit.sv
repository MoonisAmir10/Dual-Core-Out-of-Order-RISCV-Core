`timescale 1ns / 1ps

// ============================================================================
// Minimal M-Mode / U-Mode Zicsr Unit for Multi-Core RV32IMA OoO Processor
//
// Supported CSRs:
//   - 0xF14 mhartid   (RO): Hardware Thread / Core ID (parameter CORE_ID)
//   - 0xF11 mvendorid (RO): 0x0000_0000 (non-commercial)
//   - 0xF12 marchid   (RO): 0x0000_2002 (Dual-Core 2W OoO ID)
//   - 0xF13 mimpid    (RO): 0x0001_0000 (v1.0)
//   - 0x340 mscratch  (RW): General-purpose per-core scratch register
//   - 0xB00/0xB80 mcycle / mcycleh     (RW): 64-bit live cycle counter
//   - 0xB02/0xB82 minstret / minstreth (RW): 64-bit live 2-wide commit counter
//   - 0xC00/0xC80 cycle / cycleh       (RO): User-mode shadow of mcycle
//   - 0xC02/0xC82 instret / instreth   (RO): User-mode shadow of minstret
//
// Spec Compliance:
//   - CSRRS/CSRRC with rs1==x0 (or CSRRSI/CSRRCI with uimm==0) perform a pure
//     read without writing, allowing legal reads of read-only CSRs (mhartid!).
//   - Attempting to write any read-only CSR (csr_addr[11:10]==2'b11) or
//     accessing an unimplemented CSR asserts csr_illegal and performs no write.
// ============================================================================
module csr_unit #(
    parameter [31:0] CORE_ID = 32'd0
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        halted,

    // 2-Wide Retirement Pulse (0, 1, or 2 instructions committed this cycle)
    input  wire [1:0]  retire_cnt,

    // Non-speculative CSR Access Port (executed cleanly at ROB head)
    input  wire        csr_req_valid,
    input  wire [2:0]  csr_op,
    input  wire [11:0] csr_addr,
    input  wire [4:0]  csr_rs1_uimm, // rs1 index (or 5-bit uimm for *I ops)
    input  wire [31:0] csr_rs1_val,  // register operand value for non-immediate ops

    output reg  [31:0] csr_rdata,
    output wire        csr_illegal,

    // Live observability outputs
    output wire [63:0] mcycle_out,
    output wire [63:0] minstret_out
);

`include "rv32ima_defs.vh"

reg [63:0] mcycle_r;
reg [63:0] minstret_r;
reg [31:0] mscratch_r;

assign mcycle_out   = mcycle_r;
assign minstret_out = minstret_r;

// Combinational CSR Read Mux & Address Decode
reg csr_addr_valid;
always @(*) begin
    csr_addr_valid = 1'b1;
    case (csr_addr)
        CSR_MSCRATCH:  csr_rdata = mscratch_r;
        CSR_MCYCLE:    csr_rdata = mcycle_r[31:0];
        CSR_MCYCLEH:   csr_rdata = mcycle_r[63:32];
        CSR_MINSTRET:  csr_rdata = minstret_r[31:0];
        CSR_MINSTRETH: csr_rdata = minstret_r[63:32];
        CSR_CYCLE:     csr_rdata = mcycle_r[31:0];
        CSR_CYCLEH:    csr_rdata = mcycle_r[63:32];
        CSR_INSTRET:   csr_rdata = minstret_r[31:0];
        CSR_INSTRETH:  csr_rdata = minstret_r[63:32];
        CSR_MVENDORID: csr_rdata = 32'h0000_0000;
        CSR_MARCHID:   csr_rdata = 32'h0000_2002;
        CSR_MIMPID:    csr_rdata = 32'h0001_0000;
        CSR_MHARTID:   csr_rdata = CORE_ID;
        default: begin
            csr_rdata      = 32'd0;
            csr_addr_valid = 1'b0;
        end
    endcase
end

// Determine operand, write-intent, and legality per RISC-V Privileged Spec §2.1
wire        is_imm_op   = csr_op[2];
wire [1:0]  op_kind     = csr_op[1:0]; // 01: RW, 10: RS, 11: RC
wire [31:0] csr_operand = is_imm_op ? {27'd0, csr_rs1_uimm} : csr_rs1_val;
wire        op_valid    = (op_kind == 2'b01) || (op_kind == 2'b10) || (op_kind == 2'b11);
wire        wants_write = (op_kind == 2'b01) || (csr_rs1_uimm != 5'd0);
wire        is_ro_csr   = (csr_addr[11:10] == 2'b11);

assign csr_illegal = csr_req_valid && (!op_valid || !csr_addr_valid || (wants_write && is_ro_csr));

wire        do_write = csr_req_valid && wants_write && !csr_illegal;

reg [31:0] csr_wdata;
always @(*) begin
    case (op_kind)
        2'b01:   csr_wdata = csr_operand;                // CSRRW / CSRRWI
        2'b10:   csr_wdata = csr_rdata | csr_operand;    // CSRRS / CSRRSI
        2'b11:   csr_wdata = csr_rdata & (~csr_operand); // CSRRC / CSRRCI
        default: csr_wdata = csr_rdata;
    endcase
end

wire [63:0] mcycle_inc   = halted ? mcycle_r : (mcycle_r + 64'd1);
wire [63:0] minstret_inc = minstret_r + {62'd0, retire_cnt};

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        mcycle_r   <= 64'd0;
        minstret_r <= 64'd0;
        mscratch_r <= 32'd0;
    end else begin
        // Default live counter increments
        mcycle_r   <= mcycle_inc;
        minstret_r <= minstret_inc;

        // Explicit CSR write overrides counter increment for the written half
        if (do_write) begin
            case (csr_addr)
                CSR_MSCRATCH:  mscratch_r        <= csr_wdata;
                CSR_MCYCLE:    mcycle_r[31:0]    <= csr_wdata;
                CSR_MCYCLEH:   mcycle_r[63:32]   <= csr_wdata;
                CSR_MINSTRET:  minstret_r[31:0]  <= csr_wdata;
                CSR_MINSTRETH: minstret_r[63:32] <= csr_wdata;
                default: ;
            endcase
        end
    end
end

endmodule
