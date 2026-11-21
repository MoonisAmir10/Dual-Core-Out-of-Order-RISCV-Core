`timescale 1ns / 1ps

// ============================================================================
// Full M-Mode / U-Mode Zicsr, Trap/Interrupt & RV32F FCSR Unit (Phase U1 + U6)
//
// Supported CSRs:
//   - 0x001 fflags    (RW, MISA_EXT_F=1 & FS!=0): FP accrued exceptions [4:0]
//   - 0x002 frm       (RW, MISA_EXT_F=1 & FS!=0): FP dynamic rounding mode [2:0]
//   - 0x003 fcsr      (RW, MISA_EXT_F=1 & FS!=0): {24'd0, frm[2:0], fflags[4:0]}
//   - 0x300 mstatus   (RW): MIE[3], MPIE[7], MPP[12:11]=2'b11, FS[14:13]
//   - 0x301 misa      (RW-WARL): RV32IMA (+F when MISA_EXT_F=1)
//   - 0x304 mie       (RW): MSIE[3], MTIE[7], MEIE[11]
//   - 0x305 mtvec     (RW): BASE[31:2], MODE[1:0] (00=Direct, 01=Vectored)
//   - 0x340 mscratch  (RW): General-purpose per-core scratch register
//   - 0x341 mepc      (RW): Exception PC (bits [1:0] hardwired 2'b00)
//   - 0x342 mcause    (RW): Trap cause ({interrupt, 27'd0, cause[3:0]})
//   - 0x343 mtval     (RW): Faulting address or raw illegal instruction
//   - 0x344 mip       (RW-WARL): Live HW interrupt pending (MEIP[11], MTIP[7], MSIP[3])
//   - 0xB00/0xB80 mcycle / mcycleh     (RW): 64-bit live cycle counter
//   - 0xB02/0xB82 minstret / minstreth (RW): 64-bit live 2-wide commit counter
//   - 0xC00/0xC80 cycle / cycleh       (RO): User-mode shadow of mcycle
//   - 0xC02/0xC82 instret / instreth   (RO): User-mode shadow of minstret
//   - 0xF11..0xF14 mvendorid, marchid, mimpid, mhartid (RO)
// ============================================================================
module csr_unit #(
    parameter [31:0] CORE_ID    = 32'd0,
    parameter [0:0]  MISA_EXT_F = 1'b0
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        halted,

    // 2-Wide Retirement Pulse (0, 1, or 2 instructions committed this cycle)
    input  wire [1:0]  retire_cnt,

    // In-Order Non-Blocking FP Retirement Accrued Flags & FS Dirty Update (Phase U6)
    input  wire        fp_commit_dirty,
    input  wire [4:0]  fp_commit_fflags,

    // Non-speculative CSR Access Port (executed cleanly at ROB head)
    input  wire        csr_req_valid,
    input  wire [2:0]  csr_op,
    input  wire [11:0] csr_addr,
    input  wire [4:0]  csr_rs1_uimm, // rs1 index (or 5-bit uimm for *I ops)
    input  wire [31:0] csr_rs1_val,  // register operand value for non-immediate ops

    output reg  [31:0] csr_rdata,
    output wire        csr_access_illegal, // combinational legality (independent of csr_req_valid)
    output wire        csr_illegal,        // csr_req_valid && csr_access_illegal
    output wire        csr_sync_required,  // 1 when a legal CSR write mutates mstatus/frm/fcsr!

    // External Hardware Interrupt Lines (CLINT + PLIC)
    input  wire        irq_msip,
    input  wire        irq_mtip,
    input  wire        irq_meip,

    // Precise ROB-Head Trap Entry & MRET Control
    input  wire        trap_enter,
    input  wire        trap_is_irq,
    input  wire [3:0]  trap_cause,
    input  wire [31:0] trap_pc,
    input  wire [31:0] trap_tval,
    input  wire        mret_exec,

    // Live Trap/Interrupt & FP Status Outputs -> ROB / Decoder / Core
    output wire [31:0] mtvec_out,
    output wire [31:0] mepc_out,
    output wire        irq_pending,
    output wire [3:0]  irq_cause,
    output wire [1:0]  mstatus_fs_out,
    output wire [2:0]  frm_out,
    output wire [4:0]  fflags_out,

    // Live counter observability outputs
    output wire [63:0] mcycle_out,
    output wire [63:0] minstret_out
);

`include "rv32ima_defs.vh"

// M-Mode Trap, Status & FP Control Registers
reg        mstatus_mie_r;
reg        mstatus_mpie_r;
reg [1:0]  mstatus_fs_r;
reg [4:0]  fflags_r;
reg [2:0]  frm_r;
reg        mie_msie_r;
reg        mie_mtie_r;
reg        mie_meie_r;
reg [31:0] mtvec_r;
reg [31:0] mscratch_r;
reg [31:0] mepc_r;
reg [31:0] mcause_r;
reg [31:0] mtval_r;

reg [63:0] mcycle_r;
reg [63:0] minstret_r;

wire [31:0] mstatus_val = {17'd0, mstatus_fs_r, 2'b11, 3'd0, mstatus_mpie_r, 3'd0, mstatus_mie_r, 3'd0};
wire [31:0] misa_val    = 32'h4000_1101 | (MISA_EXT_F ? 32'h0000_0020 : 32'h0000_0000);
wire [31:0] mie_val     = {20'd0, mie_meie_r, 3'd0, mie_mtie_r, 3'd0, mie_msie_r, 3'd0};
wire [31:0] mip_val     = {20'd0, irq_meip,   3'd0, irq_mtip,   3'd0, irq_msip,   3'd0};

assign mtvec_out      = mtvec_r;
assign mepc_out       = mepc_r;
assign mstatus_fs_out = mstatus_fs_r;
assign frm_out        = frm_r;
assign fflags_out     = fflags_r;
assign mcycle_out     = mcycle_r;
assign minstret_out   = minstret_r;

// Live M-mode Interrupt Pending & Priority Encoder (MEI > MSI > MTI per Priv Spec §3.1.9)
wire mei_pend = irq_meip && mie_meie_r;
wire msi_pend = irq_msip && mie_msie_r;
wire mti_pend = irq_mtip && mie_mtie_r;

assign irq_pending = mstatus_mie_r && (mei_pend || msi_pend || mti_pend);
assign irq_cause   = mei_pend ? MCAUSE_IRQ_MEI :
                     msi_pend ? MCAUSE_IRQ_MSI :
                                MCAUSE_IRQ_MTI;

// Combinational CSR Read Mux & Address Decode
wire fp_csr_enabled = MISA_EXT_F && (mstatus_fs_r != 2'b00);

reg csr_addr_valid;
always @(*) begin
    csr_addr_valid = 1'b1;
    case (csr_addr)
        CSR_FFLAGS: begin
            csr_rdata      = {27'd0, fflags_r};
            csr_addr_valid = fp_csr_enabled;
        end
        CSR_FRM: begin
            csr_rdata      = {29'd0, frm_r};
            csr_addr_valid = fp_csr_enabled;
        end
        CSR_FCSR: begin
            csr_rdata      = {24'd0, frm_r, fflags_r};
            csr_addr_valid = fp_csr_enabled;
        end
        CSR_MSTATUS:   csr_rdata = mstatus_val;
        CSR_MISA:      csr_rdata = misa_val;
        CSR_MIE:       csr_rdata = mie_val;
        CSR_MTVEC:     csr_rdata = mtvec_r;
        CSR_MSCRATCH:  csr_rdata = mscratch_r;
        CSR_MEPC:      csr_rdata = mepc_r;
        CSR_MCAUSE:    csr_rdata = mcause_r;
        CSR_MTVAL:     csr_rdata = mtval_r;
        CSR_MIP:       csr_rdata = mip_val;
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

assign csr_access_illegal = (!op_valid || !csr_addr_valid || (wants_write && is_ro_csr));
assign csr_illegal        = csr_req_valid && csr_access_illegal;

// Any legal CSR write to mstatus, frm, or fcsr flushes younger speculative FP ops at commit
// so younger instructions re-decode with the exact new mstatus.FS and frm!
assign csr_sync_required  = wants_write && !csr_access_illegal &&
                            ((csr_addr == CSR_MSTATUS) || (csr_addr == CSR_FRM) || (csr_addr == CSR_FCSR));

wire        do_write = csr_req_valid && wants_write && !csr_access_illegal;

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
        mstatus_mie_r  <= 1'b0;
        mstatus_mpie_r <= 1'b0;
        mstatus_fs_r   <= 2'b01; // Initial (FP enabled on boot per bare-metal convention)
        fflags_r       <= 5'd0;
        frm_r          <= 3'd0;  // RNE default
        mie_msie_r     <= 1'b0;
        mie_mtie_r     <= 1'b0;
        mie_meie_r     <= 1'b0;
        mtvec_r        <= 32'd0;
        mscratch_r     <= 32'd0;
        mepc_r         <= 32'd0;
        mcause_r       <= 32'd0;
        mtval_r        <= 32'd0;
        mcycle_r       <= 64'd0;
        minstret_r     <= 64'd0;
    end else begin
        // Default live counter & non-blocking FP commit updates
        mcycle_r   <= mcycle_inc;
        minstret_r <= minstret_inc;
        if (MISA_EXT_F) begin
            if (fp_commit_fflags != 5'd0)
                fflags_r <= fflags_r | fp_commit_fflags;
            if (fp_commit_dirty && (mstatus_fs_r != 2'b00))
                mstatus_fs_r <= 2'b11; // Dirty!
        end

        // Atomic Trap Entry > MRET > Explicit CSR Write
        if (trap_enter) begin
            mepc_r         <= {trap_pc[31:2], 2'b00};
            mcause_r       <= {trap_is_irq, 27'd0, trap_cause};
            mtval_r        <= trap_tval;
            mstatus_mpie_r <= mstatus_mie_r;
            mstatus_mie_r  <= 1'b0;
        end else if (mret_exec) begin
            mstatus_mie_r  <= mstatus_mpie_r;
            mstatus_mpie_r <= 1'b1;
        end else if (do_write) begin
            case (csr_addr)
                CSR_FFLAGS: begin
                    fflags_r     <= csr_wdata[4:0];
                    mstatus_fs_r <= 2'b11;
                end
                CSR_FRM: begin
                    frm_r        <= csr_wdata[2:0];
                    mstatus_fs_r <= 2'b11;
                end
                CSR_FCSR: begin
                    fflags_r     <= csr_wdata[4:0];
                    frm_r        <= csr_wdata[7:5];
                    mstatus_fs_r <= 2'b11;
                end
                CSR_MSTATUS: begin
                    mstatus_mie_r  <= csr_wdata[3];
                    mstatus_mpie_r <= csr_wdata[7];
                    mstatus_fs_r   <= csr_wdata[14:13];
                end
                CSR_MIE: begin
                    mie_msie_r <= csr_wdata[3];
                    mie_mtie_r <= csr_wdata[7];
                    mie_meie_r <= csr_wdata[11];
                end
                CSR_MTVEC:     mtvec_r           <= {csr_wdata[31:2], 1'b0, csr_wdata[0]};
                CSR_MSCRATCH:  mscratch_r        <= csr_wdata;
                CSR_MEPC:      mepc_r            <= {csr_wdata[31:2], 2'b00};
                CSR_MCAUSE:    mcause_r          <= csr_wdata;
                CSR_MTVAL:     mtval_r           <= csr_wdata;
                CSR_MCYCLE:    mcycle_r[31:0]    <= csr_wdata;
                CSR_MCYCLEH:   mcycle_r[63:32]   <= csr_wdata;
                CSR_MINSTRET:  minstret_r[31:0]  <= csr_wdata;
                CSR_MINSTRETH: minstret_r[63:32] <= csr_wdata;
                default: ; // WARL-ignore writes to misa / mip
            endcase
        end
    end
end

endmodule
