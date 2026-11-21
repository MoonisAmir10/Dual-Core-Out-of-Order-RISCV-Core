`timescale 1ns / 1ps

// ============================================================================
// Unit Testbench for Phase U1 M-Mode Trap & Interrupt CSR Unit (tb_csr_traps)
// Verifies:
//   1. mstatus (MIE, MPIE, hardwired MPP=2'b11, FS reset=2'b01 & RW)
//   2. misa (MISA_EXT_F=0 -> RV32IMA 0x40001101 vs MISA_EXT_F=1 -> 0x40001121, WARL)
//   3. mtvec Direct & Vectored mode WARL masking (bit 1 forced 0)
//   4. mepc 4-byte alignment masking (bits [1:0] forced 0), mcause, mtval RW
//   5. mie & mip live hardware interrupt reflection + WARL write-ignore
//   6. Atomic trap_enter (sync exception & IRQ) and mret_exec state transitions
//   7. Interrupt priority encoder: MEI(11) > MSI(3) > MTI(7) gated by mstatus.MIE
//   8. csr_access_illegal combinational legality independent of csr_req_valid
// ============================================================================
module tb_csr_traps;

`include "rv32ima_defs.vh"

reg         clk, reset_n, halted;
reg  [1:0]  retire_cnt;
reg         csr_req_valid;
reg  [2:0]  csr_op;
reg  [11:0] csr_addr;
reg  [4:0]  csr_rs1_uimm;
reg  [31:0] csr_rs1_val;

reg         irq_msip, irq_mtip, irq_meip;
reg         trap_enter, trap_is_irq, mret_exec;
reg  [3:0]  trap_cause;
reg  [31:0] trap_pc, trap_tval;
reg         fp_dirty_f;
reg  [4:0]  fp_fflags_f;

wire [31:0] rdata0, mtvec0, mepc0;
wire        aill0, ill0, cs0, irqp0;
wire [3:0]  irqc0;
wire [1:0]  fs0;
wire [2:0]  frm0;
wire [4:0]  fflg0;
wire [63:0] cyc0, ret0;

wire [31:0] rdata_f, mtvec_f, mepc_f;
wire        aill_f, ill_f, cs_f, irqp_f;
wire [3:0]  irqc_f;
wire [1:0]  fs_f;
wire [2:0]  frm_f;
wire [4:0]  fflg_f;
wire [63:0] cyc_f, ret_f;

csr_unit #(.CORE_ID(32'd0), .MISA_EXT_F(1'b0)) u_dut (
    .clk(clk), .reset_n(reset_n), .halted(halted), .retire_cnt(retire_cnt),
    .fp_commit_dirty(1'b0), .fp_commit_fflags(5'd0),
    .csr_req_valid(csr_req_valid), .csr_op(csr_op), .csr_addr(csr_addr),
    .csr_rs1_uimm(csr_rs1_uimm), .csr_rs1_val(csr_rs1_val),
    .csr_rdata(rdata0), .csr_access_illegal(aill0), .csr_illegal(ill0), .csr_sync_required(cs0),
    .irq_msip(irq_msip), .irq_mtip(irq_mtip), .irq_meip(irq_meip),
    .trap_enter(trap_enter), .trap_is_irq(trap_is_irq), .trap_cause(trap_cause),
    .trap_pc(trap_pc), .trap_tval(trap_tval), .mret_exec(mret_exec),
    .mtvec_out(mtvec0), .mepc_out(mepc0), .irq_pending(irqp0), .irq_cause(irqc0),
    .mstatus_fs_out(fs0), .frm_out(frm0), .fflags_out(fflg0), .mcycle_out(cyc0), .minstret_out(ret0)
);

csr_unit #(.CORE_ID(32'd1), .MISA_EXT_F(1'b1)) u_dut_f (
    .clk(clk), .reset_n(reset_n), .halted(halted), .retire_cnt(retire_cnt),
    .fp_commit_dirty(fp_dirty_f), .fp_commit_fflags(fp_fflags_f),
    .csr_req_valid(csr_req_valid), .csr_op(csr_op), .csr_addr(csr_addr),
    .csr_rs1_uimm(csr_rs1_uimm), .csr_rs1_val(csr_rs1_val),
    .csr_rdata(rdata_f), .csr_access_illegal(aill_f), .csr_illegal(ill_f), .csr_sync_required(cs_f),
    .irq_msip(1'b0), .irq_mtip(1'b0), .irq_meip(1'b0),
    .trap_enter(1'b0), .trap_is_irq(1'b0), .trap_cause(4'd0),
    .trap_pc(32'd0), .trap_tval(32'd0), .mret_exec(1'b0),
    .mtvec_out(mtvec_f), .mepc_out(mepc_f), .irq_pending(irqp_f), .irq_cause(irqc_f),
    .mstatus_fs_out(fs_f), .frm_out(frm_f), .fflags_out(fflg_f), .mcycle_out(cyc_f), .minstret_out(ret_f)
);

initial clk = 1'b0;
always #5 clk = ~clk;

integer errors, checks;

task step_clk;
    begin
        @(posedge clk);
        #1;
    end
endtask

task check(input cond, input [511:0] msg);
    begin
        checks = checks + 1;
        if (!cond) begin
            $display("FAIL [tb_csr_traps]: %0s (rdata=0x%08x mtvec=0x%08x mepc=0x%08x irqp=%0d irqc=%0d)",
                     msg, rdata0, mtvec0, mepc0, irqp0, irqc0);
            errors = errors + 1;
        end
    end
endtask

task csr_write(input [11:0] addr, input [31:0] val);
    begin
        csr_req_valid = 1'b1;
        csr_op        = CSR_OP_RW;
        csr_addr      = addr;
        csr_rs1_uimm  = 5'd1;
        csr_rs1_val   = val;
        #1;
        step_clk();
        csr_req_valid = 1'b0;
    end
endtask

task csr_read_chk(input [11:0] addr, input [31:0] exp, input [511:0] msg);
    begin
        csr_req_valid = 1'b1;
        csr_op        = CSR_OP_RS;
        csr_addr      = addr;
        csr_rs1_uimm  = 5'd0;
        csr_rs1_val   = 32'd0;
        #1;
        check(!ill0 && (rdata0 == exp), msg);
        csr_req_valid = 1'b0;
    end
endtask

initial begin
    errors        = 0;
    checks        = 0;
    reset_n       = 1'b0;
    halted        = 1'b0;
    retire_cnt    = 2'd0;
    csr_req_valid = 1'b0;
    csr_op        = CSR_OP_RS;
    csr_addr      = CSR_MSTATUS;
    csr_rs1_uimm  = 5'd0;
    csr_rs1_val   = 32'd0;
    irq_msip      = 1'b0;
    irq_mtip      = 1'b0;
    irq_meip      = 1'b0;
    trap_enter    = 1'b0;
    trap_is_irq   = 1'b0;
    trap_cause    = 4'd0;
    trap_pc       = 32'd0;
    trap_tval     = 32'd0;
    mret_exec     = 1'b0;
    fp_dirty_f    = 1'b0;
    fp_fflags_f   = 5'd0;

    repeat (2) step_clk();
    reset_n = 1'b1;
    #1;

    // 1. Reset values: mstatus (FS=01, MPP=11, MPIE=0, MIE=0 -> 0x0000_3800) & misa
    csr_read_chk(CSR_MSTATUS, 32'h0000_3800, "Reset mstatus = 0x3800 (FS=01, MPP=11)");
    check(fs0 == 2'b01, "Reset mstatus_fs_out == 2'b01");

    csr_req_valid = 1'b1; csr_op = CSR_OP_RS; csr_addr = CSR_MISA; csr_rs1_uimm = 5'd0; #1;
    check(!ill0 && (rdata0 == 32'h4000_1101) && (rdata_f == 32'h4000_1121), "misa RV32IMA vs RV32IMAF");
    csr_req_valid = 1'b0;

    // Writing misa is WARL-ignored (not illegal)
    csr_write(CSR_MISA, 32'hFFFF_FFFF);
    csr_read_chk(CSR_MISA, 32'h4000_1101, "misa write WARL-ignored");

    // 2. mtvec WARL masking (bit 1 cleared; bit 0 preserved for Direct/Vectored)
    csr_write(CSR_MTVEC, 32'h0000_0403); // mode=11 -> must mask to 01!
    csr_read_chk(CSR_MTVEC, 32'h0000_0401, "mtvec masks bit 1 (0x403 -> 0x401)");
    check(mtvec0 == 32'h0000_0401, "mtvec_out matches 0x401");

    csr_write(CSR_MTVEC, 32'h0000_0802); // mode=10 -> must mask to 00!
    csr_read_chk(CSR_MTVEC, 32'h0000_0800, "mtvec masks mode=10 -> 00 (Direct)");

    // 3. mepc alignment masking (bits [1:0] forced 0)
    csr_write(CSR_MEPC, 32'h0000_1237);
    csr_read_chk(CSR_MEPC, 32'h0000_1234, "mepc masks low 2 bits (0x1237 -> 0x1234)");
    check(mepc0 == 32'h0000_1234, "mepc_out matches 0x1234");

    // 4. mstatus RW bits & MPP hardwired 2'b11
    csr_write(CSR_MSTATUS, 32'h0000_6008); // FS=11, MPP=00 (ignored!), MPIE=0, MIE=1
    csr_read_chk(CSR_MSTATUS, 32'h0000_7808, "mstatus FS=11, MPP hardwired 11, MIE=1");
    check(fs0 == 2'b11, "mstatus_fs_out updated to 2'b11");

    // 5. Atomic trap_enter (sync exception): MIE=1 -> MPIE=1, MIE=0, mepc/mcause/mtval updated
    trap_enter  = 1'b1;
    trap_is_irq = 1'b0;
    trap_cause  = MCAUSE_LOAD_MISALIGN;
    trap_pc     = 32'h0000_0142; // masks to 0x140
    trap_tval   = 32'h0001_0003;
    step_clk();
    trap_enter  = 1'b0;

    csr_read_chk(CSR_MEPC,    32'h0000_0140, "trap_enter sets aligned mepc");
    csr_read_chk(CSR_MCAUSE,  32'h0000_0004, "trap_enter sets mcause=4 (sync)");
    csr_read_chk(CSR_MTVAL,   32'h0001_0003, "trap_enter sets mtval=bad_addr");
    csr_read_chk(CSR_MSTATUS, 32'h0000_7880, "trap_enter saves MIE(1)->MPIE(1) & clears MIE(0)");

    // 6. Atomic mret_exec: restores MIE <= MPIE(1), sets MPIE <= 1
    mret_exec = 1'b1;
    step_clk();
    mret_exec = 1'b0;
    csr_read_chk(CSR_MSTATUS, 32'h0000_7888, "mret_exec restores MIE=1, MPIE=1");

    // 7. Interrupt pending & priority encoder (MEI(11) > MSI(3) > MTI(7))
    csr_write(CSR_MIE, 32'hFFFF_FFFF); // masks to bits 11, 7, 3 -> 0x888
    csr_read_chk(CSR_MIE, 32'h0000_0888, "mie masks to MEIE|MTIE|MSIE (0x888)");

    // All three HW lines active + MIE=1 -> MEI(11) wins!
    irq_msip = 1'b1; irq_mtip = 1'b1; irq_meip = 1'b1; #1;
    csr_read_chk(CSR_MIP, 32'h0000_0888, "mip reflects live MEIP|MTIP|MSIP");
    check(irqp0 && (irqc0 == MCAUSE_IRQ_MEI), "IRQ priority: MEI(11) > MSI(3) > MTI(7)");

    // Drop MEIP -> MSI(3) wins over MTI(7)!
    irq_meip = 1'b0; #1;
    check(irqp0 && (irqc0 == MCAUSE_IRQ_MSI), "IRQ priority: MSI(3) > MTI(7)");

    // Drop MSIP -> MTI(7) wins!
    irq_msip = 1'b0; #1;
    check(irqp0 && (irqc0 == MCAUSE_IRQ_MTI), "IRQ priority: MTI(7) alone");

    // Take interrupt trap -> MIE clears to 0 -> irq_pending drops to 0!
    trap_enter  = 1'b1;
    trap_is_irq = 1'b1;
    trap_cause  = irqc0;
    trap_pc     = 32'h0000_0200;
    trap_tval   = 32'd0;
    step_clk();
    trap_enter  = 1'b0;
    #1;
    check(!irqp0, "trap_enter clears MIE -> masks further IRQs");
    csr_read_chk(CSR_MCAUSE, 32'h8000_0007, "IRQ trap sets mcause[31]=1 | 7");

    // mip write is WARL-ignored (not illegal)
    csr_write(CSR_MIP, 32'd0);
    csr_read_chk(CSR_MIP, 32'h0000_0080, "mip write WARL-ignored (MTIP still live)");

    // 8. Combinational csr_access_illegal independent of csr_req_valid
    csr_req_valid = 1'b0;
    csr_op        = CSR_OP_RW;
    csr_addr      = CSR_MHARTID;
    csr_rs1_uimm  = 5'd1;
    #1;
    check(aill0 && !ill0, "csr_access_illegal=1 while csr_req_valid=0 (no CDB loop!)");

    // 9. RV32F CSRs (fflags, frm, fcsr), commit-time fflags/FS=Dirty & mstatus.FS=Off legality
    csr_req_valid = 1'b1; csr_op = CSR_OP_RS; csr_addr = CSR_FCSR; csr_rs1_uimm = 5'd0; #1;
    check(aill0 && !aill_f && (rdata_f == 32'd0), "fcsr illegal on MISA_EXT_F=0, legal on F=1");
    csr_req_valid = 1'b0;

    // Write frm=3 (RUP) & fflags=0x15 via CSR_FCSR -> must set csr_sync_required & mstatus.FS=Dirty
    csr_req_valid = 1'b1; csr_op = CSR_OP_RW; csr_addr = CSR_FCSR; csr_rs1_uimm = 5'd1; csr_rs1_val = 32'h0000_0075; #1;
    check(cs_f && !aill_f, "fcsr write asserts csr_sync_required");
    step_clk();
    csr_req_valid = 1'b0; #1;
    check((frm_f == 3'd3) && (fflg_f == 5'h15) && (fs_f == 2'b11), "fcsr write updates frm=3, fflags=0x15, FS=Dirty");

    // Non-blocking commit-time sticky fflags accumulation + mstatus.FS=Dirty
    csr_write(CSR_MSTATUS, 32'h0000_2000); // set FS=01 (Initial)
    #1;
    check(fs_f == 2'b01, "mstatus write sets FS=01");
    fp_dirty_f  = 1'b1;
    fp_fflags_f = 5'b01010; // DZ | UF -> ORs with 5'h15 => 5'h1F!
    step_clk();
    fp_dirty_f  = 1'b0;
    fp_fflags_f = 5'd0;
    #1;
    check((fflg_f == 5'h1F) && (fs_f == 2'b11), "commit-time fflags OR sticky & sets FS=Dirty");

    // Setting mstatus.FS=00 makes fflags/frm/fcsr illegal on MISA_EXT_F=1!
    csr_write(CSR_MSTATUS, 32'h0000_0000);
    csr_req_valid = 1'b1; csr_op = CSR_OP_RS; csr_addr = CSR_FFLAGS; csr_rs1_uimm = 5'd0; #1;
    check(aill_f, "fflags illegal when mstatus.FS==00");
    csr_addr = CSR_FRM; #1;
    check(aill_f, "frm illegal when mstatus.FS==00");
    csr_addr = CSR_FCSR; #1;
    check(aill_f, "fcsr illegal when mstatus.FS==00");
    csr_req_valid = 1'b0;

    if (errors == 0)
        $display("PASS [tb_csr_traps]: %0d/%0d M-mode trap/interrupt CSR checks passed!", checks, checks);
    else begin
        $display("FAIL [tb_csr_traps]: %0d errors out of %0d checks!", errors, checks);
        $fatal(1);
    end
    $finish;
end

endmodule
