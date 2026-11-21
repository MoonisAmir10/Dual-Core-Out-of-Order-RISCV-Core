`timescale 1ns / 1ps

// ============================================================================
// Unit Testbench for Zicsr Unit (Phase 0)
// Tests two instances simultaneously (CORE_ID=0 and CORE_ID=1):
//   - mhartid isolation (0 vs 1) & read-only protection
//   - All 6 CSR ops (RW, RS, RC, RWI, RSI, RCI) on mscratch
//   - CSRRS/CSRRC with rs1==0 pure-read rule on RO CSRs
//   - 64-bit mcycle / minstret live 2-wide increments & 32-bit carry wrap
//   - Unimplemented CSR illegal flagging
// ============================================================================
module tb_csr_unit;

`include "rv32ima_defs.vh"

reg         clk;
reg         reset_n;
reg         halted;
reg  [1:0]  retire_cnt;
reg         csr_req_valid;
reg  [2:0]  csr_op;
reg  [11:0] csr_addr;
reg  [4:0]  csr_rs1_uimm;
reg  [31:0] csr_rs1_val;

wire [31:0] c0_rdata, c1_rdata, c0_mtvec, c1_mtvec, c0_mepc, c1_mepc;
wire        c0_aill,  c1_aill,  c0_ill,   c1_ill;
wire        c0_irqp,  c1_irqp;
wire [3:0]  c0_irqc,  c1_irqc;
wire [1:0]  c0_fs,    c1_fs;
wire [63:0] c0_cyc,   c1_cyc;
wire [63:0] c0_ret,   c1_ret;

csr_unit #(.CORE_ID(32'd0)) u_csr0 (
    .clk(clk), .reset_n(reset_n), .halted(halted), .retire_cnt(retire_cnt),
    .csr_req_valid(csr_req_valid), .csr_op(csr_op), .csr_addr(csr_addr),
    .csr_rs1_uimm(csr_rs1_uimm), .csr_rs1_val(csr_rs1_val),
    .csr_rdata(c0_rdata), .csr_access_illegal(c0_aill), .csr_illegal(c0_ill),
    .irq_msip(1'b0), .irq_mtip(1'b0), .irq_meip(1'b0),
    .trap_enter(1'b0), .trap_is_irq(1'b0), .trap_cause(4'd0), .trap_pc(32'd0), .trap_tval(32'd0),
    .mret_exec(1'b0),
    .mtvec_out(c0_mtvec), .mepc_out(c0_mepc), .irq_pending(c0_irqp), .irq_cause(c0_irqc),
    .mstatus_fs_out(c0_fs),
    .mcycle_out(c0_cyc), .minstret_out(c0_ret)
);

csr_unit #(.CORE_ID(32'd1)) u_csr1 (
    .clk(clk), .reset_n(reset_n), .halted(halted), .retire_cnt(retire_cnt),
    .csr_req_valid(csr_req_valid), .csr_op(csr_op), .csr_addr(csr_addr),
    .csr_rs1_uimm(csr_rs1_uimm), .csr_rs1_val(csr_rs1_val),
    .csr_rdata(c1_rdata), .csr_access_illegal(c1_aill), .csr_illegal(c1_ill),
    .irq_msip(1'b0), .irq_mtip(1'b0), .irq_meip(1'b0),
    .trap_enter(1'b0), .trap_is_irq(1'b0), .trap_cause(4'd0), .trap_pc(32'd0), .trap_tval(32'd0),
    .mret_exec(1'b0),
    .mtvec_out(c1_mtvec), .mepc_out(c1_mepc), .irq_pending(c1_irqp), .irq_cause(c1_irqc),
    .mstatus_fs_out(c1_fs),
    .mcycle_out(c1_cyc), .minstret_out(c1_ret)
);

initial clk = 1'b0;
always #5 clk = ~clk;

integer errors;
integer checks;

task check(input cond, input [255:0] msg);
    begin
        checks = checks + 1;
        if (!cond) begin
            $display("FAIL [tb_csr_unit]: %0s (c0=0x%08x ill0=%0d c1=0x%08x ill1=%0d)",
                     msg, c0_rdata, c0_ill, c1_rdata, c1_ill);
            errors = errors + 1;
        end
    end
endtask

task step_clk;
    begin
        @(posedge clk);
        #1;
    end
endtask

initial begin
    errors        = 0;
    checks        = 0;
    reset_n       = 1'b0;
    halted        = 1'b0;
    retire_cnt    = 2'd0;
    csr_req_valid = 1'b0;
    csr_op        = 3'd0;
    csr_addr      = 12'd0;
    csr_rs1_uimm  = 5'd0;
    csr_rs1_val   = 32'd0;

    repeat (2) step_clk();
    reset_n = 1'b1;
    #1;

    // 1. Read mhartid via CSRRS with rs1=x0 (pure read of RO CSR -> legal!)
    csr_req_valid = 1'b1;
    csr_op        = CSR_OP_RS;
    csr_addr      = CSR_MHARTID;
    csr_rs1_uimm  = 5'd0;
    csr_rs1_val   = 32'd0;
    #1;
    check(!c0_ill && c0_rdata == 32'd0 && !c1_ill && c1_rdata == 32'd1, "CSRR mhartid (0 vs 1)");
    step_clk();

    // 2. Attempt write to RO mhartid via CSRRW -> MUST be illegal!
    csr_op        = CSR_OP_RW;
    csr_addr      = CSR_MHARTID;
    csr_rs1_uimm  = 5'd1;
    csr_rs1_val   = 32'hDEAD_BEEF;
    #1;
    check(c0_ill && c1_ill, "CSRRW RO mhartid illegal");
    step_clk();

    // 3. Attempt CSRRS with rs1!=x0 on RO cycle -> MUST be illegal!
    csr_op        = CSR_OP_RS;
    csr_addr      = CSR_CYCLE;
    csr_rs1_uimm  = 5'd2;
    csr_rs1_val   = 32'd1;
    #1;
    check(c0_ill && c1_ill, "CSRRS rs1!=0 on RO cycle illegal");
    step_clk();

    // 4. Write/Set/Clear mscratch (all 6 CSR ops)
    csr_op        = CSR_OP_RW;
    csr_addr      = CSR_MSCRATCH;
    csr_rs1_uimm  = 5'd3;
    csr_rs1_val   = 32'hA5A5_00F0;
    #1;
    check(!c0_ill && c0_rdata == 32'd0, "CSRRW mscratch old=0");
    step_clk();

    csr_op        = CSR_OP_RS;
    csr_rs1_uimm  = 5'd4;
    csr_rs1_val   = 32'h0000_0F00;
    #1;
    check(!c0_ill && c0_rdata == 32'hA5A5_00F0, "CSRRS mscratch readback");
    step_clk();

    csr_op        = CSR_OP_RC;
    csr_rs1_uimm  = 5'd4;
    csr_rs1_val   = 32'h0000_00F0;
    #1;
    check(!c0_ill && c0_rdata == 32'hA5A5_0FF0, "CSRRC mscratch readback");
    step_clk();

    csr_op        = CSR_OP_RSI;
    csr_rs1_uimm  = 5'd5;
    #1;
    check(!c0_ill && c0_rdata == 32'hA5A5_0F00, "CSRRSI mscratch readback");
    step_clk();

    csr_op        = CSR_OP_RCI;
    csr_rs1_uimm  = 5'd1;
    #1;
    check(!c0_ill && c0_rdata == 32'hA5A5_0F05, "CSRRCI mscratch readback");
    step_clk();

    csr_op        = CSR_OP_RWI;
    csr_rs1_uimm  = 5'd21;
    #1;
    check(!c0_ill && c0_rdata == 32'hA5A5_0F04, "CSRRWI mscratch readback");
    step_clk();

    csr_op        = CSR_OP_RS;
    csr_rs1_uimm  = 5'd0;
    #1;
    check(!c0_ill && c0_rdata == 32'd21, "Final mscratch == 21");
    step_clk();

    // 5. Test 2-wide minstret increment + 32-bit -> 64-bit carry wrap!
    csr_op        = CSR_OP_RW;
    csr_addr      = CSR_MINSTRET;
    csr_rs1_uimm  = 5'd1;
    csr_rs1_val   = 32'hFFFF_FFFF;
    step_clk();

    csr_req_valid = 1'b0;
    retire_cnt    = 2'd2; // dual-commit wraps low 32 bits to 1 and high 32 bits to 1!
    step_clk();
    retire_cnt    = 2'd0;
    check(c0_ret == 64'h0000_0001_0000_0001, "64-bit minstret 2-wide wrap");

    // 6. Test mcycle halts when halted==1
    halted = 1'b1;
    begin : cyc_freeze_blk
        reg [63:0] snap;
        snap = c0_cyc;
        repeat (3) step_clk();
        check(c0_cyc == snap, "mcycle freezes on halt");
    end

    // 7. Unimplemented CSR -> illegal
    csr_req_valid = 1'b1;
    csr_op        = CSR_OP_RS;
    csr_addr      = 12'h777;
    csr_rs1_uimm  = 5'd0;
    #1;
    check(c0_ill && c1_ill, "Unimplemented CSR illegal");

    if (errors == 0)
        $display("PASS [tb_csr_unit]: %0d/%0d Zicsr dual-core unit checks passed!", checks, checks);
    else begin
        $display("FAIL [tb_csr_unit]: %0d errors out of %0d checks!", errors, checks);
        $fatal(1);
    end
    $finish;
end

endmodule
