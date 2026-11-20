`timescale 1ns / 1ps

// ============================================================================
// Standalone Unit Testbench for CLINT & Uncached Synchronization MMIO Hub
// Verifies msip0/msip1, live 64-bit mtime + carry wrap + byte-strobe writes,
// and scratch[0..3] mailbox registers.
// ============================================================================
module tb_clint_mmio;

reg         clk, reset_n;
reg         req_valid, req_is_write;
wire        req_ready;
reg  [15:0] req_offset;
reg  [31:0] req_wdata;
reg  [3:0]  req_wstrb;
reg  [6:0]  req_id;

wire        resp_valid;
wire [6:0]  resp_id;
wire [31:0] resp_rdata;
wire        msip0_out, msip1_out;
wire [63:0] mtime_out;

clint_mmio #(.ID_W(7)) dut (
    .clk(clk), .reset_n(reset_n),
    .req_valid(req_valid), .req_ready(req_ready),
    .req_is_write(req_is_write), .req_offset(req_offset),
    .req_wdata(req_wdata), .req_wstrb(req_wstrb), .req_id(req_id),
    .resp_valid(resp_valid), .resp_id(resp_id), .resp_rdata(resp_rdata),
    .msip0_out(msip0_out), .msip1_out(msip1_out), .mtime_out(mtime_out)
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

task mmio_write(input [15:0] off, input [31:0] val, input [3:0] strb);
    begin
        req_valid    = 1'b1;
        req_is_write = 1'b1;
        req_offset   = off;
        req_wdata    = val;
        req_wstrb    = strb;
        req_id       = 7'd0;
        step_clk();
        req_valid    = 1'b0;
    end
endtask

task mmio_read(input [15:0] off, input [6:0] id, output [31:0] val);
    begin
        req_valid    = 1'b1;
        req_is_write = 1'b0;
        req_offset   = off;
        req_id       = id;
        step_clk();
        req_valid    = 1'b0;
        if (!resp_valid || resp_id !== id) begin
            $display("FAIL [tb_clint_mmio]: bad resp valid=%0d id=%0d exp=%0d", resp_valid, resp_id, id);
            errors = errors + 1;
        end
        val = resp_rdata;
    end
endtask

task check(input cond, input [511:0] msg);
    begin
        checks = checks + 1;
        if (!cond) begin
            $display("FAIL [tb_clint_mmio]: %0s", msg);
            errors = errors + 1;
        end
    end
endtask

reg [31:0] r0, r1;

initial begin
    errors       = 0;
    checks       = 0;
    reset_n      = 1'b0;
    req_valid    = 1'b0;
    req_is_write = 1'b0;
    req_offset   = 16'd0;
    req_wdata    = 32'd0;
    req_wstrb    = 4'hF;
    req_id       = 7'd0;

    repeat (2) step_clk();
    reset_n = 1'b1;
    #1;

    // 1. msip0 & msip1 write/readback
    mmio_write(16'h0000, 32'h0000_0001, 4'hF);
    mmio_write(16'h0004, 32'h0000_0001, 4'hF);
    check(msip0_out && msip1_out, "msip0 & msip1 outputs asserted");
    mmio_read(16'h0000, 7'd1, r0);
    mmio_read(16'h0004, 7'd2, r1);
    check(r0 == 32'd1 && r1 == 32'd1, "msip0/1 MMIO readback == 1");
    mmio_write(16'h0004, 32'd0, 4'hF);
    check(msip0_out && !msip1_out, "msip1 cleared cleanly");

    // 2. scratch[0..3] with full-word & partial byte-strobe writes
    mmio_write(16'hC000, 32'h1234_5678, 4'hF);
    mmio_write(16'hC000, 32'h00AA_0000, 4'b0100); // replace byte 2 only -> 0x12AA_5678
    mmio_read(16'hC000, 7'd3, r0);
    check(r0 == 32'h12AA_5678, "scratch[0] byte-strobe merge");

    mmio_write(16'hC00C, 32'hCAFE_BABE, 4'hF);
    mmio_read(16'hC00C, 7'd4, r1);
    check(r1 == 32'hCAFE_BABE, "scratch[3] readback");

    // 3. 64-bit mtime monotonic increment & 32->64-bit carry wrap
    mmio_read(16'hBFF8, 7'd5, r0);
    repeat (5) step_clk();
    mmio_read(16'hBFF8, 7'd6, r1);
    check(r1 > r0, "mtime ticks monotonically");

    mmio_write(16'hBFF8, 32'hFFFF_FFFE, 4'hF);
    repeat (3) step_clk();
    mmio_read(16'hBFFC, 7'd7, r1);
    check(r1 == 32'd1, "mtime 32->64-bit carry wrap");

    if (errors == 0)
        $display("PASS [tb_clint_mmio]: %0d/%0d CLINT MMIO checks passed!", checks, checks);
    else begin
        $display("FAIL [tb_clint_mmio]: %0d errors out of %0d checks!", errors, checks);
        $fatal(1);
    end
    $finish;
end

endmodule
