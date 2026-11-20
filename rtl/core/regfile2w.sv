`timescale 1ns / 1ps

// ============================================================================
// 6-Read, 2-Write Architectural Register File (ARF) for 2-Wide Superscalar
// (Phase 1: Adds raddr4/raddr5 for zero-overhead ROB-head CSR & RV32A reads!)
//
// Features:
//   - Hardwired x0 == 0
//   - Dual in-order commit write ports (w1 is younger than w0 and takes
//     priority on same-cycle write to identical rd!)
//   - Write-first internal bypass on all 6 combinational read ports
// ============================================================================
module regfile2w (
    input  wire        clk,
    input  wire        reset_n,

    // Write Port 0 (older ROB head commit)
    input  wire        we0,
    input  wire [4:0]  waddr0,
    input  wire [31:0] wdata0,

    // Write Port 1 (younger ROB head+1 commit)
    input  wire        we1,
    input  wire [4:0]  waddr1,
    input  wire [31:0] wdata1,

    // 4 Dispatch Combinational Read Ports (Lane 0 rs1/rs2, Lane 1 rs1/rs2)
    input  wire [4:0]  raddr0,
    output wire [31:0] rdata0,
    input  wire [4:0]  raddr1,
    output wire [31:0] rdata1,
    input  wire [4:0]  raddr2,
    output wire [31:0] rdata2,
    input  wire [4:0]  raddr3,
    output wire [31:0] rdata3,

    // 2 ROB-Head Architectural Read Ports (CSR rs1, RV32A LR/SC/AMO rs1 & rs2)
    input  wire [4:0]  raddr4,
    output wire [31:0] rdata4,
    input  wire [4:0]  raddr5,
    output wire [31:0] rdata5,

    output wire [31:0] x14
);

reg [31:0] reg_array [1:31];
integer i;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (i = 1; i < 32; i = i + 1)
            reg_array[i] <= 32'd0;
    end else begin
        if (we0 && (waddr0 != 5'd0))
            reg_array[waddr0] <= wdata0;
        if (we1 && (waddr1 != 5'd0))
            reg_array[waddr1] <= wdata1;
    end
end

assign rdata0 = (raddr0 == 5'd0) ? 32'd0 :
                (we1 && (waddr1 == raddr0)) ? wdata1 :
                (we0 && (waddr0 == raddr0)) ? wdata0 :
                reg_array[raddr0];

assign rdata1 = (raddr1 == 5'd0) ? 32'd0 :
                (we1 && (waddr1 == raddr1)) ? wdata1 :
                (we0 && (waddr0 == raddr1)) ? wdata0 :
                reg_array[raddr1];

assign rdata2 = (raddr2 == 5'd0) ? 32'd0 :
                (we1 && (waddr1 == raddr2)) ? wdata1 :
                (we0 && (waddr0 == raddr2)) ? wdata0 :
                reg_array[raddr2];

assign rdata3 = (raddr3 == 5'd0) ? 32'd0 :
                (we1 && (waddr1 == raddr3)) ? wdata1 :
                (we0 && (waddr0 == raddr3)) ? wdata0 :
                reg_array[raddr3];

assign rdata4 = (raddr4 == 5'd0) ? 32'd0 :
                (we1 && (waddr1 == raddr4)) ? wdata1 :
                (we0 && (waddr0 == raddr4)) ? wdata0 :
                reg_array[raddr4];

assign rdata5 = (raddr5 == 5'd0) ? 32'd0 :
                (we1 && (waddr1 == raddr5)) ? wdata1 :
                (we0 && (waddr0 == raddr5)) ? wdata0 :
                reg_array[raddr5];

assign x14 = reg_array[14];

endmodule
