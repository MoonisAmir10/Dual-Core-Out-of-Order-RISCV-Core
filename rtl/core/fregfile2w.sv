`timescale 1ns / 1ps

// ============================================================================
// 6-Read, 2-Write Floating-Point Architectural Register File (FP ARF — RV32F)
// (Phase U6 — 32x32-bit; f0 IS writable unlike integer x0!)
//
// Features:
//   - All 32 registers f0..f31 readable and writable
//   - Dual in-order commit write ports (w1 is younger than w0 and takes
//     priority on same-cycle write to identical rd!)
//   - Write-first internal bypass on all 6 combinational read ports
//     (Lane 0 rs1/rs2/rs3 + Lane 1 rs1/rs2/rs3 for 2-wide FMA dispatch!)
// ============================================================================
module fregfile2w (
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

    // 6 Dispatch Combinational Read Ports (Lane 0 rs1/rs2/rs3, Lane 1 rs1/rs2/rs3)
    input  wire [4:0]  raddr0,
    output wire [31:0] rdata0,
    input  wire [4:0]  raddr1,
    output wire [31:0] rdata1,
    input  wire [4:0]  raddr2,
    output wire [31:0] rdata2,
    input  wire [4:0]  raddr3,
    output wire [31:0] rdata3,
    input  wire [4:0]  raddr4,
    output wire [31:0] rdata4,
    input  wire [4:0]  raddr5,
    output wire [31:0] rdata5
);

reg [31:0] reg_array [0:31];
integer i;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (i = 0; i < 32; i = i + 1)
            reg_array[i] <= 32'd0;
    end else begin
        if (we0) reg_array[waddr0] <= wdata0;
        if (we1) reg_array[waddr1] <= wdata1;
    end
end

assign rdata0 = (we1 && (waddr1 == raddr0)) ? wdata1 :
                (we0 && (waddr0 == raddr0)) ? wdata0 :
                reg_array[raddr0];

assign rdata1 = (we1 && (waddr1 == raddr1)) ? wdata1 :
                (we0 && (waddr0 == raddr1)) ? wdata0 :
                reg_array[raddr1];

assign rdata2 = (we1 && (waddr1 == raddr2)) ? wdata1 :
                (we0 && (waddr0 == raddr2)) ? wdata0 :
                reg_array[raddr2];

assign rdata3 = (we1 && (waddr1 == raddr3)) ? wdata1 :
                (we0 && (waddr0 == raddr3)) ? wdata0 :
                reg_array[raddr3];

assign rdata4 = (we1 && (waddr1 == raddr4)) ? wdata1 :
                (we0 && (waddr0 == raddr4)) ? wdata0 :
                reg_array[raddr4];

assign rdata5 = (we1 && (waddr1 == raddr5)) ? wdata1 :
                (we0 && (waddr0 == raddr5)) ? wdata0 :
                reg_array[raddr5];

endmodule
