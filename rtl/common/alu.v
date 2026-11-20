`timescale 1ns / 1ps

module alu (
    input  wire [31:0] inp1,
    input  wire [31:0] inp2,
    input  wire [3:0]  alu_select,
    output reg  [31:0] alu_out
);

`include "rv32ima_defs.vh"

always @(*) begin
    case (alu_select)
        ALU_ADD:    alu_out = inp1 + inp2;
        ALU_SUB:    alu_out = inp1 - inp2;
        ALU_SLT:    alu_out = ($signed(inp1) < $signed(inp2)) ? 32'd1 : 32'd0;
        ALU_SRA:    alu_out = $signed(inp1) >>> inp2[4:0];
        ALU_SLL:    alu_out = inp1 << inp2[4:0];
        ALU_SRL:    alu_out = inp1 >> inp2[4:0];
        ALU_OR:     alu_out = inp1 | inp2;
        ALU_AND:    alu_out = inp1 & inp2;
        ALU_XOR:    alu_out = inp1 ^ inp2;
        ALU_SLTU:   alu_out = ($unsigned(inp1) < $unsigned(inp2)) ? 32'd1 : 32'd0;
        ALU_PASS_B: alu_out = inp2;
        default:    alu_out = inp1 + inp2;
    endcase
end

endmodule
