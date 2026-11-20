`timescale 1ns / 1ps

// ============================================================================
// Dual-Port Instruction Memory for 2-Wide Superscalar Fetch
// ============================================================================
module instr_mem2w #(
    parameter integer WORDS    = 1024,
    parameter         HEX_FILE = ""
) (
    input  wire [31:0] pc0,
    output wire [31:0] instr0,
    input  wire [31:0] pc1,
    output wire [31:0] instr1
);

localparam integer ADDR_W = $clog2(WORDS);

reg [31:0] mem [0:WORDS-1];
integer i;

`ifndef SYNTHESIS
initial begin
    for (i = 0; i < WORDS; i = i + 1)
        mem[i] = 32'h0000_0013; // NOP
    if (HEX_FILE != "")
        $readmemh(HEX_FILE, mem);
end

task automatic load_hex(input string filename);
    integer k;
    begin
        for (k = 0; k < WORDS; k = k + 1)
            mem[k] = 32'h0000_0013;
        $readmemh(filename, mem);
    end
endtask
`endif

assign instr0 = mem[pc0[ADDR_W+1:2]];
assign instr1 = mem[pc1[ADDR_W+1:2]];

endmodule
