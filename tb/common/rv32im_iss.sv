`timescale 1ns / 1ps

// ============================================================================
// Golden RV32IM Behavioral ISS + RVFI-Style Lockstep Commit Checker
// Portable across Icarus Verilog 13.0 (-g2012) and Verilator 5.054 (--assert)
// ============================================================================
module rv32im_iss #(
    parameter integer IMEM_WORDS  = 1024,
    parameter integer DMEM_BYTES  = 4096,
    parameter integer CHECK_EN    = 1,
    parameter integer VERBOSE     = 0
) (
    input  wire        clk,
    /* verilator lint_off SYNCASYNCNET */
    input  wire        reset_n,
    /* verilator lint_on SYNCASYNCNET */

    // RVFI-style commit port from DUT (In-Order or OoO ROB head)
    input  wire        commit_valid,
    input  wire [31:0] commit_pc,
    input  wire [31:0] commit_instr,
    input  wire        commit_rd_we,
    input  wire [4:0]  commit_rd_addr,
    input  wire [31:0] commit_rd_wdata,
    input  wire        commit_mem_valid,
    input  wire        commit_mem_we,
    input  wire [31:0] commit_mem_addr,
    input  wire [3:0]  commit_mem_wstrb,
    input  wire [31:0] commit_mem_wdata,

    output reg         halted,
    output reg  [31:0] commit_count,
    output reg  [31:0] error_count
);

reg [31:0] imem [0:IMEM_WORDS-1];
reg [7:0]  dmem [0:DMEM_BYTES-1];
reg [31:0] regs [0:31];
reg [31:0] golden_pc;

// Last-step expected commit outputs (for standalone cross-check or waveform debug)
reg [31:0] exp_pc;
reg [31:0] exp_instr;
reg        exp_rd_we;
reg [4:0]  exp_rd_addr;
reg [31:0] exp_rd_wdata;
reg        exp_mem_valid;
reg        exp_mem_we;
reg [31:0] exp_mem_addr;
reg [3:0]  exp_mem_wstrb;
reg [31:0] exp_mem_wdata;
reg        exp_halted;

integer init_i;

task automatic reset_state;
    integer k;
    begin
        golden_pc    = 32'd0;
        halted       = 1'b0;
        commit_count = 32'd0;
        error_count  = 32'd0;
        for (k = 0; k < 32; k = k + 1)
            regs[k] = 32'd0;
        for (k = 0; k < DMEM_BYTES; k = k + 1)
            dmem[k] = 8'd0;
    end
endtask

task automatic clear_imem;
    integer k;
    begin
        for (k = 0; k < IMEM_WORDS; k = k + 1)
            imem[k] = 32'h0000_0013; // NOP
    end
endtask

task automatic load_hex(input string filename);
    begin
        clear_imem();
        $readmemh(filename, imem);
    end
endtask

function automatic [31:0] read_dmem_word(input [31:0] addr);
    integer base;
    begin
        base = (addr & 32'hFFFF_FFFC) % DMEM_BYTES;
        read_dmem_word = {dmem[(base + 3) % DMEM_BYTES],
                          dmem[(base + 2) % DMEM_BYTES],
                          dmem[(base + 1) % DMEM_BYTES],
                          dmem[base]};
    end
endfunction

task automatic step_golden;
    reg [31:0] instr;
    reg [6:0]  op;
    reg [4:0]  rd, rs1, rs2;
    reg [2:0]  f3;
    reg [6:0]  f7;
    reg [31:0] v1, v2;
    reg [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;
    reg [31:0] next_pc;
    reg [31:0] addr;
    reg [1:0]  lane;
    reg        taken;
    reg signed [65:0] prod_ss;
    reg signed [65:0] prod_su;
    reg [63:0]        prod_uu;
    reg [31:0]        abs_a, abs_b, q_mag, r_mag;
    reg [7:0]         b0;
    reg [15:0]        h0;
    integer           a_idx;
    begin
        exp_pc        = golden_pc;
        instr         = imem[(golden_pc >> 2) % IMEM_WORDS];
        exp_instr     = instr;
        op            = instr[6:0];
        rd            = instr[11:7];
        f3            = instr[14:12];
        rs1           = instr[19:15];
        rs2           = instr[24:20];
        f7            = instr[31:25];

        v1            = (rs1 == 5'd0) ? 32'd0 : regs[rs1];
        v2            = (rs2 == 5'd0) ? 32'd0 : regs[rs2];

        imm_i         = {{20{instr[31]}}, instr[31:20]};
        imm_s         = {{20{instr[31]}}, instr[31:25], instr[11:7]};
        imm_b         = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
        imm_u         = {instr[31:12], 12'd0};
        imm_j         = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

        next_pc       = golden_pc + 32'd4;
        exp_rd_we     = 1'b0;
        exp_rd_addr   = 5'd0;
        exp_rd_wdata  = 32'd0;
        exp_mem_valid = 1'b0;
        exp_mem_we    = 1'b0;
        exp_mem_addr  = 32'd0;
        exp_mem_wstrb = 4'd0;
        exp_mem_wdata = 32'd0;
        exp_halted    = 1'b0;

        if (instr == 32'h0010_0073) begin
            exp_halted = 1'b1;
        end else if (op == 7'b0110111) begin // LUI
            exp_rd_we    = 1'b1;
            exp_rd_wdata = imm_u;
        end else if (op == 7'b0010111) begin // AUIPC
            exp_rd_we    = 1'b1;
            exp_rd_wdata = golden_pc + imm_u;
        end else if (op == 7'b1101111) begin // JAL
            exp_rd_we    = 1'b1;
            exp_rd_wdata = golden_pc + 32'd4;
            next_pc      = golden_pc + imm_j;
        end else if (op == 7'b1100111 && f3 == 3'b000) begin // JALR
            exp_rd_we    = 1'b1;
            exp_rd_wdata = golden_pc + 32'd4;
            next_pc      = (v1 + imm_i) & 32'hFFFF_FFFE;
        end else if (op == 7'b1100011) begin // BRANCH
            taken = 1'b0;
            case (f3)
                3'b000: taken = (v1 == v2);
                3'b001: taken = (v1 != v2);
                3'b100: taken = ($signed(v1) <  $signed(v2));
                3'b101: taken = ($signed(v1) >= $signed(v2));
                3'b110: taken = ($unsigned(v1) <  $unsigned(v2));
                3'b111: taken = ($unsigned(v1) >= $unsigned(v2));
                default: taken = 1'b0;
            endcase
            if (taken)
                next_pc = golden_pc + imm_b;
        end else if (op == 7'b0000011) begin // LOAD
            addr          = v1 + imm_i;
            a_idx         = addr % DMEM_BYTES;
            exp_mem_valid = 1'b1;
            exp_mem_we    = 1'b0;
            exp_mem_addr  = addr;
            exp_rd_we     = 1'b1;
            case (f3)
                3'b000: begin // LB
                    b0           = dmem[a_idx];
                    exp_rd_wdata = {{24{b0[7]}}, b0};
                end
                3'b001: begin // LH
                    h0           = {dmem[(a_idx + 1) % DMEM_BYTES], dmem[a_idx]};
                    exp_rd_wdata = {{16{h0[15]}}, h0};
                end
                3'b010: begin // LW
                    exp_rd_wdata = {dmem[(a_idx + 3) % DMEM_BYTES],
                                    dmem[(a_idx + 2) % DMEM_BYTES],
                                    dmem[(a_idx + 1) % DMEM_BYTES],
                                    dmem[a_idx]};
                end
                3'b100: begin // LBU
                    exp_rd_wdata = {24'd0, dmem[a_idx]};
                end
                3'b101: begin // LHU
                    exp_rd_wdata = {16'd0, dmem[(a_idx + 1) % DMEM_BYTES], dmem[a_idx]};
                end
                default: begin
                    $display("ISS ERROR: illegal load funct3=%0d at PC=0x%08x", f3, golden_pc);
                    error_count = error_count + 32'd1;
                end
            endcase
        end else if (op == 7'b0100011) begin // STORE
            addr          = v1 + imm_s;
            lane          = addr[1:0];
            a_idx         = addr % DMEM_BYTES;
            exp_mem_valid = 1'b1;
            exp_mem_we    = 1'b1;
            exp_mem_addr  = addr;
            case (f3)
                3'b000: begin // SB
                    dmem[a_idx]   = v2[7:0];
                    exp_mem_wstrb = 4'b0001 << lane;
                    exp_mem_wdata = {24'd0, v2[7:0]} << (lane * 8);
                end
                3'b001: begin // SH
                    dmem[a_idx]                  = v2[7:0];
                    dmem[(a_idx + 1) % DMEM_BYTES] = v2[15:8];
                    exp_mem_wstrb = lane[1] ? 4'b1100 : 4'b0011;
                    exp_mem_wdata = lane[1] ? {v2[15:0], 16'd0} : {16'd0, v2[15:0]};
                end
                3'b010: begin // SW
                    dmem[a_idx]                  = v2[7:0];
                    dmem[(a_idx + 1) % DMEM_BYTES] = v2[15:8];
                    dmem[(a_idx + 2) % DMEM_BYTES] = v2[23:16];
                    dmem[(a_idx + 3) % DMEM_BYTES] = v2[31:24];
                    exp_mem_wstrb = 4'b1111;
                    exp_mem_wdata = v2;
                end
                default: begin
                    $display("ISS ERROR: illegal store funct3=%0d at PC=0x%08x", f3, golden_pc);
                    error_count = error_count + 32'd1;
                end
            endcase
        end else if (op == 7'b0010011) begin // OP-IMM
            exp_rd_we = 1'b1;
            case (f3)
                3'b000: exp_rd_wdata = v1 + imm_i;
                3'b001: exp_rd_wdata = v1 << rs2;
                3'b010: exp_rd_wdata = ($signed(v1) < $signed(imm_i)) ? 32'd1 : 32'd0;
                3'b011: exp_rd_wdata = ($unsigned(v1) < $unsigned(imm_i)) ? 32'd1 : 32'd0;
                3'b100: exp_rd_wdata = v1 ^ imm_i;
                3'b101: begin
                    if (f7 == 7'b0100000)
                        exp_rd_wdata = $signed(v1) >>> rs2;
                    else
                        exp_rd_wdata = v1 >> rs2;
                end
                3'b110: exp_rd_wdata = v1 | imm_i;
                3'b111: exp_rd_wdata = v1 & imm_i;
                default: exp_rd_wdata = 32'd0;
            endcase
        end else if (op == 7'b0110011) begin // OP (RV32I + RV32M)
            exp_rd_we = 1'b1;
            if (f7 == 7'b0000001) begin // Official RV32M
                prod_ss = $signed({{34{v1[31]}}, v1}) * $signed({{34{v2[31]}}, v2});
                prod_su = $signed({{34{v1[31]}}, v1}) * $signed({34'd0, v2});
                prod_uu = {32'd0, v1} * {32'd0, v2};
                abs_a   = v1[31] ? (~v1 + 32'd1) : v1;
                abs_b   = v2[31] ? (~v2 + 32'd1) : v2;
                case (f3)
                    3'b000: exp_rd_wdata = prod_ss[31:0];  // MUL
                    3'b001: exp_rd_wdata = prod_ss[63:32]; // MULH
                    3'b010: exp_rd_wdata = prod_su[63:32]; // MULHSU
                    3'b011: exp_rd_wdata = prod_uu[63:32]; // MULHU
                    3'b100: begin // DIV (trunc toward zero)
                        if (v2 == 32'd0)
                            exp_rd_wdata = 32'hFFFF_FFFF;
                        else if (v1 == 32'h8000_0000 && v2 == 32'hFFFF_FFFF)
                            exp_rd_wdata = 32'h8000_0000;
                        else begin
                            q_mag = abs_a / abs_b;
                            exp_rd_wdata = (v1[31] ^ v2[31]) ? (~q_mag + 32'd1) : q_mag;
                        end
                    end
                    3'b101: begin // DIVU
                        exp_rd_wdata = (v2 == 32'd0) ? 32'hFFFF_FFFF : (v1 / v2);
                    end
                    3'b110: begin // REM (sign matches dividend)
                        if (v2 == 32'd0)
                            exp_rd_wdata = v1;
                        else if (v1 == 32'h8000_0000 && v2 == 32'hFFFF_FFFF)
                            exp_rd_wdata = 32'd0;
                        else begin
                            r_mag = abs_a % abs_b;
                            exp_rd_wdata = v1[31] ? (~r_mag + 32'd1) : r_mag;
                        end
                    end
                    3'b111: begin // REMU
                        exp_rd_wdata = (v2 == 32'd0) ? v1 : (v1 % v2);
                    end
                    default: exp_rd_wdata = 32'd0;
                endcase
            end else begin
                case (f3)
                    3'b000: exp_rd_wdata = (f7 == 7'b0100000) ? (v1 - v2) : (v1 + v2);
                    3'b001: exp_rd_wdata = v1 << v2[4:0];
                    3'b010: exp_rd_wdata = ($signed(v1) < $signed(v2)) ? 32'd1 : 32'd0;
                    3'b011: exp_rd_wdata = ($unsigned(v1) < $unsigned(v2)) ? 32'd1 : 32'd0;
                    3'b100: exp_rd_wdata = v1 ^ v2;
                    3'b101: begin
                        if (f7 == 7'b0100000)
                            exp_rd_wdata = $signed(v1) >>> v2[4:0];
                        else
                            exp_rd_wdata = v1 >> v2[4:0];
                    end
                    3'b110: exp_rd_wdata = v1 | v2;
                    3'b111: exp_rd_wdata = v1 & v2;
                    default: exp_rd_wdata = 32'd0;
                endcase
            end
        end else begin
            $display("ISS ERROR: illegal instr=0x%08x at PC=0x%08x", instr, golden_pc);
            error_count = error_count + 32'd1;
        end

        if (exp_rd_we && (rd != 5'd0)) begin
            exp_rd_addr = rd;
            regs[rd]    = exp_rd_wdata;
        end else begin
            exp_rd_we    = 1'b0;
            exp_rd_addr  = 5'd0;
            exp_rd_wdata = 32'd0;
        end
        regs[0] = 32'd0;

        golden_pc = next_pc;
        if (exp_halted)
            halted = 1'b1;
    end
endtask

initial begin
    clear_imem();
    reset_state();
end

// Lockstep Commit Checker
always @(posedge clk) begin
    if (!reset_n) begin
        reset_state();
    end else if (CHECK_EN != 0 && commit_valid) begin
        if (halted) begin
            $display("ISS FAIL [step %0d]: DUT committed after EBREAK! pc=0x%08x instr=0x%08x",
                     commit_count, commit_pc, commit_instr);
            error_count <= error_count + 32'd1;
        end else begin
            step_golden();
            if (commit_pc !== exp_pc || commit_instr !== exp_instr) begin
                $display("ISS FAIL [step %0d]: PC/INSTR mismatch! exp(pc=0x%08x,instr=0x%08x) got(pc=0x%08x,instr=0x%08x)",
                         commit_count, exp_pc, exp_instr, commit_pc, commit_instr);
                error_count <= error_count + 32'd1;
            end
            if (commit_rd_we !== exp_rd_we ||
                (exp_rd_we && (commit_rd_addr !== exp_rd_addr || commit_rd_wdata !== exp_rd_wdata))) begin
                $display("ISS FAIL [step %0d PC=0x%08x]: RD mismatch! exp(we=%0b x%0d=0x%08x) got(we=%0b x%0d=0x%08x)",
                         commit_count, exp_pc, exp_rd_we, exp_rd_addr, exp_rd_wdata,
                         commit_rd_we, commit_rd_addr, commit_rd_wdata);
                error_count <= error_count + 32'd1;
            end
            if (commit_mem_valid !== exp_mem_valid ||
                (exp_mem_valid && (commit_mem_we !== exp_mem_we || commit_mem_addr !== exp_mem_addr)) ||
                (exp_mem_valid && exp_mem_we && (commit_mem_wstrb !== exp_mem_wstrb || commit_mem_wdata !== exp_mem_wdata))) begin
                $display("ISS FAIL [step %0d PC=0x%08x]: MEM mismatch! exp(v=%0b we=%0b addr=0x%08x strb=%01x wdata=0x%08x) got(v=%0b we=%0b addr=0x%08x strb=%01x wdata=0x%08x)",
                         commit_count, exp_pc,
                         exp_mem_valid, exp_mem_we, exp_mem_addr, exp_mem_wstrb, exp_mem_wdata,
                         commit_mem_valid, commit_mem_we, commit_mem_addr, commit_mem_wstrb, commit_mem_wdata);
                error_count <= error_count + 32'd1;
            end
            if (VERBOSE != 0) begin
                $display("COMMIT[%04d] pc=0x%08x instr=0x%08x rd_we=%0b x%02d=0x%08x mem=%0b/%0b@0x%08x",
                         commit_count, commit_pc, commit_instr, commit_rd_we, commit_rd_addr, commit_rd_wdata,
                         commit_mem_valid, commit_mem_we, commit_mem_addr);
            end
            commit_count <= commit_count + 32'd1;
        end
    end
end

endmodule
