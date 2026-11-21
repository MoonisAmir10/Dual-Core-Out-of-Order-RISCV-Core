`timescale 1ns / 1ps

// ============================================================================
// Dual Common Data Bus (Dual-CDB) Priority Arbiter for 2-Wide Superscalar OoO
//
// Arbitrates 6 Functional Units (DIV, LSU, MUL, BR, ALU0, ALU1) onto 2 CDBs:
//   - CDB0 (full sideband): priority DIV > LSU > MUL > BR > ALU0 > ALU1
//     (BR and LSU always broadcast on CDB0 so branch/memory sidebands are unique)
//   - CDB1 (value writeback): priority MUL > ALU0 > ALU1 (excluding CDB0 winner)
//
// Guarantees up to 2 completions/cycle (e.g. DIV+MUL, LSU+ALU, BR+ALU, ALU0+ALU1)
// with zero dropped results and zero tag collisions between CDB0 and CDB1.
// ============================================================================
module cdb_arbiter2w #(
    parameter integer TAG_W = 5
) (
    // Port 0: DIV (highest priority -> CDB0)
    input  wire             div_valid,
    output reg              div_grant,
    input  wire [TAG_W-1:0] div_tag,
    input  wire [31:0]      div_seq,
    input  wire [31:0]      div_val,

    // Port 1: LSU / Head CSR (CDB0)
    input  wire             lsu_valid,
    output reg              lsu_grant,
    input  wire [TAG_W-1:0] lsu_tag,
    input  wire [31:0]      lsu_seq,
    input  wire [31:0]      lsu_val,
    input  wire             lsu_mem_we,
    input  wire [31:0]      lsu_mem_addr,
    input  wire [3:0]       lsu_mem_wstrb,
    input  wire [31:0]      lsu_mem_wdata,
    input  wire             lsu_exc_valid,
    input  wire [3:0]       lsu_exc_cause,
    input  wire [31:0]      lsu_exc_tval,

    // Port 2: MUL (CDB0 or CDB1)
    input  wire             mul_valid,
    output reg              mul_grant,
    input  wire [TAG_W-1:0] mul_tag,
    input  wire [31:0]      mul_seq,
    input  wire [31:0]      mul_val,

    // Port 3: BR (CDB0 only -> carries branch resolution & fast squash)
    input  wire             br_valid,
    output reg              br_grant,
    input  wire [TAG_W-1:0] br_tag,
    input  wire [31:0]      br_seq,
    input  wire [31:0]      br_val,
    input  wire             br_taken,
    input  wire [31:0]      br_target,
    input  wire             br_exc_valid,
    input  wire [3:0]       br_exc_cause,
    input  wire [31:0]      br_exc_tval,

    // Port 4: ALU0 (CDB0 or CDB1)
    input  wire             alu0_valid,
    output reg              alu0_grant,
    input  wire [TAG_W-1:0] alu0_tag,
    input  wire [31:0]      alu0_seq,
    input  wire [31:0]      alu0_val,

    // Port 5: ALU1 (CDB0 or CDB1)
    input  wire             alu1_valid,
    output reg              alu1_grant,
    input  wire [TAG_W-1:0] alu1_tag,
    input  wire [31:0]      alu1_seq,
    input  wire [31:0]      alu1_val,

    // CDB0 Broadcast Outputs (full sideband)
    output reg              cdb0_valid,
    output reg  [TAG_W-1:0] cdb0_tag,
    output reg  [31:0]      cdb0_seq,
    output reg  [31:0]      cdb0_val,
    output reg              cdb0_is_br,
    output reg              cdb0_br_taken,
    output reg  [31:0]      cdb0_br_target,
    output reg              cdb0_is_mem,
    output reg              cdb0_mem_we,
    output reg  [31:0]      cdb0_mem_addr,
    output reg  [3:0]       cdb0_mem_wstrb,
    output reg  [31:0]      cdb0_mem_wdata,
    output reg              cdb0_exc_valid,
    output reg  [3:0]       cdb0_exc_cause,
    output reg  [31:0]      cdb0_exc_tval,

    // CDB1 Broadcast Outputs (second value writeback port)
    output reg              cdb1_valid,
    output reg  [TAG_W-1:0] cdb1_tag,
    output reg  [31:0]      cdb1_seq,
    output reg  [31:0]      cdb1_val
);

reg cdb0_took_mul, cdb0_took_alu0, cdb0_took_alu1;

always @(*) begin
    div_grant      = 1'b0;
    lsu_grant      = 1'b0;
    mul_grant      = 1'b0;
    br_grant       = 1'b0;
    alu0_grant     = 1'b0;
    alu1_grant     = 1'b0;

    cdb0_valid     = 1'b0;
    cdb0_tag       = {TAG_W{1'b0}};
    cdb0_seq       = 32'd0;
    cdb0_val       = 32'd0;
    cdb0_is_br     = 1'b0;
    cdb0_br_taken  = 1'b0;
    cdb0_br_target = 32'd0;
    cdb0_is_mem    = 1'b0;
    cdb0_mem_we    = 1'b0;
    cdb0_mem_addr  = 32'd0;
    cdb0_mem_wstrb = 4'd0;
    cdb0_mem_wdata = 32'd0;
    cdb0_exc_valid = 1'b0;
    cdb0_exc_cause = 4'd0;
    cdb0_exc_tval  = 32'd0;

    cdb1_valid     = 1'b0;
    cdb1_tag       = {TAG_W{1'b0}};
    cdb1_seq       = 32'd0;
    cdb1_val       = 32'd0;

    cdb0_took_mul  = 1'b0;
    cdb0_took_alu0 = 1'b0;
    cdb0_took_alu1 = 1'b0;

    // --- CDB0 Selection: DIV > LSU > MUL > BR > ALU0 > ALU1 ---
    if (div_valid) begin
        div_grant  = 1'b1;
        cdb0_valid = 1'b1;
        cdb0_tag   = div_tag;
        cdb0_seq   = div_seq;
        cdb0_val   = div_val;
    end else if (lsu_valid) begin
        lsu_grant      = 1'b1;
        cdb0_valid     = 1'b1;
        cdb0_tag       = lsu_tag;
        cdb0_seq       = lsu_seq;
        cdb0_val       = lsu_val;
        cdb0_is_mem    = 1'b1;
        cdb0_mem_we    = lsu_mem_we;
        cdb0_mem_addr  = lsu_mem_addr;
        cdb0_mem_wstrb = lsu_mem_wstrb;
        cdb0_mem_wdata = lsu_mem_wdata;
        cdb0_exc_valid = lsu_exc_valid;
        cdb0_exc_cause = lsu_exc_cause;
        cdb0_exc_tval  = lsu_exc_tval;
    end else if (mul_valid) begin
        mul_grant     = 1'b1;
        cdb0_took_mul = 1'b1;
        cdb0_valid    = 1'b1;
        cdb0_tag      = mul_tag;
        cdb0_seq      = mul_seq;
        cdb0_val      = mul_val;
    end else if (br_valid) begin
        br_grant       = 1'b1;
        cdb0_valid     = 1'b1;
        cdb0_tag       = br_tag;
        cdb0_seq       = br_seq;
        cdb0_val       = br_val;
        cdb0_is_br     = 1'b1;
        cdb0_br_taken  = br_taken;
        cdb0_br_target = br_target;
        cdb0_exc_valid = br_exc_valid;
        cdb0_exc_cause = br_exc_cause;
        cdb0_exc_tval  = br_exc_tval;
    end else if (alu0_valid) begin
        alu0_grant     = 1'b1;
        cdb0_took_alu0 = 1'b1;
        cdb0_valid     = 1'b1;
        cdb0_tag       = alu0_tag;
        cdb0_seq       = alu0_seq;
        cdb0_val       = alu0_val;
    end else if (alu1_valid) begin
        alu1_grant     = 1'b1;
        cdb0_took_alu1 = 1'b1;
        cdb0_valid     = 1'b1;
        cdb0_tag       = alu1_tag;
        cdb0_seq       = alu1_seq;
        cdb0_val       = alu1_val;
    end

    // --- CDB1 Selection: remaining MUL > ALU0 > ALU1 ---
    if (mul_valid && !cdb0_took_mul) begin
        mul_grant  = 1'b1;
        cdb1_valid = 1'b1;
        cdb1_tag   = mul_tag;
        cdb1_seq   = mul_seq;
        cdb1_val   = mul_val;
    end else if (alu0_valid && !cdb0_took_alu0) begin
        alu0_grant = 1'b1;
        cdb1_valid = 1'b1;
        cdb1_tag   = alu0_tag;
        cdb1_seq   = alu0_seq;
        cdb1_val   = alu0_val;
    end else if (alu1_valid && !cdb0_took_alu1) begin
        alu1_grant = 1'b1;
        cdb1_valid = 1'b1;
        cdb1_tag   = alu1_tag;
        cdb1_seq   = alu1_seq;
        cdb1_val   = alu1_val;
    end
end

endmodule
