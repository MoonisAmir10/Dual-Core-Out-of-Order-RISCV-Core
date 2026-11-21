`timescale 1ns / 1ps

// ============================================================================
// Standardized AMBA APB4 Core-Local Interruptor (CLINT) — Phase U2
// Mapped at 0x0200_0000 .. 0x0200_FFFF (APB4 Slave 0)
//
// SiFive-Compatible Register Map (offset = paddr[15:0]):
//   0x0000 : msip0         (RW, bit 0) - Core 0 Software Interrupt (IPI)
//   0x0004 : msip1         (RW, bit 0) - Core 1 Software Interrupt (IPI)
//   0x4000 : mtimecmp0_lo  (RW, 32b)   - Core 0 Timer Compare Low  (reset 0xFFFFFFFF)
//   0x4004 : mtimecmp0_hi  (RW, 32b)   - Core 0 Timer Compare High (reset 0xFFFFFFFF)
//   0x4008 : mtimecmp1_lo  (RW, 32b)   - Core 1 Timer Compare Low  (reset 0xFFFFFFFF)
//   0x400C : mtimecmp1_hi  (RW, 32b)   - Core 1 Timer Compare High (reset 0xFFFFFFFF)
//   0xBFF8 : mtime_lo      (RW, 32b)   - Shared 64-bit Live Timer Low
//   0xBFFC : mtime_hi      (RW, 32b)   - Shared 64-bit Live Timer High
//   0xC000 : scratch[0]    (RW, 32b)   - Uncached Hardware Mailbox 0
//   0xC004 : scratch[1]    (RW, 32b)   - Uncached Hardware Mailbox 1
//   0xC008 : scratch[2]    (RW, 32b)   - Uncached Hardware Mailbox 2
//   0xC00C : scratch[3]    (RW, 32b)   - Uncached Hardware Mailbox 3
// ============================================================================
module apb4_clint (
    input  wire        pclk,
    input  wire        preset_n,

    // Standardized AMBA APB4 Slave Interface
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] paddr,
    input  wire [31:0] pwdata,
    input  wire [3:0]  pstrb,
    output wire        pready,
    output reg  [31:0] prdata,
    output wire        pslverr,

    // Hardware Interrupt & Timer Outputs -> Core 0 & Core 1
    output wire        msip0_out,
    output wire        msip1_out,
    output wire        mtip0_out,
    output wire        mtip1_out,
    output wire [63:0] mtime_out
);

reg        msip0_r, msip1_r;
reg [63:0] mtime_r;
reg [63:0] mtimecmp0_r;
reg [63:0] mtimecmp1_r;
reg [31:0] scratch_r [0:3];

assign pready    = 1'b1;
assign pslverr   = 1'b0;
assign msip0_out = msip0_r;
assign msip1_out = msip1_r;
assign mtip0_out = (mtime_r >= mtimecmp0_r);
assign mtip1_out = (mtime_r >= mtimecmp1_r);
assign mtime_out = mtime_r;

function automatic [31:0] mask_merge(
    input [31:0] old_v,
    input [31:0] new_v,
    input [3:0]  strb
);
    begin
        mask_merge = {
            strb[3] ? new_v[31:24] : old_v[31:24],
            strb[2] ? new_v[23:16] : old_v[23:16],
            strb[1] ? new_v[15:8]  : old_v[15:8],
            strb[0] ? new_v[7:0]   : old_v[7:0]
        };
    end
endfunction

wire [15:0] reg_off = {paddr[15:2], 2'b00};

always @(*) begin
    case (reg_off)
        16'h0000: prdata = {31'd0, msip0_r};
        16'h0004: prdata = {31'd0, msip1_r};
        16'h4000: prdata = mtimecmp0_r[31:0];
        16'h4004: prdata = mtimecmp0_r[63:32];
        16'h4008: prdata = mtimecmp1_r[31:0];
        16'h400C: prdata = mtimecmp1_r[63:32];
        16'hBFF8: prdata = mtime_r[31:0];
        16'hBFFC: prdata = mtime_r[63:32];
        16'hC000: prdata = scratch_r[0];
        16'hC004: prdata = scratch_r[1];
        16'hC008: prdata = scratch_r[2];
        16'hC00C: prdata = scratch_r[3];
        default:  prdata = 32'd0;
    endcase
end

wire [63:0] mtime_next = mtime_r + 64'd1;
wire        apb_write  = psel && penable && pwrite;

always @(posedge pclk or negedge preset_n) begin
    if (!preset_n) begin
        msip0_r      <= 1'b0;
        msip1_r      <= 1'b0;
        mtime_r      <= 64'd0;
        mtimecmp0_r  <= 64'hFFFF_FFFF_FFFF_FFFF;
        mtimecmp1_r  <= 64'hFFFF_FFFF_FFFF_FFFF;
        scratch_r[0] <= 32'd0;
        scratch_r[1] <= 32'd0;
        scratch_r[2] <= 32'd0;
        scratch_r[3] <= 32'd0;
    end else begin
        mtime_r <= mtime_next;

        if (apb_write) begin
            case (reg_off)
                16'h0000: if (pstrb[0]) msip0_r <= pwdata[0];
                16'h0004: if (pstrb[0]) msip1_r <= pwdata[0];
                16'h4000: mtimecmp0_r[31:0]  <= mask_merge(mtimecmp0_r[31:0],  pwdata, pstrb);
                16'h4004: mtimecmp0_r[63:32] <= mask_merge(mtimecmp0_r[63:32], pwdata, pstrb);
                16'h4008: mtimecmp1_r[31:0]  <= mask_merge(mtimecmp1_r[31:0],  pwdata, pstrb);
                16'h400C: mtimecmp1_r[63:32] <= mask_merge(mtimecmp1_r[63:32], pwdata, pstrb);
                16'hBFF8: mtime_r[31:0]      <= mask_merge(mtime_r[31:0],      pwdata, pstrb);
                16'hBFFC: mtime_r[63:32]     <= mask_merge(mtime_r[63:32],     pwdata, pstrb);
                16'hC000: scratch_r[0]       <= mask_merge(scratch_r[0],       pwdata, pstrb);
                16'hC004: scratch_r[1]       <= mask_merge(scratch_r[1],       pwdata, pstrb);
                16'hC008: scratch_r[2]       <= mask_merge(scratch_r[2],       pwdata, pstrb);
                16'hC00C: scratch_r[3]       <= mask_merge(scratch_r[3],       pwdata, pstrb);
                default: ;
            endcase
        end
    end
end

endmodule
