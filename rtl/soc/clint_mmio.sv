`timescale 1ns / 1ps

// ============================================================================
// Core-Local Interruptor (CLINT) & Uncached Synchronization MMIO Hub (Phase 2)
// Mapped at 0x0200_0000 .. 0x0200_FFFF (uncached bypass around L1D/L2!)
//
// Register Map (offset = addr[15:0]):
//   0x0000 : msip0       (RW, bit 0) - Core 0 Software Interrupt / Wakeup Flag
//   0x0004 : msip1       (RW, bit 0) - Core 1 Software Interrupt / Wakeup Flag
//   0xBFF8 : mtime_lo    (RW, 32b)   - Shared 64-bit Live Timer Low Word
//   0xBFFC : mtime_hi    (RW, 32b)   - Shared 64-bit Live Timer High Word
//   0xC000 : scratch[0]  (RW, 32b)   - Uncached Hardware Mailbox / Sync Reg 0
//   0xC004 : scratch[1]  (RW, 32b)   - Uncached Hardware Mailbox / Sync Reg 1
//   0xC008 : scratch[2]  (RW, 32b)   - Uncached Hardware Mailbox / Sync Reg 2
//   0xC00C : scratch[3]  (RW, 32b)   - Uncached Hardware Mailbox / Sync Reg 3
// ============================================================================
module clint_mmio #(
    parameter integer ID_W = 7
) (
    input  wire            clk,
    input  wire            reset_n,

    // Uncached MMIO Request Channel (1-cycle accept, 1-cycle registered read response)
    input  wire            req_valid,
    output wire            req_ready,
    input  wire            req_is_write,
    input  wire [15:0]     req_offset,
    input  wire [31:0]     req_wdata,
    input  wire [3:0]      req_wstrb,
    input  wire [ID_W-1:0] req_id,

    output reg             resp_valid,
    output reg  [ID_W-1:0] resp_id,
    output reg  [31:0]     resp_rdata,

    // Hardware Interrupt / Wakeup & Timer Observability Outputs
    output wire            msip0_out,
    output wire            msip1_out,
    output wire [63:0]     mtime_out
);

reg        msip0_r, msip1_r;
reg [63:0] mtime_r;
reg [31:0] scratch_r [0:3];

assign req_ready = 1'b1;
assign msip0_out = msip0_r;
assign msip1_out = msip1_r;
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

reg [31:0] rd_mux;
always @(*) begin
    case ({req_offset[15:2], 2'b00})
        16'h0000: rd_mux = {31'd0, msip0_r};
        16'h0004: rd_mux = {31'd0, msip1_r};
        16'hBFF8: rd_mux = mtime_r[31:0];
        16'hBFFC: rd_mux = mtime_r[63:32];
        16'hC000: rd_mux = scratch_r[0];
        16'hC004: rd_mux = scratch_r[1];
        16'hC008: rd_mux = scratch_r[2];
        16'hC00C: rd_mux = scratch_r[3];
        default:  rd_mux = 32'd0;
    endcase
end

wire [63:0] mtime_next = mtime_r + 64'd1;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        msip0_r      <= 1'b0;
        msip1_r      <= 1'b0;
        mtime_r      <= 64'd0;
        scratch_r[0] <= 32'd0;
        scratch_r[1] <= 32'd0;
        scratch_r[2] <= 32'd0;
        scratch_r[3] <= 32'd0;
        resp_valid   <= 1'b0;
        resp_id      <= {ID_W{1'b0}};
        resp_rdata   <= 32'd0;
    end else begin
        mtime_r    <= mtime_next;
        resp_valid <= 1'b0;

        if (req_valid) begin
            if (req_is_write) begin
                case ({req_offset[15:2], 2'b00})
                    16'h0000: if (req_wstrb[0]) msip0_r <= req_wdata[0];
                    16'h0004: if (req_wstrb[0]) msip1_r <= req_wdata[0];
                    16'hBFF8: mtime_r[31:0]  <= mask_merge(mtime_r[31:0],  req_wdata, req_wstrb);
                    16'hBFFC: mtime_r[63:32] <= mask_merge(mtime_r[63:32], req_wdata, req_wstrb);
                    16'hC000: scratch_r[0]   <= mask_merge(scratch_r[0],   req_wdata, req_wstrb);
                    16'hC004: scratch_r[1]   <= mask_merge(scratch_r[1],   req_wdata, req_wstrb);
                    16'hC008: scratch_r[2]   <= mask_merge(scratch_r[2],   req_wdata, req_wstrb);
                    16'hC00C: scratch_r[3]   <= mask_merge(scratch_r[3],   req_wdata, req_wstrb);
                    default: ;
                endcase
            end else begin
                resp_valid <= 1'b1;
                resp_id    <= req_id;
                resp_rdata <= rd_mux;
            end
        end
    end
end

endmodule
