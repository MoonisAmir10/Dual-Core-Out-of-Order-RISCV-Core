`timescale 1ns / 1ps

// ============================================================================
// Standardized AMBA APB4 SoC Control, TOHOST/HALT & Live Chip PMU Slave
// Mapped at 0x1004_0000 .. 0x1004_0FFF (APB4 Slave 6)
//
// Register Map (offset = paddr[11:0]):
//   0x000 : SOC_ID        (RO) - 0x2002_0002 (Dual-Core 2W OoO SoC v2.0)
//   0x004 : CORE_COUNT    (RO) - Number of harts (parameter NUM_CORES)
//   0x008 : FEATURE_MASK  (RO) - Hardware feature bitmap (parameter FEATURE_MASK)
//   0x00C : SYS_HALT      (RW) - Writing bit[0]=1 asserts sys_halt_out cleanly!
//   0x010 : TOHOST        (RW) - 32-bit software exit code / status mailbox
//   0x014 : SCRATCH       (RW) - 32-bit general SoC scratch register
//   0x018 : IRQ_INJECT    (RW) - Bits [15:1] software-driven PLIC test IRQs (bit0=0)
//   0x020 : PMU_CYCLES    (RO) - Live chip cycles
//   0x024 : PMU_C0_RET    (RO) - Core 0 retired instructions
//   0x028 : PMU_C1_RET    (RO) - Core 1 retired instructions
//   0x02C : PMU_COH_C2C   (RO) - MOESI dirty cache-to-cache transfers
//   0x030 : PMU_L2_HITS   (RO) - Shared L2 cache hits
//   0x034 : PMU_L2_MISSES (RO) - Shared L2 cache misses
//   0x038 : PMU_DRAM_RDS  (RO) - Backing DRAM reads
//   0x03C : PMU_DRAM_WRS  (RO) - Backing DRAM writes
//
// Write Protection:
//   - Attempting to write any RO register or unimplemented offset returns
//     PSLVERR=1 (triggering precise mcause=7 Store Access Fault on the core!).
// ============================================================================
module apb4_soc_ctrl #(
    parameter [31:0] SOC_ID       = 32'h2002_0002,
    parameter [31:0] NUM_CORES    = 32'd2,
    parameter [31:0] FEATURE_MASK = 32'h0000_0003 // bit0=Traps, bit1=APB4
) (
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

    // Live SoC PMU Readback Inputs
    input  wire [31:0] pmu_cycles_in,
    input  wire [31:0] pmu_c0_instret_in,
    input  wire [31:0] pmu_c1_instret_in,
    input  wire [31:0] pmu_coh_c2c_in,
    input  wire [31:0] pmu_l2_hits_in,
    input  wire [31:0] pmu_l2_misses_in,
    input  wire [31:0] pmu_dram_rds_in,
    input  wire [31:0] pmu_dram_wrs_in,

    // SoC Control Outputs
    output wire        sys_halt_out,
    output wire [31:0] tohost_out,
    output wire [15:1] irq_inject_out
);

reg        sys_halt_r;
reg [31:0] tohost_r;
reg [31:0] scratch_r;
reg [15:1] irq_inject_r;

assign pready         = 1'b1;
assign sys_halt_out   = sys_halt_r;
assign tohost_out     = tohost_r;
assign irq_inject_out = irq_inject_r;

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

wire [11:0] reg_off   = {paddr[11:2], 2'b00};
wire        is_wr_reg = (reg_off == 12'h00C) || (reg_off == 12'h010) ||
                        (reg_off == 12'h014) || (reg_off == 12'h018);

// Writing to any read-only or unimplemented register asserts PSLVERR!
assign pslverr = psel && pwrite && !is_wr_reg;

always @(*) begin
    case (reg_off)
        12'h000: prdata = SOC_ID;
        12'h004: prdata = NUM_CORES;
        12'h008: prdata = FEATURE_MASK;
        12'h00C: prdata = {31'd0, sys_halt_r};
        12'h010: prdata = tohost_r;
        12'h014: prdata = scratch_r;
        12'h018: prdata = {16'd0, irq_inject_r, 1'b0};
        12'h020: prdata = pmu_cycles_in;
        12'h024: prdata = pmu_c0_instret_in;
        12'h028: prdata = pmu_c1_instret_in;
        12'h02C: prdata = pmu_coh_c2c_in;
        12'h030: prdata = pmu_l2_hits_in;
        12'h034: prdata = pmu_l2_misses_in;
        12'h038: prdata = pmu_dram_rds_in;
        12'h03C: prdata = pmu_dram_wrs_in;
        default: prdata = 32'd0;
    endcase
end

wire apb_write = psel && penable && pwrite && is_wr_reg;

always @(posedge pclk or negedge preset_n) begin
    if (!preset_n) begin
        sys_halt_r   <= 1'b0;
        tohost_r     <= 32'd0;
        scratch_r    <= 32'd0;
        irq_inject_r <= 15'd0;
    end else if (apb_write) begin
        case (reg_off)
            12'h00C: if (pstrb[0]) sys_halt_r <= pwdata[0];
            12'h010: tohost_r  <= mask_merge(tohost_r,  pwdata, pstrb);
            12'h014: scratch_r <= mask_merge(scratch_r, pwdata, pstrb);
            12'h018: begin
                if (pstrb[0]) irq_inject_r[7:1]  <= pwdata[7:1];
                if (pstrb[1]) irq_inject_r[15:8] <= pwdata[15:8];
            end
            default: ;
        endcase
    end
end

endmodule
