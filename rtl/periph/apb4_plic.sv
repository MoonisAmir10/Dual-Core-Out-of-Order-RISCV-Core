`timescale 1ns / 1ps

// ============================================================================
// Standardized AMBA APB4 RISC-V Platform-Level Interrupt Controller (PLIC)
// Mapped at 0x0C00_0000 .. 0x0C20_FFFF (APB4 Slave 1, Phase U3)
//
// RISC-V PLIC Spec v1.0.0 Compliant (SiFive / QEMU virt Register Offsets):
//   - NUM_SOURCES = 16 (Source 0 hardwired non-existent; Sources 1..15 active)
//   - NUM_HARTS   = 2  (Context 0 = Core 0 M-mode, Context 1 = Core 1 M-mode)
//   - PRIO_BITS   = 3  (Priority 0 = disabled, 1..7 = active; lowest ID wins ties)
//   - Configurable per-source gateway: Level-sensitive (default, 0) or
//     Rising-Edge-sensitive (1) via TRIG_TYPE register (0x001080).
//   - Atomic Claim/Complete handshake: reading CLAIM returns highest-priority
//     enabled pending ID (> THRESH), atomically clears IP[id], and sets
//     in-service gateway lock claimed[id]=1 so peer harts cannot double-claim!
//     Writing COMPLETE with an enabled source ID clears claimed[id]=0.
//
// Register Map (offset = {paddr[21:2], 2'b00}):
//   0x000000          : Source 0 Priority (RO 0, writes ignored)
//   0x000004..0x00003C: Source 1..15 Priority (RW, bits [2:0])
//   0x001000          : Interrupt Pending IP[15:0] (RO, bit 0 = 0)
//   0x001080          : Gateway Trigger Type[15:0] (RW, 0=level, 1=edge)
//   0x002000          : Context 0 Enable IE0[15:0] (RW, bit 0 = 0)
//   0x002080          : Context 1 Enable IE1[15:0] (RW, bit 0 = 0)
//   0x200000          : Context 0 Priority Threshold THRESH0[2:0] (RW)
//   0x200004          : Context 0 Claim (Read) / Complete (Write)
//   0x201000          : Context 1 Priority Threshold THRESH1[2:0] (RW)
//   0x201004          : Context 1 Claim (Read) / Complete (Write)
// ============================================================================
module apb4_plic #(
    parameter integer NUM_SOURCES = 16,
    parameter integer NUM_HARTS   = 2,
    parameter integer PRIO_BITS   = 3
) (
    input  wire                   pclk,
    input  wire                   preset_n,

    // Standardized AMBA APB4 Slave Interface
    input  wire                   psel,
    input  wire                   penable,
    input  wire                   pwrite,
    input  wire [31:0]            paddr,
    input  wire [31:0]            pwdata,
    input  wire [3:0]             pstrb,
    output wire                   pready,
    output reg  [31:0]            prdata,
    output wire                   pslverr,

    // External Peripheral Interrupt Sources (IDs 1 .. NUM_SOURCES-1)
    input  wire [NUM_SOURCES-1:1] irq_src_in,

    // Per-Hart Machine External Interrupt Outputs -> Core 0 / Core 1 irq_meip
    output wire [NUM_HARTS-1:0]   meip_out
);

assign pready  = 1'b1;
assign pslverr = 1'b0;

// ----------------------------------------------------------------------------
// 1. Architectural PLIC State Registers (index 0 hardwired 0 -> 100% bounds-safe!)
// ----------------------------------------------------------------------------
reg [PRIO_BITS-1:0]   prio_r      [0:NUM_SOURCES-1];
reg [NUM_SOURCES-1:0] pending_r;
reg [NUM_SOURCES-1:0] claimed_r;
reg [NUM_SOURCES-1:0] trig_edge_r;
reg [NUM_SOURCES-1:0] src_prev_r;

reg [NUM_SOURCES-1:0] ie_r        [0:NUM_HARTS-1];
reg [PRIO_BITS-1:0]   thresh_r    [0:NUM_HARTS-1];

// ----------------------------------------------------------------------------
// 2. Per-Hart Priority Resolver (Highest Priority > Threshold; Lowest ID Tie-Break)
// ----------------------------------------------------------------------------
reg [3:0]           best_id   [0:NUM_HARTS-1];
reg [PRIO_BITS-1:0] best_prio [0:NUM_HARTS-1];

integer hi, si;
always @(*) begin
    hi = 0;
    si = 0;
    for (hi = 0; hi < NUM_HARTS; hi = hi + 1) begin
        best_id[hi]   = 4'd0;
        best_prio[hi] = {PRIO_BITS{1'b0}};
        for (si = 1; si < NUM_SOURCES; si = si + 1) begin
            if (pending_r[si] && ie_r[hi][si] && (prio_r[si] > thresh_r[hi])) begin
                // Strict '>' in ascending source order guarantees lowest ID wins ties!
                if (prio_r[si] > best_prio[hi]) begin
                    best_prio[hi] = prio_r[si];
                    best_id[hi]   = si[3:0];
                end
            end
        end
    end
end

genvar gh;
generate
    for (gh = 0; gh < NUM_HARTS; gh = gh + 1) begin : gen_meip
        assign meip_out[gh] = (best_id[gh] != 4'd0);
    end
endgenerate

// ----------------------------------------------------------------------------
// 3. APB4 Address Decode & Combinational Read Mux
// ----------------------------------------------------------------------------
wire [21:0] reg_off   = {paddr[21:2], 2'b00};
wire        apb_write = psel && penable &&  pwrite;
wire        apb_read  = psel && penable && !pwrite;

wire is_prio_reg   = (reg_off[21:6] == 16'd0) && (reg_off[5:2] != 4'd0); // 0x000004..0x00003C
wire [3:0] prio_id = reg_off[5:2];

wire is_ip_reg     = (reg_off == 22'h001000);
wire is_trig_reg   = (reg_off == 22'h001080);
wire is_ie0_reg    = (reg_off == 22'h002000);
wire is_ie1_reg    = (NUM_HARTS >= 2) && (reg_off == 22'h002080);
wire is_th0_reg    = (reg_off == 22'h200000);
wire is_clm0_reg   = (reg_off == 22'h200004);
wire is_th1_reg    = (NUM_HARTS >= 2) && (reg_off == 22'h201000);
wire is_clm1_reg   = (NUM_HARTS >= 2) && (reg_off == 22'h201004);

always @(*) begin
    prdata = 32'd0;
    if (is_prio_reg) begin
        prdata = {{(32-PRIO_BITS){1'b0}}, prio_r[prio_id]};
    end else if (is_ip_reg) begin
        prdata = {{(32-NUM_SOURCES){1'b0}}, pending_r[NUM_SOURCES-1:1], 1'b0};
    end else if (is_trig_reg) begin
        prdata = {{(32-NUM_SOURCES){1'b0}}, trig_edge_r[NUM_SOURCES-1:1], 1'b0};
    end else if (is_ie0_reg) begin
        prdata = {{(32-NUM_SOURCES){1'b0}}, ie_r[0][NUM_SOURCES-1:1], 1'b0};
    end else if (is_ie1_reg) begin
        prdata = {{(32-NUM_SOURCES){1'b0}}, ie_r[NUM_HARTS-1][NUM_SOURCES-1:1], 1'b0};
    end else if (is_th0_reg) begin
        prdata = {{(32-PRIO_BITS){1'b0}}, thresh_r[0]};
    end else if (is_clm0_reg) begin
        prdata = {28'd0, best_id[0]};
    end else if (is_th1_reg) begin
        prdata = {{(32-PRIO_BITS){1'b0}}, thresh_r[NUM_HARTS-1]};
    end else if (is_clm1_reg) begin
        prdata = {28'd0, best_id[NUM_HARTS-1]};
    end
end

// ----------------------------------------------------------------------------
// 4. Gateway Trigger Detection & Atomic Claim / Complete Sequential State
// ----------------------------------------------------------------------------
wire [NUM_SOURCES-1:0] src_in_full = {irq_src_in, 1'b0};
wire [NUM_SOURCES-1:0] src_fire;
assign src_fire[0] = 1'b0;

genvar gs;
generate
    for (gs = 1; gs < NUM_SOURCES; gs = gs + 1) begin : gen_gw
        assign src_fire[gs] = trig_edge_r[gs]
                            ? (src_in_full[gs] && !src_prev_r[gs])
                            :  src_in_full[gs];
    end
endgenerate

wire       claim0_fire = apb_read && is_clm0_reg && (best_id[0] != 4'd0);
wire       claim1_fire = apb_read && is_clm1_reg && (best_id[NUM_HARTS-1] != 4'd0);
wire [3:0] comp_id     = pwdata[3:0];
wire       comp0_fire  = apb_write && is_clm0_reg && pstrb[0] && (comp_id != 4'd0) && ie_r[0][comp_id];
wire       comp1_fire  = apb_write && is_clm1_reg && pstrb[0] && (comp_id != 4'd0) && ie_r[NUM_HARTS-1][comp_id];

integer k;
always @(posedge pclk or negedge preset_n) begin
    if (!preset_n) begin
        for (k = 0; k < NUM_SOURCES; k = k + 1)
            prio_r[k] <= {PRIO_BITS{1'b0}};
        pending_r   <= {NUM_SOURCES{1'b0}};
        claimed_r   <= {NUM_SOURCES{1'b0}};
        trig_edge_r <= {NUM_SOURCES{1'b0}};
        src_prev_r  <= {NUM_SOURCES{1'b0}};
        for (k = 0; k < NUM_HARTS; k = k + 1) begin
            ie_r[k]     <= {NUM_SOURCES{1'b0}};
            thresh_r[k] <= {PRIO_BITS{1'b0}};
        end
    end else begin
        src_prev_r <= src_in_full;

        // 1. Interrupt Gateways: forward source triggers to IP when not in-service!
        for (k = 1; k < NUM_SOURCES; k = k + 1) begin
            if (src_fire[k] && !claimed_r[k])
                pending_r[k] <= 1'b1;
        end

        // 2. Atomic Claim (clears pending_r and locks gateway claimed_r=1)
        if (claim0_fire) begin
            pending_r[best_id[0]] <= 1'b0;
            claimed_r[best_id[0]] <= 1'b1;
        end else if (claim1_fire) begin
            pending_r[best_id[NUM_HARTS-1]] <= 1'b0;
            claimed_r[best_id[NUM_HARTS-1]] <= 1'b1;
        end

        // 3. Complete Write (releases gateway lock claimed_r=0 if enabled for context)
        if (comp0_fire || comp1_fire) begin
            claimed_r[comp_id] <= 1'b0;
        end

        // 4. Configuration Register Writes (byte-strobe aware; bit 0 permanently 0)
        if (apb_write) begin
            if (is_prio_reg && pstrb[0]) begin
                prio_r[prio_id] <= pwdata[PRIO_BITS-1:0];
            end else if (is_trig_reg) begin
                if (pstrb[0]) trig_edge_r[7:1]  <= pwdata[7:1];
                if (pstrb[1]) trig_edge_r[15:8] <= pwdata[15:8];
            end else if (is_ie0_reg) begin
                if (pstrb[0]) ie_r[0][7:1]  <= pwdata[7:1];
                if (pstrb[1]) ie_r[0][15:8] <= pwdata[15:8];
            end else if (is_ie1_reg) begin
                if (pstrb[0]) ie_r[NUM_HARTS-1][7:1]  <= pwdata[7:1];
                if (pstrb[1]) ie_r[NUM_HARTS-1][15:8] <= pwdata[15:8];
            end else if (is_th0_reg && pstrb[0]) begin
                thresh_r[0] <= pwdata[PRIO_BITS-1:0];
            end else if (is_th1_reg && pstrb[0]) begin
                thresh_r[NUM_HARTS-1] <= pwdata[PRIO_BITS-1:0];
            end
        end
    end
end

endmodule
