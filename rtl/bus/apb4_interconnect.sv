`timescale 1ns / 1ps

// ============================================================================
// Expandable Standardized AMBA APB4 Interconnect Decoder & Response Mux
// (ARM IHI 0024C Compliant — PADDR, PSEL, PENABLE, PWRITE, PWDATA, PSTRB,
//  PREADY, PRDATA, PSLVERR)
//
// Expandability by Construction:
//   - Parameterizable NUM_SLAVES (1..16) and bitmask SLAVE_PRESENT[15:0].
//   - Plugging a new peripheral requires only instantiating an APB4 slave,
//     setting its SLAVE_PRESENT bit, and connecting its s_* slice.
//   - Any access to an unmapped address hole OR an unpopulated slave slot
//     completes immediately with PREADY=1, PRDATA=0, PSLVERR=1 so the
//     requesting core raises a precise Load/Store Access Fault (mcause 5 / 7)!
// ============================================================================
module apb4_interconnect #(
    parameter integer NUM_SLAVES    = 7,
    parameter [15:0]  SLAVE_PRESENT = 16'b0000_0000_0100_0001 // Default U2: S0(CLINT) + S6(SOC_CTRL)
) (
    // Master APB4 Port (from mmio_to_apb4_bridge)
    input  wire                     m_psel,
    input  wire                     m_penable,
    input  wire                     m_pwrite,
    input  wire [31:0]              m_paddr,
    input  wire [31:0]              m_pwdata,
    input  wire [3:0]               m_pstrb,
    output reg                      m_pready,
    output reg  [31:0]              m_prdata,
    output reg                      m_pslverr,

    // Slave APB4 Ports (flat packed vectors for Verilog/SV/Yosys portability)
    output reg  [NUM_SLAVES-1:0]    s_psel,
    output wire                     s_penable,
    output wire                     s_pwrite,
    output wire [31:0]              s_paddr,
    output wire [31:0]              s_pwdata,
    output wire [3:0]               s_pstrb,
    input  wire [NUM_SLAVES-1:0]    s_pready,
    input  wire [NUM_SLAVES*32-1:0] s_prdata,
    input  wire [NUM_SLAVES-1:0]    s_pslverr
);

`include "soc_addr_map.vh"

// Broadcast shared APB4 address/control/write channels to all slaves
assign s_penable = m_penable;
assign s_pwrite  = m_pwrite;
assign s_paddr   = m_paddr;
assign s_pwdata  = m_pwdata;
assign s_pstrb   = m_pstrb;

// Address Decoder -> 4-bit target slave index + decode_hit
reg       dec_hit;
reg [3:0] dec_idx;

always @(*) begin
    dec_hit = 1'b0;
    dec_idx = 4'd0;

    if (m_paddr >= SOC_CLINT_BASE && m_paddr <= SOC_CLINT_END) begin
        dec_hit = 1'b1;
        dec_idx = APB_SLV_CLINT[3:0];
    end else if (m_paddr >= SOC_PLIC_BASE && m_paddr <= SOC_PLIC_END) begin
        dec_hit = 1'b1;
        dec_idx = APB_SLV_PLIC[3:0];
    end else if (m_paddr >= SOC_GPIO_BASE && m_paddr <= SOC_GPIO_END) begin
        dec_hit = 1'b1;
        dec_idx = APB_SLV_GPIO[3:0];
    end else if (m_paddr >= SOC_UART_BASE && m_paddr <= SOC_UART_END) begin
        dec_hit = 1'b1;
        dec_idx = APB_SLV_UART[3:0];
    end else if (m_paddr >= SOC_DMA_BASE && m_paddr <= SOC_DMA_END) begin
        dec_hit = 1'b1;
        dec_idx = APB_SLV_DMA[3:0];
    end else if (m_paddr >= SOC_TIMER_BASE && m_paddr <= SOC_TIMER_END) begin
        dec_hit = 1'b1;
        dec_idx = APB_SLV_TIMER[3:0];
    end else if (m_paddr >= SOC_CTRL_BASE && m_paddr <= SOC_CTRL_END) begin
        dec_hit = 1'b1;
        dec_idx = APB_SLV_SOC_CTRL[3:0];
    end else if (m_paddr[31:20] == 12'h100 && m_paddr[19:16] >= 4'h5 &&
                 m_paddr[19:16] <= 4'hD && m_paddr[15:12] == 4'h0) begin
        // Plug-and-play expansion slots S7..S15 (0x1005_0000 .. 0x100D_0FFF, 4 KB each)
        dec_hit = 1'b1;
        dec_idx = m_paddr[19:16] + 4'd2; // 0x5->7, 0x6->8, ..., 0xD->15
    end
end

integer si;
reg     slave_active;

always @(*) begin
    si           = 0;
    s_psel       = {NUM_SLAVES{1'b0}};
    m_pready     = 1'b1;
    m_prdata     = 32'd0;
    m_pslverr    = 1'b0;
    slave_active = 1'b0;

    if (m_psel) begin
        for (si = 0; si < NUM_SLAVES; si = si + 1) begin
            if (dec_hit && (dec_idx == si[3:0]) && SLAVE_PRESENT[si]) begin
                s_psel[si]   = 1'b1;
                m_pready     = s_pready[si];
                m_prdata     = s_prdata[si*32 +: 32];
                m_pslverr    = s_pslverr[si];
                slave_active = 1'b1;
            end
        end

        // Default unmapped / unpopulated error responder: PREADY=1, PSLVERR=1!
        if (!slave_active) begin
            m_pready  = 1'b1;
            m_prdata  = 32'd0;
            m_pslverr = 1'b1;
        end
    end
end

endmodule
