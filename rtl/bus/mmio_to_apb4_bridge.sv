`timescale 1ns / 1ps

// ============================================================================
// Dual-Core 2-to-1 Round-Robin Uncached MMIO -> AMBA APB4 Master Bridge
//
// Features:
//   - Fair round-robin arbitration between Core 0 and Core 1 uncached MMIO ports
//     (Core 1 may be tied to 0 for single-core SoCs).
//   - Strict ARM IHI 0024C AMBA APB4 2-phase master FSM:
//       IDLE -> SETUP (PSEL=1, PENABLE=0, 1 cycle)
//            -> ACCESS (PSEL=1, PENABLE=1, holds addr/ctrl/data until PREADY=1)
//            -> IDLE (1-cycle registered response back to winning core)
//   - Forces PSTRB=4'b0000 on reads per AMBA APB4 §3.1.1.
//   - Returns resp_valid + resp_err (PSLVERR) on BOTH reads AND writes so
//     the requesting core's LSQ raises precise mcause=5/7 Access Faults!
// ============================================================================
module mmio_to_apb4_bridge #(
    parameter integer ID_W = 7
) (
    input  wire            clk,
    input  wire            reset_n,

    // Core 0 Uncached MMIO Request / Response Port
    input  wire            c0_req_valid,
    output wire            c0_req_ready,
    input  wire            c0_req_is_write,
    input  wire [31:0]     c0_req_addr,
    input  wire [31:0]     c0_req_wdata,
    input  wire [3:0]      c0_req_wstrb,
    input  wire [ID_W-1:0] c0_req_id,

    output reg             c0_resp_valid,
    output reg  [ID_W-1:0] c0_resp_id,
    output reg  [31:0]     c0_resp_rdata,
    output reg             c0_resp_err,

    // Core 1 Uncached MMIO Request / Response Port
    input  wire            c1_req_valid,
    output wire            c1_req_ready,
    input  wire            c1_req_is_write,
    input  wire [31:0]     c1_req_addr,
    input  wire [31:0]     c1_req_wdata,
    input  wire [3:0]      c1_req_wstrb,
    input  wire [ID_W-1:0] c1_req_id,

    output reg             c1_resp_valid,
    output reg  [ID_W-1:0] c1_resp_id,
    output reg  [31:0]     c1_resp_rdata,
    output reg             c1_resp_err,

    // Standardized AMBA APB4 Master Port -> apb4_interconnect
    output wire            m_psel,
    output wire            m_penable,
    output wire            m_pwrite,
    output wire [31:0]     m_paddr,
    output wire [31:0]     m_pwdata,
    output wire [3:0]      m_pstrb,
    input  wire            m_pready,
    input  wire [31:0]     m_prdata,
    input  wire            m_pslverr
);

localparam [1:0] ST_IDLE   = 2'd0;
localparam [1:0] ST_SETUP  = 2'd1;
localparam [1:0] ST_ACCESS = 2'd2;

reg [1:0]      state;
reg            rr_sel;      // 0: prefer Core 0, 1: prefer Core 1
reg            lat_master;  // 0: Core 0, 1: Core 1
reg [ID_W-1:0] lat_id;
reg            lat_pwrite;
reg [31:0]     lat_paddr;
reg [31:0]     lat_pwdata;
reg [3:0]      lat_pstrb;

// Round-robin grant strictly in ST_IDLE
wire pick_c1 = c1_req_valid && (!c0_req_valid || rr_sel);
wire pick_c0 = c0_req_valid && !pick_c1;

assign c0_req_ready = (state == ST_IDLE) && pick_c0;
assign c1_req_ready = (state == ST_IDLE) && pick_c1;

// Strict AMBA APB4 Master Outputs (all registered -> 0 glitches, stable across wait states!)
assign m_psel    = (state == ST_SETUP) || (state == ST_ACCESS);
assign m_penable = (state == ST_ACCESS);
assign m_pwrite  = lat_pwrite;
assign m_paddr   = lat_paddr;
assign m_pwdata  = lat_pwdata;
assign m_pstrb   = lat_pwrite ? lat_pstrb : 4'b0000; // APB4 §3.1.1: PSTRB=0 on reads

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state         <= ST_IDLE;
        rr_sel        <= 1'b0;
        lat_master    <= 1'b0;
        lat_id        <= {ID_W{1'b0}};
        lat_pwrite    <= 1'b0;
        lat_paddr     <= 32'd0;
        lat_pwdata    <= 32'd0;
        lat_pstrb     <= 4'd0;
        c0_resp_valid <= 1'b0;
        c0_resp_id    <= {ID_W{1'b0}};
        c0_resp_rdata <= 32'd0;
        c0_resp_err   <= 1'b0;
        c1_resp_valid <= 1'b0;
        c1_resp_id    <= {ID_W{1'b0}};
        c1_resp_rdata <= 32'd0;
        c1_resp_err   <= 1'b0;
    end else begin
        c0_resp_valid <= 1'b0;
        c1_resp_valid <= 1'b0;

        case (state)
            ST_IDLE: begin
                if (pick_c0) begin
                    lat_master <= 1'b0;
                    lat_id     <= c0_req_id;
                    lat_pwrite <= c0_req_is_write;
                    lat_paddr  <= c0_req_addr;
                    lat_pwdata <= c0_req_wdata;
                    lat_pstrb  <= c0_req_wstrb;
                    rr_sel     <= 1'b1; // next priority -> Core 1
                    state      <= ST_SETUP;
                end else if (pick_c1) begin
                    lat_master <= 1'b1;
                    lat_id     <= c1_req_id;
                    lat_pwrite <= c1_req_is_write;
                    lat_paddr  <= c1_req_addr;
                    lat_pwdata <= c1_req_wdata;
                    lat_pstrb  <= c1_req_wstrb;
                    rr_sel     <= 1'b0; // next priority -> Core 0
                    state      <= ST_SETUP;
                end
            end

            ST_SETUP: begin
                state <= ST_ACCESS;
            end

            ST_ACCESS: begin
                if (m_pready) begin
                    if (!lat_master) begin
                        c0_resp_valid <= 1'b1;
                        c0_resp_id    <= lat_id;
                        c0_resp_rdata <= lat_pwrite ? 32'd0 : m_prdata;
                        c0_resp_err   <= m_pslverr;
                    end else begin
                        c1_resp_valid <= 1'b1;
                        c1_resp_id    <= lat_id;
                        c1_resp_rdata <= lat_pwrite ? 32'd0 : m_prdata;
                        c1_resp_err   <= m_pslverr;
                    end
                    state <= ST_IDLE;
                end
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
