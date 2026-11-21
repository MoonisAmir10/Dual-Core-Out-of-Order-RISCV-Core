`timescale 1ns / 1ps

// ============================================================================
// Standardized AMBA AXI4 128-Bit Slave -> MOESI I/O-Coherent Interconnect Bridge
// (Phase U7: Coherent DMA System Interface)
//
// Translates standard AMBA AXI4 128-bit INCR burst reads (AR/R) and writes
// (AW/W/B) into beat-by-beat I/O-coherent transactions on coh_interconnect's
// 3rd coherent master port:
//   - AR/R burst beat -> DMA_GETS (snoops D0 & D1 for dirty C2C, else L2)
//   - AW/W burst beat (wstrb == 16'hFFFF) -> DMA_PUTM (snoop-inv D0/D1 -> L2)
//   - AW/W burst beat (partial wstrb)     -> DMA_RMW  (snoop-inv + merge -> L2)
//   - AW/W burst beat (wstrb == 16'h0000) -> Clean zero-byte no-op
//   - Physical Memory Attribute (PMA) Enforcement:
//       * Any beat with address >= RAM_SIZE_BYTES (256 KB) completes cleanly
//         with AXI DECERR (2'b11) without issuing a coherence request!
//       * Unsupported burst type (!INCR for len>0) or size (!= 16B) returns
//         AXI SLVERR (2'b10)!
//   - Purely registered AXI4 VALID/DATA/RESP outputs -> ZERO combinational loops!
// ============================================================================
module axi4_to_coh_bridge #(
    parameter integer LADDR_W        = 16,             // 16B line address width
    parameter [31:0]  RAM_SIZE_BYTES = 32'h0004_0000   // 256 KB coherent RAM window
) (
    input  wire                clk,
    input  wire                reset_n,

    // -------------------- AMBA AXI4 128-Bit Slave Interface --------------------
    // Read Address Channel (AR)
    input  wire                s_axi_arvalid,
    output wire                s_axi_arready,
    input  wire [31:0]         s_axi_araddr,
    input  wire [7:0]          s_axi_arlen,
    input  wire [2:0]          s_axi_arsize,
    input  wire [1:0]          s_axi_arburst,

    // Read Data Channel (R)
    output reg                 s_axi_rvalid,
    input  wire                s_axi_rready,
    output reg  [127:0]        s_axi_rdata,
    output reg  [1:0]          s_axi_rresp,
    output reg                 s_axi_rlast,

    // Write Address Channel (AW)
    input  wire                s_axi_awvalid,
    output wire                s_axi_awready,
    input  wire [31:0]         s_axi_awaddr,
    input  wire [7:0]          s_axi_awlen,
    input  wire [2:0]          s_axi_awsize,
    input  wire [1:0]          s_axi_awburst,

    // Write Data Channel (W)
    input  wire                s_axi_wvalid,
    output wire                s_axi_wready,
    input  wire [127:0]        s_axi_wdata,
    input  wire [15:0]         s_axi_wstrb,
    input  wire                s_axi_wlast,

    // Write Response Channel (B)
    output reg                 s_axi_bvalid,
    input  wire                s_axi_bready,
    output reg  [1:0]          s_axi_bresp,

    // -------------------- Coherent Interconnect Master 2 Port --------------------
    output wire                dma_coh_req_valid,
    input  wire                dma_coh_req_ready,
    output wire                dma_coh_req_is_write,
    output wire [LADDR_W-1:0]  dma_coh_req_laddr,
    output wire [127:0]        dma_coh_req_wdata,
    output wire [15:0]         dma_coh_req_wstrb,

    input  wire                dma_coh_resp_valid,
    input  wire [127:0]        dma_coh_resp_rdata
);

localparam [1:0] AXI_BURST_INCR = 2'b01;
localparam [2:0] AXI_SIZE_16B   = 3'b100;
localparam [1:0] AXI_RESP_OKAY  = 2'b00;
localparam [1:0] AXI_RESP_SLVERR = 2'b10;
localparam [1:0] AXI_RESP_DECERR = 2'b11;

localparam [2:0] ST_IDLE     = 3'd0;
localparam [2:0] ST_RD_REQ   = 3'd1;
localparam [2:0] ST_RD_WAIT  = 3'd2;
localparam [2:0] ST_RD_RESP  = 3'd3;
localparam [2:0] ST_WR_DATA  = 3'd4;
localparam [2:0] ST_WR_REQ   = 3'd5;
localparam [2:0] ST_WR_WAIT  = 3'd6;
localparam [2:0] ST_WR_BRESP = 3'd7;

reg [2:0]   state;
reg         rr_rw;       // 0: prefer AR, 1: prefer AW
reg [31:0]  cur_addr;
reg [7:0]   beats_left;
reg         burst_ok;
reg [1:0]   wr_err;
reg [127:0] lat_wdata;
reg [15:0]  lat_wstrb;
reg         lat_wlast;

// Fair AR / AW acceptance in ST_IDLE (no combinational VALID<-READY loops)
wire pick_aw = (state == ST_IDLE) && s_axi_awvalid && (!s_axi_arvalid || rr_rw);
wire pick_ar = (state == ST_IDLE) && s_axi_arvalid && !pick_aw;

assign s_axi_arready = pick_ar;
assign s_axi_awready = pick_aw;
assign s_axi_wready  = (state == ST_WR_DATA);

wire cur_addr_oob = (cur_addr >= RAM_SIZE_BYTES);
wire rd_issue_coh = (state == ST_RD_REQ) && burst_ok && !cur_addr_oob;
wire wr_issue_coh = (state == ST_WR_REQ);

assign dma_coh_req_valid    = rd_issue_coh || wr_issue_coh;
assign dma_coh_req_is_write = wr_issue_coh;
assign dma_coh_req_laddr    = cur_addr[LADDR_W+3:4];
assign dma_coh_req_wdata    = lat_wdata;
assign dma_coh_req_wstrb    = lat_wstrb;

wire ar_cfg_ok = ((s_axi_arburst == AXI_BURST_INCR) || (s_axi_arlen == 8'd0)) &&
                 (s_axi_arsize == AXI_SIZE_16B);
wire aw_cfg_ok = ((s_axi_awburst == AXI_BURST_INCR) || (s_axi_awlen == 8'd0)) &&
                 (s_axi_awsize == AXI_SIZE_16B);

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state        <= ST_IDLE;
        rr_rw        <= 1'b0;
        cur_addr     <= 32'd0;
        beats_left   <= 8'd0;
        burst_ok     <= 1'b1;
        wr_err       <= AXI_RESP_OKAY;
        lat_wdata    <= 128'd0;
        lat_wstrb    <= 16'd0;
        lat_wlast    <= 1'b0;
        s_axi_rvalid <= 1'b0;
        s_axi_rdata  <= 128'd0;
        s_axi_rresp  <= AXI_RESP_OKAY;
        s_axi_rlast  <= 1'b0;
        s_axi_bvalid <= 1'b0;
        s_axi_bresp  <= AXI_RESP_OKAY;
    end else begin
        case (state)
            ST_IDLE: begin
                if (pick_ar) begin
                    cur_addr   <= {s_axi_araddr[31:4], 4'd0};
                    beats_left <= s_axi_arlen;
                    burst_ok   <= ar_cfg_ok;
                    rr_rw      <= 1'b1;
                    state      <= ST_RD_REQ;
                end else if (pick_aw) begin
                    cur_addr   <= {s_axi_awaddr[31:4], 4'd0};
                    beats_left <= s_axi_awlen;
                    burst_ok   <= aw_cfg_ok;
                    wr_err     <= aw_cfg_ok ? AXI_RESP_OKAY : AXI_RESP_SLVERR;
                    rr_rw      <= 1'b0;
                    state      <= ST_WR_DATA;
                end
            end

            ST_RD_REQ: begin
                if (!burst_ok || cur_addr_oob) begin
                    s_axi_rvalid <= 1'b1;
                    s_axi_rdata  <= 128'd0;
                    s_axi_rresp  <= !burst_ok ? AXI_RESP_SLVERR : AXI_RESP_DECERR;
                    s_axi_rlast  <= (beats_left == 8'd0);
                    state        <= ST_RD_RESP;
                end else if (dma_coh_req_ready) begin
                    state <= ST_RD_WAIT;
                end
            end

            ST_RD_WAIT: begin
                if (dma_coh_resp_valid) begin
                    s_axi_rvalid <= 1'b1;
                    s_axi_rdata  <= dma_coh_resp_rdata;
                    s_axi_rresp  <= AXI_RESP_OKAY;
                    s_axi_rlast  <= (beats_left == 8'd0);
                    state        <= ST_RD_RESP;
                end
            end

            ST_RD_RESP: begin
                if (s_axi_rvalid && s_axi_rready) begin
                    s_axi_rvalid <= 1'b0;
                    s_axi_rlast  <= 1'b0;
                    if (beats_left == 8'd0) begin
                        state <= ST_IDLE;
                    end else begin
                        cur_addr   <= cur_addr + 32'd16;
                        beats_left <= beats_left - 8'd1;
                        state      <= ST_RD_REQ;
                    end
                end
            end

            ST_WR_DATA: begin
                if (s_axi_wvalid) begin
                    lat_wdata <= s_axi_wdata;
                    lat_wstrb <= s_axi_wstrb;
                    lat_wlast <= s_axi_wlast;
                    if (!burst_ok || cur_addr_oob) begin
                        if (wr_err == AXI_RESP_OKAY)
                            wr_err <= !burst_ok ? AXI_RESP_SLVERR : AXI_RESP_DECERR;
                        if (beats_left == 8'd0 || s_axi_wlast) begin
                            s_axi_bvalid <= 1'b1;
                            s_axi_bresp  <= (wr_err != AXI_RESP_OKAY) ? wr_err :
                                            (!burst_ok ? AXI_RESP_SLVERR : AXI_RESP_DECERR);
                            state        <= ST_WR_BRESP;
                        end else begin
                            cur_addr   <= cur_addr + 32'd16;
                            beats_left <= beats_left - 8'd1;
                        end
                    end else if (s_axi_wstrb == 16'h0000) begin
                        // Legal AXI4 zero-strobe beat: modifies 0 bytes
                        if (beats_left == 8'd0 || s_axi_wlast) begin
                            s_axi_bvalid <= 1'b1;
                            s_axi_bresp  <= wr_err;
                            state        <= ST_WR_BRESP;
                        end else begin
                            cur_addr   <= cur_addr + 32'd16;
                            beats_left <= beats_left - 8'd1;
                        end
                    end else begin
                        state <= ST_WR_REQ;
                    end
                end
            end

            ST_WR_REQ: begin
                if (dma_coh_req_ready)
                    state <= ST_WR_WAIT;
            end

            ST_WR_WAIT: begin
                if (dma_coh_resp_valid) begin
                    if (beats_left == 8'd0 || lat_wlast) begin
                        s_axi_bvalid <= 1'b1;
                        s_axi_bresp  <= wr_err;
                        state        <= ST_WR_BRESP;
                    end else begin
                        cur_addr   <= cur_addr + 32'd16;
                        beats_left <= beats_left - 8'd1;
                        state      <= ST_WR_DATA;
                    end
                end
            end

            ST_WR_BRESP: begin
                if (s_axi_bvalid && s_axi_bready) begin
                    s_axi_bvalid <= 1'b0;
                    state        <= ST_IDLE;
                end
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
