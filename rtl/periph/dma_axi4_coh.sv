`timescale 1ns / 1ps

// ============================================================================
// 2-Channel AMBA AXI4 128-Bit I/O-Coherent DMA Engine w/ AMBA APB4 Control
// (Phase U7: Plug-and-Play SoC APB4 Slave S4 @ 0x1002_0000 + AXI4 Master)
//
// Features:
//   - Standardized AMBA APB4 Control Slave (4 KB @ 0x1002_0000):
//       Global Regs (0x000..0x008):
//         0x000 DMA_STATUS   (RO): [1:0]=busy, [9:8]=done, [17:16]=err
//         0x004 DMA_IRQ_STAT (W1C): [0]=ch0_irq, [1]=ch1_irq
//         0x008 DMA_VERSION  (RO): 32'h0002_0100 (2-Ch, 128-bit AXI4 v1.0)
//       Per-Channel Regs (CH0 @ 0x040, CH1 @ 0x080, stride 0x40):
//         +0x00 CH_SRC        (RW): Source byte address (any alignment!)
//         +0x04 CH_DST        (RW): Destination byte address (any alignment!)
//         +0x08 CH_LEN        (RW): Transfer length in bytes (or bytes/row in 2D)
//         +0x0C CH_SRC_STRIDE (RW): Row-to-row source stride in bytes (2D mode)
//         +0x10 CH_DST_STRIDE (RW): Row-to-row dest stride in bytes (2D mode)
//         +0x14 CH_ROW_CNT    (RW): Number of rows (2D mode)
//         +0x18 CH_CTRL       (RW): [0]=START, [1]=IRQ_EN, [2]=MODE_2D,
//                                   [3]=ABORT(W1P), [7:4]=BURST_MAX (0->16, 1..15)
//         +0x1C CH_STATUS     (RO): [0]=BUSY, [1]=DONE, [2]=ERR, [5:4]=ERR_CODE,
//                                   [31:8]=BYTES_DONE (24-bit live counter)
//   - Standardized AMBA AXI4 128-Bit Master:
//       * Up to 16-beat INCR bursts (256 bytes/burst), strictly obeying the
//         ARM AMBA AXI4 4 KB burst-boundary rule!
//       * Arbitrary byte-unaligned SRC, DST, and LEN via a compact 16x128b
//         burst buffer + 256-to-128b byte-alignment barrel shifter!
//       * Burst-granularity round-robin time-slicing between CH0 and CH1!
//       * Full RRESP / BRESP bus-error capture (DECERR / SLVERR -> ERR + IRQ)!
//   - Dedicated PLIC Level-Sensitive IRQs: irq_ch0_out (Src 4), irq_ch1_out (Src 5)
// ============================================================================
module dma_axi4_coh (
    input  wire         clk,
    input  wire         reset_n,

    // -------------------- AMBA APB4 Control Slave (0x1002_0000) --------------------
    input  wire         psel,
    input  wire         penable,
    input  wire         pwrite,
    input  wire [31:0]  paddr,
    input  wire [31:0]  pwdata,
    input  wire [3:0]   pstrb,
    output wire         pready,
    output reg  [31:0]  prdata,
    output reg          pslverr,

    // -------------------- AMBA AXI4 128-Bit Master Interface --------------------
    output reg          m_axi_arvalid,
    input  wire         m_axi_arready,
    output reg  [31:0]  m_axi_araddr,
    output reg  [7:0]   m_axi_arlen,
    output wire [2:0]   m_axi_arsize,
    output wire [1:0]   m_axi_arburst,

    input  wire         m_axi_rvalid,
    output wire         m_axi_rready,
    input  wire [127:0] m_axi_rdata,
    input  wire [1:0]   m_axi_rresp,
    input  wire         m_axi_rlast,

    output reg          m_axi_awvalid,
    input  wire         m_axi_awready,
    output reg  [31:0]  m_axi_awaddr,
    output reg  [7:0]   m_axi_awlen,
    output wire [2:0]   m_axi_awsize,
    output wire [1:0]   m_axi_awburst,

    output reg          m_axi_wvalid,
    input  wire         m_axi_wready,
    output reg  [127:0] m_axi_wdata,
    output reg  [15:0]  m_axi_wstrb,
    output reg          m_axi_wlast,

    input  wire         m_axi_bvalid,
    output wire         m_axi_bready,
    input  wire [1:0]   m_axi_bresp,

    // -------------------- PLIC Interrupt Outputs --------------------
    output wire         irq_ch0_out,
    output wire         irq_ch1_out
);

localparam [31:0] DMA_VERSION_VAL = 32'h0002_0100; // 2 channels, 128-bit AXI4 v1.0
localparam [2:0]  AXI_SIZE_16B    = 3'b100;
localparam [1:0]  AXI_BURST_INCR  = 2'b01;
localparam [1:0]  AXI_RESP_OKAY   = 2'b00;

assign m_axi_arsize  = AXI_SIZE_16B;
assign m_axi_arburst = AXI_BURST_INCR;
assign m_axi_awsize  = AXI_SIZE_16B;
assign m_axi_awburst = AXI_BURST_INCR;

// Helper for APB4 PSTRB byte-masked 32-bit register writes
function automatic [31:0] apply_pstrb(
    input [31:0] old_val,
    input [31:0] new_val,
    input [3:0]  strb
);
    begin
        apply_pstrb = {
            strb[3] ? new_val[31:24] : old_val[31:24],
            strb[2] ? new_val[23:16] : old_val[23:16],
            strb[1] ? new_val[15:8]  : old_val[15:8],
            strb[0] ? new_val[7:0]   : old_val[7:0]
        };
    end
endfunction

// ----------------------------------------------------------------------------
// 1. Per-Channel Configuration, Runtime & Status Registers (CH0 & CH1)
// ----------------------------------------------------------------------------
reg [31:0] ch_src        [0:1];
reg [31:0] ch_dst        [0:1];
reg [31:0] ch_len        [0:1];
reg [31:0] ch_src_stride [0:1];
reg [31:0] ch_dst_stride [0:1];
reg [31:0] ch_row_cnt    [0:1];
reg        ch_irq_en     [0:1];
reg        ch_mode_2d    [0:1];
reg [3:0]  ch_burst_max  [0:1]; // 0 -> 16 beats (default), 1..15 -> 1..15 beats

reg [31:0] ch_row_src    [0:1];
reg [31:0] ch_row_dst    [0:1];
reg [31:0] ch_cur_src    [0:1];
reg [31:0] ch_cur_dst    [0:1];
reg [31:0] ch_row_rem    [0:1];
reg [31:0] ch_rows_left  [0:1];
reg [23:0] ch_bytes_done [0:1];

reg [1:0]  ch_busy;
reg [1:0]  ch_done;
reg [1:0]  ch_err;
reg [1:0]  ch_err_code   [0:1];
reg [1:0]  ch_abort_req;
reg [1:0]  irq_stat;

assign irq_ch0_out = irq_stat[0];
assign irq_ch1_out = irq_stat[1];

// ----------------------------------------------------------------------------
// 2. Shared AXI4 Burst Master FSM & 16x128-Bit (256B) Alignment Buffer
// ----------------------------------------------------------------------------
localparam [2:0] M_IDLE = 3'd0;
localparam [2:0] M_AR   = 3'd1;
localparam [2:0] M_R    = 3'd2;
localparam [2:0] M_AW   = 3'd3;
localparam [2:0] M_W    = 3'd4;
localparam [2:0] M_B    = 3'd5;

reg [2:0]   m_state;
reg         rr_ch;          // 0: prefer CH0, 1: prefer CH1
reg         act_ch;
reg [3:0]   act_src_off;
reg [3:0]   act_dst_off;
reg [8:0]   act_chunk_bytes; // 1..256 bytes in current read+write burst
reg [3:0]   act_wr_len;      // AWLEN (0..15 => 1..16 beats)
reg [3:0]   rd_idx;
reg [3:0]   wr_idx;
reg [1:0]   rd_err;

reg [127:0] burst_buf [0:15];

assign m_axi_rready = (m_state == M_R);
assign m_axi_bready = (m_state == M_B);

// Next channel selection & burst-chunk geometry computation (in M_IDLE)
wire        next_ch_valid = ch_busy[0] || ch_busy[1];
wire        next_ch       = (ch_busy[1] && (!ch_busy[0] || rr_ch)) ? 1'b1 : 1'b0;
wire [31:0] nc_src        = next_ch ? ch_cur_src[1]   : ch_cur_src[0];
wire [31:0] nc_dst        = next_ch ? ch_cur_dst[1]   : ch_cur_dst[0];
wire [31:0] nc_rem        = next_ch ? ch_row_rem[1]   : ch_row_rem[0];
wire [3:0]  nc_bmax_raw   = next_ch ? ch_burst_max[1] : ch_burst_max[0];
wire [4:0]  nc_max_beats  = (nc_bmax_raw == 4'd0) ? 5'd16 : {1'b0, nc_bmax_raw};
wire [8:0]  nc_max_bytes  = {nc_max_beats, 4'd0}; // 16..256 bytes

wire [3:0]  nc_src_off    = nc_src[3:0];
wire [3:0]  nc_dst_off    = nc_dst[3:0];

// Guarantee rd_beats <= nc_max_beats, wr_beats <= nc_max_beats, and 4 KB AXI boundary!
wire [8:0]  nc_cap_src    = nc_max_bytes - {5'd0, nc_src_off};
wire [8:0]  nc_cap_dst    = nc_max_bytes - {5'd0, nc_dst_off};
wire [12:0] nc_4k_src     = 13'd4096 - {1'b0, nc_src[11:0]};
wire [12:0] nc_4k_dst     = 13'd4096 - {1'b0, nc_dst[11:0]};
wire [8:0]  nc_cap_4k_s   = (nc_4k_src >= 13'd256) ? 9'd256 : nc_4k_src[8:0];
wire [8:0]  nc_cap_4k_d   = (nc_4k_dst >= 13'd256) ? 9'd256 : nc_4k_dst[8:0];
wire [8:0]  nc_cap_rem    = (nc_rem >= 32'd256) ? 9'd256 : nc_rem[8:0];

wire [8:0]  nc_min0       = (nc_cap_src  < nc_cap_dst)  ? nc_cap_src  : nc_cap_dst;
wire [8:0]  nc_min1       = (nc_cap_4k_s < nc_cap_4k_d) ? nc_cap_4k_s : nc_cap_4k_d;
wire [8:0]  nc_min2       = (nc_min0     < nc_min1)     ? nc_min0     : nc_min1;
wire [8:0]  nc_chunk      = (nc_cap_rem  < nc_min2)     ? nc_cap_rem  : nc_min2;

wire [8:0]  nc_rd_end     = {5'd0, nc_src_off} + nc_chunk - 9'd1;
wire [8:0]  nc_wr_end     = {5'd0, nc_dst_off} + nc_chunk - 9'd1;
wire [3:0]  nc_arlen      = nc_rd_end[7:4]; // 0..15 -> 1..16 beats
wire [3:0]  nc_awlen      = nc_wr_end[7:4]; // 0..15 -> 1..16 beats

// ----------------------------------------------------------------------------
// 3. Compact 256-to-128-Bit Byte-Alignment Shifter for AXI Write Beat Gen
// ----------------------------------------------------------------------------
// Beat index whose WDATA/WSTRB we prepare next:
//   - On M_AW handshake -> beat 0
//   - On M_W handshake  -> beat wr_idx + 1
wire [3:0] prep_idx    = (m_state == M_AW) ? 4'd0 : (wr_idx + 4'd1);
wire       src_ge_dst  = (act_src_off >= act_dst_off);
wire [3:0] shift_bytes = src_ge_dst ? (act_src_off - act_dst_off)
                                    : (4'd0 - (act_dst_off - act_src_off)); // = 16 - (dst-src)

wire [127:0] line_lo   = src_ge_dst
                       ? burst_buf[prep_idx]
                       : ((prep_idx == 4'd0) ? 128'd0 : burst_buf[prep_idx - 4'd1]);
wire [127:0] line_hi   = src_ge_dst
                       ? ((prep_idx == 4'd15) ? 128'd0 : burst_buf[prep_idx + 4'd1])
                       : burst_buf[prep_idx];

wire [255:0] pair_lines = {line_hi, line_lo};
wire [7:0]   shift_bits = {1'b0, shift_bytes, 3'b000};
wire [127:0] prep_wdata = pair_lines[shift_bits +: 128];

wire [8:0]   dst_end_excl = {5'd0, act_dst_off} + act_chunk_bytes;
reg  [15:0]  prep_wstrb;
integer      sb;
always @(*) begin
    for (sb = 0; sb < 16; sb = sb + 1) begin
        if (({1'b0, prep_idx, sb[3:0]} >= {5'd0, act_dst_off}) &&
            ({1'b0, prep_idx, sb[3:0]} <  dst_end_excl))
            prep_wstrb[sb] = 1'b1;
        else
            prep_wstrb[sb] = 1'b0;
    end
end

// ----------------------------------------------------------------------------
// 4. AMBA APB4 Slave Read / Write Decode (0-Wait-State, Strict PSLVERR)
// ----------------------------------------------------------------------------
assign pready = 1'b1;

wire [11:0] reg_off   = paddr[11:0];
wire        apb_acc   = psel && penable;
wire        apb_wr    = apb_acc && pwrite;
wire        is_ch0    = (reg_off[11:6] == 6'h01); // 0x040..0x07F
wire        is_ch1    = (reg_off[11:6] == 6'h02); // 0x080..0x0BF
wire [5:0]  ch_off    = reg_off[5:0];
wire        ch_sel    = is_ch1;

wire [31:0] ch0_ctrl_rd = {24'd0, ch_burst_max[0], 1'b0, ch_mode_2d[0], ch_irq_en[0], ch_busy[0]};
wire [31:0] ch1_ctrl_rd = {24'd0, ch_burst_max[1], 1'b0, ch_mode_2d[1], ch_irq_en[1], ch_busy[1]};
wire [31:0] ch0_stat_rd = {ch_bytes_done[0], 2'b00, ch_err_code[0], 1'b0, ch_err[0], ch_done[0], ch_busy[0]};
wire [31:0] ch1_stat_rd = {ch_bytes_done[1], 2'b00, ch_err_code[1], 1'b0, ch_err[1], ch_done[1], ch_busy[1]};

always @(*) begin
    prdata  = 32'd0;
    pslverr = 1'b0;
    if (psel) begin
        if (reg_off[11:5] == 7'h00) begin
            case (reg_off[4:0])
                5'h00: begin
                    prdata  = {14'd0, ch_err, 6'd0, ch_done, 6'd0, ch_busy};
                    pslverr = pwrite; // DMA_STATUS is RO
                end
                5'h04: begin
                    prdata  = {30'd0, irq_stat};
                    pslverr = 1'b0;   // DMA_IRQ_STAT is R/W1C
                end
                5'h08: begin
                    prdata  = DMA_VERSION_VAL;
                    pslverr = pwrite; // DMA_VERSION is RO
                end
                default: begin
                    prdata  = 32'd0;
                    pslverr = 1'b1;
                end
            endcase
        end else if (is_ch0 || is_ch1) begin
            case (ch_off)
                6'h00: begin
                    prdata  = ch_sel ? ch_src[1] : ch_src[0];
                    pslverr = pwrite && ch_busy[ch_sel];
                end
                6'h04: begin
                    prdata  = ch_sel ? ch_dst[1] : ch_dst[0];
                    pslverr = pwrite && ch_busy[ch_sel];
                end
                6'h08: begin
                    prdata  = ch_sel ? ch_len[1] : ch_len[0];
                    pslverr = pwrite && ch_busy[ch_sel];
                end
                6'h0C: begin
                    prdata  = ch_sel ? ch_src_stride[1] : ch_src_stride[0];
                    pslverr = pwrite && ch_busy[ch_sel];
                end
                6'h10: begin
                    prdata  = ch_sel ? ch_dst_stride[1] : ch_dst_stride[0];
                    pslverr = pwrite && ch_busy[ch_sel];
                end
                6'h14: begin
                    prdata  = ch_sel ? ch_row_cnt[1] : ch_row_cnt[0];
                    pslverr = pwrite && ch_busy[ch_sel];
                end
                6'h18: begin
                    prdata  = ch_sel ? ch1_ctrl_rd : ch0_ctrl_rd;
                    pslverr = 1'b0; // CTRL allows ABORT while busy!
                end
                6'h1C: begin
                    prdata  = ch_sel ? ch1_stat_rd : ch0_stat_rd;
                    pslverr = pwrite; // CH_STATUS is RO
                end
                default: begin
                    prdata  = 32'd0;
                    pslverr = 1'b1;
                end
            endcase
        end else begin
            prdata  = 32'd0;
            pslverr = 1'b1;
        end
    end
end

// ----------------------------------------------------------------------------
// 5. Sequential State Machine & Register Updates
// ----------------------------------------------------------------------------
integer ci, bi;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (ci = 0; ci < 2; ci = ci + 1) begin
            ch_src[ci]        <= 32'd0;
            ch_dst[ci]        <= 32'd0;
            ch_len[ci]        <= 32'd0;
            ch_src_stride[ci] <= 32'd0;
            ch_dst_stride[ci] <= 32'd0;
            ch_row_cnt[ci]    <= 32'd1;
            ch_irq_en[ci]     <= 1'b0;
            ch_mode_2d[ci]    <= 1'b0;
            ch_burst_max[ci]  <= 4'd0; // 0 -> 16-beat default burst
            ch_row_src[ci]    <= 32'd0;
            ch_row_dst[ci]    <= 32'd0;
            ch_cur_src[ci]    <= 32'd0;
            ch_cur_dst[ci]    <= 32'd0;
            ch_row_rem[ci]    <= 32'd0;
            ch_rows_left[ci]  <= 32'd0;
            ch_bytes_done[ci] <= 24'd0;
            ch_err_code[ci]   <= 2'd0;
        end
        ch_busy         <= 2'b00;
        ch_done         <= 2'b00;
        ch_err          <= 2'b00;
        ch_abort_req    <= 2'b00;
        irq_stat        <= 2'b00;
        m_state         <= M_IDLE;
        rr_ch           <= 1'b0;
        act_ch          <= 1'b0;
        act_src_off     <= 4'd0;
        act_dst_off     <= 4'd0;
        act_chunk_bytes <= 9'd0;
        act_wr_len      <= 4'd0;
        rd_idx          <= 4'd0;
        wr_idx          <= 4'd0;
        rd_err          <= AXI_RESP_OKAY;
        for (bi = 0; bi < 16; bi = bi + 1)
            burst_buf[bi] <= 128'd0;
        m_axi_arvalid   <= 1'b0;
        m_axi_araddr    <= 32'd0;
        m_axi_arlen     <= 8'd0;
        m_axi_awvalid   <= 1'b0;
        m_axi_awaddr    <= 32'd0;
        m_axi_awlen     <= 8'd0;
        m_axi_wvalid    <= 1'b0;
        m_axi_wdata     <= 128'd0;
        m_axi_wstrb     <= 16'd0;
        m_axi_wlast     <= 1'b0;
    end else begin
        // --- 5a. AMBA APB4 Register Writes ---
        if (apb_wr && !pslverr) begin
            if (reg_off == 12'h004) begin
                // DMA_IRQ_STAT W1C
                if (pstrb[0])
                    irq_stat <= irq_stat & ~pwdata[1:0];
            end else if (is_ch0 || is_ch1) begin
                case (ch_off)
                    6'h00: ch_src[ch_sel]        <= apply_pstrb(ch_src[ch_sel],        pwdata, pstrb);
                    6'h04: ch_dst[ch_sel]        <= apply_pstrb(ch_dst[ch_sel],        pwdata, pstrb);
                    6'h08: ch_len[ch_sel]        <= apply_pstrb(ch_len[ch_sel],        pwdata, pstrb);
                    6'h0C: ch_src_stride[ch_sel] <= apply_pstrb(ch_src_stride[ch_sel], pwdata, pstrb);
                    6'h10: ch_dst_stride[ch_sel] <= apply_pstrb(ch_dst_stride[ch_sel], pwdata, pstrb);
                    6'h14: ch_row_cnt[ch_sel]    <= apply_pstrb(ch_row_cnt[ch_sel],    pwdata, pstrb);
                    6'h18: begin
                        if (pstrb[0]) begin
                            if (pwdata[3]) begin
                                // ABORT request
                                if (ch_busy[ch_sel])
                                    ch_abort_req[ch_sel] <= 1'b1;
                                ch_done[ch_sel] <= 1'b0;
                                ch_err[ch_sel]  <= 1'b0;
                            end else if (!ch_busy[ch_sel]) begin
                                ch_irq_en[ch_sel]    <= pwdata[1];
                                ch_mode_2d[ch_sel]   <= pwdata[2];
                                ch_burst_max[ch_sel] <= pwdata[7:4];
                                if (pwdata[0]) begin
                                    // START channel transfer!
                                    ch_done[ch_sel]       <= 1'b0;
                                    ch_err[ch_sel]        <= 1'b0;
                                    ch_err_code[ch_sel]   <= 2'd0;
                                    ch_abort_req[ch_sel]  <= 1'b0;
                                    ch_bytes_done[ch_sel] <= 24'd0;
                                    if ((ch_len[ch_sel] == 32'd0) ||
                                        (pwdata[2] && (ch_row_cnt[ch_sel] == 32'd0))) begin
                                        // Zero-length transfer completes immediately
                                        ch_busy[ch_sel] <= 1'b0;
                                        ch_done[ch_sel] <= 1'b1;
                                        if (pwdata[1])
                                            irq_stat[ch_sel] <= 1'b1;
                                    end else begin
                                        ch_busy[ch_sel]      <= 1'b1;
                                        ch_row_src[ch_sel]   <= ch_src[ch_sel];
                                        ch_row_dst[ch_sel]   <= ch_dst[ch_sel];
                                        ch_cur_src[ch_sel]   <= ch_src[ch_sel];
                                        ch_cur_dst[ch_sel]   <= ch_dst[ch_sel];
                                        ch_row_rem[ch_sel]   <= ch_len[ch_sel];
                                        ch_rows_left[ch_sel] <= pwdata[2] ? ch_row_cnt[ch_sel] : 32'd1;
                                    end
                                end
                            end
                        end
                    end
                    default: ;
                endcase
            end
        end

        // --- 5b. Shared AMBA AXI4 128-Bit Master Burst Engine ---
        case (m_state)
            M_IDLE: begin
                if (next_ch_valid) begin
                    if (ch_abort_req[next_ch]) begin
                        ch_busy[next_ch]      <= 1'b0;
                        ch_abort_req[next_ch] <= 1'b0;
                        rr_ch                 <= !next_ch;
                    end else begin
                        act_ch          <= next_ch;
                        act_src_off     <= nc_src_off;
                        act_dst_off     <= nc_dst_off;
                        act_chunk_bytes <= nc_chunk;
                        act_wr_len      <= nc_awlen;
                        m_axi_arvalid   <= 1'b1;
                        m_axi_araddr    <= {nc_src[31:4], 4'd0};
                        m_axi_arlen     <= {4'd0, nc_arlen};
                        m_state         <= M_AR;
                    end
                end
            end

            M_AR: begin
                if (m_axi_arvalid && m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    rd_idx        <= 4'd0;
                    rd_err        <= AXI_RESP_OKAY;
                    m_state       <= M_R;
                end
            end

            M_R: begin
                if (m_axi_rvalid) begin
                    burst_buf[rd_idx] <= m_axi_rdata;
                    if ((m_axi_rresp != AXI_RESP_OKAY) && (rd_err == AXI_RESP_OKAY))
                        rd_err <= m_axi_rresp;
                    if (m_axi_rlast) begin
                        if ((rd_err != AXI_RESP_OKAY) || (m_axi_rresp != AXI_RESP_OKAY)) begin
                            // Read bus error (DECERR / SLVERR): abort cleanly & raise IRQ!
                            ch_busy[act_ch]      <= 1'b0;
                            ch_err[act_ch]       <= 1'b1;
                            ch_err_code[act_ch]  <= (rd_err != AXI_RESP_OKAY) ? rd_err : m_axi_rresp;
                            ch_abort_req[act_ch] <= 1'b0;
                            if (ch_irq_en[act_ch])
                                irq_stat[act_ch] <= 1'b1;
                            rr_ch   <= !act_ch;
                            m_state <= M_IDLE;
                        end else if (ch_abort_req[act_ch]) begin
                            ch_busy[act_ch]      <= 1'b0;
                            ch_abort_req[act_ch] <= 1'b0;
                            rr_ch                <= !act_ch;
                            m_state              <= M_IDLE;
                        end else begin
                            m_axi_awvalid <= 1'b1;
                            m_axi_awaddr  <= {ch_cur_dst[act_ch][31:4], 4'd0};
                            m_axi_awlen   <= {4'd0, act_wr_len};
                            m_state       <= M_AW;
                        end
                    end else begin
                        rd_idx <= rd_idx + 4'd1;
                    end
                end
            end

            M_AW: begin
                if (m_axi_awvalid && m_axi_awready) begin
                    m_axi_awvalid <= 1'b0;
                    wr_idx        <= 4'd0;
                    m_axi_wvalid  <= 1'b1;
                    m_axi_wdata   <= prep_wdata;
                    m_axi_wstrb   <= prep_wstrb;
                    m_axi_wlast   <= (act_wr_len == 4'd0);
                    m_state       <= M_W;
                end
            end

            M_W: begin
                if (m_axi_wvalid && m_axi_wready) begin
                    if (m_axi_wlast) begin
                        m_axi_wvalid <= 1'b0;
                        m_axi_wlast  <= 1'b0;
                        m_state      <= M_B;
                    end else begin
                        wr_idx      <= wr_idx + 4'd1;
                        m_axi_wdata <= prep_wdata;
                        m_axi_wstrb <= prep_wstrb;
                        m_axi_wlast <= ((wr_idx + 4'd1) == act_wr_len);
                    end
                end
            end

            M_B: begin
                if (m_axi_bvalid) begin
                    if (m_axi_bresp != AXI_RESP_OKAY) begin
                        ch_busy[act_ch]      <= 1'b0;
                        ch_err[act_ch]       <= 1'b1;
                        ch_err_code[act_ch]  <= m_axi_bresp;
                        ch_abort_req[act_ch] <= 1'b0;
                        if (ch_irq_en[act_ch])
                            irq_stat[act_ch] <= 1'b1;
                    end else if (ch_abort_req[act_ch]) begin
                        ch_busy[act_ch]      <= 1'b0;
                        ch_abort_req[act_ch] <= 1'b0;
                    end else begin
                        ch_bytes_done[act_ch] <= ch_bytes_done[act_ch] + {15'd0, act_chunk_bytes};
                        if (ch_row_rem[act_ch] > {23'd0, act_chunk_bytes}) begin
                            ch_row_rem[act_ch] <= ch_row_rem[act_ch] - {23'd0, act_chunk_bytes};
                            ch_cur_src[act_ch] <= ch_cur_src[act_ch] + {23'd0, act_chunk_bytes};
                            ch_cur_dst[act_ch] <= ch_cur_dst[act_ch] + {23'd0, act_chunk_bytes};
                        end else if (ch_rows_left[act_ch] > 32'd1) begin
                            // Advance to next 2D strided row!
                            ch_rows_left[act_ch] <= ch_rows_left[act_ch] - 32'd1;
                            ch_row_src[act_ch]   <= ch_row_src[act_ch] + ch_src_stride[act_ch];
                            ch_row_dst[act_ch]   <= ch_row_dst[act_ch] + ch_dst_stride[act_ch];
                            ch_cur_src[act_ch]   <= ch_row_src[act_ch] + ch_src_stride[act_ch];
                            ch_cur_dst[act_ch]   <= ch_row_dst[act_ch] + ch_dst_stride[act_ch];
                            ch_row_rem[act_ch]   <= ch_len[act_ch];
                        end else begin
                            // Channel transfer complete!
                            ch_busy[act_ch] <= 1'b0;
                            ch_done[act_ch] <= 1'b1;
                            if (ch_irq_en[act_ch])
                                irq_stat[act_ch] <= 1'b1;
                        end
                    end
                    rr_ch   <= !act_ch;
                    m_state <= M_IDLE;
                end
            end

            default: m_state <= M_IDLE;
        endcase
    end
end

endmodule
