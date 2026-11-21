`timescale 1ns / 1ps

// ============================================================================
// Standardized AMBA APB4 Full-Duplex 8N1 UART with 16-Entry TX/RX FIFOs (U4)
// Mapped at 0x1001_0000 .. 0x1001_0FFF (APB4 Slave 3, PLIC Sources 2 & 3)
//
// Features:
//   - 16-entry circular TX FIFO and 16-entry circular RX FIFO
//   - Real 8N1 serial transmitter & 1-start/8-data/1-stop receiver with
//     configurable 16-bit BAUD_DIV (clocks per bit, >= 2)
//   - Hardware internal LOOPBACK mode (CTRL[2]) routing uart_tx -> rx_pin
//   - Optional FAST_SIM bypass mode (CTRL[3]) for rapid simulation console I/O
//   - Clean 1-cycle tx_char_valid / tx_char_data console monitor tap
//   - Separate PLIC interrupt outputs:
//       * rx_irq_out (PLIC Source 2): rx_irq_en && !rx_empty
//       * tx_irq_out (PLIC Source 3): tx_irq_en &&  tx_empty && !tx_busy
//
// Register Map (offset = {paddr[11:2], 2'b00}):
//   0x000 : TX_DATA  (WO) - Push byte [7:0] into TX FIFO
//   0x004 : RX_DATA  (RO) - Pop  byte [7:0] from RX FIFO
//   0x008 : STATUS   (RO) - [0]TX_FULL [1]TX_EMPTY [2]RX_EMPTY [3]RX_FULL
//                           [4]TX_BUSY [12:8]TX_LEVEL [20:16]RX_LEVEL
//   0x00C : CTRL     (RW) - [0]TX_EN(1) [1]RX_EN(1) [2]LOOPBACK(0)
//                           [3]FAST_SIM(0) [4]RX_IRQ_EN(0) [5]TX_IRQ_EN(0)
//   0x010 : BAUD_DIV (RW) - 16-bit clocks per serial bit (reset 16'd8)
// ============================================================================
module apb4_uart (
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

    // Serial 8N1 Pins, Console Tap & PLIC Interrupt Outputs
    input  wire        uart_rx,
    output wire        uart_tx,
    output reg         tx_char_valid,
    output reg  [7:0]  tx_char_data,
    output wire        rx_irq_out,
    output wire        tx_irq_out
);

assign pready  = 1'b1;
assign pslverr = 1'b0;

// ----------------------------------------------------------------------------
// 1. Control Registers & 16-Entry TX / RX FIFOs
// ----------------------------------------------------------------------------
reg        tx_en_r;
reg        rx_en_r;
reg        loopback_r;
reg        fast_sim_r;
reg        rx_irq_en_r;
reg        tx_irq_en_r;
reg [15:0] baud_div_r;

reg [7:0]  tx_fifo [0:15];
reg [4:0]  tx_head, tx_tail;
wire [4:0] tx_count = tx_tail - tx_head;
wire       tx_empty = (tx_count == 5'd0);
wire       tx_full  = (tx_count == 5'd16);

reg [7:0]  rx_fifo [0:15];
reg [4:0]  rx_head, rx_tail;
wire [4:0] rx_count = rx_tail - rx_head;
wire       rx_empty = (rx_count == 5'd0);
wire       rx_full  = (rx_count == 5'd16);

// 8N1 TX Serializer State
reg        tx_busy_r;
reg [9:0]  tx_shift_r;   // {stop=1, data[7:0], start=0}
reg [3:0]  tx_bit_idx_r; // 0..9
reg [15:0] tx_baud_cnt_r;

// 8N1 RX Deserializer State
reg        rx_sync1_r, rx_sync2_r;
reg        rx_busy_r;
reg [7:0]  rx_shift_r;
reg [3:0]  rx_bit_idx_r; // 0=start-mid, 1..8=data, 9=stop
reg [15:0] rx_baud_cnt_r;

assign uart_tx    = tx_busy_r ? tx_shift_r[0] : 1'b1;
assign rx_irq_out = rx_irq_en_r && !rx_empty;
assign tx_irq_out = tx_irq_en_r &&  tx_empty && !tx_busy_r;

wire [15:0] baud_eff = (baud_div_r < 16'd2) ? 16'd2 : baud_div_r;
wire [15:0] half_baud = {1'b0, baud_eff[15:1]};
wire        rx_line   = loopback_r ? uart_tx : rx_sync2_r;

// ----------------------------------------------------------------------------
// 2. APB4 Decode & Combinational Read Mux
// ----------------------------------------------------------------------------
wire [11:0] reg_off   = {paddr[11:2], 2'b00};
wire        apb_write = psel && penable &&  pwrite;
wire        apb_read  = psel && penable && !pwrite;

wire [31:0] status_word = {
    11'd0,
    rx_count,          // [20:16]
    3'd0,
    tx_count,          // [12:8]
    3'd0,
    tx_busy_r,         // [4]
    rx_full,           // [3]
    rx_empty,          // [2]
    tx_empty,          // [1]
    tx_full            // [0]
};

wire [31:0] ctrl_word = {
    26'd0,
    tx_irq_en_r,       // [5]
    rx_irq_en_r,       // [4]
    fast_sim_r,        // [3]
    loopback_r,        // [2]
    rx_en_r,           // [1]
    tx_en_r            // [0]
};

always @(*) begin
    case (reg_off)
        12'h004: prdata = rx_empty ? 32'd0 : {24'd0, rx_fifo[rx_head[3:0]]};
        12'h008: prdata = status_word;
        12'h00C: prdata = ctrl_word;
        12'h010: prdata = {16'd0, baud_div_r};
        default: prdata = 32'd0;
    endcase
end

// ----------------------------------------------------------------------------
// 3. Sequential FIFO, 8N1 TX/RX FSMs & Fast-Sim Console Hook
// ----------------------------------------------------------------------------
wire tx_push_req = apb_write && (reg_off == 12'h000) && pstrb[0] && !tx_full;
wire rx_pop_req  = apb_read  && (reg_off == 12'h004) && !rx_empty;

wire tx_start_now = tx_en_r && !tx_empty && (!tx_busy_r || fast_sim_r);
wire fast_lb_push = tx_start_now && fast_sim_r && loopback_r && rx_en_r;

integer i;
always @(posedge pclk or negedge preset_n) begin
    if (!preset_n) begin
        tx_en_r       <= 1'b1;
        rx_en_r       <= 1'b1;
        loopback_r    <= 1'b0;
        fast_sim_r    <= 1'b0;
        rx_irq_en_r   <= 1'b0;
        tx_irq_en_r   <= 1'b0;
        baud_div_r    <= 16'd8;
        tx_head       <= 5'd0;
        tx_tail       <= 5'd0;
        rx_head       <= 5'd0;
        rx_tail       <= 5'd0;
        tx_busy_r     <= 1'b0;
        tx_shift_r    <= 10'h3FF;
        tx_bit_idx_r  <= 4'd0;
        tx_baud_cnt_r <= 16'd0;
        rx_sync1_r    <= 1'b1;
        rx_sync2_r    <= 1'b1;
        rx_busy_r     <= 1'b0;
        rx_shift_r    <= 8'd0;
        rx_bit_idx_r  <= 4'd0;
        rx_baud_cnt_r <= 16'd0;
        tx_char_valid <= 1'b0;
        tx_char_data  <= 8'd0;
        for (i = 0; i < 16; i = i + 1) begin
            tx_fifo[i] <= 8'd0;
            rx_fifo[i] <= 8'd0;
        end
    end else begin
        tx_char_valid <= 1'b0;
        rx_sync1_r    <= uart_rx;
        rx_sync2_r    <= rx_sync1_r;

        // A. APB4 Register Writes & TX FIFO Push
        if (tx_push_req) begin
            tx_fifo[tx_tail[3:0]] <= pwdata[7:0];
            tx_tail               <= tx_tail + 5'd1;
        end

        if (apb_write) begin
            case (reg_off)
                12'h00C: if (pstrb[0]) begin
                    tx_en_r     <= pwdata[0];
                    rx_en_r     <= pwdata[1];
                    loopback_r  <= pwdata[2];
                    fast_sim_r  <= pwdata[3];
                    rx_irq_en_r <= pwdata[4];
                    tx_irq_en_r <= pwdata[5];
                end
                12'h010: begin
                    if (pstrb[0]) baud_div_r[7:0]  <= pwdata[7:0];
                    if (pstrb[1]) baud_div_r[15:8] <= pwdata[15:8];
                end
                default: ;
            endcase
        end

        // B. TX Serializer (FAST_SIM 1-cyc drain OR real 8N1 serial FSM!)
        if (fast_sim_r) begin
            tx_busy_r <= 1'b0;
            if (tx_en_r && !tx_empty) begin
                tx_char_valid <= 1'b1;
                tx_char_data  <= tx_fifo[tx_head[3:0]];
                tx_head       <= tx_head + 5'd1;
            end
        end else if (!tx_busy_r) begin
            if (tx_en_r && !tx_empty) begin
                tx_busy_r     <= 1'b1;
                tx_shift_r    <= {1'b1, tx_fifo[tx_head[3:0]], 1'b0}; // stop, data, start
                tx_bit_idx_r  <= 4'd0;
                tx_baud_cnt_r <= baud_eff - 16'd1;
                tx_char_valid <= 1'b1;
                tx_char_data  <= tx_fifo[tx_head[3:0]];
                tx_head       <= tx_head + 5'd1;
            end
        end else begin
            if (tx_baud_cnt_r == 16'd0) begin
                if (tx_bit_idx_r == 4'd9) begin
                    tx_busy_r <= 1'b0;
                end else begin
                    tx_shift_r    <= {1'b1, tx_shift_r[9:1]};
                    tx_bit_idx_r  <= tx_bit_idx_r + 4'd1;
                    tx_baud_cnt_r <= baud_eff - 16'd1;
                end
            end else begin
                tx_baud_cnt_r <= tx_baud_cnt_r - 16'd1;
            end
        end

        // C. RX FIFO Pop & Receiver (FAST_SIM loopback OR real 8N1 serial RX!)
        if (rx_pop_req) begin
            rx_head <= rx_head + 5'd1;
        end

        if (fast_lb_push) begin
            if (!rx_full || rx_pop_req) begin
                rx_fifo[rx_tail[3:0]] <= tx_fifo[tx_head[3:0]];
                rx_tail               <= rx_tail + 5'd1;
            end
            rx_busy_r <= 1'b0;
        end else if (!rx_en_r) begin
            rx_busy_r <= 1'b0;
        end else if (!rx_busy_r) begin
            // Detect start bit (rx_line == 0) and schedule mid-bit sample
            if (rx_line == 1'b0) begin
                rx_busy_r     <= 1'b1;
                rx_bit_idx_r  <= 4'd0;
                rx_baud_cnt_r <= (half_baud > 16'd1) ? (half_baud - 16'd1) : 16'd0;
            end
        end else begin
            if (rx_baud_cnt_r == 16'd0) begin
                if (rx_bit_idx_r == 4'd0) begin
                    // Verify valid start bit at center
                    if (rx_line == 1'b0) begin
                        rx_bit_idx_r  <= 4'd1;
                        rx_baud_cnt_r <= baud_eff - 16'd1;
                    end else begin
                        rx_busy_r <= 1'b0; // Glitch rejected
                    end
                end else if (rx_bit_idx_r <= 4'd8) begin
                    rx_shift_r    <= {rx_line, rx_shift_r[7:1]}; // LSB first
                    rx_bit_idx_r  <= rx_bit_idx_r + 4'd1;
                    rx_baud_cnt_r <= baud_eff - 16'd1;
                end else begin
                    // Stop bit center -> push byte to RX FIFO if stop bit == 1
                    rx_busy_r <= 1'b0;
                    if (rx_line == 1'b1 && (!rx_full || rx_pop_req)) begin
                        rx_fifo[rx_tail[3:0]] <= rx_shift_r;
                        rx_tail               <= rx_tail + 5'd1;
                    end
                end
            end else begin
                rx_baud_cnt_r <= rx_baud_cnt_r - 16'd1;
            end
        end
    end
end

endmodule
