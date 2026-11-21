`timescale 1ns / 1ps

// ============================================================================
// Unit Testbench for AMBA APB4 32-Pin GPIO, 16-FIFO 8N1 UART & 2-Ch Timer/PWM
// (Phase U4 — Icarus Verilog + Verilator --assert -Wall + APB4 Protocol Check)
// ============================================================================
module tb_apb4_peripherals;

`include "soc_addr_map.vh"

localparam integer ID_W       = 7;
localparam integer NUM_SLAVES = 7;
// Enable S2=GPIO, S3=UART, S5=TIMER
localparam [15:0]  SLV_MASK   = 16'b0000_0000_0010_1100;

reg             clk, reset_n;
reg             c0_req_valid, c0_req_is_write;
wire            c0_req_ready;
reg  [31:0]     c0_req_addr, c0_req_wdata;
reg  [3:0]      c0_req_wstrb;
reg  [ID_W-1:0] c0_req_id;
wire            c0_resp_valid, c0_resp_err;
wire [ID_W-1:0] c0_resp_id;
wire [31:0]     c0_resp_rdata;
wire            c1_rdy_u, c1_rv_u, c1_err_u;
wire [ID_W-1:0] c1_rid_u;
wire [31:0]     c1_rd_u;

wire            m_psel, m_penable, m_pwrite, m_pready, m_pslverr;
wire [31:0]     m_paddr, m_pwdata, m_prdata;
wire [3:0]      m_pstrb;
wire [NUM_SLAVES-1:0]    s_psel, s_pready, s_pslverr;
wire [NUM_SLAVES*32-1:0] s_prdata;
wire            s_penable, s_pwrite;
wire [31:0]     s_paddr, s_pwdata;
wire [3:0]      s_pstrb;

// Peripheral external signals
reg  [31:0] gpio_in;
wire [31:0] gpio_out, gpio_oe;
wire        gpio_irq;

wire        uart_tx, uart_tx_ch_v, uart_rx_irq, uart_tx_irq;
wire [7:0]  uart_tx_ch_d;

wire [1:0]  pwm_out;
wire        tmr0_irq, tmr1_irq;

mmio_to_apb4_bridge #(.ID_W(ID_W)) u_bridge (
    .clk(clk), .reset_n(reset_n),
    .c0_req_valid(c0_req_valid), .c0_req_ready(c0_req_ready),
    .c0_req_is_write(c0_req_is_write), .c0_req_addr(c0_req_addr),
    .c0_req_wdata(c0_req_wdata), .c0_req_wstrb(c0_req_wstrb), .c0_req_id(c0_req_id),
    .c0_resp_valid(c0_resp_valid), .c0_resp_id(c0_resp_id),
    .c0_resp_rdata(c0_resp_rdata), .c0_resp_err(c0_resp_err),
    .c1_req_valid(1'b0), .c1_req_ready(c1_rdy_u),
    .c1_req_is_write(1'b0), .c1_req_addr(32'd0),
    .c1_req_wdata(32'd0), .c1_req_wstrb(4'd0), .c1_req_id(7'd0),
    .c1_resp_valid(c1_rv_u), .c1_resp_id(c1_rid_u),
    .c1_resp_rdata(c1_rd_u), .c1_resp_err(c1_err_u),
    .m_psel(m_psel), .m_penable(m_penable), .m_pwrite(m_pwrite),
    .m_paddr(m_paddr), .m_pwdata(m_pwdata), .m_pstrb(m_pstrb),
    .m_pready(m_pready), .m_prdata(m_prdata), .m_pslverr(m_pslverr)
);

apb4_interconnect #(.NUM_SLAVES(NUM_SLAVES), .SLAVE_PRESENT(SLV_MASK)) u_xbar (
    .m_psel(m_psel), .m_penable(m_penable), .m_pwrite(m_pwrite),
    .m_paddr(m_paddr), .m_pwdata(m_pwdata), .m_pstrb(m_pstrb),
    .m_pready(m_pready), .m_prdata(m_prdata), .m_pslverr(m_pslverr),
    .s_psel(s_psel), .s_penable(s_penable), .s_pwrite(s_pwrite),
    .s_paddr(s_paddr), .s_pwdata(s_pwdata), .s_pstrb(s_pstrb),
    .s_pready(s_pready), .s_prdata(s_prdata), .s_pslverr(s_pslverr)
);

apb4_gpio u_gpio (
    .pclk(clk), .preset_n(reset_n),
    .psel(s_psel[APB_SLV_GPIO]), .penable(s_penable), .pwrite(s_pwrite),
    .paddr(s_paddr), .pwdata(s_pwdata), .pstrb(s_pstrb),
    .pready(s_pready[APB_SLV_GPIO]), .prdata(s_prdata[APB_SLV_GPIO*32 +: 32]), .pslverr(s_pslverr[APB_SLV_GPIO]),
    .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_oe(gpio_oe), .irq_out(gpio_irq)
);

apb4_uart u_uart (
    .pclk(clk), .preset_n(reset_n),
    .psel(s_psel[APB_SLV_UART]), .penable(s_penable), .pwrite(s_pwrite),
    .paddr(s_paddr), .pwdata(s_pwdata), .pstrb(s_pstrb),
    .pready(s_pready[APB_SLV_UART]), .prdata(s_prdata[APB_SLV_UART*32 +: 32]), .pslverr(s_pslverr[APB_SLV_UART]),
    .uart_rx(1'b1), .uart_tx(uart_tx),
    .tx_char_valid(uart_tx_ch_v), .tx_char_data(uart_tx_ch_d),
    .rx_irq_out(uart_rx_irq), .tx_irq_out(uart_tx_irq)
);

apb4_timer_pwm u_timer (
    .pclk(clk), .preset_n(reset_n),
    .psel(s_psel[APB_SLV_TIMER]), .penable(s_penable), .pwrite(s_pwrite),
    .paddr(s_paddr), .pwdata(s_pwdata), .pstrb(s_pstrb),
    .pready(s_pready[APB_SLV_TIMER]), .prdata(s_prdata[APB_SLV_TIMER*32 +: 32]), .pslverr(s_pslverr[APB_SLV_TIMER]),
    .pwm_out(pwm_out), .irq_ch0_out(tmr0_irq), .irq_ch1_out(tmr1_irq)
);

assign s_pready[APB_SLV_CLINT]    = 1'b1; assign s_pslverr[APB_SLV_CLINT]    = 1'b1; assign s_prdata[APB_SLV_CLINT*32    +: 32] = 32'd0;
assign s_pready[APB_SLV_PLIC]     = 1'b1; assign s_pslverr[APB_SLV_PLIC]     = 1'b1; assign s_prdata[APB_SLV_PLIC*32     +: 32] = 32'd0;
assign s_pready[APB_SLV_DMA]      = 1'b1; assign s_pslverr[APB_SLV_DMA]      = 1'b1; assign s_prdata[APB_SLV_DMA*32      +: 32] = 32'd0;
assign s_pready[APB_SLV_SOC_CTRL] = 1'b1; assign s_pslverr[APB_SLV_SOC_CTRL] = 1'b1; assign s_prdata[APB_SLV_SOC_CTRL*32 +: 32] = 32'd0;

integer errors, checks, proto_errs, tx_tap_cnt;
reg prev_psel, prev_penable, prev_pwrite, prev_pready;
reg [31:0] prev_paddr, prev_pwdata;
reg [3:0]  prev_pstrb;

/* verilator lint_off BLKSEQ */
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        prev_psel <= 1'b0; prev_penable <= 1'b0; prev_pwrite <= 1'b0; prev_pready <= 1'b0;
        prev_paddr <= 32'd0; prev_pwdata <= 32'd0; prev_pstrb <= 4'd0;
        proto_errs <= 0;
        tx_tap_cnt <= 0;
    end else begin
        if (m_penable && !m_psel) proto_errs <= proto_errs + 1;
        if (m_psel && !m_pwrite && (m_pstrb != 4'b0000)) proto_errs <= proto_errs + 1;
        if (prev_psel && !prev_penable && !(m_psel && m_penable)) proto_errs <= proto_errs + 1;
        if (uart_tx_ch_v) tx_tap_cnt <= tx_tap_cnt + 1;
        prev_psel <= m_psel; prev_penable <= m_penable; prev_pwrite <= m_pwrite; prev_pready <= m_pready;
        prev_paddr <= m_paddr; prev_pwdata <= m_pwdata; prev_pstrb <= m_pstrb;
    end
end
/* verilator lint_on BLKSEQ */

initial clk = 1'b0;
always #5 clk = ~clk;

task step_clk;
    begin @(posedge clk); #1; end
endtask

task check(input cond, input [1023:0] msg);
    begin
        checks = checks + 1;
        if (!cond) begin
            $display("FAIL [tb_apb4_peripherals]: %0s", msg);
            errors = errors + 1;
        end
    end
endtask

task apb_wr(input [31:0] addr, input [31:0] wdata, input [3:0] wstrb);
    integer wd;
    begin
        c0_req_valid = 1'b1; c0_req_is_write = 1'b1;
        c0_req_addr = addr; c0_req_wdata = wdata; c0_req_wstrb = wstrb; c0_req_id = 7'd1;
        #1; while (!c0_req_ready) step_clk();
        step_clk(); c0_req_valid = 1'b0;
        wd = 0; while (!c0_resp_valid && wd < 32) begin step_clk(); wd = wd + 1; end
        check(c0_resp_valid && !c0_resp_err, "apb_wr clean ack");
        step_clk();
    end
endtask

task apb_rd(input [31:0] addr, output [31:0] rdata);
    integer wd;
    begin
        c0_req_valid = 1'b1; c0_req_is_write = 1'b0;
        c0_req_addr = addr; c0_req_wdata = 32'd0; c0_req_wstrb = 4'h0; c0_req_id = 7'd2;
        #1; while (!c0_req_ready) step_clk();
        step_clk(); c0_req_valid = 1'b0;
        wd = 0; while (!c0_resp_valid && wd < 32) begin step_clk(); wd = wd + 1; end
        check(c0_resp_valid && !c0_resp_err, "apb_rd clean ack");
        rdata = c0_resp_rdata;
        step_clk();
    end
endtask

reg [31:0] rv;
integer    k, pwm0_hi, pwm0_lo, pwm1_hi;

initial begin
    errors = 0; checks = 0;
    reset_n = 1'b0;
    gpio_in = 32'd0;
    c0_req_valid = 1'b0; c0_req_is_write = 1'b0; c0_req_addr = 32'd0; c0_req_wdata = 32'd0; c0_req_wstrb = 4'd0; c0_req_id = 7'd0;
    repeat (2) step_clk();
    reset_n = 1'b1;
    #1;

    // ==================== 1. APB4 GPIO Tests ====================
    apb_wr(SOC_GPIO_BASE + 32'h008, 32'h0000_FF00, 4'hF); // [15:8]=output, [7:0]=input
    check(gpio_oe == 32'h0000_FF00, "GPIO DIR sets gpio_oe");
    apb_wr(SOC_GPIO_BASE + 32'h004, 32'h0000_5500, 4'hF); // DATA_OUT
    check(gpio_out == 32'h0000_5500, "GPIO DATA_OUT drives gpio_out");
    apb_wr(SOC_GPIO_BASE + 32'h00C, 32'h0000_0A00, 4'hF); // Atomic OUT_SET
    check(gpio_out == 32'h0000_5F00, "GPIO atomic OUT_SET");
    apb_wr(SOC_GPIO_BASE + 32'h010, 32'h0000_0500, 4'hF); // Atomic OUT_CLR
    check(gpio_out == 32'h0000_5A00, "GPIO atomic OUT_CLR");
    apb_wr(SOC_GPIO_BASE + 32'h014, 32'h0000_FF00, 4'hF); // Atomic OUT_TGL
    check(gpio_out == 32'h0000_A500, "GPIO atomic OUT_TGL");

    // Rising-edge IRQ on pin 0 + falling-edge IRQ on pin 1
    gpio_in = 32'h0000_0002; // pin1 starts high
    repeat (2) step_clk();
    apb_wr(SOC_GPIO_BASE + 32'h01C, 32'h0000_0003, 4'hF); // pin0,1 edge-triggered
    apb_wr(SOC_GPIO_BASE + 32'h020, 32'h0000_0001, 4'hF); // pin0 rising, pin1 falling
    apb_wr(SOC_GPIO_BASE + 32'h018, 32'h0000_0003, 4'hF); // enable IRQ on pin0,1
    check(!gpio_irq, "GPIO IRQ initially quiet");
    gpio_in = 32'h0000_0003; // pin0 rising edge!
    repeat (2) step_clk();
    check(gpio_irq, "GPIO rising edge on pin 0 raises irq_out");
    apb_rd(SOC_GPIO_BASE + 32'h024, rv);
    check(rv == 32'h0000_0001, "GPIO IRQ_STAT bit 0 latched");
    apb_wr(SOC_GPIO_BASE + 32'h024, 32'h0000_0001, 4'hF); // W1C bit 0
    check(!gpio_irq, "GPIO W1C clears pin 0 IRQ even while pin 0 remains high");

    gpio_in = 32'h0000_0001; // pin1 falling edge!
    repeat (2) step_clk();
    check(gpio_irq, "GPIO falling edge on pin 1 raises irq_out");
    apb_rd(SOC_GPIO_BASE + 32'h024, rv);
    check(rv == 32'h0000_0002, "GPIO IRQ_STAT bit 1 latched");
    apb_rd(SOC_GPIO_BASE + 32'h000, rv);
    check(rv == 32'h0000_A501, "GPIO DATA_IN combines outputs[15:8] + inputs[7:0]");
    apb_wr(SOC_GPIO_BASE + 32'h024, 32'h0000_0002, 4'hF);
    check(!gpio_irq, "GPIO W1C clears pin 1 IRQ");

    // ==================== 2. APB4 UART (Real 8N1 Serial Loopback + Fast-Sim) ====================
    apb_wr(SOC_UART_BASE + 32'h010, 32'd4, 4'hF);        // BAUD_DIV = 4 clks/bit (real 8N1!)
    apb_wr(SOC_UART_BASE + 32'h00C, 32'h0000_0037, 4'hF); // TX_EN|RX_EN|LOOPBACK|RX_IRQ|TX_IRQ
    check(uart_tx_irq && !uart_rx_irq, "UART idle: TX_IRQ asserted, RX_IRQ low");
    apb_wr(SOC_UART_BASE + 32'h000, 32'h41, 4'hF); // 'A'
    apb_wr(SOC_UART_BASE + 32'h000, 32'h5A, 4'hF); // 'Z'
    apb_wr(SOC_UART_BASE + 32'h000, 32'hA5, 4'hF); // 0xA5
    // Wait for real bit-by-bit 8N1 serial framing (3 bytes * 10 bits * 4 clks = ~140 clks)
    repeat (180) step_clk();
    check(uart_rx_irq && uart_tx_irq, "UART 8N1 loopback received all 3 bytes & TX idle");
    apb_rd(SOC_UART_BASE + 32'h008, rv);
    check(rv[20:16] == 5'd3 && rv[12:8] == 5'd0, "UART RX_LEVEL==3 and TX_LEVEL==0");
    apb_rd(SOC_UART_BASE + 32'h004, rv); check(rv == 32'h41, "UART real-8N1 RX byte 0 == 'A'");
    apb_rd(SOC_UART_BASE + 32'h004, rv); check(rv == 32'h5A, "UART real-8N1 RX byte 1 == 'Z'");
    apb_rd(SOC_UART_BASE + 32'h004, rv); check(rv == 32'hA5, "UART real-8N1 RX byte 2 == 0xA5");
    check(!uart_rx_irq, "UART RX_IRQ deasserts once RX FIFO drained");

    // Fast-sim loopback mode (1-cycle FIFO burst of 8 bytes!)
    apb_wr(SOC_UART_BASE + 32'h00C, 32'h0000_001F, 4'hF); // TX_EN|RX_EN|LOOPBACK|FAST_SIM|RX_IRQ
    for (k = 0; k < 8; k = k + 1)
        apb_wr(SOC_UART_BASE + 32'h000, 32'h30 + k, 4'hF);
    step_clk();
    apb_rd(SOC_UART_BASE + 32'h008, rv);
    check(rv[20:16] == 5'd8, "UART FAST_SIM loopback burst RX_LEVEL == 8");
    for (k = 0; k < 8; k = k + 1) begin
        apb_rd(SOC_UART_BASE + 32'h004, rv);
        check(rv == (32'h30 + k), "UART FAST_SIM FIFO byte exact order");
    end
    check(tx_tap_cnt == 11, "UART console tap observed all 11 transmitted bytes");

    // ==================== 3. APB4 2-Channel Timer & PWM ====================
    // CH0: Periodic PWM (prescale=0, period=9 -> 10-cycle period, cmp=3 -> exact 30% duty!)
    apb_wr(SOC_TIMER_BASE + 32'h004, 32'd0, 4'hF); // PRESCALE=0
    apb_wr(SOC_TIMER_BASE + 32'h008, 32'd9, 4'hF); // PERIOD=9 (10 clocks)
    apb_wr(SOC_TIMER_BASE + 32'h00C, 32'd3, 4'hF); // CMP=3 (3/10 = 30% duty)
    apb_wr(SOC_TIMER_BASE + 32'h010, 32'd0, 4'hF); // COUNT=0
    apb_wr(SOC_TIMER_BASE + 32'h000, 32'h0000_000F, 4'hF); // EN|AUTO_RELOAD|PWM_EN|IRQ_EN

    // CH1: One-shot timer (prescale=1 -> /2, period=4 -> fires after 10 clocks and auto-stops!)
    apb_wr(SOC_TIMER_BASE + 32'h024, 32'd1, 4'hF); // PRESCALE=1
    apb_wr(SOC_TIMER_BASE + 32'h028, 32'd4, 4'hF); // PERIOD=4
    apb_wr(SOC_TIMER_BASE + 32'h02C, 32'd5, 4'hF); // CMP=5 (100% high while active)
    apb_wr(SOC_TIMER_BASE + 32'h030, 32'd0, 4'hF); // COUNT=0
    apb_wr(SOC_TIMER_BASE + 32'h020, 32'h0000_000D, 4'hF); // EN|PWM_EN|IRQ_EN (AUTO_RELOAD=0!)

    pwm0_hi = 0; pwm0_lo = 0; pwm1_hi = 0;
    for (k = 0; k < 40; k = k + 1) begin
        if (pwm_out[0]) pwm0_hi = pwm0_hi + 1;
        else            pwm0_lo = pwm0_lo + 1;
        if (pwm_out[1]) pwm1_hi = pwm1_hi + 1;
        step_clk();
    end
    check(pwm0_hi == 12 && pwm0_lo == 28, "Timer CH0 periodic PWM exact 30% duty (12/40 cycles)");
    check(tmr0_irq, "Timer CH0 periodic IRQ asserted");
    check(tmr1_irq, "Timer CH1 one-shot IRQ asserted");
    check(pwm1_hi > 0 && pwm1_hi <= 12, "Timer CH1 one-shot auto-stopped after 1 period");

    // Verify CH1 EN bit auto-cleared in one-shot mode, and W1C clears both IRQs!
    apb_rd(SOC_TIMER_BASE + 32'h020, rv);
    check(rv[0] == 1'b0, "Timer CH1 EN bit auto-cleared after one-shot expiration");
    apb_wr(SOC_TIMER_BASE + 32'h000, 32'h0, 4'hF); // stop CH0
    apb_wr(SOC_TIMER_BASE + 32'h014, 32'h1, 4'hF); // W1C CH0
    apb_wr(SOC_TIMER_BASE + 32'h034, 32'h1, 4'hF); // W1C CH1
    check(!tmr0_irq && !tmr1_irq, "Timer CH0 & CH1 W1C cleared both interrupts");

    check(proto_errs == 0, "Zero AMBA APB4 protocol errors across GPIO/UART/Timer tests");

    if (errors == 0 && proto_errs == 0)
        $display("PASS [tb_apb4_peripherals]: %0d/%0d AMBA APB4 GPIO, 8N1 UART & 2-Ch Timer/PWM checks passed!", checks, checks);
    else begin
        $display("FAIL [tb_apb4_peripherals]: errors=%0d proto_errs=%0d checks=%0d", errors, proto_errs, checks);
        $fatal(1);
    end
    $finish;
end

endmodule
