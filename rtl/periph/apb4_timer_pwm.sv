`timescale 1ns / 1ps

// ============================================================================
// Standardized AMBA APB4 2-Channel 32-Bit Timer & PWM Controller (Phase U4)
// Mapped at 0x1003_0000 .. 0x1003_0FFF (APB4 Slave 5, PLIC Sources 6 & 7)
//
// Features:
//   - 2 completely independent 32-bit Timer/PWM channels (CH0, CH1)
//   - 16-bit prescaler per channel (tick every PRESCALE + 1 clocks)
//   - Periodic (AUTO_RELOAD=1) and One-Shot (AUTO_RELOAD=0) modes
//   - Configurable PWM output (duty = CMP / (PERIOD + 1), optional inversion)
//   - W1C period-match interrupt flag per channel -> PLIC Sources 6 & 7
//
// Register Map (offset = {paddr[11:2], 2'b00}; CH0=0x000, CH1=0x020):
//   +0x00 : CTRL     (RW)  - [0]EN [1]AUTO_RELOAD [2]PWM_EN [3]IRQ_EN [4]PWM_INV
//   +0x04 : PRESCALE (RW)  - 16-bit clock divider (tick every PRESCALE+1 cycles)
//   +0x08 : PERIOD   (RW)  - 32-bit wrap/reload period (0..PERIOD inclusive)
//   +0x0C : CMP      (RW)  - 32-bit PWM duty compare (high while COUNT < CMP)
//   +0x10 : COUNT    (RW)  - 32-bit live timer counter
//   +0x14 : IRQ_STAT (W1C) - [0] Period-match interrupt pending (write 1 to clear)
// ============================================================================
module apb4_timer_pwm (
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

    // PWM Outputs & Per-Channel PLIC Interrupt Outputs (Sources 6 & 7)
    output wire [1:0]  pwm_out,
    output wire        irq_ch0_out,
    output wire        irq_ch1_out
);

assign pready  = 1'b1;
assign pslverr = 1'b0;

reg [4:0]  ctrl_r     [0:1];
reg [15:0] prescale_r [0:1];
reg [15:0] pre_cnt_r  [0:1];
reg [31:0] period_r   [0:1];
reg [31:0] cmp_r      [0:1];
reg [31:0] count_r    [0:1];
reg        irq_stat_r [0:1];

wire pwm0_raw = ctrl_r[0][0] && ctrl_r[0][2] && (count_r[0] < cmp_r[0]);
wire pwm1_raw = ctrl_r[1][0] && ctrl_r[1][2] && (count_r[1] < cmp_r[1]);

assign pwm_out[0]  = ctrl_r[0][2] ? (ctrl_r[0][4] ? !pwm0_raw : pwm0_raw) : 1'b0;
assign pwm_out[1]  = ctrl_r[1][2] ? (ctrl_r[1][4] ? !pwm1_raw : pwm1_raw) : 1'b0;
assign irq_ch0_out = irq_stat_r[0] && ctrl_r[0][3];
assign irq_ch1_out = irq_stat_r[1] && ctrl_r[1][3];

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
wire        apb_write = psel && penable && pwrite;
wire        is_ch0    = (reg_off[11:5] == 7'd0);
wire        is_ch1    = (reg_off[11:5] == 7'd1);
wire [4:0]  ch_off    = reg_off[4:0];

always @(*) begin
    prdata = 32'd0;
    if (is_ch0) begin
        case (ch_off)
            5'h00: prdata = {27'd0, ctrl_r[0]};
            5'h04: prdata = {16'd0, prescale_r[0]};
            5'h08: prdata = period_r[0];
            5'h0C: prdata = cmp_r[0];
            5'h10: prdata = count_r[0];
            5'h14: prdata = {31'd0, irq_stat_r[0]};
            default: prdata = 32'd0;
        endcase
    end else if (is_ch1) begin
        case (ch_off)
            5'h00: prdata = {27'd0, ctrl_r[1]};
            5'h04: prdata = {16'd0, prescale_r[1]};
            5'h08: prdata = period_r[1];
            5'h0C: prdata = cmp_r[1];
            5'h10: prdata = count_r[1];
            5'h14: prdata = {31'd0, irq_stat_r[1]};
            default: prdata = 32'd0;
        endcase
    end
end

integer c;
wire [1:0] wr_ch = {apb_write && is_ch1, apb_write && is_ch0};

always @(posedge pclk or negedge preset_n) begin
    if (!preset_n) begin
        for (c = 0; c < 2; c = c + 1) begin
            ctrl_r[c]     <= 5'd0;
            prescale_r[c] <= 16'd0;
            pre_cnt_r[c]  <= 16'd0;
            period_r[c]   <= 32'hFFFF_FFFF;
            cmp_r[c]      <= 32'd0;
            count_r[c]    <= 32'd0;
            irq_stat_r[c] <= 1'b0;
        end
    end else begin
        for (c = 0; c < 2; c = c + 1) begin
            // 1. Timer Counter & Period-Match Logic
            if (ctrl_r[c][0]) begin
                if (pre_cnt_r[c] >= prescale_r[c]) begin
                    pre_cnt_r[c] <= 16'd0;
                    if (count_r[c] >= period_r[c]) begin
                        count_r[c]    <= 32'd0;
                        irq_stat_r[c] <= 1'b1;
                        if (!ctrl_r[c][1]) begin
                            // One-shot mode: auto-disable after firing
                            ctrl_r[c][0] <= 1'b0;
                        end
                    end else begin
                        count_r[c] <= count_r[c] + 32'd1;
                    end
                end else begin
                    pre_cnt_r[c] <= pre_cnt_r[c] + 16'd1;
                end
            end else begin
                pre_cnt_r[c] <= 16'd0;
            end

            // 2. APB4 Register Writes (priority over auto-update on explicit write)
            if (wr_ch[c]) begin
                case (ch_off)
                    5'h00: if (pstrb[0]) ctrl_r[c] <= pwdata[4:0];
                    5'h04: begin
                        if (pstrb[0]) prescale_r[c][7:0]  <= pwdata[7:0];
                        if (pstrb[1]) prescale_r[c][15:8] <= pwdata[15:8];
                    end
                    5'h08: period_r[c] <= mask_merge(period_r[c], pwdata, pstrb);
                    5'h0C: cmp_r[c]    <= mask_merge(cmp_r[c],    pwdata, pstrb);
                    5'h10: begin
                        count_r[c]   <= mask_merge(count_r[c], pwdata, pstrb);
                        pre_cnt_r[c] <= 16'd0;
                    end
                    5'h14: if (pstrb[0] && pwdata[0]) irq_stat_r[c] <= 1'b0; // W1C
                    default: ;
                endcase
            end
        end
    end
end

endmodule
