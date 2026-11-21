`timescale 1ns / 1ps

// ============================================================================
// Standardized AMBA APB4 32-Pin GPIO Controller (Phase U4)
// Mapped at 0x1000_0000 .. 0x1000_0FFF (APB4 Slave 2, PLIC Source 1)
//
// Features:
//   - 32 bidirectional pins: gpio_in[31:0], gpio_out[31:0], gpio_oe[31:0]
//   - Atomic bit-manipulation registers: OUT_SET, OUT_CLR, OUT_TGL
//     (zero read-modify-write hazards across dual cores!)
//   - Per-pin configurable interrupt engine:
//       * IRQ_EN   : per-pin interrupt enable
//       * IRQ_TYPE : 0 = level-sensitive, 1 = edge-sensitive
//       * IRQ_POL  : 1 = active-high / rising-edge, 0 = active-low / falling-edge
//       * IRQ_STAT : W1C latched status -> irq_out = |(irq_stat_r & irq_en_r)
//
// Register Map (offset = {paddr[11:2], 2'b00}):
//   0x000 : DATA_IN   (RO)  - Sampled pin state (outputs reflect DATA_OUT)
//   0x004 : DATA_OUT  (RW)  - 32-bit output data register
//   0x008 : DIR       (RW)  - 32-bit direction (1=output, 0=input)
//   0x00C : OUT_SET   (WO)  - Atomic bitwise set on DATA_OUT
//   0x010 : OUT_CLR   (WO)  - Atomic bitwise clear on DATA_OUT
//   0x014 : OUT_TGL   (WO)  - Atomic bitwise toggle on DATA_OUT
//   0x018 : IRQ_EN    (RW)  - Per-pin interrupt enable mask
//   0x01C : IRQ_TYPE  (RW)  - Per-pin trigger mode (0=level, 1=edge)
//   0x020 : IRQ_POL   (RW)  - Per-pin polarity (1=high/rising, 0=low/falling)
//   0x024 : IRQ_STAT  (W1C) - Per-pin latched interrupt status (write 1 to clear)
// ============================================================================
module apb4_gpio (
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

    // External 32-Pin GPIO Interface & PLIC Interrupt Output (Source 1)
    input  wire [31:0] gpio_in,
    output wire [31:0] gpio_out,
    output wire [31:0] gpio_oe,
    output wire        irq_out
);

assign pready  = 1'b1;
assign pslverr = 1'b0;

reg [31:0] data_out_r;
reg [31:0] dir_r;
reg [31:0] irq_en_r;
reg [31:0] irq_type_r;
reg [31:0] irq_pol_r;
reg [31:0] irq_stat_r;

reg [31:0] pin_cur_r;
reg [31:0] pin_prev_r;

assign gpio_out = data_out_r;
assign gpio_oe  = dir_r;
assign irq_out  = |(irq_stat_r & irq_en_r);

wire [31:0] data_in_eff = (data_out_r & dir_r) | (pin_cur_r & ~dir_r);

function automatic [31:0] strb_mask(input [3:0] strb);
    begin
        strb_mask = {
            {8{strb[3]}},
            {8{strb[2]}},
            {8{strb[1]}},
            {8{strb[0]}}
        };
    end
endfunction

wire [31:0] bmask     = strb_mask(pstrb);
wire [11:0] reg_off   = {paddr[11:2], 2'b00};
wire        apb_write = psel && penable && pwrite;

// Per-pin level & edge event detector (evaluates on synchronized input pins)
wire [31:0] pin_rise =  pin_cur_r & ~pin_prev_r;
wire [31:0] pin_fall = ~pin_cur_r &  pin_prev_r;
wire [31:0] edge_evt = (irq_pol_r & pin_rise) | (~irq_pol_r & pin_fall);
wire [31:0] lvl_evt  = (irq_pol_r & pin_cur_r) | (~irq_pol_r & ~pin_cur_r);
wire [31:0] pin_evt  = ((irq_type_r & edge_evt) | (~irq_type_r & lvl_evt)) & ~dir_r;

wire [31:0] w1c_clr  = (apb_write && (reg_off == 12'h024)) ? (pwdata & bmask) : 32'd0;

always @(*) begin
    case (reg_off)
        12'h000: prdata = data_in_eff;
        12'h004: prdata = data_out_r;
        12'h008: prdata = dir_r;
        12'h018: prdata = irq_en_r;
        12'h01C: prdata = irq_type_r;
        12'h020: prdata = irq_pol_r;
        12'h024: prdata = irq_stat_r;
        default: prdata = 32'd0;
    endcase
end

always @(posedge pclk or negedge preset_n) begin
    if (!preset_n) begin
        data_out_r <= 32'd0;
        dir_r      <= 32'd0;
        irq_en_r   <= 32'd0;
        irq_type_r <= 32'd0;
        irq_pol_r  <= 32'hFFFF_FFFF; // default rising / active-high
        irq_stat_r <= 32'd0;
        pin_cur_r  <= 32'd0;
        pin_prev_r <= 32'd0;
    end else begin
        pin_cur_r  <= gpio_in;
        pin_prev_r <= pin_cur_r;

        // Latch enabled input events; W1C clears acknowledged bits
        irq_stat_r <= (irq_stat_r & ~w1c_clr) | (pin_evt & irq_en_r);

        if (apb_write) begin
            case (reg_off)
                12'h004: data_out_r <= (data_out_r & ~bmask) | (pwdata & bmask);
                12'h008: dir_r      <= (dir_r      & ~bmask) | (pwdata & bmask);
                12'h00C: data_out_r <= data_out_r |  (pwdata & bmask); // Atomic OUT_SET
                12'h010: data_out_r <= data_out_r & ~(pwdata & bmask); // Atomic OUT_CLR
                12'h014: data_out_r <= data_out_r ^  (pwdata & bmask); // Atomic OUT_TGL
                12'h018: irq_en_r   <= (irq_en_r   & ~bmask) | (pwdata & bmask);
                12'h01C: irq_type_r <= (irq_type_r & ~bmask) | (pwdata & bmask);
                12'h020: irq_pol_r  <= (irq_pol_r  & ~bmask) | (pwdata & bmask);
                default: ;
            endcase
        end
    end
end

endmodule
