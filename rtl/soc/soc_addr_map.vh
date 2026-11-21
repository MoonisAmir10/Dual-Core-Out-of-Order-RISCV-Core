// ============================================================================
// Unified 32-Bit SoC Physical Memory Map & AMBA APB4 Slave Table (Phase U2+)
// Single source of truth shared across apb4_interconnect, dual_ooo2w_soc,
// single_ooo2w_soc, and verification testbenches.
//
// Physical Address Map (PMA):
//   0x0000_0000 .. 0x0003_FFFF (256 KB) : Cacheable Coherent Main RAM
//   0x0200_0000 .. 0x0200_FFFF ( 64 KB) : APB4 S0 - CLINT (msip/mtime/mtimecmp)
//   0x0C00_0000 .. 0x0C20_FFFF (~2.1MB) : APB4 S1 - PLIC (Phase U3)
//   0x1000_0000 .. 0x1000_0FFF (  4 KB) : APB4 S2 - 32-Pin GPIO (Phase U4)
//   0x1001_0000 .. 0x1001_0FFF (  4 KB) : APB4 S3 - 16-FIFO UART (Phase U4)
//   0x1002_0000 .. 0x1002_0FFF (  4 KB) : APB4 S4 - Coherent DMA Regs (Phase U7)
//   0x1003_0000 .. 0x1003_0FFF (  4 KB) : APB4 S5 - 2-Ch Timer/PWM (Phase U4)
//   0x1004_0000 .. 0x1004_0FFF (  4 KB) : APB4 S6 - SoC Control & PMU (Phase U2)
//   0x1005_0000 .. 0x100D_0FFF ( 9x4KB) : APB4 S7..S15 - Plug-and-Play Expansion
//   All other addresses                 : Unmapped (PSLVERR=1 -> mcause 5 / 7)
// ============================================================================

localparam [31:0] SOC_RAM_BASE      = 32'h0000_0000;
localparam [31:0] SOC_RAM_SIZE      = 32'h0004_0000; // 256 KB cacheable RAM

// Standardized AMBA APB4 Slave Slot Indices
localparam integer APB_MAX_SLAVES   = 16;
localparam integer APB_SLV_CLINT    = 0;
localparam integer APB_SLV_PLIC     = 1;
localparam integer APB_SLV_GPIO     = 2;
localparam integer APB_SLV_UART     = 3;
localparam integer APB_SLV_DMA      = 4;
localparam integer APB_SLV_TIMER    = 5;
localparam integer APB_SLV_SOC_CTRL = 6;

// Base & End Addresses (inclusive)
localparam [31:0] SOC_CLINT_BASE    = 32'h0200_0000;
localparam [31:0] SOC_CLINT_END     = 32'h0200_FFFF;

localparam [31:0] SOC_PLIC_BASE     = 32'h0C00_0000;
localparam [31:0] SOC_PLIC_END      = 32'h0C20_FFFF;

localparam [31:0] SOC_GPIO_BASE     = 32'h1000_0000;
localparam [31:0] SOC_GPIO_END      = 32'h1000_0FFF;

localparam [31:0] SOC_UART_BASE     = 32'h1001_0000;
localparam [31:0] SOC_UART_END      = 32'h1001_0FFF;

localparam [31:0] SOC_DMA_BASE      = 32'h1002_0000;
localparam [31:0] SOC_DMA_END       = 32'h1002_0FFF;

localparam [31:0] SOC_TIMER_BASE    = 32'h1003_0000;
localparam [31:0] SOC_TIMER_END     = 32'h1003_0FFF;

localparam [31:0] SOC_CTRL_BASE     = 32'h1004_0000;
localparam [31:0] SOC_CTRL_END      = 32'h1004_0FFF;

// Expansion slots S7..S15: 0x1005_0000 + (slot-7)*0x10000, 4 KB window each
localparam [31:0] SOC_EXP_BASE      = 32'h1005_0000;
