/* ============================================================================
 * Unified 32-Bit SoC Physical Address Map & MMIO Register Definitions (HAL)
 * Mirrors rtl/soc/soc_addr_map.vh as the software single source of truth!
 * ============================================================================ */
#ifndef SOC_REGS_H
#define SOC_REGS_H

#include <stdint.h>

/* ----------------------------------------------------------------------------
 * Memory & AMBA APB4 Peripheral Base Addresses
 * ---------------------------------------------------------------------------- */
#define SOC_RAM_BASE        0x00000000u
#define SOC_RAM_SIZE        0x00040000u  /* 256 KB cacheable coherent RAM */

#define SOC_CLINT_BASE      0x02000000u  /* APB4 S0: CLINT (64 KB) */
#define SOC_PLIC_BASE       0x0C000000u  /* APB4 S1: PLIC  (~2.1 MB) */
#define SOC_GPIO_BASE       0x10000000u  /* APB4 S2: 32-Pin GPIO (4 KB) */
#define SOC_UART_BASE       0x10010000u  /* APB4 S3: 16-FIFO UART (4 KB) */
#define SOC_DMA_BASE        0x10020000u  /* APB4 S4: Coherent DMA (4 KB) */
#define SOC_TIMER_BASE      0x10030000u  /* APB4 S5: 2-Ch Timer/PWM (4 KB) */
#define SOC_CTRL_BASE       0x10040000u  /* APB4 S6: SoC Control & PMU (4 KB) */

/* ----------------------------------------------------------------------------
 * Standard MMIO Accessors
 * ---------------------------------------------------------------------------- */
#define MMIO_REG32(addr)    (*(volatile uint32_t *)(uintptr_t)(addr))

static inline uint32_t mmio_read32(uint32_t addr) {
    return MMIO_REG32(addr);
}

static inline void mmio_write32(uint32_t addr, uint32_t val) {
    MMIO_REG32(addr) = val;
}

/* ----------------------------------------------------------------------------
 * APB4 S0: CLINT Register Offsets (SiFive-Compatible)
 * ---------------------------------------------------------------------------- */
#define CLINT_MSIP_OFF(hart)        (0x0000u + 4u * (hart))
#define CLINT_MTIMECMP_LO_OFF(hart) (0x4000u + 8u * (hart))
#define CLINT_MTIMECMP_HI_OFF(hart) (0x4004u + 8u * (hart))
#define CLINT_MTIME_LO_OFF          0xBFF8u
#define CLINT_MTIME_HI_OFF          0xBFFCu
#define CLINT_SCRATCH_OFF(idx)      (0xC000u + 4u * (idx))

/* ----------------------------------------------------------------------------
 * APB4 S1: RISC-V PLIC Register Offsets (16 Sources, 2 M-Mode Contexts)
 * ---------------------------------------------------------------------------- */
#define PLIC_NUM_SOURCES            16u
#define PLIC_PRIO_OFF(src)          (0x000000u + 4u * (src))
#define PLIC_PENDING_OFF            0x001000u
#define PLIC_TRIG_EDGE_OFF          0x001080u
#define PLIC_ENABLE_OFF(hart)       (0x002000u + 0x0080u * (hart))
#define PLIC_THRESH_OFF(hart)       (0x200000u + 0x1000u * (hart))
#define PLIC_CLAIM_OFF(hart)        (0x200004u + 0x1000u * (hart))

/* Standard SoC External PLIC Interrupt Source Map */
#define PLIC_IRQ_GPIO               1u
#define PLIC_IRQ_UART_RX            2u
#define PLIC_IRQ_UART_TX            3u
#define PLIC_IRQ_DMA_CH0            4u
#define PLIC_IRQ_DMA_CH1            5u
#define PLIC_IRQ_TIMER0             6u
#define PLIC_IRQ_TIMER1             7u

/* ----------------------------------------------------------------------------
 * APB4 S6: SoC Control, TOHOST/HALT, IRQ_INJECT & Live PMU Counters
 * ---------------------------------------------------------------------------- */
#define SOC_CTRL_ID_OFF             0x000u
#define SOC_CTRL_CORES_OFF          0x004u
#define SOC_CTRL_FEATURES_OFF       0x008u
#define SOC_CTRL_SYS_HALT_OFF       0x00Cu
#define SOC_CTRL_TOHOST_OFF         0x010u
#define SOC_CTRL_SCRATCH_OFF        0x014u
#define SOC_CTRL_IRQ_INJECT_OFF     0x018u
#define SOC_CTRL_PMU_CYCLES_OFF     0x020u
#define SOC_CTRL_PMU_C0_RET_OFF     0x024u
#define SOC_CTRL_PMU_C1_RET_OFF     0x028u
#define SOC_CTRL_PMU_COH_C2C_OFF    0x02Cu
#define SOC_CTRL_PMU_L2_HITS_OFF    0x030u
#define SOC_CTRL_PMU_L2_MISSES_OFF  0x034u
#define SOC_CTRL_PMU_DRAM_RDS_OFF   0x038u
#define SOC_CTRL_PMU_DRAM_WRS_OFF   0x03Cu

#define SOC_FEAT_TRAPS              (1u << 0)
#define SOC_FEAT_APB4               (1u << 1)
#define SOC_FEAT_PLIC               (1u << 2)
#define SOC_FEAT_PERIPH             (1u << 3)
#define SOC_FEAT_FPU                (1u << 4)
#define SOC_FEAT_DMA                (1u << 5)

/* Clean Software Halt via APB4 SoC Control (works even while mtvec != 0!) */
static inline void soc_halt(uint32_t exit_code) {
    mmio_write32(SOC_CTRL_BASE + SOC_CTRL_TOHOST_OFF, exit_code);
    register uint32_t a4_reg __asm__("a4") = exit_code;
    uint32_t halt_addr = SOC_CTRL_BASE + SOC_CTRL_SYS_HALT_OFF;
    uint32_t one = 1u;
    __asm__ volatile (
        "sw %1, 0(%2)\n\t"
        "1: j 1b\n\t"
        :
        : "r"(a4_reg), "r"(one), "r"(halt_addr)
        : "memory"
    );
    __builtin_unreachable();
}

/* Park secondary hart with deterministic x14 (a4) visible before primary halts */
static inline void soc_park_secondary(uint32_t exit_code, volatile uint32_t *done_flag) {
    register uint32_t a4_reg __asm__("a4") = exit_code;
    uint32_t one = 1u;
    __asm__ volatile (
        "fence w, w\n\t"
        "sw %1, 0(%2)\n\t"
        "1: j 1b\n\t"
        :
        : "r"(a4_reg), "r"(one), "r"(done_flag)
        : "memory"
    );
    __builtin_unreachable();
}

#endif /* SOC_REGS_H */
