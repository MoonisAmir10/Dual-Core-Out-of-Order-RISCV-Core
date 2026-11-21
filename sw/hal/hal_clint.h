/* ============================================================================
 * Hardware Abstraction Layer: AMBA APB4 CLINT (Timer & Cross-Core IPI)
 * ============================================================================ */
#ifndef HAL_CLINT_H
#define HAL_CLINT_H

#include <stdint.h>
#include "soc_regs.h"

static inline uint32_t hal_clint_mtime_lo(void) {
    return mmio_read32(SOC_CLINT_BASE + CLINT_MTIME_LO_OFF);
}

static inline uint32_t hal_clint_mtime_hi(void) {
    return mmio_read32(SOC_CLINT_BASE + CLINT_MTIME_HI_OFF);
}

/* Tear-free 64-bit mtime read on RV32 (RISC-V Priv Spec recommendation) */
static inline uint64_t hal_clint_mtime64(void) {
    uint32_t hi0, lo, hi1;
    do {
        hi0 = hal_clint_mtime_hi();
        lo  = hal_clint_mtime_lo();
        hi1 = hal_clint_mtime_hi();
    } while (hi0 != hi1);
    return ((uint64_t)hi0 << 32) | (uint64_t)lo;
}

/* Glitch-free 64-bit mtimecmp update on RV32 (RISC-V Priv Spec §3.2.1) */
static inline void hal_clint_set_mtimecmp(uint32_t hartid, uint64_t cmp_val) {
    uint32_t lo_addr = SOC_CLINT_BASE + CLINT_MTIMECMP_LO_OFF(hartid);
    uint32_t hi_addr = SOC_CLINT_BASE + CLINT_MTIMECMP_HI_OFF(hartid);
    mmio_write32(hi_addr, 0xFFFFFFFFu);
    mmio_write32(lo_addr, (uint32_t)(cmp_val & 0xFFFFFFFFu));
    mmio_write32(hi_addr, (uint32_t)(cmp_val >> 32));
}

static inline void hal_clint_set_alarm_rel(uint32_t hartid, uint32_t delta_cycles) {
    uint64_t now = hal_clint_mtime64();
    hal_clint_set_mtimecmp(hartid, now + (uint64_t)delta_cycles);
}

static inline void hal_clint_disable_timer(uint32_t hartid) {
    hal_clint_set_mtimecmp(hartid, 0xFFFFFFFFFFFFFFFFull);
}

static inline void hal_clint_send_ipi(uint32_t target_hart) {
    mmio_write32(SOC_CLINT_BASE + CLINT_MSIP_OFF(target_hart), 1u);
}

static inline void hal_clint_clear_ipi(uint32_t hartid) {
    mmio_write32(SOC_CLINT_BASE + CLINT_MSIP_OFF(hartid), 0u);
}

static inline uint32_t hal_clint_get_ipi(uint32_t hartid) {
    return mmio_read32(SOC_CLINT_BASE + CLINT_MSIP_OFF(hartid)) & 1u;
}

static inline uint32_t hal_clint_scratch_read(uint32_t idx) {
    return mmio_read32(SOC_CLINT_BASE + CLINT_SCRATCH_OFF(idx));
}

static inline void hal_clint_scratch_write(uint32_t idx, uint32_t val) {
    mmio_write32(SOC_CLINT_BASE + CLINT_SCRATCH_OFF(idx), val);
}

#endif /* HAL_CLINT_H */
