/* ============================================================================
 * Hardware Abstraction Layer: AMBA APB4 RISC-V PLIC (External Interrupts)
 * ============================================================================ */
#ifndef HAL_PLIC_H
#define HAL_PLIC_H

#include <stdint.h>
#include "soc_regs.h"

static inline void hal_plic_set_priority(uint32_t src, uint32_t prio) {
    if (src >= 1u && src < PLIC_NUM_SOURCES) {
        mmio_write32(SOC_PLIC_BASE + PLIC_PRIO_OFF(src), prio & 0x7u);
    }
}

static inline uint32_t hal_plic_get_priority(uint32_t src) {
    if (src >= 1u && src < PLIC_NUM_SOURCES) {
        return mmio_read32(SOC_PLIC_BASE + PLIC_PRIO_OFF(src)) & 0x7u;
    }
    return 0u;
}

static inline uint32_t hal_plic_get_pending(void) {
    return mmio_read32(SOC_PLIC_BASE + PLIC_PENDING_OFF);
}

static inline void hal_plic_set_trigger_edge(uint32_t src, uint32_t is_edge) {
    if (src >= 1u && src < PLIC_NUM_SOURCES) {
        uint32_t cur = mmio_read32(SOC_PLIC_BASE + PLIC_TRIG_EDGE_OFF);
        if (is_edge) cur |=  (1u << src);
        else         cur &= ~(1u << src);
        mmio_write32(SOC_PLIC_BASE + PLIC_TRIG_EDGE_OFF, cur);
    }
}

static inline void hal_plic_enable_irq(uint32_t hartid, uint32_t src) {
    if (src >= 1u && src < PLIC_NUM_SOURCES) {
        uint32_t addr = SOC_PLIC_BASE + PLIC_ENABLE_OFF(hartid);
        mmio_write32(addr, mmio_read32(addr) | (1u << src));
    }
}

static inline void hal_plic_disable_irq(uint32_t hartid, uint32_t src) {
    if (src >= 1u && src < PLIC_NUM_SOURCES) {
        uint32_t addr = SOC_PLIC_BASE + PLIC_ENABLE_OFF(hartid);
        mmio_write32(addr, mmio_read32(addr) & ~(1u << src));
    }
}

static inline void hal_plic_set_enable_mask(uint32_t hartid, uint32_t mask) {
    mmio_write32(SOC_PLIC_BASE + PLIC_ENABLE_OFF(hartid), mask & ~1u);
}

static inline void hal_plic_set_threshold(uint32_t hartid, uint32_t thresh) {
    mmio_write32(SOC_PLIC_BASE + PLIC_THRESH_OFF(hartid), thresh & 0x7u);
}

static inline uint32_t hal_plic_get_threshold(uint32_t hartid) {
    return mmio_read32(SOC_PLIC_BASE + PLIC_THRESH_OFF(hartid)) & 0x7u;
}

static inline uint32_t hal_plic_claim(uint32_t hartid) {
    return mmio_read32(SOC_PLIC_BASE + PLIC_CLAIM_OFF(hartid));
}

static inline void hal_plic_complete(uint32_t hartid, uint32_t src) {
    mmio_write32(SOC_PLIC_BASE + PLIC_CLAIM_OFF(hartid), src);
}

#endif /* HAL_PLIC_H */
