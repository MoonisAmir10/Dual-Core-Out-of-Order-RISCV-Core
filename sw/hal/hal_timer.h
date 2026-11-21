/* ============================================================================
 * Hardware Abstraction Layer: AMBA APB4 2-Channel Timer & PWM (Phase U4)
 * ============================================================================ */
#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#include <stdint.h>
#include "soc_regs.h"

#define TIMER_CH_STRIDE     0x20u
#define TIMER_CTRL_OFF(ch)  ((ch) * TIMER_CH_STRIDE + 0x00u)
#define TIMER_PRE_OFF(ch)   ((ch) * TIMER_CH_STRIDE + 0x04u)
#define TIMER_PER_OFF(ch)   ((ch) * TIMER_CH_STRIDE + 0x08u)
#define TIMER_CMP_OFF(ch)   ((ch) * TIMER_CH_STRIDE + 0x0Cu)
#define TIMER_CNT_OFF(ch)   ((ch) * TIMER_CH_STRIDE + 0x10u)
#define TIMER_STAT_OFF(ch)  ((ch) * TIMER_CH_STRIDE + 0x14u)

#define TIMER_CTRL_EN          (1u << 0)
#define TIMER_CTRL_AUTO_RELOAD (1u << 1)
#define TIMER_CTRL_PWM_EN      (1u << 2)
#define TIMER_CTRL_IRQ_EN      (1u << 3)
#define TIMER_CTRL_PWM_INV     (1u << 4)

static inline void hal_timer_init(
    uint32_t ch,
    uint32_t prescale,
    uint32_t period,
    uint32_t cmp,
    uint32_t ctrl_flags
) {
    mmio_write32(SOC_TIMER_BASE + TIMER_CTRL_OFF(ch), 0u);
    mmio_write32(SOC_TIMER_BASE + TIMER_STAT_OFF(ch), 1u); /* clear stale IRQ */
    mmio_write32(SOC_TIMER_BASE + TIMER_PRE_OFF(ch),  prescale & 0xFFFFu);
    mmio_write32(SOC_TIMER_BASE + TIMER_PER_OFF(ch),  period);
    mmio_write32(SOC_TIMER_BASE + TIMER_CMP_OFF(ch),  cmp);
    mmio_write32(SOC_TIMER_BASE + TIMER_CNT_OFF(ch),  0u);
    mmio_write32(SOC_TIMER_BASE + TIMER_CTRL_OFF(ch), ctrl_flags);
}

static inline void hal_timer_stop(uint32_t ch) {
    uint32_t c = mmio_read32(SOC_TIMER_BASE + TIMER_CTRL_OFF(ch));
    mmio_write32(SOC_TIMER_BASE + TIMER_CTRL_OFF(ch), c & ~TIMER_CTRL_EN);
}

static inline void hal_timer_set_duty(uint32_t ch, uint32_t cmp) {
    mmio_write32(SOC_TIMER_BASE + TIMER_CMP_OFF(ch), cmp);
}

static inline uint32_t hal_timer_get_count(uint32_t ch) {
    return mmio_read32(SOC_TIMER_BASE + TIMER_CNT_OFF(ch));
}

static inline uint32_t hal_timer_irq_status(uint32_t ch) {
    return mmio_read32(SOC_TIMER_BASE + TIMER_STAT_OFF(ch)) & 1u;
}

static inline void hal_timer_irq_clear(uint32_t ch) {
    mmio_write32(SOC_TIMER_BASE + TIMER_STAT_OFF(ch), 1u);
}

#endif /* HAL_TIMER_H */
