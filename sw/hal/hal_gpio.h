/* ============================================================================
 * Hardware Abstraction Layer: AMBA APB4 32-Pin GPIO Controller (Phase U4)
 * ============================================================================ */
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>
#include "soc_regs.h"

#define GPIO_DATA_IN_OFF    0x000u
#define GPIO_DATA_OUT_OFF   0x004u
#define GPIO_DIR_OFF        0x008u
#define GPIO_OUT_SET_OFF    0x00Cu
#define GPIO_OUT_CLR_OFF    0x010u
#define GPIO_OUT_TGL_OFF    0x014u
#define GPIO_IRQ_EN_OFF     0x018u
#define GPIO_IRQ_TYPE_OFF   0x01Cu
#define GPIO_IRQ_POL_OFF    0x020u
#define GPIO_IRQ_STAT_OFF   0x024u

static inline void hal_gpio_set_dir(uint32_t dir_mask) {
    mmio_write32(SOC_GPIO_BASE + GPIO_DIR_OFF, dir_mask);
}

static inline uint32_t hal_gpio_get_dir(void) {
    return mmio_read32(SOC_GPIO_BASE + GPIO_DIR_OFF);
}

static inline uint32_t hal_gpio_read(void) {
    return mmio_read32(SOC_GPIO_BASE + GPIO_DATA_IN_OFF);
}

static inline void hal_gpio_write(uint32_t val) {
    mmio_write32(SOC_GPIO_BASE + GPIO_DATA_OUT_OFF, val);
}

static inline void hal_gpio_set_bits(uint32_t mask) {
    mmio_write32(SOC_GPIO_BASE + GPIO_OUT_SET_OFF, mask);
}

static inline void hal_gpio_clear_bits(uint32_t mask) {
    mmio_write32(SOC_GPIO_BASE + GPIO_OUT_CLR_OFF, mask);
}

static inline void hal_gpio_toggle_bits(uint32_t mask) {
    mmio_write32(SOC_GPIO_BASE + GPIO_OUT_TGL_OFF, mask);
}

static inline void hal_gpio_irq_config(uint32_t pin_mask, uint32_t is_edge, uint32_t pol_high) {
    uint32_t t = mmio_read32(SOC_GPIO_BASE + GPIO_IRQ_TYPE_OFF);
    uint32_t p = mmio_read32(SOC_GPIO_BASE + GPIO_IRQ_POL_OFF);
    if (is_edge)  t |=  pin_mask; else t &= ~pin_mask;
    if (pol_high) p |=  pin_mask; else p &= ~pin_mask;
    mmio_write32(SOC_GPIO_BASE + GPIO_IRQ_TYPE_OFF, t);
    mmio_write32(SOC_GPIO_BASE + GPIO_IRQ_POL_OFF,  p);
}

static inline void hal_gpio_irq_enable(uint32_t pin_mask) {
    uint32_t e = mmio_read32(SOC_GPIO_BASE + GPIO_IRQ_EN_OFF);
    mmio_write32(SOC_GPIO_BASE + GPIO_IRQ_EN_OFF, e | pin_mask);
}

static inline void hal_gpio_irq_disable(uint32_t pin_mask) {
    uint32_t e = mmio_read32(SOC_GPIO_BASE + GPIO_IRQ_EN_OFF);
    mmio_write32(SOC_GPIO_BASE + GPIO_IRQ_EN_OFF, e & ~pin_mask);
}

static inline uint32_t hal_gpio_irq_status(void) {
    return mmio_read32(SOC_GPIO_BASE + GPIO_IRQ_STAT_OFF);
}

static inline void hal_gpio_irq_clear(uint32_t pin_mask) {
    mmio_write32(SOC_GPIO_BASE + GPIO_IRQ_STAT_OFF, pin_mask);
}

#endif /* HAL_GPIO_H */
