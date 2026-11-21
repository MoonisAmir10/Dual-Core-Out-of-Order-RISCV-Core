/* ============================================================================
 * Hardware Abstraction Layer: AMBA APB4 16-FIFO 8N1 UART & printf (Phase U4)
 * ============================================================================ */
#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include "soc_regs.h"

#define UART_TX_DATA_OFF    0x000u
#define UART_RX_DATA_OFF    0x004u
#define UART_STATUS_OFF     0x008u
#define UART_CTRL_OFF       0x00Cu
#define UART_BAUD_DIV_OFF   0x010u

#define UART_STAT_TX_FULL   (1u << 0)
#define UART_STAT_TX_EMPTY  (1u << 1)
#define UART_STAT_RX_EMPTY  (1u << 2)
#define UART_STAT_RX_FULL   (1u << 3)
#define UART_STAT_TX_BUSY   (1u << 4)

#define UART_CTRL_TX_EN     (1u << 0)
#define UART_CTRL_RX_EN     (1u << 1)
#define UART_CTRL_LOOPBACK  (1u << 2)
#define UART_CTRL_FAST_SIM  (1u << 3)
#define UART_CTRL_RX_IRQ_EN (1u << 4)
#define UART_CTRL_TX_IRQ_EN (1u << 5)

static inline void hal_uart_init(uint32_t baud_div, uint32_t fast_sim, uint32_t loopback) {
    mmio_write32(SOC_UART_BASE + UART_BAUD_DIV_OFF, baud_div & 0xFFFFu);
    uint32_t ctrl = UART_CTRL_TX_EN | UART_CTRL_RX_EN |
                    (loopback ? UART_CTRL_LOOPBACK : 0u) |
                    (fast_sim ? UART_CTRL_FAST_SIM : 0u);
    mmio_write32(SOC_UART_BASE + UART_CTRL_OFF, ctrl);
}

static inline void hal_uart_irq_enable(uint32_t rx_en, uint32_t tx_en) {
    uint32_t ctrl = mmio_read32(SOC_UART_BASE + UART_CTRL_OFF);
    ctrl &= ~(UART_CTRL_RX_IRQ_EN | UART_CTRL_TX_IRQ_EN);
    if (rx_en) ctrl |= UART_CTRL_RX_IRQ_EN;
    if (tx_en) ctrl |= UART_CTRL_TX_IRQ_EN;
    mmio_write32(SOC_UART_BASE + UART_CTRL_OFF, ctrl);
}

static inline uint32_t hal_uart_status(void) {
    return mmio_read32(SOC_UART_BASE + UART_STATUS_OFF);
}

static inline uint32_t hal_uart_rx_available(void) {
    return (hal_uart_status() & UART_STAT_RX_EMPTY) == 0u;
}

static inline void hal_uart_putc(char c) {
    while (hal_uart_status() & UART_STAT_TX_FULL) {
        /* Backpressure cleanly if 16-entry TX FIFO is full */
    }
    mmio_write32(SOC_UART_BASE + UART_TX_DATA_OFF, (uint32_t)(uint8_t)c);
}

static inline void hal_uart_puts(const char *s) {
    while (*s) {
        hal_uart_putc(*s++);
    }
}

static inline uint32_t hal_uart_getc_nonblock(uint8_t *out) {
    if (!hal_uart_rx_available()) return 0u;
    *out = (uint8_t)(mmio_read32(SOC_UART_BASE + UART_RX_DATA_OFF) & 0xFFu);
    return 1u;
}

static inline uint8_t hal_uart_getc(void) {
    while (!hal_uart_rx_available()) {}
    return (uint8_t)(mmio_read32(SOC_UART_BASE + UART_RX_DATA_OFF) & 0xFFu);
}

static inline void hal_uart_tx_flush(void) {
    while ((hal_uart_status() & (UART_STAT_TX_EMPTY | UART_STAT_TX_BUSY)) != UART_STAT_TX_EMPTY) {
        /* Wait until TX FIFO empty AND serializer idle */
    }
}

/* Bare-metal formatted console printer (implemented in sw/hal/hal_uart.c) */
void hal_uart_printf(const char *fmt, ...);

#endif /* HAL_UART_H */
