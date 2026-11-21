/* ============================================================================
 * Hardware Abstraction Layer: 2-Channel I/O-Coherent AMBA AXI4 DMA Engine
 * (Phase U7 — APB4 Slave S4 @ 0x1002_0000, PLIC Sources 4 & 5)
 *
 * Fully hardware cache-coherent with BOTH cores' MOESI L1D caches & Shared L2:
 *   - NO manual cache flushes required before DMA reads from dirty L1D buffers!
 *   - NO manual cache invalidations required after DMA writes (hardware snoops
 *     both L1Ds and replays any speculative loads in lsq2w automatically)!
 * ============================================================================ */
#ifndef HAL_DMA_H
#define HAL_DMA_H

#include <stdint.h>
#include "soc_regs.h"

/* Global DMA Register Offsets */
#define DMA_STATUS_OFF          0x000u
#define DMA_IRQ_STAT_OFF        0x004u
#define DMA_VERSION_OFF         0x008u

/* Per-Channel Register Offsets (ch = 0 -> 0x040, ch = 1 -> 0x080) */
#define DMA_CH_BASE(ch)         (SOC_DMA_BASE + 0x040u + 0x040u * ((ch) & 1u))
#define DMA_CH_SRC_OFF          0x00u
#define DMA_CH_DST_OFF          0x04u
#define DMA_CH_LEN_OFF          0x08u
#define DMA_CH_SRC_STRIDE_OFF   0x0Cu
#define DMA_CH_DST_STRIDE_OFF   0x10u
#define DMA_CH_ROW_CNT_OFF      0x14u
#define DMA_CH_CTRL_OFF         0x18u
#define DMA_CH_STATUS_OFF       0x1Cu

/* DMA_CH_CTRL Bitfields */
#define DMA_CTRL_START          (1u << 0)
#define DMA_CTRL_IRQ_EN         (1u << 1)
#define DMA_CTRL_MODE_2D        (1u << 2)
#define DMA_CTRL_ABORT          (1u << 3)
#define DMA_CTRL_BURST_SHIFT    4u
#define DMA_CTRL_BURST_MASK     (0xFu << DMA_CTRL_BURST_SHIFT)

/* DMA_CH_STATUS Bitfields */
#define DMA_STAT_BUSY           (1u << 0)
#define DMA_STAT_DONE           (1u << 1)
#define DMA_STAT_ERR            (1u << 2)
#define DMA_STAT_ERR_CODE_SHIFT 4u
#define DMA_STAT_ERR_CODE_MASK  (0x3u << DMA_STAT_ERR_CODE_SHIFT)
#define DMA_STAT_BYTES_SHIFT    8u

/* AXI4 Error Codes Reported in DMA_STAT_ERR_CODE */
#define DMA_ERR_SLVERR          2u
#define DMA_ERR_DECERR          3u

static inline uint32_t hal_dma_version(void) {
    return mmio_read32(SOC_DMA_BASE + DMA_VERSION_OFF);
}

static inline uint32_t hal_dma_global_status(void) {
    return mmio_read32(SOC_DMA_BASE + DMA_STATUS_OFF);
}

static inline uint32_t hal_dma_irq_status(void) {
    return mmio_read32(SOC_DMA_BASE + DMA_IRQ_STAT_OFF) & 0x3u;
}

static inline void hal_dma_irq_clear(uint32_t ch_mask) {
    mmio_write32(SOC_DMA_BASE + DMA_IRQ_STAT_OFF, ch_mask & 0x3u);
}

static inline uint32_t hal_dma_ch_status(uint32_t ch) {
    return mmio_read32(DMA_CH_BASE(ch) + DMA_CH_STATUS_OFF);
}

static inline uint32_t hal_dma_ch_bytes_done(uint32_t ch) {
    return hal_dma_ch_status(ch) >> DMA_STAT_BYTES_SHIFT;
}

static inline uint32_t hal_dma_ch_is_busy(uint32_t ch) {
    return (hal_dma_ch_status(ch) & DMA_STAT_BUSY) != 0u;
}

static inline uint32_t hal_dma_ch_has_err(uint32_t ch) {
    return (hal_dma_ch_status(ch) & DMA_STAT_ERR) != 0u;
}

static inline uint32_t hal_dma_ch_err_code(uint32_t ch) {
    return (hal_dma_ch_status(ch) & DMA_STAT_ERR_CODE_MASK) >> DMA_STAT_ERR_CODE_SHIFT;
}

static inline void hal_dma_abort(uint32_t ch) {
    mmio_write32(DMA_CH_BASE(ch) + DMA_CH_CTRL_OFF, DMA_CTRL_ABORT);
    while (hal_dma_ch_is_busy(ch)) {}
}

/* Start a 1D coherent memory copy (arbitrary byte alignment & length!).
 * burst_max: 0 => default max 16-beat (256B) AXI4 bursts; 1..15 => 1..15 beats. */
static inline void hal_dma_start_ex(uint32_t ch, uint32_t src, uint32_t dst,
                                    uint32_t len_bytes, uint32_t irq_en, uint32_t burst_max) {
    uint32_t base = DMA_CH_BASE(ch);
    /* Ensure prior core stores into source buffer are ordered before MMIO trigger */
    __asm__ volatile ("fence w, o" ::: "memory");
    mmio_write32(base + DMA_CH_SRC_OFF, src);
    mmio_write32(base + DMA_CH_DST_OFF, dst);
    mmio_write32(base + DMA_CH_LEN_OFF, len_bytes);
    uint32_t ctrl = DMA_CTRL_START |
                    (irq_en ? DMA_CTRL_IRQ_EN : 0u) |
                    ((burst_max & 0xFu) << DMA_CTRL_BURST_SHIFT);
    mmio_write32(base + DMA_CH_CTRL_OFF, ctrl);
}

static inline void hal_dma_start(uint32_t ch, uint32_t src, uint32_t dst,
                                 uint32_t len_bytes, uint32_t irq_en) {
    hal_dma_start_ex(ch, src, dst, len_bytes, irq_en, 0u);
}

/* Start a 2D strided coherent transfer (row_cnt rows of row_bytes each) */
static inline void hal_dma_start_2d(uint32_t ch, uint32_t src, uint32_t dst,
                                    uint32_t row_bytes, uint32_t src_stride,
                                    uint32_t dst_stride, uint32_t row_cnt, uint32_t irq_en) {
    uint32_t base = DMA_CH_BASE(ch);
    __asm__ volatile ("fence w, o" ::: "memory");
    mmio_write32(base + DMA_CH_SRC_OFF, src);
    mmio_write32(base + DMA_CH_DST_OFF, dst);
    mmio_write32(base + DMA_CH_LEN_OFF, row_bytes);
    mmio_write32(base + DMA_CH_SRC_STRIDE_OFF, src_stride);
    mmio_write32(base + DMA_CH_DST_STRIDE_OFF, dst_stride);
    mmio_write32(base + DMA_CH_ROW_CNT_OFF, row_cnt);
    uint32_t ctrl = DMA_CTRL_START | DMA_CTRL_MODE_2D | (irq_en ? DMA_CTRL_IRQ_EN : 0u);
    mmio_write32(base + DMA_CH_CTRL_OFF, ctrl);
}

/* Pointer convenience wrapper for asynchronous coherent memcpy */
static inline void hal_dma_memcpy_async(uint32_t ch, void *dst, const void *src,
                                        uint32_t len_bytes, uint32_t irq_en) {
    hal_dma_start(ch, (uint32_t)(uintptr_t)src, (uint32_t)(uintptr_t)dst, len_bytes, irq_en);
}

/* Poll until channel finishes (DONE or ERR); returns CH_STATUS */
static inline uint32_t hal_dma_wait(uint32_t ch) {
    uint32_t st;
    do {
        st = hal_dma_ch_status(ch);
    } while ((st & DMA_STAT_BUSY) != 0u);
    __asm__ volatile ("fence i, r" ::: "memory");
    return st;
}

#endif /* HAL_DMA_H */
