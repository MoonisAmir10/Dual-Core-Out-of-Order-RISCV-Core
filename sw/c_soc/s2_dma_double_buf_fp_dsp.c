/* ============================================================================
 * Phase U8 Flagship Benchmark S2: Double-Buffered Streaming FP DSP Pipeline
 * (Async I/O-Coherent AXI4 DMA Overlap [-DUSE_DMA=1] vs. CPU Copy [-DUSE_DMA=0])
 *
 * Processes 8 streaming 2 KB IEEE-754 float frames (512 floats/frame = 16 KB
 * input stream -> 16 KB output stream):
 *   - USE_DMA=1: Double-buffered ping-pong pipeline where DMA Ch0 prefetches
 *     frame f+1 (2 KB) AND DMA Ch1 drains previous result f-1 (2 KB, snooping
 *     dirty M lines directly out of BOTH cores' L1Ds with ZERO cache flushes!)
 *     in the background while Core 0 & Core 1 execute a parallel 8-tap IEEE-754
 *     FP FIR filter + energy dot-product on frame f, synchronized via PLIC
 *     DMA Ch0 (Source 4 -> Core 0) and Ch1 (Source 5 -> Core 1) C ISRs!
 *   - USE_DMA=0: Identical dual-core 8-tap FP FIR + energy kernel, but frame
 *     staging into/out of ping-pong buffers is performed by CPU load/store loops.
 *   - Both modes produce the EXACT SAME bit-exact IEEE-754 output checksum!
 * ============================================================================ */
#include <stdint.h>
#include "sync.h"
#include "hal_irq.h"
#include "hal_dma.h"
#include "hal_uart.h"

#ifndef USE_DMA
#define USE_DMA 1
#endif

#define NUM_FRAMES   8u
#define FRAME_FLOATS 512u          /* 2048 bytes = 128 cache lines per frame */
#define FRAME_BYTES  (FRAME_FLOATS * 4u)

/* 16 KB Input Stream + 16 KB Output Stream + 8 KB Ping-Pong Buffers */
static float g_in_stream[NUM_FRAMES][FRAME_FLOATS]  __attribute__((aligned(64)));
static float g_out_stream[NUM_FRAMES][FRAME_FLOATS] __attribute__((aligned(64)));
static float g_in_ping[2][FRAME_FLOATS]             __attribute__((aligned(64)));
static float g_out_ping[2][FRAME_FLOATS]            __attribute__((aligned(64)));

static const float g_fir_taps[8] = {
    0.0625f, 0.125f, 0.1875f, 0.25f, 0.25f, 0.1875f, 0.125f, 0.0625f
};

static volatile uint32_t g_ch0_irqs;
static volatile uint32_t g_ch1_irqs;
static volatile uint32_t g_energy_c0[NUM_FRAMES];
static volatile uint32_t g_energy_c1[NUM_FRAMES];
static volatile uint32_t g_c1_done;
static barrier_t         g_bar __attribute__((aligned(64)));

static inline uint32_t f2u(float x) {
    union { float f; uint32_t u; } v;
    v.f = x;
    return v.u;
}

static inline uint32_t rotl32(uint32_t x, uint32_t r) {
    return (x << r) | (x >> (32u - r));
}

#if USE_DMA
static void dma_ch0_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid;
    (void)src_id;
    uint32_t st = hal_dma_ch_status(0u);
    hal_dma_irq_clear(1u << 0);
    if ((st & DMA_STAT_ERR) != 0u) {
        soc_halt(0xBAD80201u);
    }
    g_ch0_irqs += 1u;
}

static void dma_ch1_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid;
    (void)src_id;
    uint32_t st = hal_dma_ch_status(1u);
    hal_dma_irq_clear(1u << 1);
    if ((st & DMA_STAT_ERR) != 0u) {
        soc_halt(0xBAD80202u);
    }
    g_ch1_irqs += 1u;
}
#endif

/* Parallel 8-tap IEEE-754 FP FIR + dot-product energy kernel (4-wide unrolled) */
static uint32_t dsp_fir_half(const float *in_buf, float *out_buf, uint32_t start, uint32_t end) {
    float h0 = g_fir_taps[0], h1 = g_fir_taps[1], h2 = g_fir_taps[2], h3 = g_fir_taps[3];
    float h4 = g_fir_taps[4], h5 = g_fir_taps[5], h6 = g_fir_taps[6], h7 = g_fir_taps[7];
    float e0 = 0.0f, e1 = 0.0f, e2 = 0.0f, e3 = 0.0f;

    uint32_t i = start;
    /* Handle boundary samples i < 7 cleanly */
    while (i < end && i < 7u) {
        float acc = 0.0f;
        for (uint32_t k = 0u; k < 8u; k++) {
            float s = (i >= k) ? in_buf[i - k] : 0.0f;
            acc += g_fir_taps[k] * s;
        }
        out_buf[i] = acc;
        e0 += acc * acc;
        i++;
    }
    /* Align to multiple of 4 */
    while (i < end && (i & 3u) != 0u) {
        float acc = h0 * in_buf[i] + h1 * in_buf[i - 1u] + h2 * in_buf[i - 2u] + h3 * in_buf[i - 3u]
                  + h4 * in_buf[i - 4u] + h5 * in_buf[i - 5u] + h6 * in_buf[i - 6u] + h7 * in_buf[i - 7u];
        out_buf[i] = acc;
        e0 += acc * acc;
        i++;
    }
    /* High-ILP 4-wide unrolled FMADD.S steady-state loop */
    for (; i + 3u < end; i += 4u) {
        float x_7 = in_buf[i - 7u], x_6 = in_buf[i - 6u], x_5 = in_buf[i - 5u], x_4 = in_buf[i - 4u];
        float x_3 = in_buf[i - 3u], x_2 = in_buf[i - 2u], x_1 = in_buf[i - 1u];
        float x0  = in_buf[i + 0u], x1  = in_buf[i + 1u], x2  = in_buf[i + 2u], x3  = in_buf[i + 3u];

        float y0 = h0 * x0 + h1 * x_1 + h2 * x_2 + h3 * x_3 + h4 * x_4 + h5 * x_5 + h6 * x_6 + h7 * x_7;
        float y1 = h0 * x1 + h1 * x0  + h2 * x_1 + h3 * x_2 + h4 * x_3 + h5 * x_4 + h6 * x_5 + h7 * x_6;
        float y2 = h0 * x2 + h1 * x1  + h2 * x0  + h3 * x_1 + h4 * x_2 + h5 * x_3 + h6 * x_4 + h7 * x_5;
        float y3 = h0 * x3 + h1 * x2  + h2 * x1  + h3 * x0  + h4 * x_1 + h5 * x_2 + h6 * x_3 + h7 * x_4;

        out_buf[i + 0u] = y0;
        out_buf[i + 1u] = y1;
        out_buf[i + 2u] = y2;
        out_buf[i + 3u] = y3;

        e0 += y0 * y0;
        e1 += y1 * y1;
        e2 += y2 * y2;
        e3 += y3 * y3;
    }
    while (i < end) {
        float acc = h0 * in_buf[i] + h1 * in_buf[i - 1u] + h2 * in_buf[i - 2u] + h3 * in_buf[i - 3u]
                  + h4 * in_buf[i - 4u] + h5 * in_buf[i - 5u] + h6 * in_buf[i - 6u] + h7 * in_buf[i - 7u];
        out_buf[i] = acc;
        e0 += acc * acc;
        i++;
    }
    return f2u((e0 + e1) + (e2 + e3));
}

#if !USE_DMA
static void cpu_copy_half(uint32_t *dst, const uint32_t *src, uint32_t start, uint32_t end) {
    for (uint32_t i = start; i < end; i += 4u) {
        uint32_t w0 = src[i + 0u];
        uint32_t w1 = src[i + 1u];
        uint32_t w2 = src[i + 2u];
        uint32_t w3 = src[i + 3u];
        dst[i + 0u] = w0;
        dst[i + 1u] = w1;
        dst[i + 2u] = w2;
        dst[i + 3u] = w3;
    }
}
#endif

uint32_t main(uint32_t hartid) {
    uint32_t local_sense = 0u;
    uint32_t half_s = hartid * (FRAME_FLOATS / 2u);
    uint32_t half_e = (hartid + 1u) * (FRAME_FLOATS / 2u);

    if (hartid == 0u) {
        g_ch0_irqs = 0u;
        g_ch1_irqs = 0u;
        g_c1_done  = 0u;
        hal_uart_init(4u, 1u, 0u);
        hal_uart_irq_enable(0u, 0u);
    }

    /* Parallel initialization of all 8 streaming 2 KB input frames */
    for (uint32_t f = 0u; f < NUM_FRAMES; f++) {
        for (uint32_t i = half_s; i < half_e; i++) {
            int32_t s = (int32_t)(((f * 19u + i * 7u) & 31u)) - 16;
            g_in_stream[f][i]  = (float)s * 0.125f;
            g_out_stream[f][i] = 0.0f;
        }
    }

#if USE_DMA
    hal_irq_init(hartid);
    hal_irq_set_mtvec_direct();
    hal_irq_enable_mie(MIE_MEIE);
    hal_plic_set_threshold(hartid, 0u);

    if (hartid == 0u) {
        hal_dma_irq_clear(0x3u);
        hal_plic_set_trigger_edge(PLIC_IRQ_DMA_CH0, 0u);
        hal_plic_set_priority(PLIC_IRQ_DMA_CH0, 5u);
        hal_irq_register_ext(0u, PLIC_IRQ_DMA_CH0, dma_ch0_isr);
        hal_plic_enable_irq(0u, PLIC_IRQ_DMA_CH0);
    } else {
        hal_plic_set_trigger_edge(PLIC_IRQ_DMA_CH1, 0u);
        hal_plic_set_priority(PLIC_IRQ_DMA_CH1, 5u);
        hal_irq_register_ext(1u, PLIC_IRQ_DMA_CH1, dma_ch1_isr);
        hal_plic_enable_irq(1u, PLIC_IRQ_DMA_CH1);
    }
    barrier_wait(&g_bar, 2u, &local_sense);
    hal_irq_global_enable();
#else
    barrier_wait(&g_bar, 2u, &local_sense);
#endif

    /* Measure exact steady-state streaming pipeline latency (excluding init/checksum) */
    uint32_t pipe_t0 = (hartid == 0u) ? hal_clint_mtime_lo() : 0u;

#if USE_DMA
    /* Prologue: Prefetch Frame 0 via Coherent DMA Ch0 */
    uint32_t exp_ch0 = 0u, exp_ch1 = 0u;
    if (hartid == 0u) {
        hal_dma_memcpy_async(0u, g_in_ping[0], g_in_stream[0], FRAME_BYTES, 1u);
        exp_ch0 += 1u;
        while (g_ch0_irqs < exp_ch0) {}
        __asm__ volatile ("fence i, r" ::: "memory");
    }
    barrier_wait(&g_bar, 2u, &local_sense);

    /* Double-Buffered Async Coherent DMA + Dual-Core FP DSP Main Pipeline */
    for (uint32_t f = 0u; f < NUM_FRAMES; f++) {
        uint32_t cur = f & 1u;
        uint32_t nxt = (f + 1u) & 1u;

        /* Core 0 launches background Ch0 prefetch (f+1) AND Ch1 drain (f-1) */
        if (hartid == 0u) {
            if (f + 1u < NUM_FRAMES) {
                hal_dma_memcpy_async(0u, g_in_ping[nxt], g_in_stream[f + 1u], FRAME_BYTES, 1u);
                exp_ch0 += 1u;
            }
            if (f > 0u) {
                /* ZERO cache flush! DMA Ch1 snoops dirty M lines from BOTH L1D0 & L1D1! */
                hal_dma_memcpy_async(1u, g_out_stream[f - 1u], g_out_ping[nxt], FRAME_BYTES, 1u);
            }
        } else if (f > 0u) {
            exp_ch1 += 1u;
        }

        /* BOTH Cores compute 8-tap IEEE-754 FP FIR + dot-product CONCURRENTLY with DMA! */
        uint32_t e = dsp_fir_half(g_in_ping[cur], g_out_ping[cur], half_s, half_e);
        if (hartid == 0u) g_energy_c0[f] = e;
        else              g_energy_c1[f] = e;

        /* Synchronize background DMA Ch0 (on Core 0) & Ch1 (on Core 1) via PLIC ISRs */
        if (hartid == 0u) {
            while (g_ch0_irqs < exp_ch0) {}
            __asm__ volatile ("fence i, r" ::: "memory");
        } else {
            while (g_ch1_irqs < exp_ch1) {}
            __asm__ volatile ("fence i, r" ::: "memory");
        }
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* Epilogue: Drain final Frame 7 via Coherent DMA Ch1 */
    if (hartid == 0u) {
        hal_dma_memcpy_async(1u, g_out_stream[NUM_FRAMES - 1u], g_out_ping[(NUM_FRAMES - 1u) & 1u], FRAME_BYTES, 1u);
    } else {
        exp_ch1 += 1u;
        while (g_ch1_irqs < exp_ch1) {}
        __asm__ volatile ("fence i, r" ::: "memory");
    }
    barrier_wait(&g_bar, 2u, &local_sense);
#else
    /* CPU-Copy Double-Buffered Baseline (No DMA Hardware Overlap) */
    for (uint32_t f = 0u; f < NUM_FRAMES; f++) {
        uint32_t cur = f & 1u;
        cpu_copy_half((uint32_t *)g_in_ping[cur], (const uint32_t *)g_in_stream[f], half_s, half_e);
        barrier_wait(&g_bar, 2u, &local_sense);

        uint32_t e = dsp_fir_half(g_in_ping[cur], g_out_ping[cur], half_s, half_e);
        if (hartid == 0u) g_energy_c0[f] = e;
        else              g_energy_c1[f] = e;
        barrier_wait(&g_bar, 2u, &local_sense);

        cpu_copy_half((uint32_t *)g_out_stream[f], (const uint32_t *)g_out_ping[cur], half_s, half_e);
        barrier_wait(&g_bar, 2u, &local_sense);
    }
#endif

    uint32_t pipe_cyc = (hartid == 0u) ? (hal_clint_mtime_lo() - pipe_t0) : 0u;

    /* Bit-exact IEEE-754 checksum over all 8*512 output floats + frame energies */
    uint32_t hash = 0xD5000008u;
    for (uint32_t f = 0u; f < NUM_FRAMES; f++) {
        hash = rotl32(hash ^ g_energy_c0[f] ^ g_energy_c1[f], 3u);
        for (uint32_t i = 0u; i < FRAME_FLOATS; i++) {
            hash = rotl32(hash ^ f2u(g_out_stream[f][i]), 5u) + 0x9E3779B9u;
        }
    }

    if (hartid == 0u) {
#if USE_DMA
        hal_uart_printf("S2_DSP mode=DMA pipe_cyc=%u ch0_irqs=%u ch1_irqs=%u hash=0x%08x\n",
                        pipe_cyc, g_ch0_irqs, g_ch1_irqs, hash);
#else
        hal_uart_printf("S2_DSP mode=CPU pipe_cyc=%u ch0_irqs=0 ch1_irqs=0 hash=0x%08x\n",
                        pipe_cyc, hash);
#endif
        hal_uart_tx_flush();
    }

    barrier_wait(&g_bar, 2u, &local_sense);
#if USE_DMA
    hal_irq_global_disable();
#endif

    if (hartid == 1u) {
        soc_park_secondary(hash, &g_c1_done);
    } else {
        while (!g_c1_done) {}
        soc_halt(hash);
    }
    return hash;
}
