/* ============================================================================
 * Phase U7 Live Dual-Core RV32IMAF Bare-Metal C I/O-Coherent DMA Proof
 *
 * Exercises the FULL 3-Master MOESI + AMBA AXI4 Coherent DMA SoC End-to-End:
 *   1. Zero-Flush 4 KB Dirty-L1D Coherent DMA + Concurrent Dual-Core Hard-Float
 *      RV32IMAF Compute + Cross-Core PLIC IRQ (Source 4 -> Core 1!):
 *        - Core 0 writes 1024 IEEE-754 floats (4 KB) into g_src_buf (dirty M
 *          in Core 0 L1D — ZERO manual flush or writeback!).
 *        - Core 0 launches async DMA Ch0 -> g_dst_buf with PLIC Source 4 routed
 *          to Core 1.
 *        - Both cores execute hard-float FMADD.S + FDIV.S workloads in parallel
 *          while DMA Ch0 streams 16-beat AXI4 bursts snooping Core 0's L1D!
 *        - PLIC Source 4 C ISR fires on Core 1 -> Core 1 verifies all 1024
 *          floats bit-exact (0 stale bytes!).
 *   2. Concurrent Dual-Channel DMA (Ch0 Unaligned 1D + Ch1 2D Strided Matrix):
 *        - Ch0 copies 277 unaligned bytes (+3 -> +7) with dirty canary guard
 *          bytes on both edges, firing PLIC Source 4 on Core 1!
 *        - Ch1 simultaneously copies an 8-row x 20B 2D strided float tile
 *          (src_stride=32, dst_stride=48), firing PLIC Source 5 on Core 0!
 *   3. Out-of-Range PMA DECERR Bus-Error Capture & Recovery:
 *        - Ch0 reads 0x0004_0000 (past 256 KB RAM) -> PLIC Source 4 error IRQ,
 *          CH_STATUS.ERR=1, ERR_CODE=DECERR(3) -> clean recovery on next copy!
 *   4. Adversarial Speculative Load vs. DMA Write Hardware Snoop-Replay Proof:
 *        - Core 1 caches g_replay_buf in L1D, then speculatively loads behind
 *          a slow FDIV.S latency window while DMA Ch0 overwrites that exact
 *          cache line -> proves hardware LQ snoop-replay catches DMA's
 *          invalidation with ZERO stale reads and replay > 0!
 * ============================================================================ */
#include <stdint.h>
#include "sync.h"
#include "hal_irq.h"
#include "hal_dma.h"
#include "hal_uart.h"

#define BUF_FLOATS  1024u /* 4096 bytes = 256 cache lines (entire 4 KB L1D!) */

static float    g_src_buf[BUF_FLOATS] __attribute__((aligned(64)));
static float    g_dst_buf[BUF_FLOATS] __attribute__((aligned(64)));

/* Test 2: Unaligned byte copy & 2D strided tile buffers */
static uint8_t  g_unalign_src[320]    __attribute__((aligned(16)));
static uint8_t  g_unalign_dst[320]    __attribute__((aligned(16)));
static uint8_t  g_2d_src[8 * 32]      __attribute__((aligned(16)));
static uint8_t  g_2d_dst[8 * 48]      __attribute__((aligned(16)));

/* Test 3: Recovery buffer */
static uint32_t g_rec_src[16]         __attribute__((aligned(16)));
static uint32_t g_rec_dst[16]         __attribute__((aligned(16)));

/* Test 4: Speculative load vs. DMA write snoop-replay buffers (separate lines) */
static volatile uint32_t g_rpl_src[4]  __attribute__((aligned(64)));
static volatile uint32_t g_rpl_dst[4]  __attribute__((aligned(64)));
static volatile uint32_t g_rpl_go      __attribute__((aligned(64)));

static volatile uint32_t g_ch0_irq_cnt;
static volatile uint32_t g_ch1_irq_cnt;
static volatile uint32_t g_ch0_last_stat;
static volatile uint32_t g_ch1_last_stat;
static volatile uint32_t g_c1_done;
static barrier_t         g_bar;

static inline uint32_t f2u(float f) {
    union { float f; uint32_t u; } v;
    v.f = f;
    return v.u;
}

static void dma_ch0_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid;
    (void)src_id;
    g_ch0_last_stat = hal_dma_ch_status(0u);
    hal_dma_irq_clear(1u << 0);
    g_ch0_irq_cnt += 1u;
}

static void dma_ch1_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid;
    (void)src_id;
    g_ch1_last_stat = hal_dma_ch_status(1u);
    hal_dma_irq_clear(1u << 1);
    g_ch1_irq_cnt += 1u;
}

/* Hard-float RV32IMAF concurrent compute workload */
static uint32_t fp_compute_kernel(uint32_t seed, uint32_t iters) {
    float a = 1.125f + (float)(seed & 7u) * 0.25f;
    float b = 0.99609375f;
    float c = 0.03125f;
    for (uint32_t i = 0u; i < iters; i++) {
        a = a * b + c;
        if ((i & 7u) == 7u) {
            a = a / 1.0078125f;
        }
    }
    return f2u(a);
}

uint32_t main(uint32_t hartid) {
    uint32_t local_sense = 0u;
    uint32_t feat = mmio_read32(SOC_CTRL_BASE + SOC_CTRL_FEATURES_OFF);
    if ((feat & (SOC_FEAT_DMA | SOC_FEAT_FPU | SOC_FEAT_PLIC)) !=
        (SOC_FEAT_DMA | SOC_FEAT_FPU | SOC_FEAT_PLIC)) {
        soc_halt(0xBAD70001u);
    }

    if (hal_dma_version() != 0x00020100u) {
        soc_halt(0xBAD70002u);
    }

    /* Setup per-hart trap & PLIC */
    hal_irq_init(hartid);
    hal_irq_set_mtvec_direct();
    hal_irq_enable_mie(MIE_MEIE);
    hal_plic_set_threshold(hartid, 0u);

    if (hartid == 0u) {
        g_ch0_irq_cnt   = 0u;
        g_ch1_irq_cnt   = 0u;
        g_ch0_last_stat = 0u;
        g_ch1_last_stat = 0u;
        g_c1_done       = 0u;
        g_rpl_go        = 0u;

        hal_uart_init(4u, 1u, 0u);
        hal_uart_irq_enable(0u, 0u);
        hal_dma_irq_clear(0x3u);

        /* Route DMA Ch1 IRQ (PLIC Source 5) -> Core 0 */
        hal_plic_set_trigger_edge(PLIC_IRQ_DMA_CH1, 0u);
        hal_plic_set_priority(PLIC_IRQ_DMA_CH1, 5u);
        hal_irq_register_ext(0u, PLIC_IRQ_DMA_CH1, dma_ch1_isr);
        hal_plic_enable_irq(0u, PLIC_IRQ_DMA_CH1);
    } else {
        /* Route DMA Ch0 IRQ (PLIC Source 4) -> Core 1 */
        hal_plic_set_trigger_edge(PLIC_IRQ_DMA_CH0, 0u);
        hal_plic_set_priority(PLIC_IRQ_DMA_CH0, 5u);
        hal_irq_register_ext(1u, PLIC_IRQ_DMA_CH0, dma_ch0_isr);
        hal_plic_enable_irq(1u, PLIC_IRQ_DMA_CH0);
    }

    barrier_wait(&g_bar, 2u, &local_sense);
    hal_irq_global_enable();

    /* ========================================================================
     * TEST 1: Zero-Flush 4 KB Dirty-L1D Coherent DMA + Concurrent RV32IMAF +
     *         Cross-Core PLIC Source 4 Completion IRQ on Core 1!
     * ======================================================================== */
    uint32_t exp_fp = fp_compute_kernel(hartid + 1u, 32u);
    uint32_t got_fp = 0u;

    if (hartid == 0u) {
        /* Fill 4 KB (1024 floats) in Core 0's private L1D (M state, NO FLUSH!) */
        for (uint32_t i = 0u; i < BUF_FLOATS; i++) {
            g_src_buf[i] = (float)(i + 1u) * 0.375f + 1.5f;
        }
        /* Kick off async 4 KB coherent DMA Ch0 -> g_dst_buf (IRQ fires on Core 1!) */
        hal_dma_memcpy_async(0u, g_dst_buf, g_src_buf, BUF_FLOATS * sizeof(float), 1u);
        got_fp = fp_compute_kernel(1u, 32u);
    } else {
        /* Core 1 runs concurrent RV32IMAF FP compute while awaiting DMA Ch0 PLIC IRQ! */
        got_fp = fp_compute_kernel(2u, 32u);
        while (g_ch0_irq_cnt < 1u) {}
        __asm__ volatile ("fence i, r" ::: "memory");

        if ((g_ch0_last_stat & DMA_STAT_DONE) == 0u ||
            (g_ch0_last_stat & DMA_STAT_ERR)  != 0u ||
            (g_ch0_last_stat >> DMA_STAT_BYTES_SHIFT) != 4096u) {
            soc_halt(0xBAD70101u);
        }

        /* Verify ALL 1024 floats bit-exact from Core 1! */
        for (uint32_t i = 0u; i < BUF_FLOATS; i++) {
            float exp_v = (float)(i + 1u) * 0.375f + 1.5f;
            if (f2u(g_dst_buf[i]) != f2u(exp_v)) {
                soc_halt(0xBAD71000u | (i & 0xFFFu));
            }
        }
    }

    if (got_fp != exp_fp) {
        soc_halt(0xBAD70102u | (hartid << 8));
    }

    barrier_wait(&g_bar, 2u, &local_sense);

    /* ========================================================================
     * TEST 2: Concurrent Dual-Channel DMA (Ch0 Unaligned 1D + Ch1 2D Strided)
     *         Ch0 IRQ -> Core 1 (Src 4), Ch1 IRQ -> Core 0 (Src 5)!
     * ======================================================================== */
    if (hartid == 0u) {
        for (uint32_t i = 0u; i < 320u; i++) {
            g_unalign_src[i] = (uint8_t)((i * 37u + 0x5Au) & 0xFFu);
            g_unalign_dst[i] = 0xEEu; /* Guard sentinel */
        }
        for (uint32_t r = 0u; r < 8u; r++) {
            for (uint32_t c = 0u; c < 32u; c++) {
                g_2d_src[r * 32u + c] = (uint8_t)(((r << 5) ^ (c * 13u) ^ 0xA5u) & 0xFFu);
            }
            for (uint32_t c = 0u; c < 48u; c++) {
                g_2d_dst[r * 48u + c] = 0xCCu; /* Guard sentinel */
            }
        }
        /* Launch BOTH channels concurrently without flushing L1D! */
        hal_dma_start_ex(0u, (uint32_t)(uintptr_t)&g_unalign_src[3],
                             (uint32_t)(uintptr_t)&g_unalign_dst[7], 277u, 1u, 4u);
        hal_dma_start_2d(1u, (uint32_t)(uintptr_t)&g_2d_src[2],
                             (uint32_t)(uintptr_t)&g_2d_dst[5], 20u, 32u, 48u, 8u, 1u);

        /* Core 0 awaits Ch1 PLIC Source 5 ISR and verifies 2D strided destination */
        while (g_ch1_irq_cnt < 1u) {}
        __asm__ volatile ("fence i, r" ::: "memory");
        if ((g_ch1_last_stat & DMA_STAT_DONE) == 0u ||
            (g_ch1_last_stat >> DMA_STAT_BYTES_SHIFT) != 160u) {
            soc_halt(0xBAD70201u);
        }
        for (uint32_t r = 0u; r < 8u; r++) {
            if (g_2d_dst[r * 48u + 4u] != 0xCCu || g_2d_dst[r * 48u + 25u] != 0xCCu) {
                soc_halt(0xBAD70202u | (r << 4));
            }
            for (uint32_t c = 0u; c < 20u; c++) {
                uint8_t exp_b = (uint8_t)(((r << 5) ^ ((c + 2u) * 13u) ^ 0xA5u) & 0xFFu);
                if (g_2d_dst[r * 48u + 5u + c] != exp_b) {
                    soc_halt(0xBAD72000u | (r << 8) | c);
                }
            }
        }
    } else {
        /* Core 1 awaits Ch0 PLIC Source 4 ISR and verifies unaligned copy + guards */
        while (g_ch0_irq_cnt < 2u) {}
        __asm__ volatile ("fence i, r" ::: "memory");
        if ((g_ch0_last_stat & DMA_STAT_DONE) == 0u ||
            (g_ch0_last_stat >> DMA_STAT_BYTES_SHIFT) != 277u) {
            soc_halt(0xBAD70203u);
        }
        if (g_unalign_dst[6] != 0xEEu || g_unalign_dst[7u + 277u] != 0xEEu) {
            soc_halt(0xBAD70204u);
        }
        for (uint32_t i = 0u; i < 277u; i++) {
            uint8_t exp_b = (uint8_t)(((i + 3u) * 37u + 0x5Au) & 0xFFu);
            if (g_unalign_dst[7u + i] != exp_b) {
                soc_halt(0xBAD73000u | i);
            }
        }
    }

    barrier_wait(&g_bar, 2u, &local_sense);

    /* ========================================================================
     * TEST 3: Out-of-Range PMA DECERR Bus-Error Capture & Clean Recovery
     * ======================================================================== */
    if (hartid == 0u) {
        for (uint32_t i = 0u; i < 16u; i++) {
            g_rec_src[i] = 0x70000000u + i;
            g_rec_dst[i] = 0u;
        }
        /* Trigger OOB read @ 0x0004_0000 (past 256 KB RAM) -> DECERR IRQ on Core 1 */
        hal_dma_start(0u, 0x00040000u, (uint32_t)(uintptr_t)g_rec_dst, 64u, 1u);
    } else {
        while (g_ch0_irq_cnt < 3u) {}
        if ((g_ch0_last_stat & DMA_STAT_ERR) == 0u ||
            ((g_ch0_last_stat & DMA_STAT_ERR_CODE_MASK) >> DMA_STAT_ERR_CODE_SHIFT) != DMA_ERR_DECERR) {
            soc_halt(0xBAD70301u);
        }
        /* Prove clean recovery: launch valid 64B copy on Ch0 */
        hal_dma_memcpy_async(0u, g_rec_dst, g_rec_src, 64u, 1u);
        while (g_ch0_irq_cnt < 4u) {}
        __asm__ volatile ("fence i, r" ::: "memory");
        if ((g_ch0_last_stat & DMA_STAT_DONE) == 0u || (g_ch0_last_stat & DMA_STAT_ERR) != 0u) {
            soc_halt(0xBAD70302u);
        }
        for (uint32_t i = 0u; i < 16u; i++) {
            if (g_rec_dst[i] != (0x70000000u + i)) {
                soc_halt(0xBAD70310u | i);
            }
        }
    }

    barrier_wait(&g_bar, 2u, &local_sense);

    /* ========================================================================
     * TEST 4: Adversarial Speculative Load vs. DMA Write Snoop-Replay Proof!
     *   Core 1 caches g_rpl_dst in L1D, then speculatively executes BOTH:
     *     ld_a = g_rpl_dst[0] (word 0)
     *     <slow FDIV.S stalls ROB head so younger loads sit in LQ!>
     *     ld_b = g_rpl_dst[0] (word 0, SAME line!)
     *   while Core 0 launches DMA Ch0 overwriting g_rpl_dst[0..3] with fresh
     *   monotonically increasing tags! Under RVWMO CoRR, if ld_a saw tag K
     *   and DMA invalidated g_rpl_dst, ld_b MUST NEVER see < K; even stronger:
     *   by placing a slow FDIV.S ahead of ld_a = g_rpl_dst[0] and then loading
     *   the updated line, lsq2w's snoop_inv_valid triggers hardware LQ replay!
     * ======================================================================== */
    if (hartid == 0u) {
        for (uint32_t i = 0u; i < 4u; i++) {
            g_rpl_dst[i] = 0x00000000u;
            g_rpl_src[i] = 0x00000000u;
        }
    }
    barrier_wait(&g_bar, 2u, &local_sense);

    uint32_t rpl_obs_sum = 0u;
    for (uint32_t iter = 1u; iter <= 24u; iter++) {
        if (hartid == 1u) {
            /* Pre-warm g_rpl_dst cleanly into Core 1 L1D so DMA_PUTM hits & invalidates it! */
            uint32_t warm = g_rpl_dst[0];
            rpl_obs_sum ^= warm;
        }
        barrier_wait(&g_bar, 2u, &local_sense);

        if (hartid == 0u) {
            uint32_t tag = 0xD0000000u | iter;
            g_rpl_src[0] = tag;
            g_rpl_src[1] = tag ^ 0x55AA0000u;
            g_rpl_src[2] = tag ^ 0xAA550000u;
            g_rpl_src[3] = tag;
            /* Disable IRQ for tight synchronous DMA hammer (polling) */
            hal_dma_start(0u, (uint32_t)(uintptr_t)g_rpl_src,
                              (uint32_t)(uintptr_t)g_rpl_dst, 16u, 0u);
            g_rpl_go = iter;
            hal_dma_wait(0u);
        } else {
            /* Spin until Core 0 kicks off DMA, then execute speculative loads behind slow FDIV.S! */
            while (g_rpl_go < iter) {}
            volatile float fa = 123.456f + (float)iter;
            volatile float fb = 7.125f;
            uint32_t prev_val = 0u;
            for (uint32_t probe = 0u; probe < 12u; probe++) {
                uint32_t v0, v1, v2;
                float fd = fa / fb;
                float fd2 = fd / fb;
                /* Speculative loads issued OoO while FDIV.S holds ROB head -> LQ snoop replay! */
                __asm__ volatile (
                    "lw %0, 0(%3)\n\t"
                    "lw %1, 12(%3)\n\t"
                    "lw %2, 0(%3)\n\t"
                    : "=&r"(v0), "=&r"(v1), "=&r"(v2)
                    : "r"(g_rpl_dst), "f"(fd2)
                    : "memory"
                );
                /* Check 1: Single 128b cache line atomicity & CoRR monotonicity! */
                if ((v0 & 0xFFFFu) > (v2 & 0xFFFFu)) {
                    soc_halt(0xBAD70401u | (iter << 8));
                }
                if ((v0 & 0xFFFFu) < (prev_val & 0xFFFFu)) {
                    soc_halt(0xBAD70402u | (iter << 8));
                }
                if (v0 != v1) {
                    /* Same 16B line written atomically by DMA_PUTM -> word0 == word3! */
                    soc_halt(0xBAD70403u | (iter << 8));
                }
                prev_val = v2;
            }
            /* Final check after DMA completes */
            while (hal_dma_ch_is_busy(0u)) {}
            __asm__ volatile ("fence i, r" ::: "memory");
            uint32_t exp_tag = 0xD0000000u | iter;
            if (g_rpl_dst[0] != exp_tag || g_rpl_dst[3] != exp_tag) {
                soc_halt(0xBAD70404u | (iter << 8));
            }
            rpl_obs_sum += g_rpl_dst[0];
        }
        barrier_wait(&g_bar, 2u, &local_sense);
    }
    (void)rpl_obs_sum;

    if (hartid == 0u) {
        hal_uart_printf("SOC_U7_DMA_DONE: ch0_irqs=%u ch1_irqs=%u decerr=OK 4kb_fp=OK unalign=OK 2d=OK rpl=24\n",
                        g_ch0_irq_cnt, g_ch1_irq_cnt);
        hal_uart_tx_flush();
    }

    barrier_wait(&g_bar, 2u, &local_sense);
    hal_irq_global_disable();

    uint32_t checksum = 0x00070000u | (g_ch0_irq_cnt << 4) | g_ch1_irq_cnt;
    if (hartid == 1u) {
        soc_park_secondary(checksum | 0x00100000u, &g_c1_done);
    } else {
        while (!g_c1_done) {}
        soc_halt(checksum);
    }
    return checksum;
}
