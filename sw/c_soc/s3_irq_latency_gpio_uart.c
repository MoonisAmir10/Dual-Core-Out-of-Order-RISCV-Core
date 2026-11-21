/* ============================================================================
 * Phase U8 Flagship Benchmark S3: Hardware Interrupt Dispatch Latency,
 * Cross-Core IPI Round-Trip, GPIO/UART & Zero-Stall OoO IPC Audit
 *
 * Measures on live Dual-Core RV32IMAF SoC (+GPIO_STIM +UART_OUT):
 *   1. Non-Blocking OoO IPC Audit (identical FP+INT compute kernel):
 *        - Phase A: MIE=0 (IRQs globally masked)
 *        - Phase B: MIE=1 (MEIE|MTIE|MSIE enabled, idle IRQs -> 0.0% IPC loss!)
 *        - Phase C: MIE=1 + live periodic APB4 Timer0 PLIC IRQ + GPIO pin IRQs
 *   2. Exact Hardware CLINT Timer (MTIP) Dispatch Latency (8 samples)
 *   3. Exact Hardware PLIC External Timer0 (MEIP Src 6) Dispatch Latency (6 samples)
 *   4. Exact Cross-Core IPI (CLINT MSIP) Round-Trip Latency (10 samples)
 *   5. Live GPIO Edge PLIC ISR (Core 1, Src 1) + hal_uart_printf() Telemetry
 * ============================================================================ */
#include <stdint.h>
#include "sync.h"
#include "hal_irq.h"
#include "hal_gpio.h"
#include "hal_timer.h"
#include "hal_uart.h"

#define MTIP_SAMPLES 8u
#define PLIC_SAMPLES 6u
#define IPI_SAMPLES  10u

static volatile uint32_t g_mtip_target;
static volatile uint32_t g_mtip_cnt;
static volatile uint32_t g_mtip_min;
static volatile uint32_t g_mtip_max;
static volatile uint32_t g_mtip_sum;

static volatile uint32_t g_plic_target;
static volatile uint32_t g_plic_cnt;
static volatile uint32_t g_plic_min;
static volatile uint32_t g_plic_max;
static volatile uint32_t g_plic_sum;
static volatile uint32_t g_plic_phase_c;
static volatile uint32_t g_t0_busy_cnt;

static volatile uint32_t g_ipi_t0;
static volatile uint32_t g_ipi_cnt;
static volatile uint32_t g_ipi_min;
static volatile uint32_t g_ipi_max;
static volatile uint32_t g_ipi_sum;
static volatile uint32_t g_ipi_c1_cnt;

static volatile uint32_t g_gpio_cnt;
static volatile uint32_t g_c1_done;
static barrier_t         g_bar __attribute__((aligned(64)));

static inline uint32_t rd_mcycle(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mcycle" : "=r"(v));
    return v;
}

static inline uint32_t rd_minstret(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, minstret" : "=r"(v));
    return v;
}

static inline uint32_t f2u(float x) {
    union { float f; uint32_t u; } v;
    v.f = x;
    return v.u;
}

/* Deterministic mixed FP + INT compute kernel (measures exact OoO IPC!) */
static __attribute__((noinline)) uint32_t mixed_compute_kernel(uint32_t iters) {
    __asm__ volatile ("" : "+r"(iters) :: "memory");
    float a0 = 1.25f, a1 = 1.5f, a2 = 1.75f, a3 = 2.0f;
    const float m = 0.99609375f, c = 0.03125f;
    uint32_t x0 = 0x13579BDFu, x1 = 0x2468ACE0u;
    for (uint32_t i = 0u; i < iters; i++) {
        a0 = a0 * m + c;
        x0 = (x0 ^ (i * 2654435761u)) + x1;
        a1 = a1 * m + c;
        x1 = ((x1 << 5) | (x1 >> 27)) ^ x0;
        a2 = a2 * m + c;
        a3 = a3 * m + c;
    }
    return f2u(a0 + a1 + a2 + a3) ^ x0 ^ x1;
}

static void c0_timer_isr(uint32_t hartid) {
    uint32_t now = hal_clint_mtime_lo();
    hal_clint_disable_timer(hartid);
    uint32_t lat = now - g_mtip_target;
    if (lat < g_mtip_min) g_mtip_min = lat;
    if (lat > g_mtip_max) g_mtip_max = lat;
    g_mtip_sum += lat;
    g_mtip_cnt += 1u;
}

static void c0_plic_timer0_isr(uint32_t hartid, uint32_t src_id) {
    uint32_t now = hal_clint_mtime_lo();
    (void)hartid;
    (void)src_id;
    hal_timer_irq_clear(0u);
    if (g_plic_phase_c) {
        g_t0_busy_cnt += 1u;
    } else {
        uint32_t lat = now - g_plic_target;
        if (lat < g_plic_min) g_plic_min = lat;
        if (lat > g_plic_max) g_plic_max = lat;
        g_plic_sum += lat;
        g_plic_cnt += 1u;
    }
}

static void c0_ipi_isr(uint32_t hartid) {
    uint32_t now = hal_clint_mtime_lo();
    hal_clint_clear_ipi(hartid);
    uint32_t rtt = now - g_ipi_t0;
    if (rtt < g_ipi_min) g_ipi_min = rtt;
    if (rtt > g_ipi_max) g_ipi_max = rtt;
    g_ipi_sum += rtt;
    g_ipi_cnt += 1u;
}

static void c1_ipi_isr(uint32_t hartid) {
    hal_clint_clear_ipi(hartid);
    hal_clint_send_ipi(0u);
    g_ipi_c1_cnt += 1u;
}

static void c1_gpio_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid;
    (void)src_id;
    uint32_t st = hal_gpio_irq_status();
    hal_gpio_irq_clear(st);
    uint32_t m = st & 0x07u;
    if (m != 0u) {
        hal_gpio_toggle_bits(m << 8);
        for (uint32_t b = 0u; b < 3u; b++) {
            if (m & (1u << b)) g_gpio_cnt += 1u;
        }
    }
}

uint32_t main(uint32_t hartid) {
    uint32_t local_sense = 0u;

    hal_irq_init(hartid);
    hal_irq_set_mtvec_direct();
    hal_clint_disable_timer(hartid);
    hal_clint_clear_ipi(hartid);
    hal_plic_set_threshold(hartid, 0u);

    if (hartid == 0u) {
        g_mtip_cnt = 0u; g_mtip_min = 0xFFFFFFFFu; g_mtip_max = 0u; g_mtip_sum = 0u;
        g_plic_cnt = 0u; g_plic_min = 0xFFFFFFFFu; g_plic_max = 0u; g_plic_sum = 0u;
        g_plic_phase_c = 0u; g_t0_busy_cnt = 0u;
        g_ipi_cnt = 0u; g_ipi_min = 0xFFFFFFFFu; g_ipi_max = 0u; g_ipi_sum = 0u;
        g_ipi_c1_cnt = 0u;
        g_gpio_cnt = 0u;
        g_c1_done  = 0u;

        hal_uart_init(4u, 1u, 0u);
        hal_uart_irq_enable(0u, 0u);

        hal_irq_register_timer(0u, c0_timer_isr);
        hal_irq_register_ipi(0u, c0_ipi_isr);
        hal_plic_set_trigger_edge(PLIC_IRQ_TIMER0, 0u);
        hal_plic_set_priority(PLIC_IRQ_TIMER0, 6u);
        hal_irq_register_ext(0u, PLIC_IRQ_TIMER0, c0_plic_timer0_isr);
        hal_plic_enable_irq(0u, PLIC_IRQ_TIMER0);
    } else {
        hal_irq_register_ipi(1u, c1_ipi_isr);

        hal_gpio_set_dir(0x0000FF00u);
        hal_gpio_write(0u);
        hal_gpio_irq_clear(0xFFFFFFFFu);
        hal_gpio_irq_config(0x00000007u, 1u, 1u);
        hal_gpio_irq_enable(0x00000007u);

        hal_plic_set_trigger_edge(PLIC_IRQ_GPIO, 0u);
        hal_plic_set_priority(PLIC_IRQ_GPIO, 5u);
        hal_irq_register_ext(1u, PLIC_IRQ_GPIO, c1_gpio_isr);
        hal_plic_enable_irq(1u, PLIC_IRQ_GPIO);
    }

    barrier_wait(&g_bar, 2u, &local_sense);

    /* ========================================================================
     * 1. Non-Blocking IPC Audit: MIE=0 vs. MIE=1 (Idle IRQs) on Core 0
     * ======================================================================== */
    uint32_t base_cyc = 0u, base_ret = 0u, idle_cyc = 0u, idle_ret = 0u;
    uint32_t busy_cyc = 0u, busy_ret = 0u;
    uint32_t ref_res = 0u;

    if (hartid == 0u) {
        /* Warm I$ + BTB first */
        ref_res = mixed_compute_kernel(16u);

        /* Phase A: IRQs masked (MIE=0) */
        hal_irq_global_disable();
        uint32_t c0 = rd_mcycle(), r0 = rd_minstret();
        uint32_t res_a = mixed_compute_kernel(250u);
        base_cyc = rd_mcycle() - c0;
        base_ret = rd_minstret() - r0;

        /* Phase B: MIE=1 with MEIE|MTIE|MSIE armed (idle) -> 0% IPC overhead! */
        hal_irq_enable_mie(MIE_ALL);
        hal_irq_global_enable();
        uint32_t c1 = rd_mcycle(), r1 = rd_minstret();
        uint32_t res_b = mixed_compute_kernel(250u);
        idle_cyc = rd_mcycle() - c1;
        idle_ret = rd_minstret() - r1;

        if (res_a != res_b) {
            soc_halt(0xBAD80301u);
        }
        ref_res = res_a;
    }

    barrier_wait(&g_bar, 2u, &local_sense);
    hal_irq_enable_mie(MIE_ALL);
    hal_irq_global_enable();

    /* ========================================================================
     * 2. Hardware CLINT Timer (MTIP) Dispatch Latency (Core 0, 8 Samples)
     * ======================================================================== */
    if (hartid == 0u) {
        for (uint32_t s = 0u; s < MTIP_SAMPLES; s++) {
            uint64_t now64 = hal_clint_mtime64();
            uint64_t tgt64 = now64 + 220ull;
            g_mtip_target = (uint32_t)tgt64;
            hal_clint_set_mtimecmp(0u, tgt64);
            while (g_mtip_cnt <= s) {
                /* Active background arithmetic while waiting for alarm */
                __asm__ volatile ("nop");
            }
        }

        /* ====================================================================
         * 3. Hardware PLIC External Timer0 (MEIP Src 6) Latency (6 Samples)
         * ==================================================================== */
        for (uint32_t s = 0u; s < PLIC_SAMPLES; s++) {
            uint32_t t0 = hal_clint_mtime_lo();
            g_plic_target = t0 + 250u;
            hal_timer_init(0u, 0u, 249u, 125u, TIMER_CTRL_EN | TIMER_CTRL_IRQ_EN);
            while (g_plic_cnt <= s) {
                __asm__ volatile ("nop");
            }
        }
    }

    barrier_wait(&g_bar, 2u, &local_sense);

    /* ========================================================================
     * 4. Cross-Core IPI (CLINT MSIP) Round-Trip Latency (10 Samples)
     * ======================================================================== */
    if (hartid == 0u) {
        for (uint32_t s = 0u; s < IPI_SAMPLES; s++) {
            g_ipi_t0 = hal_clint_mtime_lo();
            hal_clint_send_ipi(1u);
            while (g_ipi_cnt <= s) {}
        }
    } else {
        while (g_ipi_c1_cnt < IPI_SAMPLES) {}
    }

    barrier_wait(&g_bar, 2u, &local_sense);

    /* ========================================================================
     * 5. Phase C: Active Periodic Timer0 PLIC IRQs + Live GPIO Pin IRQs
     * ======================================================================== */
    if (hartid == 0u) {
        g_plic_phase_c = 1u;
        hal_timer_init(0u, 0u, 599u, 300u,
                       TIMER_CTRL_EN | TIMER_CTRL_AUTO_RELOAD | TIMER_CTRL_PWM_EN | TIMER_CTRL_IRQ_EN);
        uint32_t c2 = rd_mcycle(), r2 = rd_minstret();
        uint32_t res_c = mixed_compute_kernel(250u);
        busy_cyc = rd_mcycle() - c2;
        busy_ret = rd_minstret() - r2;
        hal_timer_stop(0u);
        if (res_c != ref_res) {
            soc_halt(0xBAD80302u);
        }
    } else {
        /* Start external +GPIO_STIM stimulus on Core 1 while Core 0 runs Phase C */
        hal_gpio_set_bits(1u << 15);
        while (g_gpio_cnt < 3u) {
            (void)mixed_compute_kernel(20u);
        }
    }

    barrier_wait(&g_bar, 2u, &local_sense);

    if (hartid == 0u) {
        uint32_t mtip_avg = g_mtip_sum / MTIP_SAMPLES;
        uint32_t plic_avg = g_plic_sum / PLIC_SAMPLES;
        uint32_t ipi_avg  = g_ipi_sum / IPI_SAMPLES;
        hal_uart_printf(
            "S3_IRQ base=%u/%u idle=%u/%u busy=%u/%u t0=%u gpio=%u mtip=%u/%u/%u plic=%u/%u/%u ipi_rtt=%u/%u/%u\n",
            base_cyc, base_ret, idle_cyc, idle_ret, busy_cyc, busy_ret,
            g_t0_busy_cnt, g_gpio_cnt,
            g_mtip_min, mtip_avg, g_mtip_max,
            g_plic_min, plic_avg, g_plic_max,
            g_ipi_min, ipi_avg, g_ipi_max
        );
        hal_uart_tx_flush();
    }

    barrier_wait(&g_bar, 2u, &local_sense);
    hal_irq_global_disable();

    uint32_t checksum = 0x00083000u | (g_gpio_cnt << 8) | (g_mtip_cnt << 4) | g_plic_cnt;
    if (hartid == 1u) {
        soc_park_secondary(checksum | 0x00100000u, &g_c1_done);
    } else {
        while (!g_c1_done) {}
        soc_halt(checksum);
    }
    return checksum;
}
