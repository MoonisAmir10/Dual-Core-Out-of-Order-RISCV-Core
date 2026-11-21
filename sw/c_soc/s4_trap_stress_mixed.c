/* ============================================================================
 * Phase U8 Flagship Benchmark S4: Full-SoC Adversarial Mixed Trap + IRQ +
 * Coherent AXI4 DMA + RV32IMAF + RV32A Coherence Stress Workload
 *
 * Exercises EVERY Phase U1..U7 hardware subsystem SIMULTANEOUSLY across both
 * cores under +JITTER DRAM latency jitter + always-on 3-master coh_checker:
 *   - Illegal instruction software emulation (mcause=2, custom .word 0x0000000B)
 *   - Misaligned load (mcause=4) & misaligned store (mcause=6) trapping w/ 0 corruption
 *   - Unmapped APB4 load/store bus faults (mcause=5 & 7 @ 0x1008_0000)
 *   - ECALL syscalls (mcause=11) with register return values
 *   - Concurrent CLINT Timer IRQs (Core 0), PLIC Timer0 & GPIO IRQs (Core 1),
 *     AND Background 2-Channel Coherent AXI4 DMA transfers (PLIC Src 4 & Src 5)
 *   - Concurrent RV32IMAF hard-float FMA/FDIV compute & RV32A AMOADD.W atomics!
 * ============================================================================ */
#include <stdint.h>
#include "sync.h"
#include "hal_irq.h"
#include "hal_gpio.h"
#include "hal_timer.h"
#include "hal_dma.h"
#include "hal_uart.h"

#define ROUNDS 12u

static volatile uint32_t g_exc_ill[2];
static volatile uint32_t g_exc_ld_mis[2];
static volatile uint32_t g_exc_st_mis[2];
static volatile uint32_t g_exc_ld_flt[2];
static volatile uint32_t g_exc_st_flt[2];
static volatile uint32_t g_exc_ecall[2];

static volatile uint32_t g_clint_t_cnt;
static volatile uint32_t g_plic_t0_cnt;
static volatile uint32_t g_gpio_cnt;
static volatile uint32_t g_dma0_cnt;
static volatile uint32_t g_dma1_cnt;

static volatile uint32_t g_shared_amo __attribute__((aligned(64)));
static volatile uint32_t g_canary_mem[2][4] __attribute__((aligned(64)));
static float             g_dma_src0[64] __attribute__((aligned(64)));
static float             g_dma_dst0[64] __attribute__((aligned(64)));
static float             g_dma_src1[64] __attribute__((aligned(64)));
static float             g_dma_dst1[64] __attribute__((aligned(64)));

static volatile uint32_t g_c1_done;
static barrier_t         g_bar __attribute__((aligned(64)));

static inline uint32_t f2u(float x) {
    union { float f; uint32_t u; } v;
    v.f = x;
    return v.u;
}

/* Unified M-mode Synchronous Exception Handler (handles 6 fault classes!) */
static void mixed_exc_handler(uint32_t hartid, uint32_t cause, uint32_t mtval, trap_frame_t *tf) {
    switch (cause) {
        case EXC_ILLEGAL_INSTR:
            if (mtval != 0x0000000Bu) soc_halt(0xBAD84001u | (hartid << 8));
            tf->a0 += 7u; /* Emulate custom instruction: a0 += 7 */
            g_exc_ill[hartid] += 1u;
            break;
        case EXC_LOAD_MISALIGN:
            g_exc_ld_mis[hartid] += 1u;
            break;
        case EXC_STORE_MISALIGN:
            g_exc_st_mis[hartid] += 1u;
            break;
        case EXC_LOAD_FAULT:
            if (mtval != 0x10080000u) soc_halt(0xBAD84002u | (hartid << 8));
            g_exc_ld_flt[hartid] += 1u;
            break;
        case EXC_STORE_FAULT:
            if (mtval != 0x10080000u) soc_halt(0xBAD84003u | (hartid << 8));
            g_exc_st_flt[hartid] += 1u;
            break;
        case EXC_ECALL_M:
            tf->a0 = tf->a0 * 3u + 1u;
            g_exc_ecall[hartid] += 1u;
            break;
        default:
            soc_halt(0xBAD84F00u | (cause & 0xFFu));
            break;
    }
    tf->mepc += 4u;
}

static void c0_clint_timer_isr(uint32_t hartid) {
    g_clint_t_cnt += 1u;
    if (g_clint_t_cnt < 16u) {
        hal_clint_set_alarm_rel(hartid, 900u);
    } else {
        hal_clint_disable_timer(hartid);
    }
}

static void c0_dma1_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid; (void)src_id;
    uint32_t st = hal_dma_ch_status(1u);
    hal_dma_irq_clear(1u << 1);
    if ((st & DMA_STAT_ERR) != 0u) soc_halt(0xBAD84011u);
    g_dma1_cnt += 1u;
}

static void c1_dma0_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid; (void)src_id;
    uint32_t st = hal_dma_ch_status(0u);
    hal_dma_irq_clear(1u << 0);
    if ((st & DMA_STAT_ERR) != 0u) soc_halt(0xBAD84012u);
    g_dma0_cnt += 1u;
}

static void c1_timer0_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid; (void)src_id;
    hal_timer_irq_clear(0u);
    g_plic_t0_cnt += 1u;
    if (g_plic_t0_cnt >= 16u) hal_timer_stop(0u);
}

static void c1_gpio_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid; (void)src_id;
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

static uint32_t do_custom_illegal(uint32_t val) {
    register uint32_t a0_r __asm__("a0") = val;
    __asm__ volatile (".word 0x0000000B" : "+r"(a0_r) :: "memory");
    return a0_r;
}

static uint32_t do_ecall(uint32_t val) {
    register uint32_t a0_r __asm__("a0") = val;
    __asm__ volatile ("ecall" : "+r"(a0_r) :: "memory");
    return a0_r;
}

uint32_t main(uint32_t hartid) {
    uint32_t local_sense = 0u;

    hal_irq_init(hartid);
    hal_irq_set_mtvec_direct();
    hal_irq_register_exc(hartid, mixed_exc_handler);
    hal_clint_disable_timer(hartid);
    hal_clint_clear_ipi(hartid);
    hal_plic_set_threshold(hartid, 0u);

    g_exc_ill[hartid]    = 0u;
    g_exc_ld_mis[hartid] = 0u;
    g_exc_st_mis[hartid] = 0u;
    g_exc_ld_flt[hartid] = 0u;
    g_exc_st_flt[hartid] = 0u;
    g_exc_ecall[hartid]  = 0u;
    g_canary_mem[hartid][0] = 0xA5A55A5Au ^ hartid;
    g_canary_mem[hartid][1] = 0x12345678u ^ hartid;

    if (hartid == 0u) {
        g_clint_t_cnt = 0u; g_plic_t0_cnt = 0u; g_gpio_cnt = 0u;
        g_dma0_cnt = 0u; g_dma1_cnt = 0u; g_shared_amo = 0u; g_c1_done = 0u;

        hal_uart_init(4u, 1u, 0u);
        hal_uart_irq_enable(0u, 0u);
        hal_dma_irq_clear(0x3u);

        hal_irq_register_timer(0u, c0_clint_timer_isr);
        hal_plic_set_trigger_edge(PLIC_IRQ_DMA_CH1, 0u);
        hal_plic_set_priority(PLIC_IRQ_DMA_CH1, 6u);
        hal_irq_register_ext(0u, PLIC_IRQ_DMA_CH1, c0_dma1_isr);
        hal_plic_enable_irq(0u, PLIC_IRQ_DMA_CH1);
    } else {
        hal_gpio_set_dir(0x0000FF00u);
        hal_gpio_write(0u);
        hal_gpio_irq_clear(0xFFFFFFFFu);
        hal_gpio_irq_config(0x00000007u, 1u, 1u);
        hal_gpio_irq_enable(0x00000007u);

        hal_plic_set_trigger_edge(PLIC_IRQ_GPIO, 0u);
        hal_plic_set_priority(PLIC_IRQ_GPIO, 4u);
        hal_irq_register_ext(1u, PLIC_IRQ_GPIO, c1_gpio_isr);
        hal_plic_enable_irq(1u, PLIC_IRQ_GPIO);

        hal_plic_set_trigger_edge(PLIC_IRQ_TIMER0, 0u);
        hal_plic_set_priority(PLIC_IRQ_TIMER0, 5u);
        hal_irq_register_ext(1u, PLIC_IRQ_TIMER0, c1_timer0_isr);
        hal_plic_enable_irq(1u, PLIC_IRQ_TIMER0);

        hal_plic_set_trigger_edge(PLIC_IRQ_DMA_CH0, 0u);
        hal_plic_set_priority(PLIC_IRQ_DMA_CH0, 6u);
        hal_irq_register_ext(1u, PLIC_IRQ_DMA_CH0, c1_dma0_isr);
        hal_plic_enable_irq(1u, PLIC_IRQ_DMA_CH0);
    }

    barrier_wait(&g_bar, 2u, &local_sense);

    /* Enable live asynchronous CLINT + PLIC interrupts on both cores! */
    hal_irq_enable_mie(MIE_MEIE | MIE_MTIE);
    hal_irq_global_enable();

    if (hartid == 0u) {
        hal_clint_set_alarm_rel(0u, 800u);
    } else {
        hal_timer_init(0u, 0u, 999u, 500u,
                       TIMER_CTRL_EN | TIMER_CTRL_AUTO_RELOAD | TIMER_CTRL_PWM_EN | TIMER_CTRL_IRQ_EN);
        hal_gpio_set_bits(1u << 15); /* Kick off +GPIO_STIM edges */
    }

    /* Concurrent Mixed Trap + IRQ + Coherent DMA + RV32IMAF + RV32A Stress Loop */
    float fp_acc = 1.0f + (float)hartid * 0.5f;
    uint32_t mis_addr = (uint32_t)(uintptr_t)&g_canary_mem[hartid][0] + 2u;
    uint32_t bad_apb  = 0x10080000u;

    for (uint32_t r = 0u; r < ROUNDS; r++) {
        /* 1. Prepare dirty L1D float buffer & launch async coherent DMA (Ch0 on C0, Ch1 on C1) */
        if (hartid == 0u) {
            for (uint32_t i = 0u; i < 64u; i++) g_dma_src0[i] = (float)(r * 64u + i) * 0.25f + 1.0f;
            hal_dma_memcpy_async(0u, g_dma_dst0, g_dma_src0, 256u, 1u);
        } else {
            for (uint32_t i = 0u; i < 64u; i++) g_dma_src1[i] = (float)(r * 64u + i) * 0.5f + 2.0f;
            hal_dma_memcpy_async(1u, g_dma_dst1, g_dma_src1, 256u, 1u);
        }

        /* 2. Execute all 6 synchronous exception classes while DMA & IRQs are live! */
        uint32_t r_ill = do_custom_illegal(r + 10u);
        if (r_ill != r + 17u) soc_halt(0xBAD84101u | (hartid << 8));

        uint32_t r_ec = do_ecall(r + 2u);
        if (r_ec != (r + 2u) * 3u + 1u) soc_halt(0xBAD84102u | (hartid << 8));

        /* Misaligned LW & SW (must trap with 0 memory corruption!) */
        uint32_t sink = 0u;
        __asm__ volatile ("lw %0, 0(%1)" : "=r"(sink) : "r"(mis_addr) : "memory");
        __asm__ volatile ("sw %0, 0(%1)" :: "r"(0xDEADBEEFu), "r"(mis_addr) : "memory");

        /* Unmapped APB4 Load & Store Bus Faults (PSLVERR -> mcause 5 & 7!) */
        __asm__ volatile ("lw %0, 0(%1)" : "=r"(sink) : "r"(bad_apb) : "memory");
        __asm__ volatile ("sw %0, 0(%1)" :: "r"(0xCAFEBABEu), "r"(bad_apb) : "memory");
        (void)sink;

        /* Verify canary RAM untouched by misaligned store */
        if (g_canary_mem[hartid][0] != (0xA5A55A5Au ^ hartid) ||
            g_canary_mem[hartid][1] != (0x12345678u ^ hartid)) {
            soc_halt(0xBAD84103u | (hartid << 8));
        }

        /* 3. Mixed RV32IMAF hard-float + RV32A contended AMOADD.W */
        for (uint32_t k = 0u; k < 8u; k++) {
            fp_acc = fp_acc * 0.99609375f + 0.125f;
            (void)amo_add_w(&g_shared_amo, 1u);
        }
        fp_acc = fp_acc / 1.0078125f;

        /* 4. Wait for peer's DMA completion PLIC ISR & verify coherent float buffer! */
        if (hartid == 0u) {
            while (g_dma1_cnt <= r) {}
            __asm__ volatile ("fence i, r" ::: "memory");
            for (uint32_t i = 0u; i < 64u; i++) {
                float exp_v = (float)(r * 64u + i) * 0.5f + 2.0f;
                if (f2u(g_dma_dst1[i]) != f2u(exp_v)) soc_halt(0xBAD84200u | i);
            }
        } else {
            while (g_dma0_cnt <= r) {}
            __asm__ volatile ("fence i, r" ::: "memory");
            for (uint32_t i = 0u; i < 64u; i++) {
                float exp_v = (float)(r * 64u + i) * 0.25f + 1.0f;
                if (f2u(g_dma_dst0[i]) != f2u(exp_v)) soc_halt(0xBAD84300u | i);
            }
        }

        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* Wait for all 3 GPIO stimulus edges on Core 1 before stopping */
    if (hartid == 1u) {
        while (g_gpio_cnt < 3u) {}
        hal_timer_stop(0u);
    } else {
        hal_clint_disable_timer(0u);
    }
    barrier_wait(&g_bar, 2u, &local_sense);

    /* Verify exact trap counts on BOTH cores and exact atomic total! */
    if (g_exc_ill[hartid] != ROUNDS || g_exc_ld_mis[hartid] != ROUNDS ||
        g_exc_st_mis[hartid] != ROUNDS || g_exc_ld_flt[hartid] != ROUNDS ||
        g_exc_st_flt[hartid] != ROUNDS || g_exc_ecall[hartid] != ROUNDS) {
        soc_halt(0xBAD84400u | hartid);
    }
    if (g_shared_amo != (2u * ROUNDS * 8u)) {
        soc_halt(0xBAD84410u);
    }

    /* Verify deterministic FP result (identical regardless of arbitrary IRQ preemption!) */
    float exp_fp = 1.0f + (float)hartid * 0.5f;
    for (uint32_t r = 0u; r < ROUNDS; r++) {
        for (uint32_t k = 0u; k < 8u; k++) exp_fp = exp_fp * 0.99609375f + 0.125f;
        exp_fp = exp_fp / 1.0078125f;
    }
    if (f2u(fp_acc) != f2u(exp_fp)) {
        soc_halt(0xBAD84420u | hartid);
    }

    if (hartid == 0u) {
        uint32_t tot_traps = (g_exc_ill[0] + g_exc_ld_mis[0] + g_exc_st_mis[0] +
                              g_exc_ld_flt[0] + g_exc_st_flt[0] + g_exc_ecall[0]) * 2u;
        hal_uart_printf("S4_MIXED traps=%u clint_t=%u plic_t0=%u gpio=%u dma=%u/%u amo=%u fp=OK\n",
                        tot_traps, g_clint_t_cnt, g_plic_t0_cnt, g_gpio_cnt,
                        g_dma0_cnt, g_dma1_cnt, g_shared_amo);
        hal_uart_tx_flush();
    }

    barrier_wait(&g_bar, 2u, &local_sense);
    hal_irq_global_disable();

    uint32_t checksum = 0x00084000u | (g_dma0_cnt << 4) | g_dma1_cnt;
    if (hartid == 1u) {
        soc_park_secondary(checksum | 0x00100000u, &g_c1_done);
    } else {
        while (!g_c1_done) {}
        soc_halt(checksum);
    }
    return checksum;
}
