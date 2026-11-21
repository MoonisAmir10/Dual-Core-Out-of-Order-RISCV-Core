/* ============================================================================
 * Phase U3 Live Bare-Metal C ISR Suite (Direct mtvec Mode — 1C & 2C!)
 *
 * Exercises ALL 3 RISC-V M-mode hardware interrupt classes simultaneously with
 * real out-of-order foreground compute:
 *   1. Machine Timer Interrupt (MTIP, mcause=0x80000007):
 *        - Core 0: periodic 600-cycle timer alarm (5 ticks)
 *        - Core 1: independent 450-cycle timer alarm (5 ticks)
 *        - Foreground loop computes deterministic integer dot-product checksum
 *          to prove async trap preemption NEVER corrupts registers or memory!
 *   2. Machine Software Interrupt / Cross-Core IPI (MSIP, mcause=0x80000003):
 *        - 2C mode: 4x Core 0 -> Core 1 -> Core 0 IPI ping-pong round-trips!
 *        - 1C mode: 4x self-IPI triggers!
 *   3. Machine External Interrupt via AMBA APB4 PLIC (MEIP, mcause=0x8000000B):
 *        - Priority & Threshold Proof: injects Src 2 (prio=3) + Src 5 (prio=6)
 *          simultaneously with thresh=4 -> Src 5 MUST preempt & service first
 *          while Src 2 stays masked; lowering thresh=0 immediately services Src 2!
 *        - Dual-Core Atomic Claim Race Proof (2C): Shared edge-triggered Src 9
 *          enabled on BOTH Core 0 & Core 1, pulsed 6 times -> proves EXACTLY 6
 *          total claims across {Core 0 + Core 1} (0 lost, 0 double-claimed)!
 * ============================================================================ */
#include <stdint.h>
#include "sync.h"
#include "hal_irq.h"

#define TARGET_TIMER_TICKS  5u
#define TARGET_IPI_ROUNDS   4u
#define TARGET_SHARED_PULSES 6u

static volatile uint32_t g_timer_ticks[2];
static volatile uint32_t g_ipi_cnt[2];
static volatile uint32_t g_prio_order[4];
static volatile uint32_t g_prio_idx;
static volatile uint32_t g_shared_claims;
static volatile uint32_t g_shared_per_core[2];
static volatile uint32_t g_c1_done;
static barrier_t         g_bar;

static inline void soc_irq_inject_set(uint32_t mask) {
    uint32_t addr = SOC_CTRL_BASE + SOC_CTRL_IRQ_INJECT_OFF;
    mmio_write32(addr, mmio_read32(addr) | mask);
}

static inline void soc_irq_inject_clr(uint32_t mask) {
    uint32_t addr = SOC_CTRL_BASE + SOC_CTRL_IRQ_INJECT_OFF;
    mmio_write32(addr, mmio_read32(addr) & ~mask);
}

static void timer_isr(uint32_t hartid) {
    uint32_t t = g_timer_ticks[hartid] + 1u;
    g_timer_ticks[hartid] = t;
    if (t < TARGET_TIMER_TICKS) {
        uint32_t period = (hartid == 0u) ? 600u : 450u;
        hal_clint_set_alarm_rel(hartid, period);
    } else {
        hal_clint_disable_timer(hartid);
    }
}

static void ipi_isr(uint32_t hartid) {
    hal_clint_clear_ipi(hartid);
    g_ipi_cnt[hartid] += 1u;
    /* In 2C mode, Core 1 replies to Core 0 immediately from inside its C ISR! */
    if (hartid == 1u) {
        hal_clint_send_ipi(0u);
    }
}

static void plic_prio_isr(uint32_t hartid, uint32_t src_id) {
    (void)hartid;
    uint32_t idx = g_prio_idx;
    if (idx < 4u) {
        g_prio_order[idx] = src_id;
        g_prio_idx = idx + 1u;
    }
    /* Clear level-sensitive injected source before hal_irq calls hal_plic_complete! */
    soc_irq_inject_clr(1u << src_id);
}

static void plic_shared_isr(uint32_t hartid, uint32_t src_id) {
    (void)src_id;
    amo_add_w(&g_shared_claims, 1u);
    g_shared_per_core[hartid] += 1u;
}

/* Deterministic foreground ALU/MUL compute (stressed under live async preemption!) */
static uint32_t compute_workload(uint32_t seed, uint32_t iters) {
    uint32_t acc = 0x13572468u ^ seed;
    for (uint32_t i = 1u; i <= iters; i++) {
        uint32_t a = (i * 2654435761u) ^ acc;
        acc = ((acc << 5) | (acc >> 27)) + (a ^ (i * 17u));
    }
    return acc;
}

uint32_t main(uint32_t hartid) {
    uint32_t ncores = mmio_read32(SOC_CTRL_BASE + SOC_CTRL_CORES_OFF);
    uint32_t local_sense = 0u;

    /* Sanity check FEATURE_MASK includes Traps + APB4 + PLIC */
    uint32_t feat = mmio_read32(SOC_CTRL_BASE + SOC_CTRL_FEATURES_OFF);
    if ((feat & (SOC_FEAT_TRAPS | SOC_FEAT_APB4 | SOC_FEAT_PLIC)) !=
        (SOC_FEAT_TRAPS | SOC_FEAT_APB4 | SOC_FEAT_PLIC)) {
        soc_halt(0xBAD30001u);
    }

    /* 1. Initialize per-hart HAL & install Direct-mode trap trampoline */
    hal_irq_init(hartid);
    g_timer_ticks[hartid]     = 0u;
    g_ipi_cnt[hartid]         = 0u;
    g_shared_per_core[hartid] = 0u;
    if (hartid == 0u) {
        g_prio_idx      = 0u;
        g_shared_claims = 0u;
        g_c1_done       = 0u;
        mmio_write32(SOC_CTRL_BASE + SOC_CTRL_IRQ_INJECT_OFF, 0u);
    }

    hal_irq_register_timer(hartid, timer_isr);
    hal_irq_register_ipi(hartid, ipi_isr);
    hal_irq_register_ext(hartid, 2u, plic_prio_isr);
    hal_irq_register_ext(hartid, 5u, plic_prio_isr);
    hal_irq_register_ext(hartid, 9u, plic_shared_isr);
    hal_irq_set_mtvec_direct();

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* 2. Periodic Timer Alarm (MTIP) concurrent with foreground compute! */
    hal_irq_enable_mie(MIE_MTIE | MIE_MSIE | MIE_MEIE);
    hal_clint_set_alarm_rel(hartid, (hartid == 0u) ? 400u : 300u);
    hal_irq_global_enable();

    uint32_t expected_work = compute_workload(hartid + 1u, 120u);
    uint32_t actual_work   = 0u;
    /* Recompute while timer alarms keep firing in the background! */
    while (g_timer_ticks[hartid] < TARGET_TIMER_TICKS) {
        actual_work = compute_workload(hartid + 1u, 120u);
    }
    if (actual_work != expected_work) {
        soc_halt(0xBAD30002u | (hartid << 8));
    }

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* 3. Cross-Core IPI Ping-Pong (2C) or Self-IPI (1C) via CLINT MSIP */
    if (ncores == 2u) {
        if (hartid == 0u) {
            for (uint32_t r = 0u; r < TARGET_IPI_ROUNDS; r++) {
                uint32_t before = g_ipi_cnt[0];
                hal_clint_send_ipi(1u);
                while (g_ipi_cnt[0] == before) {
                    /* Wait for Core 1's IPI ISR reply! */
                }
            }
        }
        barrier_wait(&g_bar, 2u, &local_sense);
        if (g_ipi_cnt[hartid] != TARGET_IPI_ROUNDS) {
            soc_halt(0xBAD30003u | (hartid << 8));
        }
    } else {
        for (uint32_t r = 0u; r < TARGET_IPI_ROUNDS; r++) {
            uint32_t before = g_ipi_cnt[0];
            hal_clint_send_ipi(0u);
            while (g_ipi_cnt[0] == before) {}
        }
        if (g_ipi_cnt[0] != TARGET_IPI_ROUNDS) {
            soc_halt(0xBAD30003u);
        }
    }

    /* 4. PLIC Priority Ordering & Threshold Masking Proof (Target = Core 0 or Core 1) */
    uint32_t prio_target = (ncores == 2u) ? 1u : 0u;
    if (hartid == prio_target) {
        hal_irq_global_disable();
        hal_plic_set_trigger_edge(2u, 0u); /* level */
        hal_plic_set_trigger_edge(5u, 0u); /* level */
        hal_plic_set_priority(2u, 3u);     /* low prio = 3 */
        hal_plic_set_priority(5u, 6u);     /* high prio = 6 */
        hal_plic_set_threshold(hartid, 4u);/* masks prio<=4 (Src 2), allows prio=6 (Src 5)! */
        hal_plic_enable_irq(hartid, 2u);
        hal_plic_enable_irq(hartid, 5u);

        /* Inject BOTH sources simultaneously while MIE=0 */
        soc_irq_inject_set((1u << 2) | (1u << 5));
        hal_irq_global_enable();

        /* Wait for high-priority Src 5 to fire */
        while (g_prio_idx < 1u) {}

        /* Spin 40 cycles to verify Src 2 remains masked while thresh==4 */
        for (volatile uint32_t d = 0u; d < 20u; d++) {}
        if (g_prio_idx != 1u || g_prio_order[0] != 5u) {
            soc_halt(0xBAD30004u);
        }

        /* Lower threshold to 0 -> pending Src 2 MUST immediately fire! */
        hal_plic_set_threshold(hartid, 0u);
        while (g_prio_idx < 2u) {}
        if (g_prio_idx != 2u || g_prio_order[1] != 2u) {
            soc_halt(0xBAD30005u);
        }

        hal_plic_disable_irq(hartid, 2u);
        hal_plic_disable_irq(hartid, 5u);
    }

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* 5. Shared PLIC Atomic Dual-Core Claim Race (Edge-Triggered Source 9) */
    if (hartid == 0u) {
        hal_plic_set_trigger_edge(9u, 1u);
        hal_plic_set_priority(9u, 5u);
        hal_plic_set_threshold(0u, 0u);
        hal_plic_enable_irq(0u, 9u);
        if (ncores == 2u) {
            hal_plic_set_threshold(1u, 0u);
            hal_plic_enable_irq(1u, 9u);
        }
    }
    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    if (hartid == 0u) {
        for (uint32_t p = 0u; p < TARGET_SHARED_PULSES; p++) {
            /* Generate one clean 0->1->0 edge pulse on PLIC Source 9! */
            soc_irq_inject_set(1u << 9);
            soc_irq_inject_clr(1u << 9);
            while (g_shared_claims < (p + 1u)) {
                /* Wait until either Core 0 or Core 1 atomically claims & handles it! */
            }
        }
        /* Brief settle to ensure zero duplicate late claims */
        for (volatile uint32_t d = 0u; d < 30u; d++) {}
    }

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    if (g_shared_claims != TARGET_SHARED_PULSES) {
        soc_halt(0xBAD30006u);
    }

    /* Cleanly disable interrupts & return deterministic checksum */
    hal_irq_global_disable();
    uint32_t checksum = (ncores << 16) |
                        (g_timer_ticks[hartid] << 12) |
                        (g_ipi_cnt[hartid] << 8) |
                        (g_prio_idx << 4) |
                        g_shared_claims;

    if (ncores == 2u) {
        if (hartid == 1u) {
            /* Expose Core 1 x14 and signal Core 0 before halting */
            soc_park_secondary(checksum | 0x00100000u, &g_c1_done);
        } else {
            while (!g_c1_done) {}
            soc_halt(checksum);
        }
    } else {
        soc_halt(checksum);
    }
    return checksum;
}
