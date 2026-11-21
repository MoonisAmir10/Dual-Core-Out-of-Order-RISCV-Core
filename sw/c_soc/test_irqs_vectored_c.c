/* ============================================================================
 * Phase U3 Live Bare-Metal Vectored mtvec (MODE=01) Proof (1C & 2C)
 *
 * Installs _trap_vector_table | 1 into mtvec and overrides the 4 distinct
 * vectored C hooks in sw/common/trap.S:
 *   - Slot  0 (+0x00): soc_vec_exc_handler  (triggered via ECALL -> mcause=11)
 *   - Slot  3 (+0x0C): soc_vec_msi_handler  (triggered via CLINT MSIP)
 *   - Slot  7 (+0x1C): soc_vec_mti_handler  (triggered via CLINT MTIMECMP)
 *   - Slot 11 (+0x2C): soc_vec_mei_handler  (triggered via PLIC Src 3/4)
 *
 * Proves hardware jumped directly to BASE + 4*cause for each async IRQ and
 * BASE + 0 for synchronous exceptions on BOTH single_ooo2w_soc & dual_ooo2w_soc!
 * ============================================================================ */
#include <stdint.h>
#include "sync.h"
#include "hal_irq.h"

static volatile uint32_t g_vec_exc[2];
static volatile uint32_t g_vec_msi[2];
static volatile uint32_t g_vec_mti[2];
static volatile uint32_t g_vec_mei[2];
static volatile uint32_t g_vec_err[2];
static volatile uint32_t g_c1_done;
static barrier_t         g_bar;

void soc_vec_exc_handler(uint32_t mcause, uint32_t mepc, uint32_t mtval, trap_frame_t *tf) {
    uint32_t h = hal_csr_mhartid();
    (void)mepc; (void)mtval;
    if (mcause == EXC_ECALL_M) {
        g_vec_exc[h] += 1u;
        tf->a0 = 0x55AA0000u | h; /* Return syscall value in a0! */
    } else {
        g_vec_err[h] += 1u;
    }
    tf->mepc += 4u;
}

void soc_vec_msi_handler(uint32_t mcause, uint32_t mepc, uint32_t mtval, trap_frame_t *tf) {
    uint32_t h = hal_csr_mhartid();
    (void)mepc; (void)mtval; (void)tf;
    if (mcause == (MCAUSE_INT_BIT | IRQ_CAUSE_MSI)) {
        g_vec_msi[h] += 1u;
    } else {
        g_vec_err[h] += 1u;
    }
    hal_clint_clear_ipi(h);
}

void soc_vec_mti_handler(uint32_t mcause, uint32_t mepc, uint32_t mtval, trap_frame_t *tf) {
    uint32_t h = hal_csr_mhartid();
    (void)mepc; (void)mtval; (void)tf;
    if (mcause == (MCAUSE_INT_BIT | IRQ_CAUSE_MTI)) {
        g_vec_mti[h] += 1u;
    } else {
        g_vec_err[h] += 1u;
    }
    hal_clint_disable_timer(h);
}

void soc_vec_mei_handler(uint32_t mcause, uint32_t mepc, uint32_t mtval, trap_frame_t *tf) {
    uint32_t h = hal_csr_mhartid();
    (void)mepc; (void)mtval; (void)tf;
    if (mcause == (MCAUSE_INT_BIT | IRQ_CAUSE_MEI)) {
        uint32_t src = hal_plic_claim(h);
        if (src == (3u + h)) {
            g_vec_mei[h] += 1u;
        } else {
            g_vec_err[h] += 1u;
        }
        if (src != 0u) {
            hal_plic_complete(h, src);
        }
    } else {
        g_vec_err[h] += 1u;
    }
}

uint32_t main(uint32_t hartid) {
    uint32_t ncores = mmio_read32(SOC_CTRL_BASE + SOC_CTRL_CORES_OFF);
    uint32_t local_sense = 0u;
    uint32_t my_src = 3u + hartid; /* Core 0 -> PLIC Src 3, Core 1 -> PLIC Src 4 */

    hal_irq_init(hartid);
    g_vec_exc[hartid] = 0u;
    g_vec_msi[hartid] = 0u;
    g_vec_mti[hartid] = 0u;
    g_vec_mei[hartid] = 0u;
    g_vec_err[hartid] = 0u;
    if (hartid == 0u) {
        g_c1_done = 0u;
        mmio_write32(SOC_CTRL_BASE + SOC_CTRL_IRQ_INJECT_OFF, 0u);
    }

    /* Install VECTORED mtvec (MODE = 01)! */
    hal_irq_set_mtvec_vectored();
    hal_irq_enable_mie(MIE_ALL);

    /* Configure per-hart edge-triggered PLIC source */
    if (hartid == 0u) {
        hal_plic_set_trigger_edge(3u, 1u);
        hal_plic_set_priority(3u, 4u);
        if (ncores == 2u) {
            hal_plic_set_trigger_edge(4u, 1u);
            hal_plic_set_priority(4u, 4u);
        }
    }
    if (ncores == 2u) barrier_wait(&g_bar, 2u, &local_sense);

    hal_plic_set_threshold(hartid, 0u);
    hal_plic_enable_irq(hartid, my_src);
    hal_irq_global_enable();

    /* 1. Trigger Synchronous Exception (ECALL -> Slot 0 +0x00) */
    register uint32_t ret_a0 __asm__("a0") = 0u;
    __asm__ volatile ("ecall" : "+r"(ret_a0) :: "memory");
    if (ret_a0 != (0x55AA0000u | hartid)) soc_halt(0xBAD40001u);

    /* 2. Trigger Machine Software Interrupt (MSIP -> Slot 3 +0x0C) */
    hal_clint_send_ipi(hartid);
    while (g_vec_msi[hartid] < 1u) {}

    /* 3. Trigger Machine Timer Interrupt (MTIP -> Slot 7 +0x1C) */
    hal_clint_set_alarm_rel(hartid, 150u);
    while (g_vec_mti[hartid] < 1u) {}

    /* 4. Trigger Machine External Interrupt (PLIC MEIP -> Slot 11 +0x2C) */
    if (ncores == 2u) barrier_wait(&g_bar, 2u, &local_sense);
    if (hartid == 0u) {
        uint32_t mask = (ncores == 2u) ? ((1u << 3) | (1u << 4)) : (1u << 3);
        mmio_write32(SOC_CTRL_BASE + SOC_CTRL_IRQ_INJECT_OFF, mask);
        mmio_write32(SOC_CTRL_BASE + SOC_CTRL_IRQ_INJECT_OFF, 0u);
    }
    while (g_vec_mei[hartid] < 1u) {}

    if (ncores == 2u) barrier_wait(&g_bar, 2u, &local_sense);
    hal_irq_global_disable();

    if (g_vec_err[hartid] != 0u) soc_halt(0xBAD40002u);

    uint32_t res = (ncores << 16) |
                   (g_vec_exc[hartid] << 12) |
                   (g_vec_msi[hartid] << 8) |
                   (g_vec_mti[hartid] << 4) |
                   g_vec_mei[hartid];

    if (ncores == 2u) {
        if (hartid == 1u) {
            soc_park_secondary(res | 0x00100000u, &g_c1_done);
        } else {
            while (!g_c1_done) {}
            soc_halt(res);
        }
    } else {
        soc_halt(res);
    }
    return res;
}
