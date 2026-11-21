/* ============================================================================
 * Hardware Abstraction Layer: Per-Hart C Trap & ISR Dispatch Implementation
 * ============================================================================ */
#include "hal_irq.h"

static hal_isr_fn_t     g_timer_isr[SOC_MAX_HARTS];
static hal_isr_fn_t     g_ipi_isr[SOC_MAX_HARTS];
static hal_ext_isr_fn_t g_ext_isr[SOC_MAX_HARTS][PLIC_NUM_SOURCES];
static hal_exc_fn_t     g_exc_handler[SOC_MAX_HARTS];

void hal_irq_init(uint32_t hartid) {
    if (hartid >= SOC_MAX_HARTS) return;
    g_timer_isr[hartid]   = (hal_isr_fn_t)0;
    g_ipi_isr[hartid]     = (hal_isr_fn_t)0;
    g_exc_handler[hartid] = (hal_exc_fn_t)0;
    for (uint32_t s = 0u; s < PLIC_NUM_SOURCES; s++) {
        g_ext_isr[hartid][s] = (hal_ext_isr_fn_t)0;
    }
    hal_irq_global_disable();
    hal_irq_disable_mie(MIE_ALL);
    hal_clint_disable_timer(hartid);
    hal_clint_clear_ipi(hartid);
    hal_plic_set_enable_mask(hartid, 0u);
    hal_plic_set_threshold(hartid, 0u);
}

void hal_irq_register_timer(uint32_t hartid, hal_isr_fn_t fn) {
    if (hartid < SOC_MAX_HARTS) g_timer_isr[hartid] = fn;
}

void hal_irq_register_ipi(uint32_t hartid, hal_isr_fn_t fn) {
    if (hartid < SOC_MAX_HARTS) g_ipi_isr[hartid] = fn;
}

void hal_irq_register_ext(uint32_t hartid, uint32_t src_id, hal_ext_isr_fn_t fn) {
    if (hartid < SOC_MAX_HARTS && src_id >= 1u && src_id < PLIC_NUM_SOURCES) {
        g_ext_isr[hartid][src_id] = fn;
    }
}

void hal_irq_register_exc(uint32_t hartid, hal_exc_fn_t fn) {
    if (hartid < SOC_MAX_HARTS) g_exc_handler[hartid] = fn;
}

void soc_trap_handler(uint32_t mcause, uint32_t mepc, uint32_t mtval, trap_frame_t *tf) {
    uint32_t hartid = hal_csr_mhartid();
    (void)mepc;

    if (mcause & MCAUSE_INT_BIT) {
        uint32_t code = mcause & MCAUSE_CODE_MASK;
        if (code == IRQ_CAUSE_MTI) {
            if (hartid < SOC_MAX_HARTS && g_timer_isr[hartid]) {
                g_timer_isr[hartid](hartid);
            } else {
                hal_clint_disable_timer(hartid);
            }
        } else if (code == IRQ_CAUSE_MSI) {
            if (hartid < SOC_MAX_HARTS && g_ipi_isr[hartid]) {
                g_ipi_isr[hartid](hartid);
            } else {
                hal_clint_clear_ipi(hartid);
            }
        } else if (code == IRQ_CAUSE_MEI) {
            uint32_t src = hal_plic_claim(hartid);
            if (src >= 1u && src < PLIC_NUM_SOURCES) {
                if (hartid < SOC_MAX_HARTS && g_ext_isr[hartid][src]) {
                    g_ext_isr[hartid][src](hartid, src);
                }
                hal_plic_complete(hartid, src);
            }
        }
    } else {
        uint32_t code = mcause & MCAUSE_CODE_MASK;
        if (hartid < SOC_MAX_HARTS && g_exc_handler[hartid]) {
            g_exc_handler[hartid](hartid, code, mtval, tf);
        } else {
            /* Default fallback: skip offending 32-bit instruction */
            tf->mepc += 4u;
        }
    }
}

/* Weak default vectored entry points (overrideable by user C code!) */
__attribute__((weak))
void soc_vec_exc_handler(uint32_t mcause, uint32_t mepc, uint32_t mtval, trap_frame_t *tf) {
    soc_trap_handler(mcause, mepc, mtval, tf);
}

__attribute__((weak))
void soc_vec_msi_handler(uint32_t mcause, uint32_t mepc, uint32_t mtval, trap_frame_t *tf) {
    soc_trap_handler(mcause, mepc, mtval, tf);
}

__attribute__((weak))
void soc_vec_mti_handler(uint32_t mcause, uint32_t mepc, uint32_t mtval, trap_frame_t *tf) {
    soc_trap_handler(mcause, mepc, mtval, tf);
}

__attribute__((weak))
void soc_vec_mei_handler(uint32_t mcause, uint32_t mepc, uint32_t mtval, trap_frame_t *tf) {
    soc_trap_handler(mcause, mepc, mtval, tf);
}
