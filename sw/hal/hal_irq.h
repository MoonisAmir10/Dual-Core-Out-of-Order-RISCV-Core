/* ============================================================================
 * Hardware Abstraction Layer: M-Mode Trap & Interrupt Sub-Routine (ISR) API
 *
 * Works seamlessly with sw/common/trap.S and sw/hal/hal_irq.c in BOTH:
 *   - Direct mtvec mode   (hal_irq_set_mtvec_direct)
 *   - Vectored mtvec mode (hal_irq_set_mtvec_vectored)
 *
 * Supports independent per-hart Timer (MTIP), Software/IPI (MSIP), PLIC
 * External (MEIP, Sources 1..15), and Synchronous Exception C callbacks!
 * ============================================================================ */
#ifndef HAL_IRQ_H
#define HAL_IRQ_H

#include <stdint.h>
#include "soc_regs.h"
#include "hal_clint.h"
#include "hal_plic.h"

#define SOC_MAX_HARTS       2u

/* M-Mode CSR Bit Masks */
#define MSTATUS_MIE         (1u << 3)
#define MSTATUS_MPIE        (1u << 7)

#define MIE_MSIE            (1u << 3)
#define MIE_MTIE            (1u << 7)
#define MIE_MEIE            (1u << 11)
#define MIE_ALL             (MIE_MSIE | MIE_MTIE | MIE_MEIE)

#define MCAUSE_INT_BIT      0x80000000u
#define MCAUSE_CODE_MASK    0x7FFFFFFFu

#define IRQ_CAUSE_MSI       3u
#define IRQ_CAUSE_MTI       7u
#define IRQ_CAUSE_MEI       11u

#define EXC_INSTR_MISALIGN  0u
#define EXC_ILLEGAL_INSTR   2u
#define EXC_BREAKPOINT      3u
#define EXC_LOAD_MISALIGN   4u
#define EXC_LOAD_FAULT      5u
#define EXC_STORE_MISALIGN  6u
#define EXC_STORE_FAULT     7u
#define EXC_ECALL_M         11u

/* Exact Trap Frame pushed by sw/common/trap.S (80 bytes) */
typedef struct {
    uint32_t ra;
    uint32_t t0;
    uint32_t t1;
    uint32_t t2;
    uint32_t a0;
    uint32_t a1;
    uint32_t a2;
    uint32_t a3;
    uint32_t a4;
    uint32_t a5;
    uint32_t a6;
    uint32_t a7;
    uint32_t t3;
    uint32_t t4;
    uint32_t t5;
    uint32_t t6;
    uint32_t mepc;
    uint32_t mstatus;
    uint32_t mcause;
    uint32_t mtval;
} trap_frame_t;

/* C ISR Callback Signatures */
typedef void (*hal_isr_fn_t)(uint32_t hartid);
typedef void (*hal_ext_isr_fn_t)(uint32_t hartid, uint32_t src_id);
typedef void (*hal_exc_fn_t)(uint32_t hartid, uint32_t cause, uint32_t mtval, trap_frame_t *tf);

/* Assembly Trampoline Symbols (from sw/common/trap.S) */
extern void _trap_entry_direct(void);
extern void _trap_vector_table(void);

/* ----------------------------------------------------------------------------
 * Inline M-Mode CSR Helpers
 * ---------------------------------------------------------------------------- */
static inline uint32_t hal_csr_mhartid(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mhartid" : "=r"(v));
    return v;
}

static inline uint32_t hal_csr_mstatus(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mstatus" : "=r"(v));
    return v;
}

static inline uint32_t hal_csr_mip(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mip" : "=r"(v));
    return v;
}

static inline void hal_irq_global_enable(void) {
    __asm__ volatile ("csrsi mstatus, 8" ::: "memory");
}

static inline void hal_irq_global_disable(void) {
    __asm__ volatile ("csrci mstatus, 8" ::: "memory");
}

static inline void hal_irq_enable_mie(uint32_t mask) {
    __asm__ volatile ("csrs mie, %0" :: "r"(mask) : "memory");
}

static inline void hal_irq_disable_mie(uint32_t mask) {
    __asm__ volatile ("csrc mie, %0" :: "r"(mask) : "memory");
}

static inline void hal_irq_set_mtvec_direct(void) {
    uint32_t v = (uint32_t)(uintptr_t)&_trap_entry_direct;
    __asm__ volatile ("csrw mtvec, %0" :: "r"(v & ~3u) : "memory");
}

static inline void hal_irq_set_mtvec_vectored(void) {
    uint32_t v = (uint32_t)(uintptr_t)&_trap_vector_table;
    __asm__ volatile ("csrw mtvec, %0" :: "r"((v & ~3u) | 1u) : "memory");
}

static inline void hal_irq_clear_mtvec(void) {
    __asm__ volatile ("csrw mtvec, zero" ::: "memory");
}

/* ----------------------------------------------------------------------------
 * Per-Hart C Callback Registration API (implemented in sw/hal/hal_irq.c)
 * ---------------------------------------------------------------------------- */
void hal_irq_init(uint32_t hartid);
void hal_irq_register_timer(uint32_t hartid, hal_isr_fn_t fn);
void hal_irq_register_ipi(uint32_t hartid, hal_isr_fn_t fn);
void hal_irq_register_ext(uint32_t hartid, uint32_t src_id, hal_ext_isr_fn_t fn);
void hal_irq_register_exc(uint32_t hartid, hal_exc_fn_t fn);

/* C Trap Dispatcher invoked by sw/common/trap.S */
void soc_trap_handler(uint32_t mcause, uint32_t mepc, uint32_t mtval, trap_frame_t *tf);

#endif /* HAL_IRQ_H */
