/* ============================================================================
 * Phase U2 Software SYS_HALT & TOHOST Verification (with active mtvec != 0!)
 *
 * Proves that a C program with an active M-mode trap handler (mtvec != 0) can
 * cleanly terminate simulation via apb4_soc_ctrl.TOHOST + SYS_HALT without
 * disabling mtvec or relying on EBREAK!
 * ============================================================================ */

#include <stdint.h>

#define SOC_CTRL_BASE  0x10040000u
#define SOC_SYS_HALT   (*(volatile uint32_t *)(SOC_CTRL_BASE + 0x00Cu))
#define SOC_TOHOST     (*(volatile uint32_t *)(SOC_CTRL_BASE + 0x010u))

static volatile uint32_t g_traps;

__attribute__((interrupt("machine"), aligned(4)))
void halt_test_trap(void) {
    uint32_t epc;
    __asm__ volatile ("csrr %0, mepc" : "=r"(epc));
    g_traps += 1u;
    __asm__ volatile ("csrw mepc, %0" :: "r"(epc + 4u));
}

uint32_t main(uint32_t hartid) {
    if (hartid != 0u) {
        while (1) { __asm__ volatile ("nop"); }
    }
    g_traps = 0u;
    uint32_t handler = (uint32_t)(uintptr_t)&halt_test_trap;
    __asm__ volatile ("csrw mtvec, %0" :: "r"(handler));

    /* Take 1 illegal-instruction trap while mtvec != 0 */
    __asm__ volatile (".word 0xFFFFFFFF" ::: "memory");

    uint32_t code = (g_traps == 1u) ? 0x0000CAFEu : 0xBAD0FFFFu;
    SOC_TOHOST = code;

    /* Put result in hardware x14 (a4) then assert SYS_HALT while mtvec is STILL != 0! */
    __asm__ volatile (
        "mv   a4, %0\n"
        "sw   %1, 0(%2)\n"
        "1: j 1b\n"
        :: "r"(code), "r"(1u), "r"(SOC_CTRL_BASE + 0x00Cu)
        : "a4", "memory"
    );
    return code;
}
