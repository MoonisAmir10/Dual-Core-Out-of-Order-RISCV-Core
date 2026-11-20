#include <stdint.h>
#include "sync.h"

/*
 * Phase 0 Full RV32IMA_Zicsr_Zifencei C Toolchain Smoke Test
 * Exercises crt0.S (mhartid stack setup), sync.h (AMO, LR/SC, spinlock, FENCE,
 * FENCE.I, mhartid), compiled with riscv64-unknown-elf-gcc -march=rv32ima -O2.
 *
 * Expected result for hartid=1:
 *   shared_cnt starts at 10 -> amo_add_w(+5) -> 15 (old=10)
 *   lr/sc loop adds 7 -> 22
 *   spinlock protects +100 -> 122
 *   return = shared_cnt (122) + old (10) + hartid (1) = 133 (0x00000085)
 */
static spinlock_t g_lock = { 0 };
static volatile uint32_t g_cnt = 10u;

uint32_t main(uint32_t hartid) {
    uint32_t id = read_mhartid();
    fence_rw();

    uint32_t old = amo_add_w(&g_cnt, 5u);

    /* Lock-free CAS via LR.W / SC.W */
    for (;;) {
        uint32_t cur = lr_w(&g_cnt);
        if (sc_w(&g_cnt, cur + 7u) == 0u) {
            break;
        }
    }

    /* Spinlock critical section */
    spin_lock(&g_lock);
    g_cnt += 100u;
    spin_unlock(&g_lock);

    fence_i();
    return g_cnt + old + id + (hartid - id);
}
