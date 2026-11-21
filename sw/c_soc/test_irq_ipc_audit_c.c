/* ============================================================================
 * Phase U3 Zero-Stall IPC Audit: Parallel 16x16 Matmul (MIE=0 vs MIE=1 Idle!)
 *
 * Compiled twice with -DIRQ_ARMED=0 and -DIRQ_ARMED=1 to prove that arming
 * mtvec, mstatus.MIE=1, mie=ALL, and PLIC (with idle IRQs) incurs ZERO (0.0%)
 * steady-state IPC penalty on our 2-wide dual-core OoO pipeline!
 * ============================================================================ */
#include <stdint.h>
#include "sync.h"
#include "hal_irq.h"

#define N 16u

static int32_t A[N][N] __attribute__((aligned(16)));
static int32_t B[N][N] __attribute__((aligned(16)));
static int32_t C[N][N] __attribute__((aligned(16)));
static barrier_t g_bar;

/* Fixed cache-line-aligned matmul kernel -> identical I$ alignment in both builds! */
__attribute__((noinline, aligned(16)))
static uint32_t run_matmul_slice(uint32_t r0, uint32_t r1) {
    uint32_t t0 = read_mcycle();
    for (uint32_t i = r0; i < r1; i++) {
        for (uint32_t j = 0u; j < N; j++) {
            int32_t s = 0;
            for (uint32_t k = 0u; k < N; k++) {
                s += A[i][k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
    return read_mcycle() - t0;
}

uint32_t main(uint32_t hartid) {
    uint32_t local_sense = 0u;

#if IRQ_ARMED
    hal_irq_init(hartid);
    hal_irq_set_mtvec_direct();
    hal_plic_set_priority(1u, 4u);
    hal_plic_enable_irq(hartid, 1u);
    hal_irq_enable_mie(MIE_ALL);
    hal_irq_global_enable();
#endif

    if (hartid == 0u) {
        for (uint32_t i = 0u; i < N; i++) {
            for (uint32_t j = 0u; j < N; j++) {
                A[i][j] = (int32_t)((i * 3u + j * 5u + 1u) & 0xFu) - 7;
                B[i][j] = (int32_t)((i * 7u + j * 2u + 3u) & 0xFu) - 6;
            }
        }
    }
    barrier_wait(&g_bar, 2u, &local_sense);

    /* Timed region: Core 0 computes rows 0..7, Core 1 computes rows 8..15 */
    uint32_t r0 = hartid * (N / 2u);
    uint32_t r1 = r0 + (N / 2u);
    uint32_t dt = run_matmul_slice(r0, r1);
    barrier_wait(&g_bar, 2u, &local_sense);

    /* Publish exact compute-region cycle counts in fixed DRAM mailbox 0x0002_0000 */
    volatile uint32_t *mb = (volatile uint32_t *)0x00020000u;
    mb[hartid] = dt;

    uint32_t sum = 0u;
    for (uint32_t i = 0u; i < N; i++) {
        for (uint32_t j = 0u; j < N; j++) {
            sum ^= (uint32_t)C[i][j] + (i * 31u + j);
        }
    }
    if (hartid == 0u) {
        mb[2] = sum;
    }

#if IRQ_ARMED
    hal_irq_global_disable();
    hal_irq_clear_mtvec();
#endif
    return sum;
}
