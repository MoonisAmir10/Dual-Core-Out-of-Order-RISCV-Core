#include <stdint.h>
#include "sync.h"

/*
 * P5: False-Sharing (-DFALSE_SHARE=1) vs. Cache-Line-Padded (-DFALSE_SHARE=0)
 *   - Quantifies coherence line ping-pong penalty when two cores update independent
 *     counters packed inside the SAME 16-byte cache line vs. 16-byte aligned lines!
 *   - 1C (NUM_CORES=1): Core 0 runs both 1,500-iter loops sequentially (3,000 updates)
 *   - 2C (NUM_CORES=2): Core 0 updates slot 0 (1,500 iters), Core 1 updates slot 1 (1,500 iters)
 *   - All 3 builds (1C, 2C false-shared, 2C padded) return the EXACT SAME 32-bit checksum!
 */

#ifndef NUM_CORES
#define NUM_CORES 2
#endif

#ifndef FALSE_SHARE
#define FALSE_SHARE 0
#endif

#define ITERS 1000u

#if FALSE_SHARE
/* Packed inside the EXACT SAME 16-byte cache line (offsets +0 and +4)! */
static struct __attribute__((aligned(16))) {
    volatile uint32_t c0;
    volatile uint32_t c1;
    uint32_t pad[2];
} g_pack = { 0, 0, { 0, 0 } };
#define SLOT_PTR(id) ((id) == 0u ? &g_pack.c0 : &g_pack.c1)
#else
/* Padded to DISJOINT 16-byte cache lines (offsets +0 and +16)! */
typedef struct __attribute__((aligned(16))) {
    volatile uint32_t val;
    uint32_t pad[3];
} padded_cnt_t;
static padded_cnt_t g_pad[2] __attribute__((aligned(16))) = { { 0, {0,0,0} }, { 0, {0,0,0} } };
#define SLOT_PTR(id) (&g_pad[(id)].val)
#endif

static barrier_t g_bar __attribute__((aligned(16))) = { 0, 0 };

static inline void run_worker(uint32_t id, volatile uint32_t *slot) {
    uint32_t bias = id * 3u + 1u;
    for (uint32_t i = 0; i < ITERS; ++i) {
        /* Per-thread atomic counter increment (canonical false-sharing benchmark):
         *   Padded (16B): hits local L1D M-state in 1 cycle -> near-2x scaling!
         *   Packed (same line): forces GetM + C2C ping-pong on every atomic -> cliff! */
        amo_add_w(slot, (i & 7u) + bias);
    }
}

uint32_t main(uint32_t hartid) {
    uint32_t sense = 0u;
    barrier_wait(&g_bar, NUM_CORES, &sense);

#if NUM_CORES == 1
    (void)hartid;
    run_worker(0u, SLOT_PTR(0u));
    run_worker(1u, SLOT_PTR(1u));
#else
    run_worker(hartid, SLOT_PTR(hartid));
#endif

    barrier_wait(&g_bar, NUM_CORES, &sense);
    uint32_t v0 = *SLOT_PTR(0u);
    uint32_t v1 = *SLOT_PTR(1u);
    return (v0 * 31u) ^ (v1 + 0x5A5A5A5Au);
}
