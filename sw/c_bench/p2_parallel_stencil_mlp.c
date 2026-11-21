#include <stdint.h>
#include "sync.h"

/*
 * P2: Parallel 1D 5-Point Multi-Stream Stencil (16 KB Working Set -> L2 & MLP Showcase!)
 *   - Two ping-pong buffers buf_a[2048] + buf_b[2048] = 16 KB total
 *     (> 4x L1D size -> hammers DRAM if L2 disabled, hits cleanly in 32 KB 2-Bank L2!)
 *   - 4 sweeps with read-only halo boundary sharing at idx=1024 across MOESI S state
 *   - Unrolled x4 per core for 2-MSHR MLP + 2-bank L2 interleaving
 *   - Both 1C and 2C return the EXACT SAME 32-bit checksum!
 */

#ifndef NUM_CORES
#define NUM_CORES 2
#endif

#define LEN    2048
#define SWEEPS 4

static int32_t buf_a[LEN] __attribute__((aligned(16)));
static int32_t buf_b[LEN] __attribute__((aligned(16)));
static barrier_t g_bar __attribute__((aligned(16))) = { 0, 0 };

uint32_t main(uint32_t hartid) {
    uint32_t sense = 0u;
    uint32_t start = (NUM_CORES == 2) ? (hartid * (LEN / 2)) : 0u;
    uint32_t end   = (NUM_CORES == 2) ? (start + (LEN / 2)) : (uint32_t)LEN;

    for (uint32_t i = start; i < end; ++i) {
        buf_a[i] = (int32_t)((i * 37u + 19u) & 0xFFFu) - 2048;
        buf_b[i] = 0;
    }
    barrier_wait(&g_bar, NUM_CORES, &sense);

    int32_t s_lo = (start < 2u) ? 2 : (int32_t)start;
    int32_t s_hi = (end > (uint32_t)(LEN - 2)) ? (LEN - 2) : (int32_t)end;

    for (uint32_t sw = 0; sw < SWEEPS; ++sw) {
        const int32_t *src = (sw & 1u) ? buf_b : buf_a;
        int32_t       *dst = (sw & 1u) ? buf_a : buf_b;

        /* Boundary preservation */
        if (hartid == 0u) {
            dst[0] = src[0];
            dst[1] = src[1];
            dst[LEN - 2] = src[LEN - 2];
            dst[LEN - 1] = src[LEN - 1];
        }

        int32_t s_vec_end = s_lo + ((s_hi - s_lo) & ~3);
        for (int32_t i = s_lo; i < s_vec_end; i += 4) {
            int32_t v0 = (src[i - 2] + 2 * src[i - 1] + 4 * src[i] + 2 * src[i + 1] + src[i + 2]) >> 3;
            int32_t v1 = (src[i - 1] + 2 * src[i] + 4 * src[i + 1] + 2 * src[i + 2] + src[i + 3]) >> 3;
            int32_t v2 = (src[i] + 2 * src[i + 1] + 4 * src[i + 2] + 2 * src[i + 3] + src[i + 4]) >> 3;
            int32_t v3 = (src[i + 1] + 2 * src[i + 2] + 4 * src[i + 3] + 2 * src[i + 4] + src[i + 5]) >> 3;
            dst[i + 0] = v0;
            dst[i + 1] = v1;
            dst[i + 2] = v2;
            dst[i + 3] = v3;
        }
        for (int32_t i = s_vec_end; i < s_hi; ++i) {
            dst[i] = (src[i - 2] + 2 * src[i - 1] + 4 * src[i] + 2 * src[i + 1] + src[i + 2]) >> 3;
        }
        barrier_wait(&g_bar, NUM_CORES, &sense);
    }

    /* After 4 sweeps, final output is in buf_a */
    uint32_t h = 0x243F6A88u;
    for (uint32_t i = 0; i < LEN; ++i) {
        h ^= (uint32_t)buf_a[i];
        h = (h << 3) | (h >> 29);
        h += 0x85EBCA6Bu;
    }
    return h;
}
