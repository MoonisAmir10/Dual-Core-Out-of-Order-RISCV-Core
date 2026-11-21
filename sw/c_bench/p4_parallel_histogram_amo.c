#include <stdint.h>
#include "sync.h"

/*
 * P4: Parallel 32-Bin Histogram Reduction (AMOADD.W + AMOSWAP Spinlock + Local Bins)
 *   - 2,048 input samples (8 KB > 4 KB L1D, fits in 32 KB L2)
 *   - Each core processes its half (1,024 samples on 2C, 2,048 on 1C):
 *       * Accumulates fast thread-local 32-bin histogram (zero false sharing!)
 *       * Every 8th sample performs a direct global AMOADD.W on shared_amo_bins[bin]
 *   - Merges local histograms into global_hist[32] under an AMOSWAP.W.aq/.rl spinlock
 *   - Both 1C and 2C return the EXACT SAME 32-bit checksum over all 32 bins!
 */

#ifndef NUM_CORES
#define NUM_CORES 2
#endif

#define N_SAMPLES 2048u
#define N_BINS    32u

static uint32_t samples[N_SAMPLES] __attribute__((aligned(16)));
static uint32_t local_hist[2][N_BINS] __attribute__((aligned(16)));
static uint32_t global_hist[N_BINS] __attribute__((aligned(16)));
static volatile uint32_t shared_amo_bins[N_BINS] __attribute__((aligned(16)));
static spinlock_t g_hist_lock __attribute__((aligned(16))) = { 0 };
static barrier_t g_bar __attribute__((aligned(16))) = { 0, 0 };

uint32_t main(uint32_t hartid) {
    uint32_t sense = 0u;
    uint32_t start = (NUM_CORES == 2) ? (hartid * (N_SAMPLES / 2u)) : 0u;
    uint32_t end   = (NUM_CORES == 2) ? (start + (N_SAMPLES / 2u)) : N_SAMPLES;

    /* Parallel deterministic sample generation */
    for (uint32_t i = start; i < end; ++i) {
        uint32_t x = i * 2654435761u + 0x9E3779B9u;
        x ^= x >> 16;
        x *= 0x7FEB352Du;
        samples[i] = x;
    }
    for (uint32_t b = 0; b < N_BINS; ++b) {
        local_hist[hartid][b] = 0u;
    }
    barrier_wait(&g_bar, NUM_CORES, &sense);

    /* Parallel histogramming: fast local bins + periodic global AMOADD.W */
    uint32_t *lh = local_hist[hartid];
    for (uint32_t i = start; i < end; ++i) {
        uint32_t v = samples[i];
        uint32_t bin = (v ^ (v >> 11) ^ (v >> 21)) & (N_BINS - 1u);
        if ((i & 7u) == 0u) {
            amo_add_w(&shared_amo_bins[bin], 1u);
        } else {
            lh[bin] += 1u;
        }
    }

    /* Protected reduction into global_hist via AMOSWAP TTAS spinlock */
    spin_lock(&g_hist_lock);
    for (uint32_t b = 0; b < N_BINS; ++b) {
        global_hist[b] += lh[b];
    }
    spin_unlock(&g_hist_lock);

    barrier_wait(&g_bar, NUM_CORES, &sense);

    /* Combine global_hist + shared_amo_bins -> bit-exact 32-bit digest */
    uint32_t h = 0xCAFEBABEu;
    for (uint32_t b = 0; b < N_BINS; ++b) {
        uint32_t tot = global_hist[b] + shared_amo_bins[b];
        h ^= tot * (b + 17u);
        h = (h << 5) | (h >> 27);
        h += 0x9E3779B9u;
    }
    return h;
}
