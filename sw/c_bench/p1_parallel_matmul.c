#include <stdint.h>
#include "sync.h"

/*
 * P1: Parallel 24x24 Integer Matrix Multiply (C = A x B)
 *   - Working set: 3 x 24 x 24 x 4 B = 6,912 B (> 4 KB L1D, fits in 32 KB L2!)
 *   - 1C (NUM_CORES=1): Core 0 computes all 24 rows
 *   - 2C (NUM_CORES=2): Core 0 computes rows 0..11, Core 1 computes rows 12..23
 *   - Inner k-loop unrolled x4 for dual-issue ALU/MUL saturation (~3.0 chip IPC!)
 *   - Both 1C and 2C return the EXACT SAME 32-bit FNV/XOR checksum of matrix C!
 */

#ifndef NUM_CORES
#define NUM_CORES 2
#endif

#define N 24

static int32_t mat_a[N][N] __attribute__((aligned(16)));
static int32_t mat_b[N][N] __attribute__((aligned(16)));
static int32_t mat_c[N][N] __attribute__((aligned(16)));
static barrier_t g_bar __attribute__((aligned(16))) = { 0, 0 };

uint32_t main(uint32_t hartid) {
    uint32_t sense = 0u;
    uint32_t r_start = (NUM_CORES == 2) ? (hartid * (N / 2)) : 0u;
    uint32_t r_end   = (NUM_CORES == 2) ? (r_start + (N / 2)) : (uint32_t)N;

    /* Parallel initialization of A and B (read-only shared B after barrier!) */
    for (uint32_t i = r_start; i < r_end; ++i) {
        for (uint32_t j = 0; j < N; ++j) {
            mat_a[i][j] = (int32_t)(((i * 7u + j * 13u + 5u) & 31u) - 15u);
            mat_b[i][j] = (int32_t)(((i * 11u + j * 3u + 9u) & 31u) - 14u);
        }
    }
    barrier_wait(&g_bar, NUM_CORES, &sense);

    /* Parallel tiled/unrolled Matrix Multiply: C[i][j] = sum_k A[i][k] * B[k][j] */
    for (uint32_t i = r_start; i < r_end; ++i) {
        const int32_t *a_row = mat_a[i];
        int32_t *c_row = mat_c[i];
        for (uint32_t j = 0; j < N; ++j) {
            int32_t s0 = 0, s1 = 0, s2 = 0, s3 = 0;
            for (uint32_t k = 0; k < N; k += 4) {
                s0 += a_row[k + 0] * mat_b[k + 0][j];
                s1 += a_row[k + 1] * mat_b[k + 1][j];
                s2 += a_row[k + 2] * mat_b[k + 2][j];
                s3 += a_row[k + 3] * mat_b[k + 3][j];
            }
            c_row[j] = (s0 + s1) + (s2 + s3);
        }
    }
    barrier_wait(&g_bar, NUM_CORES, &sense);

    /* Deterministic bit-exact checksum across full C matrix */
    uint32_t h = 0x811C9DC5u;
    for (uint32_t i = 0; i < N; ++i) {
        for (uint32_t j = 0; j < N; ++j) {
            h ^= (uint32_t)mat_c[i][j];
            h = (h << 5) | (h >> 27);
            h += 0x9E3779B9u;
        }
    }
    return h;
}
