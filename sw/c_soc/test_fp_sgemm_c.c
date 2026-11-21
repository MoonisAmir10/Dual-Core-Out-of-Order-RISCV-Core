/* ============================================================================
 * Phase U6 Parallel 24x24 Single-Precision IEEE-754 Float SGEMM Benchmark
 * (C = A * B, compiled with GCC -march=rv32imaf -mabi=ilp32f -O2)
 *
 * Uses classic 4x4 register-blocked FMA micro-kernel (16 independent float
 * accumulators c00..c33 -> 16 FMADD.S per 8 FLWs, 2:1 compute-to-load ratio!)
 * with odd-cache-line-padded B stride (28 floats = 112B = 7x 16B lines ->
 * zero L2 2-bank conflict!) to saturate our 4-stage II=1 fpu_fma_pipe +
 * dedicated 3rd CDB (cdb_fp) + 2-wide superscalar INT/FLW issue!
 *   - 1C (single_ooo2w_soc): Core 0 computes all 24 rows (0..23)
 *   - 2C (dual_ooo2w_soc)  : Core 0 computes rows 0..11, Core 1 computes rows 12..23
 *   - Both 1C and 2C return the EXACT SAME bit-exact IEEE-754 checksum of C!
 * ============================================================================ */
#include <stdint.h>
#include "sync.h"
#include "soc_regs.h"

#define N      24u
#define B_STR  28u   /* 28 floats = 112B = 7 cache lines (odd -> eliminates 2-bank L2 conflicts!) */

static float g_A[N][N]     __attribute__((aligned(16)));
static float g_B[N][B_STR] __attribute__((aligned(16)));
static float g_C[N][N]     __attribute__((aligned(16)));

static barrier_t g_bar __attribute__((aligned(16))) = { 0u, 0u };

static inline uint32_t f2u(float x) {
    union { float f; uint32_t u; } v;
    v.f = x;
    return v.u;
}

static inline uint32_t rotl32(uint32_t x, uint32_t r) {
    return (x << r) | (x >> (32u - r));
}

uint32_t main(uint32_t hartid) {
    uint32_t ncores = mmio_read32(SOC_CTRL_BASE + SOC_CTRL_CORES_OFF);
    uint32_t local_sense = 0u;

    uint32_t r_start = (ncores == 2u) ? (hartid * (N / 2u)) : 0u;
    uint32_t r_end   = (ncores == 2u) ? ((hartid + 1u) * (N / 2u)) : N;

    /* Parallel initialization of A and B (exact binary32 values, div-free) */
    for (uint32_t i = r_start; i < r_end; i++) {
        for (uint32_t j = 0u; j < N; j++) {
            int32_t av = (int32_t)((i * 7u + j * 3u) & 15u) - 7;
            int32_t bv = (int32_t)((i * 5u + j * 11u) & 15u) - 8;
            g_A[i][j] = (float)av * 0.25f;
            g_B[i][j] = (float)bv * 0.125f;
        }
    }

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* Parallel 24x24 IEEE-754 SGEMM: 4x4 register-blocked FMA micro-kernel */
    for (uint32_t i = r_start; i < r_end; i += 4u) {
        const float *ai0 = &g_A[i + 0u][0];
        const float *ai1 = &g_A[i + 1u][0];
        const float *ai2 = &g_A[i + 2u][0];
        const float *ai3 = &g_A[i + 3u][0];

        for (uint32_t j = 0u; j < N; j += 4u) {
            float c00 = 0.0f, c01 = 0.0f, c02 = 0.0f, c03 = 0.0f;
            float c10 = 0.0f, c11 = 0.0f, c12 = 0.0f, c13 = 0.0f;
            float c20 = 0.0f, c21 = 0.0f, c22 = 0.0f, c23 = 0.0f;
            float c30 = 0.0f, c31 = 0.0f, c32 = 0.0f, c33 = 0.0f;

            for (uint32_t k = 0u; k < N; k++) {
                const float *bk = &g_B[k][j];
                float b0 = bk[0];
                float b1 = bk[1];
                float b2 = bk[2];
                float b3 = bk[3];

                float a0 = ai0[k];
                c00 += a0 * b0;
                c01 += a0 * b1;
                c02 += a0 * b2;
                c03 += a0 * b3;

                float a1 = ai1[k];
                c10 += a1 * b0;
                c11 += a1 * b1;
                c12 += a1 * b2;
                c13 += a1 * b3;

                float a2 = ai2[k];
                c20 += a2 * b0;
                c21 += a2 * b1;
                c22 += a2 * b2;
                c23 += a2 * b3;

                float a3 = ai3[k];
                c30 += a3 * b0;
                c31 += a3 * b1;
                c32 += a3 * b2;
                c33 += a3 * b3;
            }

            g_C[i + 0u][j + 0u] = c00; g_C[i + 0u][j + 1u] = c01;
            g_C[i + 0u][j + 2u] = c02; g_C[i + 0u][j + 3u] = c03;
            g_C[i + 1u][j + 0u] = c10; g_C[i + 1u][j + 1u] = c11;
            g_C[i + 1u][j + 2u] = c12; g_C[i + 1u][j + 3u] = c13;
            g_C[i + 2u][j + 0u] = c20; g_C[i + 2u][j + 1u] = c21;
            g_C[i + 2u][j + 2u] = c22; g_C[i + 2u][j + 3u] = c23;
            g_C[i + 3u][j + 0u] = c30; g_C[i + 3u][j + 1u] = c31;
            g_C[i + 3u][j + 2u] = c32; g_C[i + 3u][j + 3u] = c33;
        }
    }

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* Both cores verify deterministic bit-exact IEEE-754 checksum over all 576 floats of C */
    uint32_t hash = 0x56E00024u;
    for (uint32_t i = 0u; i < N; i++) {
        for (uint32_t j = 0u; j < N; j++) {
            hash = rotl32(hash ^ f2u(g_C[i][j]), 5u) + 0x9E3779B9u;
        }
    }
    return hash;
}
