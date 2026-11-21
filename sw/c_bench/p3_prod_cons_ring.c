#include <stdint.h>
#include "sync.h"

/*
 * P3: Lock-Free SPSC Producer-Consumer Ring Buffer (MOESI M->O C2C Showcase!)
 *   - 32 cache-line-aligned (16 B) packet slots + cache-line-separated head/tail pointers
 *   - 2C (NUM_CORES=2):
 *       Core 0 (Producer): synthesizes 256 4-word packets (dirty M lines) & publishes tail w/ FENCE
 *       Core 1 (Consumer): reads published packets (GetS -> MOESI M->O C2C with 0 L2 WBs vs
 *                          MESI M->S forcing 256+ dirty L2 writebacks!), hashes payload & advances head!
 *   - 1C (NUM_CORES=1): Core 0 performs the exact same 256-packet produce + consume stream sequentially
 *   - Both 1C and 2C return the EXACT SAME 32-bit consumer digest!
 */

#ifndef NUM_CORES
#define NUM_CORES 2
#endif

#define RING_SLOTS 32u
#define RING_MASK  (RING_SLOTS - 1u)
#define NUM_PKTS   256u

typedef struct __attribute__((aligned(16))) {
    uint32_t w0;
    uint32_t w1;
    uint32_t w2;
    uint32_t w3;
} pkt_t;

static pkt_t ring_buf[RING_SLOTS] __attribute__((aligned(16)));
static struct __attribute__((aligned(16))) { volatile uint32_t val; } g_tail = { 0 };
static struct __attribute__((aligned(16))) { volatile uint32_t val; } g_head = { 0 };
static struct __attribute__((aligned(16))) { volatile uint32_t val; } g_result = { 0 };
#if NUM_CORES > 1
static barrier_t g_bar __attribute__((aligned(16))) = { 0, 0 };
#endif

static inline uint32_t prod_step(uint32_t idx, pkt_t *dst) {
    uint32_t a = idx * 0x9E3779B1u + 0x13579BDFu;
    uint32_t b = a ^ (a >> 13);
    b *= 0x85EBCA6Bu;
    uint32_t c = b ^ (b >> 15);
    uint32_t d = (a + b) ^ (c << 3);
    dst->w0 = a;
    dst->w1 = b;
    dst->w2 = c;
    dst->w3 = d;
    return d;
}

static inline uint32_t cons_mix(uint32_t acc, const pkt_t *src) {
    uint32_t x = src->w0 ^ src->w1;
    uint32_t y = src->w2 + src->w3;
    uint32_t m = (x * 33u) ^ (y + 0x7F4A7C15u);
    acc ^= m;
    acc = (acc << 7) | (acc >> 25);
    acc += (x ^ y);
    return acc;
}

uint32_t main(uint32_t hartid) {
#if NUM_CORES == 1
    (void)hartid;
    uint32_t acc = 0x12345678u;
    for (uint32_t i = 0; i < NUM_PKTS; ++i) {
        pkt_t *slot = &ring_buf[i & RING_MASK];
        prod_step(i, slot);
        fence_ww();
        g_tail.val = i + 1u;
        fence_rr();
        acc = cons_mix(acc, slot);
        g_head.val = i + 1u;
    }
    g_result.val = acc;
    return acc;
#else
    uint32_t sense = 0u;
    barrier_wait(&g_bar, NUM_CORES, &sense);
    if (hartid == 0u) {
        /* Producer: writes 16-byte dirty lines in M -> consumer GetS triggers MOESI M->O C2C! */
        for (uint32_t i = 0; i < NUM_PKTS; ++i) {
            while ((i - g_head.val) >= RING_SLOTS) {
                /* Wait for ring space */
            }
            prod_step(i, &ring_buf[i & RING_MASK]);
            fence_ww();
            g_tail.val = i + 1u;
        }
    } else {
        /* Consumer: pulls dirty packets via C2C, computes rolling digest */
        uint32_t acc = 0x12345678u;
        for (uint32_t i = 0; i < NUM_PKTS; ++i) {
            while (g_tail.val <= i) {
                /* Wait for published packet */
            }
            fence_rr();
            acc = cons_mix(acc, &ring_buf[i & RING_MASK]);
            fence_rw();
            g_head.val = i + 1u;
        }
        g_result.val = acc;
        fence_ww();
    }
    barrier_wait(&g_bar, NUM_CORES, &sense);
    return g_result.val;
#endif
}
