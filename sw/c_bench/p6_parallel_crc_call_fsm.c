#include <stdint.h>
#include "sync.h"

/*
 * P6: Parallel Branchy Call-Heavy Packet CRC-32 & FSM Classifier
 *   - Exercises RAS (multi-level noinline function calls), Tournament Branch Predictor
 *     (data-dependent FSM + CRC branches), L1I Next-Line Prefetcher (multi-KB .text
 *     footprint), and Shared L2 cache across 64 packets x 32 bytes!
 *   - 1C (NUM_CORES=1): Core 0 processes all 64 packets
 *   - 2C (NUM_CORES=2): Core 0 processes packets 0..31, Core 1 processes packets 32..63
 *   - Both 1C and 2C return the EXACT SAME 32-bit aggregate packet digest!
 */

#ifndef NUM_CORES
#define NUM_CORES 2
#endif

#define N_PKTS   64u
#define PKT_LEN  32u

static uint8_t  pkt_buf[N_PKTS][PKT_LEN] __attribute__((aligned(16)));
static uint32_t pkt_out[N_PKTS] __attribute__((aligned(16)));
static barrier_t g_bar __attribute__((aligned(16))) = { 0, 0 };

__attribute__((noinline))
static uint32_t sbox_mix(uint32_t x) {
    x ^= (x >> 7);
    if (x & 1u) x = (x * 0x45D9F3Bu) ^ 0xA5A5u;
    else        x = (x * 0x27D4EB2Du) + 0x3C3Cu;
    x ^= (x >> 11);
    return x;
}

__attribute__((noinline))
static uint32_t crc32_byte(uint32_t crc, uint8_t b) {
    crc ^= (uint32_t)b;
    /* Unrolled 8-bit IEEE CRC-32 with branchy LFSR polynomial feedback */
    for (uint32_t bit = 0; bit < 8u; ++bit) {
        if (crc & 1u) crc = (crc >> 1) ^ 0xEDB88320u;
        else          crc = (crc >> 1);
    }
    return crc;
}

__attribute__((noinline))
static uint32_t fsm_classify(const uint8_t *p, uint32_t len) {
    uint32_t state = 0u;
    uint32_t score = 0x101u;
    for (uint32_t i = 0; i < len; ++i) {
        uint32_t b = p[i];
        switch (state) {
            case 0u:
                if (b < 64u)       { state = 1u; score += b + 3u; }
                else if (b < 160u) { state = 2u; score ^= (b << 2); }
                else               { state = 3u; score += sbox_mix(b); }
                break;
            case 1u:
                if (b & 0x10u)     { state = 2u; score = (score << 1) ^ b; }
                else               { state = 0u; score += (b * 5u); }
                break;
            case 2u:
                if (b > 128u)      { state = 3u; score ^= sbox_mix(b + score); }
                else if (b & 1u)   { state = 1u; score += b ^ 0x55u; }
                else               { state = 0u; score -= b; }
                break;
            default:
                if ((b & 3u) == 0u) { state = 0u; score += (b << 3); }
                else                { state = 1u; score ^= (b * 17u); }
                break;
        }
    }
    return score ^ (state << 24);
}

__attribute__((noinline))
static uint32_t process_packet(const uint8_t *p) {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < PKT_LEN; ++i) {
        crc = crc32_byte(crc, p[i]);
    }
    uint32_t cls = fsm_classify(p, PKT_LEN);
    return sbox_mix(crc ^ cls);
}

uint32_t main(uint32_t hartid) {
    uint32_t sense = 0u;
    uint32_t start = (NUM_CORES == 2) ? (hartid * (N_PKTS / 2u)) : 0u;
    uint32_t end   = (NUM_CORES == 2) ? (start + (N_PKTS / 2u)) : N_PKTS;

    /* Parallel packet synthesis */
    for (uint32_t p = start; p < end; ++p) {
        uint32_t s = (p + 1u) * 0x9E3779B9u;
        for (uint32_t i = 0; i < PKT_LEN; ++i) {
            s = s * 1664525u + 1013904223u;
            pkt_buf[p][i] = (uint8_t)(s >> 24);
        }
    }
    barrier_wait(&g_bar, NUM_CORES, &sense);

    /* Parallel CRC-32 + FSM classification (heavy calls -> RAS + BP + L1I!) */
    for (uint32_t p = start; p < end; ++p) {
        pkt_out[p] = process_packet(pkt_buf[p]);
    }
    barrier_wait(&g_bar, NUM_CORES, &sense);

    uint32_t h = 0x31415926u;
    for (uint32_t p = 0; p < N_PKTS; ++p) {
        h ^= pkt_out[p];
        h = (h << 5) | (h >> 27);
        h += 0x27182818u;
    }
    return h;
}
