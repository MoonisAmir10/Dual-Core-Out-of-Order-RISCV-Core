#include <stdint.h>

/*
 * Phase 0 Bare-Metal C Smoke Benchmark (RV32IM-compatible for baseline core!)
 * Computes Fibonacci(12) + 8-element dot product + integer division checksum.
 * Expected result:
 *   fib(12) = 144
 *   dot     = sum_{i=0..7} (i+1)*(2*i+3) = 1*3 + 2*5 + 3*7 + 4*9 + 5*11 + 6*13 + 7*15 + 8*17
 *           = 3 + 10 + 21 + 36 + 55 + 78 + 105 + 136 = 444
 *   q       = (444 * 144) / 13 = 63936 / 13 = 4918
 *   return  = 144 + 444 + 4918 = 5506 (0x00001582)
 */
static uint32_t fib(uint32_t n) {
    uint32_t a = 0, b = 1;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t t = a + b;
        a = b;
        b = t;
    }
    return a;
}

uint32_t main(uint32_t hartid) {
    volatile uint32_t vec_a[8];
    volatile uint32_t vec_b[8];

    for (uint32_t i = 0; i < 8; i++) {
        vec_a[i] = i + 1u;
        vec_b[i] = 2u * i + 3u;
    }

    uint32_t dot = 0;
    for (uint32_t i = 0; i < 8; i++) {
        dot += vec_a[i] * vec_b[i];
    }

    uint32_t f = fib(12u);
    uint32_t q = (dot * f) / 13u;
    return f + dot + q + hartid;
}
