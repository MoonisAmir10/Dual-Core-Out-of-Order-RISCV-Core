#ifndef DUAL_OOO_SYNC_H
#define DUAL_OOO_SYNC_H

#include <stdint.h>

/* ----------------------------------------------------------------------------
 * CSR & Memory-Ordering Intrinsics (RV32IMA_Zicsr_Zifencei)
 * ---------------------------------------------------------------------------- */
static inline uint32_t read_mhartid(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mhartid" : "=r"(v));
    return v;
}

static inline uint32_t read_mcycle(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mcycle" : "=r"(v));
    return v;
}

static inline uint32_t read_minstret(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, minstret" : "=r"(v));
    return v;
}

static inline void fence_rw(void) {
    __asm__ volatile ("fence rw, rw" ::: "memory");
}

static inline void fence_ww(void) {
    __asm__ volatile ("fence w, w" ::: "memory");
}

static inline void fence_rr(void) {
    __asm__ volatile ("fence r, r" ::: "memory");
}

static inline void fence_i(void) {
    __asm__ volatile ("fence.i" ::: "memory");
}

/* ----------------------------------------------------------------------------
 * RV32A Atomic Primitives
 * ---------------------------------------------------------------------------- */
static inline uint32_t amo_add_w(volatile uint32_t *addr, uint32_t val) {
    uint32_t old;
    __asm__ volatile ("amoadd.w.aqrl %0, %2, (%1)"
                      : "=r"(old) : "r"(addr), "r"(val) : "memory");
    return old;
}

static inline uint32_t amo_swap_w_aq(volatile uint32_t *addr, uint32_t val) {
    uint32_t old;
    __asm__ volatile ("amoswap.w.aq %0, %2, (%1)"
                      : "=r"(old) : "r"(addr), "r"(val) : "memory");
    return old;
}

static inline uint32_t amo_swap_w_rl(volatile uint32_t *addr, uint32_t val) {
    uint32_t old;
    __asm__ volatile ("amoswap.w.rl %0, %2, (%1)"
                      : "=r"(old) : "r"(addr), "r"(val) : "memory");
    return old;
}

static inline uint32_t lr_w(volatile uint32_t *addr) {
    uint32_t v;
    __asm__ volatile ("lr.w %0, (%1)" : "=r"(v) : "r"(addr) : "memory");
    return v;
}

static inline uint32_t sc_w(volatile uint32_t *addr, uint32_t val) {
    uint32_t fail;
    __asm__ volatile ("sc.w %0, %2, (%1)"
                      : "=r"(fail) : "r"(addr), "r"(val) : "memory");
    return fail;
}

/* ----------------------------------------------------------------------------
 * Synchronization Primitives (Spinlock, Ticket Lock, Sense-Reversing Barrier)
 * ---------------------------------------------------------------------------- */
typedef struct __attribute__((aligned(16))) {
    volatile uint32_t locked;
} spinlock_t;

static inline void spin_lock(spinlock_t *lk) {
    while (amo_swap_w_aq(&lk->locked, 1u) != 0u) {
        while (lk->locked != 0u) {
            /* Test-and-Test-and-Set (TTAS): spins locally in Shared (S) state! */
        }
    }
}

static inline void spin_unlock(spinlock_t *lk) {
    amo_swap_w_rl(&lk->locked, 0u);
}

typedef struct __attribute__((aligned(16))) {
    volatile uint32_t count;
    volatile uint32_t sense;
} barrier_t;

static inline void barrier_wait(barrier_t *b, uint32_t num_cores, uint32_t *local_sense) {
    uint32_t s = !(*local_sense);
    *local_sense = s;
    uint32_t prev = amo_add_w(&b->count, 1u);
    if (prev == num_cores - 1u) {
        b->count = 0u;
        fence_ww();
        b->sense = s;
    } else {
        while (b->sense != s) {
            /* Spin quietly in Shared (S) cache state */
        }
        fence_rr();
    }
}

#endif /* DUAL_OOO_SYNC_H */
