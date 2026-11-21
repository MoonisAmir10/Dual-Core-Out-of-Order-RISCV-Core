/* ============================================================================
 * Phase U1 Bare-Metal C M-Mode Trap Handler Verification (GCC -O2)
 *
 * Proves that real GCC-compiled C code using __attribute__((interrupt("machine")))
 * cleanly installs mtvec, handles illegal instructions (mcause=2), misaligned
 * load/store (mcause=4,6), and ECALL (mcause=11), advances mepc += 4, returns
 * via compiler-emitted MRET, and preserves all memory/register invariants on
 * both single-core and dual-core MOESI SoCs!
 * ============================================================================ */

#include <stdint.h>

static volatile uint32_t g_trap_cnt[2];
static volatile uint32_t g_cause_sum[2];
static volatile uint32_t g_tval_xor[2];
static volatile uint32_t g_sentinel[2];

static inline uint32_t read_csr_mhartid(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mhartid" : "=r"(v));
    return v;
}

static inline uint32_t read_csr_mcause(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mcause" : "=r"(v));
    return v;
}

static inline uint32_t read_csr_mtval(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mtval" : "=r"(v));
    return v;
}

static inline uint32_t read_csr_mepc(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, mepc" : "=r"(v));
    return v;
}

static inline void write_csr_mepc(uint32_t v) {
    __asm__ volatile ("csrw mepc, %0" :: "r"(v));
}

static inline void write_csr_mtvec(uint32_t v) {
    __asm__ volatile ("csrw mtvec, %0" :: "r"(v));
}

__attribute__((interrupt("machine"), aligned(4)))
void c_trap_handler(void) {
    uint32_t cid   = read_csr_mhartid();
    uint32_t cause = read_csr_mcause();
    uint32_t tval  = read_csr_mtval();
    uint32_t epc   = read_csr_mepc();

    g_trap_cnt[cid]  += 1u;
    g_cause_sum[cid] += cause;
    g_tval_xor[cid]  ^= tval;

    /* Advance past faulting 32-bit instruction */
    write_csr_mepc(epc + 4u);
}

uint32_t main(uint32_t hartid) {
    g_trap_cnt[hartid]  = 0u;
    g_cause_sum[hartid] = 0u;
    g_tval_xor[hartid]  = 0u;
    g_sentinel[hartid]  = 0xCAFE0000u | hartid;

    /* Install C machine-mode trap handler */
    write_csr_mtvec((uint32_t)(uintptr_t)&c_trap_handler);

    /* 1. Illegal instruction trap (mcause = 2, mtval = 0xFFFFFFFF) */
    __asm__ volatile (".word 0xFFFFFFFF" ::: "memory");

    /* 2. Misaligned LW trap (mcause = 4, mtval = &g_sentinel[hartid] + 1) */
    uint32_t bad_addr = (uint32_t)(uintptr_t)&g_sentinel[hartid] + 1u;
    uint32_t dummy = 0x12345678u;
    __asm__ volatile ("lw %0, 0(%1)" : "+r"(dummy) : "r"(bad_addr) : "memory");

    /* 3. Misaligned SW trap (mcause = 6, mtval = &g_sentinel[hartid] + 2) */
    uint32_t bad_sw_addr = (uint32_t)(uintptr_t)&g_sentinel[hartid] + 2u;
    uint32_t poison = 0xBAD0BAD0u;
    __asm__ volatile ("sw %0, 0(%1)" :: "r"(poison), "r"(bad_sw_addr) : "memory");

    /* 4. ECALL trap (mcause = 11, mtval = 0) */
    __asm__ volatile ("ecall" ::: "memory");

    /* Disable trap vector before returning to crt0.S EBREAK halt */
    write_csr_mtvec(0u);

    /* Verify sentinel untouched, dummy untouched, 4 traps, cause_sum = 2+4+6+11 = 23 */
    if (g_sentinel[hartid] != (0xCAFE0000u | hartid)) return 0xBAD00010u;
    if (dummy != 0x12345678u)                         return 0xBAD00011u;
    if (g_trap_cnt[hartid] != 4u)                     return 0xBAD00012u;
    if (g_cause_sum[hartid] != 23u)                   return 0xBAD00013u;

    /* Return deterministic checksum incorporating hartid, count, cause_sum, and mtval check */
    uint32_t exp_tval_xor = 0xFFFFFFFFu ^ bad_addr ^ bad_sw_addr ^ 0u;
    if (g_tval_xor[hartid] != exp_tval_xor)           return 0xBAD00014u;

    return (hartid << 16) | (g_trap_cnt[hartid] << 8) | g_cause_sum[hartid];
}
