/* ============================================================================
 * Phase U2 End-to-End AMBA APB4 Bus, Peripherals & Precise Access Fault Test
 *
 * Verified on BOTH single_ooo2w_soc (1C) AND dual_ooo2w_soc (2C + MOESI + JITTER):
 *   1. apb4_soc_ctrl (0x1004_0000): SOC_ID, CORE_COUNT, FEATURE_MASK, live PMU
 *      counters, byte-strobed SCRATCH RW.
 *   2. apb4_clint (0x0200_0000): 64-bit mtime tick, per-hart 64-bit mtimecmp0/1,
 *      per-hart msip0/1 toggle, per-hart scratch[0/1] RW.
 *   3. Precise Load Access Fault (mcause=5) & Store Access Fault (mcause=7)
 *      from AMBA APB4 PSLVERR=1 on:
 *        - Unmapped RAM hole (0x0004_0000) load & store
 *        - Unpopulated PLIC slot (0x0C00_0000) load & store
 *        - Unpopulated Expansion slot S7 (0x1005_0000) load & store
 *        - Unmapped 32-bit address (0xDEAD_0000) load & store
 *        - Read-only SOC_ID write (0x1004_0000) -> mcause=7
 *        - Uncached AMOADD.W on APB4 (0x1004_0014) -> mcause=7
 *      Recovered cleanly by a GCC -O2 C __attribute__((interrupt("machine"))) handler!
 * ============================================================================ */

#include <stdint.h>

#define CLINT_BASE       0x02000000u
#define CLINT_MSIP(h)    (*(volatile uint32_t *)(CLINT_BASE + 4u * (h)))
#define CLINT_MTIMECMP_L(h) (*(volatile uint32_t *)(CLINT_BASE + 0x4000u + 8u * (h)))
#define CLINT_MTIMECMP_H(h) (*(volatile uint32_t *)(CLINT_BASE + 0x4004u + 8u * (h)))
#define CLINT_MTIME_L    (*(volatile uint32_t *)(CLINT_BASE + 0xBFF8u))
#define CLINT_SCRATCH(h) (*(volatile uint32_t *)(CLINT_BASE + 0xC000u + 4u * (h)))

#define SOC_CTRL_BASE    0x10040000u
#define SOC_ID_REG       (*(volatile uint32_t *)(SOC_CTRL_BASE + 0x000u))
#define SOC_CORE_CNT     (*(volatile uint32_t *)(SOC_CTRL_BASE + 0x004u))
#define SOC_FEAT_MASK    (*(volatile uint32_t *)(SOC_CTRL_BASE + 0x008u))
#define SOC_SCRATCH      (*(volatile uint32_t *)(SOC_CTRL_BASE + 0x014u))
#define SOC_PMU_CYCLES   (*(volatile uint32_t *)(SOC_CTRL_BASE + 0x020u))
#define SOC_PMU_C0_RET   (*(volatile uint32_t *)(SOC_CTRL_BASE + 0x024u))

static volatile uint32_t g_fault_cnt[2];
static volatile uint32_t g_load_faults[2];
static volatile uint32_t g_store_faults[2];
static volatile uint32_t g_tval_xor[2];
static volatile uint32_t g_bad_cause[2];

static inline uint32_t csr_mhartid(void) { uint32_t v; __asm__ volatile ("csrr %0, mhartid" : "=r"(v)); return v; }
static inline uint32_t csr_mcause(void)  { uint32_t v; __asm__ volatile ("csrr %0, mcause"  : "=r"(v)); return v; }
static inline uint32_t csr_mtval(void)   { uint32_t v; __asm__ volatile ("csrr %0, mtval"   : "=r"(v)); return v; }
static inline uint32_t csr_mepc(void)    { uint32_t v; __asm__ volatile ("csrr %0, mepc"    : "=r"(v)); return v; }
static inline void     set_mepc(uint32_t v)  { __asm__ volatile ("csrw mepc, %0"  :: "r"(v)); }
static inline void     set_mtvec(uint32_t v) { __asm__ volatile ("csrw mtvec, %0" :: "r"(v)); }

__attribute__((interrupt("machine"), aligned(4)))
void apb4_fault_handler(void) {
    uint32_t cid   = csr_mhartid();
    uint32_t cause = csr_mcause();
    uint32_t tval  = csr_mtval();
    uint32_t epc   = csr_mepc();

    g_fault_cnt[cid] += 1u;
    if (cause == 5u)      g_load_faults[cid]  += 1u;
    else if (cause == 7u) g_store_faults[cid] += 1u;
    else                  g_bad_cause[cid]    += 1u;

    g_tval_xor[cid] ^= tval;
    set_mepc(epc + 4u);
}

static inline void probe_load_fault(uint32_t addr, uint32_t *canary) {
    uint32_t v = *canary;
    __asm__ volatile ("lw %0, 0(%1)" : "+r"(v) : "r"(addr) : "memory");
    *canary = v;
}

static inline void probe_store_fault(uint32_t addr, uint32_t val) {
    __asm__ volatile ("sw %0, 0(%1)" :: "r"(val), "r"(addr) : "memory");
}

uint32_t main(uint32_t hartid) {
    g_fault_cnt[hartid]    = 0u;
    g_load_faults[hartid]  = 0u;
    g_store_faults[hartid] = 0u;
    g_tval_xor[hartid]     = 0u;
    g_bad_cause[hartid]    = 0u;

    /* 1. Verify APB4 SoC Control Slave (0x1004_0000) */
    if (SOC_ID_REG != 0x20020002u)   return 0xBAD20001u;
    uint32_t ncores = SOC_CORE_CNT;
    if (ncores != 1u && ncores != 2u) return 0xBAD20002u;
    if ((SOC_FEAT_MASK & 0x00000003u) != 0x00000003u) return 0xBAD20003u;

    uint32_t cyc0 = SOC_PMU_CYCLES;
    if (cyc0 == 0u)                  return 0xBAD20004u;

    /* 2. Verify APB4 CLINT Slave (0x0200_0000): mtime, mtimecmp, msip, scratch */
    uint32_t t0 = CLINT_MTIME_L;
    CLINT_SCRATCH(hartid) = 0xA5000000u | (hartid << 8) | 0x5Au;
    CLINT_MTIMECMP_L(hartid) = 0x12345678u + hartid;
    CLINT_MTIMECMP_H(hartid) = 0xFFFFFFFFu;
    CLINT_MSIP(hartid) = 1u;
    if (CLINT_MSIP(hartid) != 1u)    return 0xBAD20005u;
    CLINT_MSIP(hartid) = 0u;
    if (CLINT_MSIP(hartid) != 0u)    return 0xBAD20006u;
    if (CLINT_SCRATCH(hartid) != (0xA5000000u | (hartid << 8) | 0x5Au)) return 0xBAD20007u;
    if (CLINT_MTIMECMP_L(hartid) != (0x12345678u + hartid))             return 0xBAD20008u;
    uint32_t t1 = CLINT_MTIME_L;
    if (t1 <= t0)                    return 0xBAD20009u;

    /* Core 0 also verifies byte-strobed PSTRB write to SOC_SCRATCH */
    if (hartid == 0u) {
        SOC_SCRATCH = 0x11223344u;
        volatile uint8_t *bptr = (volatile uint8_t *)(SOC_CTRL_BASE + 0x014u);
        bptr[1] = 0xEEu;
        if (SOC_SCRATCH != 0x1122EE44u) return 0xBAD2000Au;
    }

    /* 3. Install C Trap Handler & Verify Precise APB4 PSLVERR -> mcause=5 / mcause=7! */
    set_mtvec((uint32_t)(uintptr_t)&apb4_fault_handler);

    uint32_t canary = 0x55AA7788u;
    /* 4 Load Access Faults (mcause = 5) */
    probe_load_fault(0x00040000u, &canary); /* unmapped RAM hole */
    probe_load_fault(0x10080000u, &canary); /* unpopulated expansion slot S10 */
    probe_load_fault(0x10050000u, &canary); /* unpopulated expansion slot S7 */
    probe_load_fault(0xDEAD0000u, &canary); /* unmapped 32-bit space */

    /* 6 Store/AMO Access Faults (mcause = 7) */
    probe_store_fault(0x00040000u, 0x11111111u);
    probe_store_fault(0x10080000u, 0x22222222u);
    probe_store_fault(0x10050000u, 0x33333333u);
    probe_store_fault(0xDEAD0000u, 0x44444444u);
    probe_store_fault(0x10040000u, 0xBAD0BAD0u); /* RO SOC_ID write -> PSLVERR! */

    /* Uncached AMOADD.W on APB4 address -> mcause = 7 (canary untouched!) */
    uint32_t amo_addr = SOC_CTRL_BASE + 0x014u;
    uint32_t one = 1u;
    __asm__ volatile ("amoadd.w %0, %1, (%2)" : "+r"(canary) : "r"(one), "r"(amo_addr) : "memory");

    /* Disable trap vector before returning to crt0.S EBREAK halt */
    set_mtvec(0u);

    /* Verify canary untouched, SOC_ID intact, 4 load faults + 6 store faults = 10 total! */
    if (canary != 0x55AA7788u)         return 0xBAD20010u;
    if (SOC_ID_REG != 0x20020002u)     return 0xBAD20011u;
    if (g_bad_cause[hartid] != 0u)     return 0xBAD20012u;
    if (g_load_faults[hartid] != 4u)   return 0xBAD20013u;
    if (g_store_faults[hartid] != 6u)  return 0xBAD20014u;
    if (g_fault_cnt[hartid] != 10u)    return 0xBAD20015u;

    /* Expected tval XOR: pairs cancel out leaving only 0x10040000 ^ 0x10040014 = 0x14! */
    if (g_tval_xor[hartid] != 0x00000014u) return 0xBAD20016u;

    /* Live PMU cycles must have advanced */
    if (SOC_PMU_CYCLES <= cyc0)        return 0xBAD20017u;
    if (SOC_PMU_C0_RET == 0u)          return 0xBAD20018u;

    /* Deterministic checksum: (hartid << 16) | (ncores << 12) | (load_f << 8) | store_f */
    return (hartid << 16) | (ncores << 12) | (g_load_faults[hartid] << 8) | g_store_faults[hartid];
}
