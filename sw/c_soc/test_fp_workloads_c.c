/* ============================================================================
 * Phase U6 Live Hard-Float C Workload, Rounding-Mode, FS=Off Trap &
 * Live FP-Preemption Context Save/Restore Suite (RV32IMAF / ilp32f)
 *
 * Compiled with: -march=rv32imaf -mabi=ilp32f -O2 -fno-math-errno
 * Verified on BOTH single_ooo2w_soc (1C) AND dual_ooo2w_soc (2C + MOESI + JITTER):
 *   1. Verifies SOC_FEAT_FPU (bit 4) and misa.F (bit 5)
 *   2. SAXPY (N=32) using hardware fmadd.s -> bit-exact IEEE-754 checksum
 *   3. Hardware fsqrt.s (normal, subnormal, +/-0, negative -> qNaN+NV) + fdiv/0 (DZ)
 *   4. Dynamic rounding modes (frm = RNE/RTZ/RDN/RUP/RMM via csrw frm -> csr_sync)
 *   5. mstatus.FS = Off (00) illegal-instruction trap + C handler recovery (no double-fault!)
 *   6. Live hard-float foreground compute preempted >=4x by CLINT MTI C ISR that
 *      heavily clobbers ft0..ft11, fa0..fa7, frm, and fflags (proves trap.S lazy FP
 *      context save/restore preserves 100% bit-exact IEEE-754 state & fcsr!)
 * ============================================================================ */
#include <stdint.h>
#include "sync.h"
#include "hal_irq.h"
#include "hal_clint.h"
#include "hal_uart.h"

#define SAXPY_N         32u
#define TARGET_FP_IRQS  4u

static inline uint32_t f2u(float x) {
    union { float f; uint32_t u; } v;
    v.f = x;
    return v.u;
}

static inline float u2f(uint32_t u) {
    union { float f; uint32_t u; } v;
    v.u = u;
    return v.f;
}

static inline uint32_t rotl32(uint32_t x, uint32_t r) {
    return (x << r) | (x >> (32u - r));
}

static inline uint32_t mix_hash(uint32_t h, uint32_t v) {
    return rotl32(h ^ v, 5u) + 0x9E3779B9u;
}

static inline uint32_t read_misa(void) {
    uint32_t v;
    __asm__ volatile ("csrr %0, misa" : "=r"(v));
    return v;
}

static inline uint32_t read_fflags(void) {
    uint32_t v;
    __asm__ volatile ("frflags %0" : "=r"(v));
    return v;
}

static inline void write_fflags(uint32_t v) {
    __asm__ volatile ("fsflags %0" :: "r"(v) : "memory");
}

static inline uint32_t read_frm(void) {
    uint32_t v;
    __asm__ volatile ("frrm %0" : "=r"(v));
    return v;
}

static inline void write_frm(uint32_t v) {
    __asm__ volatile ("fsrm %0" :: "r"(v) : "memory");
}

static inline float hw_fsqrt(float x) {
    float r;
    __asm__ volatile ("fsqrt.s %0, %1" : "=f"(r) : "f"(x));
    return r;
}

static inline float hw_fdiv_dyn(float a, float b) {
    float r;
    __asm__ volatile ("fdiv.s %0, %1, %2, dyn" : "=f"(r) : "f"(a), "f"(b));
    return r;
}

static inline int32_t hw_fcvt_ws_dyn(float a) {
    int32_t r;
    __asm__ volatile ("fcvt.w.s %0, %1, dyn" : "=r"(r) : "f"(a));
    return r;
}

static inline float hw_fmadd_dyn(float a, float b, float c) {
    float r;
    __asm__ volatile ("fmadd.s %0, %1, %2, %3, dyn" : "=f"(r) : "f"(a), "f"(b), "f"(c));
    return r;
}

/* Per-hart shared state */
static volatile uint32_t g_fs_off_traps[2];
static volatile uint32_t g_fp_irq_ticks[2];
static volatile uint32_t g_isr_fp_sink[2];
static volatile uint32_t g_c1_done;
static barrier_t         g_bar;

/* Exception handler for Test 5: mstatus.FS = Off illegal FP instruction */
static void fs_off_exc_handler(uint32_t hartid, uint32_t cause, uint32_t tval, trap_frame_t *tf) {
    (void)tval;
    if (cause != 2u) {
        soc_halt(0xBAD60050u | (cause & 0xFu));
    }
    /* Re-enable mstatus.FS = Initial (2'b01 -> bit 13 = 0x2000) before returning! */
    uint32_t fs_init = 0x2000u;
    __asm__ volatile ("csrs mstatus, %0" :: "r"(fs_init) : "memory");
    g_fs_off_traps[hartid] += 1u;
    tf->mepc += 4u;
}

/* Live CLINT MTI ISR that aggressively clobbers ALL caller-saved FP regs & fcsr! */
static void fp_clobber_mti_isr(uint32_t hartid) {
    uint32_t cnt = g_fp_irq_ticks[hartid] + 1u;
    g_fp_irq_ticks[hartid] = cnt;
    if (cnt < TARGET_FP_IRQS + 2u) {
        hal_clint_set_alarm_rel(hartid, 450u);
    } else {
        hal_clint_disable_timer(hartid);
    }

    /* Deliberately mutate frm=RUP (3) & fflags=0x1F inside the ISR, plus clobber
     * caller-saved FP registers ft0..ft11 and fa0..fa7 via real FP instructions! */
    write_frm(3u);
    write_fflags(0x1Fu);

    volatile float seed = (float)(int32_t)(cnt * 7u + hartid + 3u);
    float a = seed * 0.375f + 1.25f;
    float b = hw_fsqrt(a + 2.0f);
    float c = hw_fdiv_dyn(a, b + 0.5f);
    float d = hw_fmadd_dyn(a, b, c);
    float e = hw_fmadd_dyn(c, d, -b);

    /* Directly stomp ft0..ft11 and fa0..fa7 with poison floats so any missing
     * register save/restore in trap.S is caught instantaneously! */
    uint32_t poison = 0x43210000u | (cnt << 4) | hartid;
    __asm__ volatile (
        "fmv.w.x ft0,  %0\n\t"
        "fmv.w.x ft1,  %0\n\t"
        "fmv.w.x ft2,  %0\n\t"
        "fmv.w.x ft3,  %0\n\t"
        "fmv.w.x ft4,  %0\n\t"
        "fmv.w.x ft5,  %0\n\t"
        "fmv.w.x ft6,  %0\n\t"
        "fmv.w.x ft7,  %0\n\t"
        "fmv.w.x ft8,  %0\n\t"
        "fmv.w.x ft9,  %0\n\t"
        "fmv.w.x ft10, %0\n\t"
        "fmv.w.x ft11, %0\n\t"
        "fmv.w.x fa0,  %0\n\t"
        "fmv.w.x fa1,  %0\n\t"
        "fmv.w.x fa2,  %0\n\t"
        "fmv.w.x fa3,  %0\n\t"
        "fmv.w.x fa4,  %0\n\t"
        "fmv.w.x fa5,  %0\n\t"
        "fmv.w.x fa6,  %0\n\t"
        "fmv.w.x fa7,  %0\n\t"
        :: "r"(poison)
        : "ft0","ft1","ft2","ft3","ft4","ft5","ft6","ft7",
          "ft8","ft9","ft10","ft11",
          "fa0","fa1","fa2","fa3","fa4","fa5","fa6","fa7"
    );
    g_isr_fp_sink[hartid] = f2u(d + e);
}

/* Foreground hard-float kernel that holds live values across caller-saved FP regs! */
__attribute__((noinline))
static uint32_t run_saxpy_kernel(void) {
    float x[SAXPY_N];
    float y[SAXPY_N];
    float a = 1.375f;
    for (uint32_t i = 0u; i < SAXPY_N; i++) {
        x[i] = (float)((int32_t)(i * 3u) - 40) * 0.125f;
        y[i] = (float)((int32_t)(i * 5u) - 70) * 0.25f;
    }
    uint32_t h = 0x12345678u;
    for (uint32_t i = 0u; i < SAXPY_N; i++) {
        float r = hw_fmadd_dyn(a, x[i], y[i]);
        y[i] = r;
        h = mix_hash(h, f2u(r));
    }
    return h;
}

/* Multi-register live FP chain designed to span ft0..ft11 & fa0..fa7 during preemption */
__attribute__((noinline))
static uint32_t run_live_fp_chain(uint32_t seed) {
    float v0 = (float)(int32_t)(seed + 1u) * 0.5f;
    float v1 = (float)(int32_t)(seed + 2u) * 0.25f;
    float v2 = (float)(int32_t)(seed + 3u) * 0.125f;
    float v3 = 1.75f;
    float v4 = -0.625f;
    float v5 = 3.125f;
    float v6 = 0.875f;
    float v7 = -2.25f;

    for (uint32_t k = 0u; k < 24u; k++) {
        float s0 = hw_fmadd_dyn(v0, 0.75f, v1);
        float s1 = hw_fmadd_dyn(v2, -0.5f, v3);
        float s2 = hw_fmadd_dyn(v4, 0.25f, v5);
        float s3 = hw_fmadd_dyn(v6, 0.125f, v7);
        float d0 = hw_fdiv_dyn(s0 + 16.0f, s1 + 32.0f);
        float q0 = hw_fsqrt(s2 * s2 + 1.0f);
        v0 = s0 * 0.5f + d0;
        v1 = s1 * 0.5f - q0;
        v2 = s2 * 0.25f + v0;
        v3 = s3 * 0.25f - v1;
        v4 = hw_fmadd_dyn(d0, q0, v4 * 0.5f);
        v5 = hw_fmadd_dyn(q0, 0.25f, v5 * 0.5f);
        v6 = v6 * 0.75f + d0;
        v7 = v7 * 0.75f - q0;
    }

    uint32_t h = 0xA5A55A5Au;
    h = mix_hash(h, f2u(v0));
    h = mix_hash(h, f2u(v1));
    h = mix_hash(h, f2u(v2));
    h = mix_hash(h, f2u(v3));
    h = mix_hash(h, f2u(v4));
    h = mix_hash(h, f2u(v5));
    h = mix_hash(h, f2u(v6));
    h = mix_hash(h, f2u(v7));
    return h;
}

uint32_t main(uint32_t hartid) {
    uint32_t ncores = mmio_read32(SOC_CTRL_BASE + SOC_CTRL_CORES_OFF);
    uint32_t local_sense = 0u;

    /* 1. Verify SOC_FEAT_FPU & misa.F */
    uint32_t feat = mmio_read32(SOC_CTRL_BASE + SOC_CTRL_FEATURES_OFF);
    if ((feat & SOC_FEAT_FPU) == 0u) {
        soc_halt(0xBAD60001u);
    }
    if ((read_misa() & (1u << 5)) == 0u) {
        soc_halt(0xBAD60002u);
    }

    if (hartid == 0u) {
        g_fs_off_traps[0] = 0u;
        g_fs_off_traps[1] = 0u;
        g_fp_irq_ticks[0] = 0u;
        g_fp_irq_ticks[1] = 0u;
        g_c1_done         = 0u;
        hal_uart_init(4u, 1u, 0u);
    }
    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    /* 2. SAXPY (hardware fmadd.s) */
    write_frm(0u);
    write_fflags(0u);
    uint32_t saxpy_h = run_saxpy_kernel();

    /* 3. Hardware FSQRT.S + IEEE-754 sticky fflags (NX, NV, DZ) */
    static const uint32_t sqrt_inputs[8] = {
        0x00000000u, /* +0.0 */
        0x80000000u, /* -0.0 */
        0x3F800000u, /* 1.0 */
        0x40100000u, /* 2.25 -> 1.5 exact */
        0x40000000u, /* 2.0 -> 0x3FB504F3 (NX) */
        0x40400000u, /* 3.0 -> 0x3FDDB3D7 (NX) */
        0x00000004u, /* subnormal 2^-147 -> exact normal! */
        0x461C4000u, /* 10000.0 -> 100.0 exact */
    };
    write_fflags(0u);
    uint32_t sqrt_h = 0x33445566u;
    for (uint32_t i = 0u; i < 8u; i++) {
        float y = hw_fsqrt(u2f(sqrt_inputs[i]));
        sqrt_h = mix_hash(sqrt_h, f2u(y));
    }
    if (read_fflags() != 0x01u) { /* Only NX must be raised! */
        soc_halt(0xBAD60010u | read_fflags());
    }
    /* sqrt(-1.0) -> canonical qNaN + NV (0x10) */
    write_fflags(0u);
    float qn = hw_fsqrt(-1.0f);
    if (f2u(qn) != 0x7FC00000u || read_fflags() != 0x10u) {
        soc_halt(0xBAD60011u);
    }
    /* 1.0 / 0.0 -> +Inf + DZ (0x08) */
    write_fflags(0u);
    float pinf = hw_fdiv_dyn(1.0f, 0.0f);
    if (f2u(pinf) != 0x7F800000u || read_fflags() != 0x08u) {
        soc_halt(0xBAD60012u);
    }

    /* 4. Dynamic Rounding Modes (frm = 0..4 -> proves csr_sync flush!) */
    static const uint32_t exp_pos_third[5] = {
        0x3EAAAAABu, 0x3EAAAAAAu, 0x3EAAAAAAu, 0x3EAAAAABu, 0x3EAAAAABu
    };
    static const uint32_t exp_neg_third[5] = {
        0xBEAAAAABu, 0xBEAAAAAAu, 0xBEAAAAABu, 0xBEAAAAAAu, 0xBEAAAAABu
    };
    static const int32_t exp_cvt_pos[5] = { 2, 2, 2, 3, 3 };
    static const int32_t exp_cvt_neg[5] = { -2, -2, -3, -2, -3 };

    for (uint32_t rm = 0u; rm < 5u; rm++) {
        write_frm(rm);
        float p = hw_fdiv_dyn(1.0f, 3.0f);
        float n = hw_fdiv_dyn(-1.0f, 3.0f);
        int32_t cp = hw_fcvt_ws_dyn(2.5f);
        int32_t cn = hw_fcvt_ws_dyn(-2.5f);
        if (f2u(p) != exp_pos_third[rm] || f2u(n) != exp_neg_third[rm] ||
            cp != exp_cvt_pos[rm] || cn != exp_cvt_neg[rm]) {
            soc_halt(0xBAD60020u | rm);
        }
    }
    write_frm(0u); /* Back to RNE */
    write_fflags(0u);

    /* 5. mstatus.FS = Off Illegal-Instruction Trap & C Handler Recovery */
    hal_irq_init(hartid);
    hal_irq_register_exc(hartid, fs_off_exc_handler);
    hal_irq_register_timer(hartid, fp_clobber_mti_isr);
    hal_irq_set_mtvec_vectored();

    uint32_t fs_mask = 0x6000u;
    __asm__ volatile ("csrc mstatus, %0" :: "r"(fs_mask) : "memory");
    /* This fadd.s MUST trap (mcause=2) without double-faulting in trap.S! */
    __asm__ volatile ("fadd.s ft0, ft1, ft2" ::: "ft0", "memory");

    if (g_fs_off_traps[hartid] != 1u) {
        soc_halt(0xBAD60030u | g_fs_off_traps[hartid]);
    }
    /* Verify FP resumes cleanly right after handler restored mstatus.FS */
    if (f2u(hw_fmadd_dyn(2.0f, 3.0f, 4.0f)) != 0x41200000u) {
        soc_halt(0xBAD60031u);
    }

    /* 6. Live FP Compute Preempted by CLINT Timer C ISR (trap.S FP Save/Restore!) */
    write_frm(0u);
    write_fflags(0u);
    uint32_t golden_chain = run_live_fp_chain(hartid + 5u);

    hal_clint_set_alarm_rel(hartid, 250u);
    hal_irq_enable_mie(MIE_MTIE);
    hal_irq_global_enable();

    uint32_t iters = 0u;
    while (g_fp_irq_ticks[hartid] < TARGET_FP_IRQS) {
        uint32_t got = run_live_fp_chain(hartid + 5u);
        if (got != golden_chain) {
            soc_halt(0xBAD60040u | (hartid << 8));
        }
        if (read_frm() != 0u) {
            soc_halt(0xBAD60041u | (hartid << 8));
        }
        iters++;
    }

    hal_irq_global_disable();
    hal_clint_disable_timer(hartid);
    if (iters == 0u || g_fp_irq_ticks[hartid] < TARGET_FP_IRQS) {
        soc_halt(0xBAD60042u);
    }

    if (ncores == 2u) {
        barrier_wait(&g_bar, 2u, &local_sense);
    }

    if (hartid == 0u) {
        hal_uart_printf("FP_U6_WORKLOADS: cores=%u saxpy=0x%08x sqrt=0x%08x chain0=0x%08x fs_trap=%u irqs=%u/%u\n",
                        ncores, saxpy_h, sqrt_h, golden_chain,
                        g_fs_off_traps[0], g_fp_irq_ticks[0], g_fp_irq_ticks[1]);
        hal_uart_tx_flush();
    }

    uint32_t code = 0xF6000000u | (ncores << 16) | ((saxpy_h ^ sqrt_h) & 0xFFFFu);
    if (ncores == 2u) {
        if (hartid == 1u) {
            soc_park_secondary(code | 0x00100000u, &g_c1_done);
        } else {
            while (!g_c1_done) {}
            soc_halt(code);
        }
    } else {
        soc_halt(code);
    }
    return code;
}
