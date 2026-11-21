#!/usr/bin/env python3
"""
Phase U5 Standalone IEEE-754-2008 / RISC-V RV32F Verification Driver (56,000+ Vectors!)

Generates exhaustive directed corner-case + randomized test vectors computed by
scripts/ieee754_oracle.py (exact Python fractions.Fraction & math.isqrt — zero host FPU
dependencies!) across ALL 24 RV32F arithmetic/div/sqrt/misc/compare/convert operations
and ALL 5 IEEE-754 rounding modes, then runs:
  1. Verilator --assert -Wall (Vtb_fpu) on ALL 56,000+ vectors + OoO squash/backpressure!
  2. Icarus Verilog (tb_fpu_ieee754.vvp) dual-simulator cross-check on directed corner cases!
"""

from __future__ import annotations

import random
import subprocess
import sys
import time
from pathlib import Path
from typing import List

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

import ieee754_oracle as orc

BUILD = ROOT / "build" / "phase_u5"
VTB = ROOT / "build" / "v_tb_fpu" / "Vtb_fpu"
IVVP = ROOT / "build" / "tb_fpu_ieee754.vvp"
VVP = "/home/moonis/auto_job_seeker/.eda_tools/envs/dft/bin/vvp"

ROUNDING_MODES = [orc.RM_RNE, orc.RM_RTZ, orc.RM_RDN, orc.RM_RUP, orc.RM_RMM]

# High-value IEEE-754 binary32 corner operands
CORNER_FLOATS: List[int] = [
    0x0000_0000,  # +0.0
    0x8000_0000,  # -0.0
    0x0000_0001,  # +min subnormal (2^-149)
    0x8000_0001,  # -min subnormal
    0x0000_0002,  # +2 * 2^-149
    0x0040_0000,  # +0.5 * 2^-126 (half min normal)
    0x007F_FFFF,  # +max subnormal
    0x807F_FFFF,  # -max subnormal
    0x0080_0000,  # +min normal (2^-126)
    0x8080_0000,  # -min normal
    0x0080_0001,  # +min normal + 1 ULP
    0x0100_0000,  # 2^-125
    0x3300_0000,  # 2^-25
    0x3380_0000,  # 2^-24
    0x3400_0000,  # 2^-23
    0x3E80_0000,  # +0.25
    0xBE99_999A,  # -0.3
    0x3F00_0000,  # +0.5
    0xBF00_0000,  # -0.5
    0xBF40_0000,  # -0.75
    0x3F7F_FFFF,  # 1.0 - 2^-24
    0x3F80_0000,  # +1.0
    0xBF80_0000,  # -1.0
    0x3F80_0001,  # 1.0 + 2^-23
    0x3FC0_0000,  # +1.5
    0xBFC0_0000,  # -1.5
    0x4000_0000,  # +2.0
    0x4010_0000,  # +2.25 (exact square!)
    0x4040_0000,  # +3.0
    0x4B00_0000,  # 2^23 (exact integer boundary)
    0x4B80_0000,  # 2^24
    0x4EFF_FFFF,  # 2^31 - 128 (max int32 float!)
    0x4F00_0000,  # +2^31 (just out of int32!)
    0xCF00_0000,  # -2^31 (exact INT_MIN!)
    0xCF00_0001,  # -2^31 - 256 (just below INT_MIN!)
    0x4F7F_FFFF,  # 2^32 - 256 (max uint32 float!)
    0x4F80_0000,  # +2^32 (just out of uint32!)
    0x7F00_0000,  # 2^127
    0x7F7F_FFFF,  # +MAX_NORMAL
    0xFF7F_FFFF,  # -MAX_NORMAL
    0x7F80_0000,  # +Inf
    0xFF80_0000,  # -Inf
    0x7F80_0001,  # +sNaN (payload 1)
    0xFFBF_FFFF,  # -sNaN (max payload)
    0x7FC0_0000,  # +canonical qNaN
    0xFFC0_1234,  # -qNaN (custom payload)
]

CORNER_INTS: List[int] = [
    0x0000_0000, 0x0000_0001, 0xFFFF_FFFF, 0x0000_0002, 0xFFFF_FFFE,
    0x007F_FFFF, 0x0080_0000, 0x0080_0001, 0x0100_0000, 0x0100_0001,
    0x0100_0002, 0x0100_0003, 0x7FFF_FF80, 0x7FFF_FFBF, 0x7FFF_FFC0,
    0x7FFF_FFFF, 0x8000_0000, 0x8000_0001, 0x8000_0040, 0x8000_0080,
    0xFFFF_FF00, 0xFFFF_FF7F, 0xFFFF_FF80, 0xFFFF_FFBF, 0xFFFF_FFC0,
]


def rand_f32(rng: random.Random) -> int:
    """Weighted random float covering subnormals, tiny, normal, huge, ties, and specials."""
    bucket = rng.randrange(20)
    sign = rng.randrange(2) << 31
    if bucket == 0:
        return rng.choice(CORNER_FLOATS)
    if bucket <= 3:
        # Subnormal (e=0, non-zero frac)
        return sign | rng.randint(1, 0x7F_FFFF)
    if bucket <= 5:
        # Near underflow boundary (e in 1..4)
        e = rng.randint(1, 4)
        return sign | (e << 23) | rng.randrange(1 << 23)
    if bucket <= 7:
        # Near overflow boundary (e in 250..254)
        e = rng.randint(250, 254)
        return sign | (e << 23) | rng.randrange(1 << 23)
    if bucket <= 9:
        # Near integer / unit range (e in 124..160)
        e = rng.randint(124, 160)
        return sign | (e << 23) | rng.randrange(1 << 23)
    # Full arbitrary 32-bit pattern (normal/subnormal/inf/nan)
    return rng.getrandbits(32)


def generate_vectors(seed: int = 20261121) -> tuple[Path, Path, Path, dict[str, int]]:
    BUILD.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)

    fma_path = BUILD / "fma_vectors.txt"
    div_path = BUILD / "div_vectors.txt"
    misc_path = BUILD / "misc_vectors.txt"

    # ------------------------------------------------------------------
    # 1. FMA Vectors (FMADD, FMSUB, FNMSUB, FNMADD, FADD, FSUB, FMUL)
    # ------------------------------------------------------------------
    fma_lines: List[str] = []

    def add_fma(op: int, rm: int, a: int, b: int, c: int) -> None:
        res, flg = orc.eval_fma(op, a, b, c, rm)
        fma_lines.append(f"{op:x} {rm:x} {a:08x} {b:08x} {c:08x} {res:08x} {flg:02x}\n")

    # 1a. Directed notorious IEEE-754 / RISC-V corner cases across ALL 5 rounding modes!
    special_fma_triples = [
        # Tininess AFTER rounding vs BEFORE rounding!
        (orc.FMA_FMUL,   0x0080_0000, 0x3F7F_FFFF, 0x0000_0000),  # 2^-126 * (1 - 2^-24)
        (orc.FMA_FMADD,  0x80C0_0000, 0x3300_0000, 0x0080_0000),  # (2^24 - 0.75)*2^-150 -> UF=1, res=min_norm!
        (orc.FMA_FMADD,  0x8080_0000, 0x3300_0000, 0x0080_0000),  # (2^24 - 0.5)*2^-150 -> UF=0 in RNE!
        (orc.FMA_FMADD,  0x8040_0000, 0x3300_0000, 0x0080_0000),  # (2^24 - 0.25)*2^-150
        (orc.FMA_FMADD,  0x0080_0000, 0x3F00_0000, 0x0000_0000),  # Exact subnormal (UF=0, NX=0)
        (orc.FMA_FMUL,   0x0000_0001, 0x3F00_0000, 0x0000_0000),  # Exact 0.5 * MIN_SUBNORM tie!
        (orc.FMA_FMUL,   0x8000_0001, 0x3F00_0000, 0x0000_0000),  # -0.5 * MIN_SUBNORM tie!
        (orc.FMA_FMUL,   0x0000_0003, 0x3F00_0000, 0x0000_0000),  # 1.5 * MIN_SUBNORM tie to even!
        # RISC-V 0 * Inf + qNaN MUST raise NV=1!
        (orc.FMA_FMADD,  0x0000_0000, 0x7F80_0000, 0x7FC0_0000),
        (orc.FMA_FMADD,  0xFF80_0000, 0x8000_0000, 0xFFC0_1234),
        (orc.FMA_FNMADD, 0x0000_0000, 0x7F80_0000, 0x7FC0_0000),
        # Exact cancellation & single-rounding precision (would FAIL double rounding!)
        (orc.FMA_FMSUB,  0x3F80_0001, 0x3F80_0001, 0x3F80_0002),  # (1+2^-23)^2 - (1+2^-22) = 2^-46 exact!
        (orc.FMA_FMADD,  0x7F7F_FFFF, 0x4000_0000, 0xFF7F_FFFF),  # Overflow product saved by huge negative c!
        (orc.FMA_FMSUB,  0x7F7F_FFFF, 0x7F7F_FFFF, 0x7F7F_FFFF),  # Huge overflow
        (orc.FMA_FMADD,  0x0000_0001, 0x0000_0001, 0x3F80_0000),  # Tiny p added to 1.0 (c_exp >> p_exp!)
        (orc.FMA_FMSUB,  0x0000_0001, 0x0000_0001, 0x3F80_0000),  # Tiny p - 1.0 (sub with c_exp >> p_exp!)
        (orc.FMA_FNMSUB, 0x0000_0001, 0x0000_0001, 0x3F80_0000),  # 1.0 - tiny p (exact borrow at 1.0!)
        (orc.FMA_FNMSUB, 0x0000_0001, 0x0000_0001, 0x0000_0001),  # MIN_SUBNORM - tiny p!
        (orc.FMA_FMADD,  0x3F80_0000, 0x3F80_0000, 0x0000_0001),  # 1.0 + MIN_SUBNORM (p_exp >> c_exp!)
        (orc.FMA_FMSUB,  0x3F80_0000, 0x3F80_0000, 0x0000_0001),  # 1.0 - MIN_SUBNORM (p - tiny c sticky!)
        (orc.FMA_FMSUB,  0x3F80_0000, 0x3F80_0000, 0x3F80_0000),  # Exact zero cancellation!
        (orc.FMA_FMADD,  0x0000_0000, 0x3F80_0000, 0x8000_0000),  # +0 + (-0)
        (orc.FMA_FMADD,  0x8000_0000, 0x3F80_0000, 0x8000_0000),  # -0 + (-0)
        (orc.FMA_FMUL,   0x0000_0000, 0x3F80_0000, 0x0000_0000),  # +0 * +1 under RDN MUST be +0!
        (orc.FMA_FMUL,   0x8000_0000, 0x3F80_0000, 0x0000_0000),  # -0 * +1 under RUP MUST be -0!
        (orc.FMA_FADD,   0x0000_0000, 0x8000_0000, 0x0000_0000),  # +0 + -0
        (orc.FMA_FSUB,   0x0000_0000, 0x0000_0000, 0x0000_0000),  # +0 - +0
        (orc.FMA_FSUB,   0x8000_0000, 0x0000_0000, 0x0000_0000),  # -0 - +0
        (orc.FMA_FSUB,   0x8000_0000, 0x8000_0000, 0x0000_0000),  # -0 - -0
        (orc.FMA_FADD,   0x7F80_0000, 0xFF80_0000, 0x0000_0000),  # +Inf + -Inf
        (orc.FMA_FSUB,   0x7F80_0000, 0x7F80_0000, 0x0000_0000),  # +Inf - +Inf
    ]
    for op, a, b, c in special_fma_triples:
        for rm in ROUNDING_MODES:
            add_fma(op, rm, a, b, c)

    # 1b. Exhaustive corner grid for FADD, FSUB, FMUL (subset of key corners x 5 modes)
    key_c = [
        0x0000_0000, 0x8000_0000, 0x0000_0001, 0x8000_0001, 0x007F_FFFF, 0x0080_0000,
        0x3F80_0000, 0xBF80_0000, 0x3F7F_FFFF, 0x3F80_0001, 0x7F7F_FFFF, 0xFF7F_FFFF,
        0x7F80_0000, 0xFF80_0000, 0x7F80_0001, 0x7FC0_0000,
    ]
    for op in (orc.FMA_FADD, orc.FMA_FSUB, orc.FMA_FMUL):
        for a in key_c:
            for b in key_c:
                add_fma(op, ROUNDING_MODES[(a + b + op) % 5], a, b, 0)

    # 1c. Sweep every alignment shift sh in [-4 .. +80] for FMADD/FMSUB/FNMSUB/FNMADD!
    for sh in range(-4, 81):
        ea = 127
        eb = 127
        ec = max(1, min(254, 127 + 27 - sh))
        a = (ea << 23) | rng.randrange(1 << 23)
        b = (eb << 23) | rng.randrange(1 << 23)
        c = (ec << 23) | rng.randrange(1 << 23)
        for op in (orc.FMA_FMADD, orc.FMA_FMSUB, orc.FMA_FNMSUB, orc.FMA_FNMADD):
            for rm in ROUNDING_MODES:
                add_fma(op, rm, a, b, c)
                # Also test exact 1.0 * 1.0 ± c (borrow boundary!)
                add_fma(op, rm, 0x3F80_0000, 0x3F80_0000, c)

    # 1d. 22,000 weighted randomized FMA/FADD/FSUB/FMUL vectors across all 7 ops & 5 modes!
    for i in range(22000):
        op = i % 7
        rm = ROUNDING_MODES[rng.randrange(5)]
        a = rand_f32(rng)
        b = rand_f32(rng)
        c = rand_f32(rng) if op < 4 else 0
        # 20% of the time, craft close-exponent operands to stress cancellation & exact rounding!
        if op < 4 and (i % 5 == 0):
            sa = rng.randrange(2) << 31
            sb = rng.randrange(2) << 31
            sc = rng.randrange(2) << 31
            ea = rng.randint(60, 190)
            eb = rng.randint(60, 190)
            pe = ea + eb - 127
            ec = max(0, min(254, pe + rng.randint(-30, 30)))
            a = sa | (ea << 23) | rng.randrange(1 << 23)
            b = sb | (eb << 23) | rng.randrange(1 << 23)
            c = sc | (ec << 23) | rng.randrange(1 << 23)
        add_fma(op, rm, a, b, c)

    fma_path.write_text("".join(fma_lines), encoding="utf-8")

    # ------------------------------------------------------------------
    # 2. FDIV.S & FSQRT.S Vectors (14,000+ vectors!)
    # ------------------------------------------------------------------
    div_lines: List[str] = []

    def add_div(op: int, rm: int, a: int, b: int) -> None:
        res, flg = orc.eval_div_sqrt(op, a, b, rm)
        div_lines.append(f"{op:x} {rm:x} {a:08x} {b:08x} {res:08x} {flg:02x}\n")

    # 2a. Directed FDIV & FSQRT corner matrix
    for a in CORNER_FLOATS:
        for rm in ROUNDING_MODES:
            add_div(orc.FDIV_SQRT, rm, a, 0)
    for a in key_c:
        for b in key_c:
            for rm in ROUNDING_MODES:
                add_div(orc.FDIV_DIV, rm, a, b)

    # Tininess-after-rounding FDIV boundary!
    # 2^-126 / (1 + 2^-23) -> just below 2^-126!
    for b_div in (0x3F80_0001, 0x3F80_0002, 0x3F80_0003, 0x4000_0000, 0x4000_0001, 0x7F7F_FFFF):
        for rm in ROUNDING_MODES:
            add_div(orc.FDIV_DIV, rm, 0x0080_0000, b_div)
            add_div(orc.FDIV_DIV, rm, 0x0080_0001, b_div)
            add_div(orc.FDIV_DIV, rm, 0x0000_0001, b_div)

    # Exact square roots & adjacent ULPs (normal & subnormal, even & odd exponents!)
    for k in (1, 2, 3, 4, 9, 16, 25, 100, 0x100, 0x400000, 0x0080_0000):
        sq_f, _ = orc.eval_misc(orc.FMISC_CVT_SWU, k * k, 0, orc.RM_RNE)
        for delta in (-1, 0, 1):
            cand = (sq_f + delta) & 0x7FFF_FFFF
            for rm in ROUNDING_MODES:
                add_div(orc.FDIV_SQRT, rm, cand, 0)

    # 2b. 12,000 randomized FDIV & FSQRT vectors
    for i in range(12000):
        op = orc.FDIV_SQRT if (i % 3 == 0) else orc.FDIV_DIV
        rm = ROUNDING_MODES[rng.randrange(5)]
        a = rand_f32(rng)
        if op == orc.FDIV_SQRT and (i % 6 != 0):
            a &= 0x7FFF_FFFF  # mostly positive so we exercise full 25-bit radix-2 core!
        b = rand_f32(rng) if op == orc.FDIV_DIV else 0
        add_div(op, rm, a, b)

    div_path.write_text("".join(div_lines), encoding="utf-8")

    # ------------------------------------------------------------------
    # 3. FMISC / COMPARE / CONVERT Vectors (18,000+ vectors!)
    # ------------------------------------------------------------------
    misc_lines: List[str] = []

    def add_misc(op: int, rm: int, a: int, b: int) -> None:
        res, flg = orc.eval_misc(op, a, b, rm)
        misc_lines.append(f"{op:x} {rm:x} {a:08x} {b:08x} {res:08x} {flg:02x}\n")

    # 3a. Exhaustive corners for ALL 15 misc ops!
    for a in CORNER_FLOATS:
        add_misc(orc.FMISC_CLASS, 0, a, 0)
        add_misc(orc.FMISC_MV_XW, 0, a, 0)
        add_misc(orc.FMISC_MV_WX, 0, a, 0)
        for rm in ROUNDING_MODES:
            add_misc(orc.FMISC_CVT_WS, rm, a, 0)
            add_misc(orc.FMISC_CVT_WUS, rm, a, 0)
    for iv in CORNER_INTS:
        for rm in ROUNDING_MODES:
            add_misc(orc.FMISC_CVT_SW, rm, iv, 0)
            add_misc(orc.FMISC_CVT_SWU, rm, iv, 0)
    for op in (orc.FMISC_SGNJ, orc.FMISC_SGNJN, orc.FMISC_SGNJX,
               orc.FMISC_MIN, orc.FMISC_MAX, orc.FMISC_EQ, orc.FMISC_LT, orc.FMISC_LE):
        for a in key_c:
            for b in key_c:
                add_misc(op, 0, a, b)

    # 3b. 15,000 randomized vectors across all 15 misc/cmp/cvt ops & 5 modes
    for i in range(15000):
        op = i % 15
        rm = ROUNDING_MODES[rng.randrange(5)]
        if op in (orc.FMISC_CVT_SW, orc.FMISC_CVT_SWU):
            a = rng.getrandbits(32)
            b = 0
        else:
            a = rand_f32(rng)
            b = rand_f32(rng)
        add_misc(op, rm, a, b)

    misc_path.write_text("".join(misc_lines), encoding="utf-8")

    counts = {"fma": len(fma_lines), "div": len(div_lines), "misc": len(misc_lines)}
    return fma_path, div_path, misc_path, counts


def run_sim(cmd: List[str], tag: str) -> str:
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0 or tag not in res.stdout:
        print(f"FAIL running: {' '.join(cmd)}")
        print(res.stdout)
        print(res.stderr)
        sys.exit(1)
    for ln in res.stdout.splitlines():
        if tag in ln:
            return ln
    return res.stdout.strip()


def main() -> int:
    print("=== Phase U5 Verification: IEEE-754-2008 / RISC-V RV32F Standalone FPU Units ===")
    t0 = time.time()

    # 1. Self-test Python exact rational oracle
    orc.self_test()

    # 2. Generate 56,000+ directed & randomized IEEE-754 vectors
    fma_p, div_p, misc_p, cnts = generate_vectors()
    tot = cnts["fma"] + cnts["div"] + cnts["misc"]
    print(
        f"  [GEN] Generated {tot:,d} exact rational vectors in {time.time() - t0:.2f}s "
        f"(FMA={cnts['fma']:,d}, DIV/SQRT={cnts['div']:,d}, MISC/CMP/CVT={cnts['misc']:,d})"
    )

    # 3. Verilator --assert -Wall full sweep (ALL 56,000+ vectors + OoO squash/backpressure!)
    t1 = time.time()
    v_out = run_sim(
        [str(VTB), f"+FMA_VEC={fma_p}", f"+DIV_VEC={div_p}", f"+MISC_VEC={misc_p}"],
        "PASS [tb_fpu_ieee754]",
    )
    print(f"  [OK] Verilator Full Sweep ({time.time() - t1:.2f}s): {v_out}")

    # 4. Icarus Verilog dual-simulator cross-check (2,000 directed corner vectors per unit!)
    t2 = time.time()
    i_out = run_sim(
        [VVP, str(IVVP), f"+FMA_VEC={fma_p}", f"+DIV_VEC={div_p}", f"+MISC_VEC={misc_p}", "+MAX_VEC=2000"],
        "PASS [tb_fpu_ieee754]",
    )
    print(f"  [OK] Icarus Dual-Sim Cross-Check ({time.time() - t2:.2f}s): {i_out}")

    print(f"PASS [test_phase_u5_fpu]: All {tot:,d}+ IEEE-754-2008 & RISC-V RV32F vectors + OoO hazards PASSED (0 mismatches)!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
