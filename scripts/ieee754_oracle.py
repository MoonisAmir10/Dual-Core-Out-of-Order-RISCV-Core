#!/usr/bin/env python3
"""
Exact Mathematical IEEE-754-2008 / RISC-V RV32F Reference Oracle (Phase U5)

Uses Python arbitrary-precision `fractions.Fraction` and `math.isqrt` — ZERO
reliance on host CPU floating-point rounding modes, FMA quirks, or host tininess
detection! Implements exact RISC-V Unprivileged ISA v2.2 (F Extension) semantics:
  - Canonical quiet NaN: 0x7FC0_0000
  - Tininess detected AFTER rounding (exact IEEE-754-2008 §7.5)
  - FMA(0, Inf, qNaN) raises NV = 1 (RISC-V spec requirement)
  - FMIN/FMAX: -0.0 < +0.0, sNaN sets NV only
  - FCVT.W/WU.S: rounds BEFORE range check; NaN clamps to INT_MAX / UINT_MAX + NV
  - All 5 rounding modes: RNE(0), RTZ(1), RDN(2), RUP(3), RMM(4)
  - All 5 fflags: NV(bit4), DZ(bit3), OF(bit2), UF(bit1), NX(bit0)

Reusable by Phase U5 standalone FPU testbench AND Phase U6 RV32IMAF golden ISS!
"""

from __future__ import annotations

import math
from fractions import Fraction
from typing import Tuple

# Rounding Modes
RM_RNE = 0
RM_RTZ = 1
RM_RDN = 2
RM_RUP = 3
RM_RMM = 4

# fflags bit masks
FLAG_NX = 1 << 0
FLAG_UF = 1 << 1
FLAG_OF = 1 << 2
FLAG_DZ = 1 << 3
FLAG_NV = 1 << 4

CANON_QNAN = 0x7FC0_0000
POS_INF = 0x7F80_0000
NEG_INF = 0xFF80_0000
POS_MAX = 0x7F7F_FFFF
NEG_MAX = 0xFF7F_FFFF

# FMA Ops
FMA_FMADD = 0
FMA_FMSUB = 1
FMA_FNMSUB = 2
FMA_FNMADD = 3
FMA_FADD = 4
FMA_FSUB = 5
FMA_FMUL = 6

# DIV/SQRT Ops
FDIV_DIV = 0
FDIV_SQRT = 1

# MISC Ops
FMISC_SGNJ = 0
FMISC_SGNJN = 1
FMISC_SGNJX = 2
FMISC_MIN = 3
FMISC_MAX = 4
FMISC_EQ = 5
FMISC_LT = 6
FMISC_LE = 7
FMISC_CLASS = 8
FMISC_CVT_WS = 9
FMISC_CVT_WUS = 10
FMISC_CVT_SW = 11
FMISC_CVT_SWU = 12
FMISC_MV_XW = 13
FMISC_MV_WX = 14


def unpack_f32(u: int) -> Tuple[int, int, int]:
    u &= 0xFFFF_FFFF
    return (u >> 31) & 1, (u >> 23) & 0xFF, u & 0x7F_FFFF


def classify_f32(u: int) -> str:
    _, e, f = unpack_f32(u)
    if e == 0xFF:
        if f == 0:
            return "inf"
        return "qnan" if (f & 0x40_0000) else "snan"
    if e == 0:
        return "zero" if f == 0 else "sub"
    return "norm"


def f32_to_fraction(u: int) -> Fraction:
    s, e, f = unpack_f32(u)
    if e == 0:
        if f == 0:
            return Fraction(0)
        val = Fraction(f, 1 << 149)
    else:
        val = Fraction((1 << 23) + f) * (Fraction(1 << (e - 150)) if e >= 150 else Fraction(1, 1 << (150 - e)))
    return -val if s else val


def round_nonneg_fraction_to_int(x: Fraction, sign: int, rm: int) -> Tuple[int, bool]:
    """Exact rounding of a non-negative rational magnitude x >= 0 with sign bit (0=+, 1=-)."""
    n, d = x.numerator, x.denominator
    q, r = divmod(n, d)
    if r == 0:
        return q, False
    twice_r = r << 1
    if rm == RM_RNE:
        inc = (twice_r > d) or (twice_r == d and (q & 1) == 1)
    elif rm == RM_RTZ:
        inc = False
    elif rm == RM_RDN:
        inc = (sign == 1)
    elif rm == RM_RUP:
        inc = (sign == 0)
    elif rm == RM_RMM:
        inc = (twice_r >= d)
    else:
        raise ValueError(f"Invalid rm={rm}")
    return q + (1 if inc else 0), True


def floor_log2_fraction(x: Fraction) -> int:
    """Exact integer e such that 2^e <= x < 2^(e+1) for rational x > 0."""
    n, d = x.numerator, x.denominator
    e = n.bit_length() - d.bit_length()
    if e >= 0:
        if n < (d << e):
            e -= 1
    else:
        if (n << (-e)) < d:
            e -= 1
    return e


def pack_exact_nonzero_f32(val: Fraction, sign: int, rm: int) -> Tuple[int, int]:
    """
    Round exact non-zero rational `val` (magnitude |val| > 0, sign in {0,1}) to IEEE-754
    binary32 with EXACT RISC-V tininess-after-rounding and exact fflags!
    """
    mag = -val if val < 0 else val
    assert mag > 0
    e = floor_log2_fraction(mag)

    # 1. Compute unbounded-exponent rounded magnitude (for exact tininess-after-rounding!)
    # Scaled by 2^(23 - e) so significand integer is in [2^23, 2^24)
    shift_unb = 23 - e
    scaled_unb = mag * (1 << shift_unb) if shift_unb >= 0 else mag / (1 << (-shift_unb))
    q_unb, inexact_unb = round_nonneg_fraction_to_int(scaled_unb, sign, rm)
    e_unb = e
    if q_unb == (1 << 24):
        q_unb >>= 1
        e_unb += 1

    tiny_after = (e_unb < -126)

    # 2. Subnormal vs. Normal packing
    if e < -126:
        # Subnormal binary point is fixed at 2^-149
        scaled_sub = mag * (1 << 149)
        q_sub, inexact_sub = round_nonneg_fraction_to_int(scaled_sub, sign, rm)
        flags = 0
        if inexact_sub:
            flags |= FLAG_NX
            if tiny_after:
                flags |= FLAG_UF
        if q_sub == (1 << 23):
            return (sign << 31) | (1 << 23), flags
        return (sign << 31) | q_sub, flags

    # Normal range (e >= -126)
    if e_unb >= 128:
        flags = FLAG_OF | FLAG_NX
        if rm in (RM_RNE, RM_RMM):
            to_inf = True
        elif rm == RM_RTZ:
            to_inf = False
        elif rm == RM_RDN:
            to_inf = (sign == 1)
        elif rm == RM_RUP:
            to_inf = (sign == 0)
        else:
            raise ValueError(f"Invalid rm={rm}")
        res = ((sign << 31) | POS_INF) if to_inf else ((sign << 31) | POS_MAX)
        return res, flags

    flags = FLAG_NX if inexact_unb else 0
    return (sign << 31) | ((e_unb + 127) << 23) | (q_unb & 0x7F_FFFF), flags


def eval_fma(op: int, a: int, b: int, c: int, rm: int) -> Tuple[int, int]:
    """Exact IEEE-754-2008 / RISC-V FMADD/FMSUB/FNMSUB/FNMADD/FADD/FSUB/FMUL."""
    a &= 0xFFFF_FFFF
    b &= 0xFFFF_FFFF
    c &= 0xFFFF_FFFF

    if op == FMA_FADD:
        oa, ob, oc, neg_p, neg_c = a, 0x3F80_0000, b, 0, 0
    elif op == FMA_FSUB:
        oa, ob, oc, neg_p, neg_c = a, 0x3F80_0000, b, 0, 1
    elif op == FMA_FMUL:
        ps = ((a >> 31) ^ (b >> 31)) & 1
        oa, ob, oc, neg_p, neg_c = a, b, (ps << 31), 0, 0
    elif op == FMA_FMADD:
        oa, ob, oc, neg_p, neg_c = a, b, c, 0, 0
    elif op == FMA_FMSUB:
        oa, ob, oc, neg_p, neg_c = a, b, c, 0, 1
    elif op == FMA_FNMSUB:
        oa, ob, oc, neg_p, neg_c = a, b, c, 1, 0
    elif op == FMA_FNMADD:
        oa, ob, oc, neg_p, neg_c = a, b, c, 1, 1
    else:
        raise ValueError(f"Unknown FMA op={op}")

    sa, _, _ = unpack_f32(oa)
    sb, _, _ = unpack_f32(ob)
    sc, _, _ = unpack_f32(oc)
    ca, cb, cc = classify_f32(oa), classify_f32(ob), classify_f32(oc)

    p_sign = sa ^ sb ^ neg_p
    c_sign = sc ^ neg_c

    any_nan = any(k in ("snan", "qnan") for k in (ca, cb, cc))
    any_snan = any(k == "snan" for k in (ca, cb, cc))
    mul_zero_inf = (ca == "zero" and cb == "inf") or (ca == "inf" and cb == "zero")

    if any_nan or mul_zero_inf:
        return CANON_QNAN, (FLAG_NV if (any_snan or mul_zero_inf) else 0)

    p_inf = (ca == "inf" or cb == "inf")
    c_inf = (cc == "inf")
    if p_inf and c_inf:
        if p_sign != c_sign:
            return CANON_QNAN, FLAG_NV
        return (p_sign << 31) | POS_INF, 0
    if p_inf:
        return (p_sign << 31) | POS_INF, 0
    if c_inf:
        return (c_sign << 31) | POS_INF, 0

    p_zero = (ca == "zero" or cb == "zero")
    c_zero = (cc == "zero")
    if p_zero and c_zero:
        zsign = p_sign if (p_sign == c_sign) else (1 if rm == RM_RDN else 0)
        return (zsign << 31), 0

    fa = f32_to_fraction(oa)
    fb = f32_to_fraction(ob)
    fc = f32_to_fraction(oc)
    prod = -(fa * fb) if neg_p else (fa * fb)
    addend = -fc if neg_c else fc
    exact = prod + addend

    if exact == 0:
        # Exact cancellation of two non-zero terms
        return ((1 if rm == RM_RDN else 0) << 31), 0

    res_sign = 1 if exact < 0 else 0
    return pack_exact_nonzero_f32(exact, res_sign, rm)


def eval_div_sqrt(op: int, a: int, b: int, rm: int) -> Tuple[int, int]:
    """Exact IEEE-754-2008 / RISC-V FDIV.S & FSQRT.S."""
    a &= 0xFFFF_FFFF
    b &= 0xFFFF_FFFF
    sa, ea, fa = unpack_f32(a)
    sb, _, _ = unpack_f32(b)
    ca, cb = classify_f32(a), classify_f32(b)

    if op == FDIV_SQRT:
        if ca in ("snan", "qnan"):
            return CANON_QNAN, (FLAG_NV if ca == "snan" else 0)
        if ca == "zero":
            return (sa << 31), 0
        if sa == 1:
            return CANON_QNAN, FLAG_NV
        if ca == "inf":
            return POS_INF, 0

        # Exact integer square root at scale 2^-24 relative to normalized exponent!
        # Value a = sig * 2^(exp - 23), sig in [2^23, 2^24)
        if ea == 0:
            lz = 24 - fa.bit_length()
            sig = fa << lz
            exp = -126 - lz
        else:
            sig = (1 << 23) + fa
            exp = ea - 127

        if exp & 1:
            q_exp = (exp - 1) >> 1
            rad = sig << 26  # M * 2^48 where M in [2, 4)
        else:
            q_exp = exp >> 1
            rad = sig << 25  # M * 2^48 where M in [1, 2)

        q_int = math.isqrt(rad)  # Exact 25-bit floor(sqrt(M) * 2^24)
        rem = rad - q_int * q_int
        sig24 = q_int >> 1
        rb = q_int & 1
        sb_bit = 1 if rem != 0 else 0
        inexact = (rb != 0) or (sb_bit != 0)

        if rm == RM_RNE:
            inc = (rb == 1) and (sb_bit == 1 or (sig24 & 1) == 1)
        elif rm == RM_RTZ:
            inc = False
        elif rm == RM_RDN:
            inc = False  # sqrt result is always positive
        elif rm == RM_RUP:
            inc = inexact
        elif rm == RM_RMM:
            inc = (rb == 1)
        else:
            raise ValueError(f"Invalid rm={rm}")

        sig_r = sig24 + (1 if inc else 0)
        if sig_r == (1 << 24):
            sig_r >>= 1
            q_exp += 1
        return ((q_exp + 127) << 23) | (sig_r & 0x7F_FFFF), (FLAG_NX if inexact else 0)

    elif op == FDIV_DIV:
        res_sign = sa ^ sb
        if ca in ("snan", "qnan") or cb in ("snan", "qnan"):
            return CANON_QNAN, (FLAG_NV if (ca == "snan" or cb == "snan") else 0)
        if (ca == "inf" and cb == "inf") or (ca == "zero" and cb == "zero"):
            return CANON_QNAN, FLAG_NV
        if cb == "zero":
            return (res_sign << 31) | POS_INF, FLAG_DZ
        if ca == "inf":
            return (res_sign << 31) | POS_INF, 0
        if ca == "zero" or cb == "inf":
            return (res_sign << 31), 0

        exact = f32_to_fraction(a) / f32_to_fraction(b)
        return pack_exact_nonzero_f32(exact, res_sign, rm)
    else:
        raise ValueError(f"Unknown div_sqrt op={op}")


def eval_misc(op: int, a: int, b: int, rm: int) -> Tuple[int, int]:
    """Exact IEEE-754-2008 / RISC-V FSGNJ/FMIN/FMAX/FEQ/FLT/FLE/FCLASS/FCVT/FMV."""
    a &= 0xFFFF_FFFF
    b &= 0xFFFF_FFFF
    sa, _, _ = unpack_f32(a)
    sb, _, _ = unpack_f32(b)
    ca, cb = classify_f32(a), classify_f32(b)
    a_nan = ca in ("snan", "qnan")
    b_nan = cb in ("snan", "qnan")
    a_snan = ca == "snan"
    b_snan = cb == "snan"

    if op == FMISC_SGNJ:
        return ((sb << 31) | (a & 0x7FFF_FFFF)), 0
    if op == FMISC_SGNJN:
        return (((sb ^ 1) << 31) | (a & 0x7FFF_FFFF)), 0
    if op == FMISC_SGNJX:
        return (((sa ^ sb) << 31) | (a & 0x7FFF_FFFF)), 0
    if op in (FMISC_MV_XW, FMISC_MV_WX):
        return a, 0

    if op in (FMISC_MIN, FMISC_MAX):
        flg = FLAG_NV if (a_snan or b_snan) else 0
        if a_nan and b_nan:
            return CANON_QNAN, flg
        if a_nan:
            return b, flg
        if b_nan:
            return a, flg
        if ca == "zero" and cb == "zero":
            a_lt = (sa == 1 and sb == 0)
        else:
            fa_v = Fraction(-1 if sa else 1) * (Fraction(1 << 200) if ca == "inf" else abs(f32_to_fraction(a)))
            fb_v = Fraction(-1 if sb else 1) * (Fraction(1 << 200) if cb == "inf" else abs(f32_to_fraction(b)))
            a_lt = fa_v < fb_v
        if op == FMISC_MIN:
            return (a if a_lt else b), flg
        return (b if a_lt else a), flg

    if op == FMISC_EQ:
        flg = FLAG_NV if (a_snan or b_snan) else 0
        if a_nan or b_nan:
            return 0, flg
        if ca == "zero" and cb == "zero":
            return 1, flg
        return (1 if a == b else 0), flg

    if op in (FMISC_LT, FMISC_LE):
        flg = FLAG_NV if (a_nan or b_nan) else 0
        if a_nan or b_nan:
            return 0, flg
        if ca == "zero" and cb == "zero":
            return (1 if op == FMISC_LE else 0), flg
        fa_v = Fraction(-1 if sa else 1) * (Fraction(1 << 200) if ca == "inf" else abs(f32_to_fraction(a)))
        fb_v = Fraction(-1 if sb else 1) * (Fraction(1 << 200) if cb == "inf" else abs(f32_to_fraction(b)))
        res = (fa_v <= fb_v) if op == FMISC_LE else (fa_v < fb_v)
        return (1 if res else 0), flg

    if op == FMISC_CLASS:
        bit_map = {
            (1, "inf"): 0, (1, "norm"): 1, (1, "sub"): 2, (1, "zero"): 3,
            (0, "zero"): 4, (0, "sub"): 5, (0, "norm"): 6, (0, "inf"): 7,
        }
        if ca == "snan":
            return 1 << 8, 0
        if ca == "qnan":
            return 1 << 9, 0
        return 1 << bit_map[(sa, ca)], 0

    if op == FMISC_CVT_WS:
        if a_nan:
            return 0x7FFF_FFFF, FLAG_NV
        if ca == "inf":
            return (0x8000_0000 if sa else 0x7FFF_FFFF), FLAG_NV
        fv = f32_to_fraction(a)
        q, inexact = round_nonneg_fraction_to_int(abs(fv), sa, rm)
        signed_q = -q if sa else q
        if signed_q > 0x7FFF_FFFF:
            return 0x7FFF_FFFF, FLAG_NV
        if signed_q < -0x8000_0000:
            return 0x8000_0000, FLAG_NV
        return (signed_q & 0xFFFF_FFFF), (FLAG_NX if inexact else 0)

    if op == FMISC_CVT_WUS:
        if a_nan:
            return 0xFFFF_FFFF, FLAG_NV
        if ca == "inf":
            return (0 if sa else 0xFFFF_FFFF), FLAG_NV
        fv = f32_to_fraction(a)
        q, inexact = round_nonneg_fraction_to_int(abs(fv), sa, rm)
        signed_q = -q if sa else q
        if signed_q < 0:
            return 0, FLAG_NV
        if signed_q > 0xFFFF_FFFF:
            return 0xFFFF_FFFF, FLAG_NV
        return (signed_q & 0xFFFF_FFFF), (FLAG_NX if inexact else 0)

    if op == FMISC_CVT_SW:
        sv = a - 0x1_0000_0000 if (a & 0x8000_0000) else a
        if sv == 0:
            return 0, 0
        return pack_exact_nonzero_f32(Fraction(sv), 1 if sv < 0 else 0, rm)

    if op == FMISC_CVT_SWU:
        if a == 0:
            return 0, 0
        return pack_exact_nonzero_f32(Fraction(a), 0, rm)

    raise ValueError(f"Unknown misc op={op}")


def self_test() -> None:
    """Verify notorious IEEE-754-2008 & RISC-V F edge cases inside the oracle."""
    # 1. Tininess AFTER rounding vs BEFORE rounding!
    # 1a. a = 0x0080_0000 (2^-126), b = 0x3F7F_FFFF (1 - 2^-24) -> exact = (2^24 - 1)*2^-150
    #     Unbounded: EXACT (2^24 - 1)*2^-150 < 2^-126 -> TINY AFTER ROUNDING!
    #     Bounded subnormal: (2^23 - 0.5)*2^-149 ties to even -> 0x0080_0000, UF=1, NX=1!
    res, flg = eval_fma(FMA_FMUL, 0x0080_0000, 0x3F7F_FFFF, 0, RM_RNE)
    assert res == 0x0080_0000 and flg == (FLAG_UF | FLAG_NX), f"Tiny-after subnormal round-up failed: {res:#x} {flg:#x}"

    # 1b. (2^24 - 0.5) * 2^-150 = 2^-126 - 2^-151 = fmadd(-2^-126, 2^-25, 2^-126)
    #     Unbounded RNE: tie between odd (2^24 - 1) and even 2^24 -> rounds UP to 2^-126 -> NOT tiny!
    #     Result = 0x0080_0000, UF=0, NX=1!
    res, flg = eval_fma(FMA_FMADD, 0x8080_0000, 0x3300_0000, 0x0080_0000, RM_RNE)
    assert res == 0x0080_0000 and flg == FLAG_NX, f"Not-tiny-after tie failed: {res:#x} {flg:#x}"

    # 1c. (2^24 - 0.75) * 2^-150 = 2^-126 - 1.5*2^-151 = fmadd(-1.5*2^-126, 2^-25, 2^-126)
    #     Unbounded RNE: rounds DOWN to (2^24 - 1)*2^-150 < 2^-126 -> TINY AFTER!
    #     Bounded subnormal: (2^23 - 0.375)*2^-149 rounds UP to 0x0080_0000 -> UF=1, NX=1!
    res, flg = eval_fma(FMA_FMADD, 0x80C0_0000, 0x3300_0000, 0x0080_0000, RM_RNE)
    assert res == 0x0080_0000 and flg == (FLAG_UF | FLAG_NX), f"Tiny-after classic case failed: {res:#x} {flg:#x}"

    # 2. RISC-V FMA(0, Inf, qNaN) MUST raise NV=1!
    res, flg = eval_fma(FMA_FMADD, 0x0000_0000, POS_INF, CANON_QNAN, RM_RNE)
    assert res == CANON_QNAN and flg == FLAG_NV

    # 3. FCVT.WU.S(-0.3) rounds BEFORE range check!
    neg_0p3 = 0xBE99_999A
    assert eval_misc(FMISC_CVT_WUS, neg_0p3, 0, RM_RNE) == (0, FLAG_NX)
    assert eval_misc(FMISC_CVT_WUS, neg_0p3, 0, RM_RTZ) == (0, FLAG_NX)
    assert eval_misc(FMISC_CVT_WUS, neg_0p3, 0, RM_RUP) == (0, FLAG_NX)
    assert eval_misc(FMISC_CVT_WUS, neg_0p3, 0, RM_RDN) == (0, FLAG_NV)
    assert eval_misc(FMISC_CVT_WUS, 0xBF00_0000, 0, RM_RNE) == (0, FLAG_NX)  # -0.5 ties to even 0!
    assert eval_misc(FMISC_CVT_WUS, 0xBF00_0000, 0, RM_RMM) == (0, FLAG_NV)  # -0.5 ties away to -1!

    # 4. FMIN/FMAX signed zero & NaN
    assert eval_misc(FMISC_MIN, 0x0000_0000, 0x8000_0000, RM_RNE) == (0x8000_0000, 0)
    assert eval_misc(FMISC_MAX, 0x8000_0000, 0x0000_0000, RM_RNE) == (0x0000_0000, 0)
    assert eval_misc(FMISC_MIN, CANON_QNAN, 0x3F80_0000, RM_RNE) == (0x3F80_0000, 0)
    assert eval_misc(FMISC_MIN, 0x7F80_0001, 0x3F80_0000, RM_RNE) == (0x3F80_0000, FLAG_NV)

    # 5. FSQRT(-0.0) == -0.0, FSQRT(-1.0) == qNaN + NV
    assert eval_div_sqrt(FDIV_SQRT, 0x8000_0000, 0, RM_RNE) == (0x8000_0000, 0)
    assert eval_div_sqrt(FDIV_SQRT, 0xBF80_0000, 0, RM_RNE) == (CANON_QNAN, FLAG_NV)
    assert eval_div_sqrt(FDIV_SQRT, 0x4010_0000, 0, RM_RNE) == (0x3FC0_0000, 0)  # sqrt(2.25) = 1.5 exact!

    print("PASS [ieee754_oracle --self-test]: All IEEE-754-2008 & RISC-V F corner invariants verified!")


if __name__ == "__main__":
    self_test()
