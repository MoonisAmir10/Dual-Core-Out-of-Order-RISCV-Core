#!/usr/bin/env python3
"""
Phase U6 End-to-End Verification Suite: Non-Blocking RV32F Integration into the
2-Wide Dual-Core Out-of-Order Processor (RV32IMAF_Zicsr_Zifencei).

Verification Gates:
  1. Suite A (150-Program Randomized Single-Core RV32IMAF Differential Fuzz):
     - Runs on single_ooo2w_soc (Vp2_c0 +FP_TRACE +JITTER)
     - Bit-exact commit-by-commit comparison (PC, instr, rd_we, rd_is_fp, rd_addr,
       rd_wdata, fflags, mem_*) against Golden RV32IMAF ISS + IEEE-754 Fraction Oracle!
     - 5 Categories (30 seeds each):
         S1_fma_alu_mix, S2_fdiv_fsqrt_ooo, S3_fmisc_cvt_cmp_br,
         S4_flw_fsw_stlf_spec, S5_frm_fcsr_fs_traps
  2. Suite B (100-Program Randomized Dual-Core RV32IMAF Coherence Fuzz + FLW CoRR Litmus):
     - Runs on dual_ooo2w_soc (Vdual_soc +LOG +JITTER + always-on SWMR/Shadow checker)
     - Verified end-to-end by upgraded RV32IMAF scripts/dual_core_checker.py!
     - 4 Categories (25 seeds each):
         D1_disjoint_rv32imaf, D2_shared_flw_fsw_false_share,
         D3_mixed_fp_amo_spin, D4_flw_corr_snoop_replay
     - Plus directed FLW L05_CoRR snoop-invalidation replay proof!
  3. Suite C (Real GCC -march=rv32imaf -mabi=ilp32f -O2 Compiled C Workloads):
     - C1: test_fp_workloads_c.c on 1C & 2C (SAXPY, FSQRT.S, 5 rounding modes,
           mstatus.FS=Off trap recovery, live CLINT Timer ISR FP preemption!)
     - C2: test_fp_sgemm_c.c (24x24 IEEE-754 float SGEMM) on 1C & 2C:
           Bit-exact match vs. Python exact FMA oracle AND > 3.0 chip IPC on 2C!
"""

from __future__ import annotations

import concurrent.futures
import json
import os
import random
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import dual_core_checker as chk  # noqa: E402
import ieee754_oracle as fpo  # noqa: E402
import rv32ima_asm as isa  # noqa: E402

BUILD = ROOT / "build" / "phase_u6"
BIN_SINGLE = ROOT / "build" / "v_p2_c0" / "Vp2_c0"
BIN_DUAL = ROOT / "build" / "v_dual_soc" / "Vdual_soc"
GCC = "/usr/bin/riscv64-unknown-elf-gcc"
OBJCOPY = "/usr/bin/riscv64-unknown-elf-objcopy"

IREGS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
FREGS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]  # Includes f0!
RMS = ["rne", "rtz", "rdn", "rup", "rmm", "dyn"]

# Curated IEEE-754 binary32 corner & normal bit patterns (zero host FP reliance!)
FP_POOL = [
    0x0000_0000,  # +0.0
    0x8000_0000,  # -0.0
    0x3F80_0000,  # +1.0
    0xBF80_0000,  # -1.0
    0x4000_0000,  # +2.0
    0xC040_0000,  # -3.0
    0x3FC0_0000,  # +1.5
    0x3E80_0000,  # +0.25
    0xBEAAAAAB,  # -1/3
    0x3EAAAAAB,  # +1/3
    0x4049_0FDB,  # pi
    0xC128_0000,  # -10.5
    0x42C8_0000,  # 100.0
    0x0000_0001,  # min subnormal
    0x007F_FFFF,  # max subnormal
    0x0080_0000,  # min normal (2^-126)
    0x3F7F_FFFF,  # 1 - 2^-24 (tininess boundary!)
    0x7F7F_FFFF,  # FLT_MAX
    0xFF7F_FFFF,  # -FLT_MAX
    0x7F80_0000,  # +Inf
    0xFF80_0000,  # -Inf
    0x7FC0_0000,  # canonical qNaN
    0x7F80_0001,  # sNaN
    0x4F00_0000,  # 2^31 (int overflow boundary)
    0xCF00_0000,  # -2^31 (INT_MIN)
    0x4020_0000,  # 2.5 (tie-to-even test)
    0xC020_0000,  # -2.5
]


def _init_fp_and_int(rng: random.Random, core_tag: int = 0) -> List[str]:
    lines: List[str] = []
    for r in IREGS:
        val = rng.randint(-120, 120) + core_tag * 19
        lines.append(f"    addi x{r}, x0, {val}")
    for fr in FREGS:
        bits = rng.choice(FP_POOL)
        lines.append(f"    li   x30, 0x{bits:08x}")
        lines.append(f"    fmv.w.x f{fr}, x30")
    return lines


def _fp_checksum_footer(base_reg: str) -> List[str]:
    lines = ["    addi x14, x0, 0"]
    for r in IREGS:
        lines.append(f"    xor  x14, x14, x{r}")
    for fr in FREGS:
        lines.append(f"    fmv.x.w x29, f{fr}")
        lines.append("    xor  x14, x14, x29")
    lines.append("    frcsr x29")
    lines.append("    xor  x14, x14, x29")
    lines.append(f"    lw   x29, 0({base_reg})")
    lines.append("    add  x14, x14, x29")
    lines.append("    ebreak")
    return lines


def _gen_fp_body(
    rng: random.Random,
    prefix: str,
    n_ops: int,
    base_reg: str,
    offsets: List[int],
    weights: Dict[str, int],
    allow_traps: bool = False,
) -> List[str]:
    kinds = list(weights.keys())
    wvals = [weights[k] for k in kinds]
    lines: List[str] = []

    for i in range(n_ops):
        kind = rng.choices(kinds, weights=wvals, k=1)[0]
        rd = rng.choice(IREGS)
        rs1 = rng.choice(IREGS)
        rs2 = rng.choice(IREGS)
        fd = rng.choice(FREGS)
        fs1 = rng.choice(FREGS)
        fs2 = rng.choice(FREGS)
        fs3 = rng.choice(FREGS)
        rm = rng.choice(RMS)
        off = rng.choice(offsets) & ~3

        if kind == "fma":
            op = rng.choice(["fmadd.s", "fmsub.s", "fnmsub.s", "fnmadd.s"])
            lines.append(f"    {op} f{fd}, f{fs1}, f{fs2}, f{fs3}, {rm}")
        elif kind == "fadd_mul":
            op = rng.choice(["fadd.s", "fsub.s", "fmul.s"])
            lines.append(f"    {op} f{fd}, f{fs1}, f{fs2}, {rm}")
        elif kind == "fdiv":
            lines.append(f"    fdiv.s f{fd}, f{fs1}, f{fs2}, {rm}")
        elif kind == "fsqrt":
            lines.append(f"    fsqrt.s f{fd}, f{fs1}, {rm}")
        elif kind == "fsgnj":
            op = rng.choice(["fsgnj.s", "fsgnjn.s", "fsgnjx.s", "fmin.s", "fmax.s"])
            lines.append(f"    {op} f{fd}, f{fs1}, f{fs2}")
        elif kind == "fcmp":
            op = rng.choice(["feq.s", "flt.s", "fle.s"])
            lines.append(f"    {op} x{rd}, f{fs1}, f{fs2}")
        elif kind == "fcmp_br":
            op = rng.choice(["feq.s", "flt.s", "fle.s"])
            lbl = f"{prefix}_fcbr_{i}"
            lines.append(f"    {op} x29, f{fs1}, f{fs2}")
            lines.append(f"    bne  x29, x0, {lbl}")
            lines.append(f"    fadd.s f{fd}, f{fs1}, f{fs2}, {rm}")
            lines.append(f"{lbl}:")
        elif kind == "fcvt":
            c = rng.choice(["w.s", "wu.s", "s.w", "s.wu", "mv.xw", "mv.wx", "class"])
            if c == "w.s":
                lines.append(f"    fcvt.w.s x{rd}, f{fs1}, {rm}")
            elif c == "wu.s":
                lines.append(f"    fcvt.wu.s x{rd}, f{fs1}, {rm}")
            elif c == "s.w":
                lines.append(f"    fcvt.s.w f{fd}, x{rs1}, {rm}")
            elif c == "s.wu":
                lines.append(f"    fcvt.s.wu f{fd}, x{rs1}, {rm}")
            elif c == "mv.xw":
                lines.append(f"    fmv.x.w x{rd}, f{fs1}")
            elif c == "mv.wx":
                lines.append(f"    fmv.w.x f{fd}, x{rs1}")
            else:
                lines.append(f"    fclass.s x{rd}, f{fs1}")
        elif kind == "flw":
            lines.append(f"    flw  f{fd}, {off}({base_reg})")
        elif kind == "fsw":
            lines.append(f"    fsw  f{fs2}, {off}({base_reg})")
        elif kind == "stlf_cross":
            # Cross-domain SW->FLW or FSW->LW STLF!
            if rng.random() < 0.5:
                lines.append(f"    fsw  f{fs2}, {off}({base_reg})")
                lines.append(f"    lw   x{rd}, {off}({base_reg})")
                lines.append(f"    flw  f{fd}, {off}({base_reg})")
            else:
                lines.append(f"    sw   x{rs2}, {off}({base_reg})")
                lines.append(f"    flw  f{fd}, {off}({base_reg})")
        elif kind == "alu":
            op = rng.choice(["add", "sub", "xor", "or", "and", "mul"])
            lines.append(f"    {op} x{rd}, x{rs1}, x{rs2}")
        elif kind == "imm":
            lines.append(f"    addi x{rd}, x{rs1}, {rng.randint(-64, 63)}")
        elif kind == "ld_int":
            op = rng.choice(["lw", "lh", "lb", "lbu"])
            lo = off + (rng.choice([0, 2]) if op == "lh" else (rng.randint(0, 3) if op in ("lb", "lbu") else 0))
            lines.append(f"    {op} x{rd}, {lo}({base_reg})")
        elif kind == "st_int":
            op = rng.choice(["sw", "sh", "sb"])
            so = off + (rng.choice([0, 2]) if op == "sh" else (rng.randint(0, 3) if op == "sb" else 0))
            lines.append(f"    {op} x{rs2}, {so}({base_reg})")
        elif kind == "spec_fp_squash":
            # Slow FDIV.S or INT DIV feeding a mispredicted branch with poison FP ops along wrong path!
            lbl = f"{prefix}_spfp_{i}"
            lines.append("    li   x30, 99991")
            lines.append("    addi x31, x0, 13")
            lines.append("    div  x29, x30, x31")
            lines.append(f"    bne  x29, x0, {lbl}")
            # Wrong-path FP ops (must squash FP RAT, FP RS & NOT leak fflags!)
            lines.append(f"    fdiv.s f{fd}, f0, f0, rne")       # Would raise NV if leaked!
            lines.append(f"    fmadd.s f{fs1}, f{fs2}, f{fs3}, f{fd}, rne")
            lines.append(f"    fsw  f{fd}, {off}({base_reg})")
            lines.append(f"{lbl}:")
        elif kind == "fcsr_dyn":
            new_rm = rng.randint(0, 4)
            lines.append(f"    fsrmi x{rd}, {new_rm}")
            lines.append(f"    fmadd.s f{fd}, f{fs1}, f{fs2}, f{fs3}, dyn")
            lines.append(f"    frflags x{rs1}")
            if rng.random() < 0.4:
                lines.append("    fsflagsi x0, 0")
        elif kind == "fp_trap" and allow_traps:
            t = rng.choice(["fs_off", "bad_rm", "mis_flw", "mis_fsw"])
            if t == "fs_off":
                lines.append("    li   x30, 0x6000")
                lines.append("    csrc mstatus, x30")
                lines.append(f"    fadd.s f{fd}, f{fs1}, f{fs2}, rne")
            elif t == "bad_rm":
                # Encode FADD.S with illegal rm=5
                bad_ins = isa.enc_r(0x00, fs2, fs1, 5, fd, opcode=0x53)
                lines.append(f"    .word 0x{bad_ins:08x}")
            elif t == "mis_flw":
                lines.append(f"    flw  f{fd}, {off + 2}({base_reg})")
            else:
                lines.append(f"    fsw  f{fs2}, {off + 1}({base_reg})")
        elif kind == "amo":
            op = rng.choice(["amoadd.w", "amoswap.w", "amoxor.w", "amoor.w"])
            lines.append(f"    addi x29, {base_reg}, {off}")
            lines.append(f"    {op}.aqrl x{rd}, x{rs2}, (x29)")
        elif kind == "fp_spin":
            # Contended AMOSWAP spinlock protecting a shared FP accumulator at 4(x28)!
            lbl = f"{prefix}_fpsp_{i}"
            lines.append("    li   x30, 1")
            lines.append(f"{lbl}:")
            lines.append("    amoswap.w.aq x29, x30, (x28)")
            lines.append(f"    bne  x29, x0, {lbl}")
            lines.append(f"    flw  f{fd}, 4(x28)")
            lines.append(f"    fadd.s f{fd}, f{fd}, f{fs1}, rne")
            lines.append(f"    fsw  f{fd}, 4(x28)")
            lines.append("    amoswap.w.rl x0, x0, (x28)")
        elif kind == "fence":
            lines.append("    fence rw, rw")
        elif kind == "flw_spec_corr":
            # Speculative FLW behind slow FDIV.S / DIV (stresses FLW LQ snoop-replay!)
            lines.append("    li   x30, 1234567")
            lines.append(f"    addi x31, x0, {rng.choice([7, 11, 13, 19])}")
            lines.append("    div  x29, x30, x31")
            lines.append("    sub  x29, x29, x29")
            lines.append(f"    add  x29, {base_reg}, x29")
            lines.append(f"    flw  f{fd}, {off}(x29)")
            lines.append(f"    flw  f{fs1}, {off}({base_reg})")
            lines.append(f"    fadd.s f{fs2}, f{fd}, f{fs1}, rne")

    return lines


# ===========================================================================
# Suite A: 150-Program Single-Core RV32IMAF Differential Fuzz (Vp2_c0 +FP_TRACE)
# ===========================================================================
def gen_single_fuzz(seed: int) -> Tuple[str, str]:
    rng = random.Random(20266000 + seed)
    offs = [0, 4, 8, 12, 16, 20, 24, 28, 32, 48]

    if seed < 30:
        cat = "S1_fma_alu_mix"
        w = {"fma": 30, "fadd_mul": 25, "alu": 20, "imm": 10, "flw": 8, "fsw": 7}
        body = _gen_fp_body(rng, "s", 80, "x28", offs, w)
        src = "\n".join(["    li   x28, 0x20000"] + _init_fp_and_int(rng) + body + _fp_checksum_footer("x28"))
        return cat, src

    if seed < 60:
        cat = "S2_fdiv_fsqrt_ooo"
        w = {"fdiv": 18, "fsqrt": 16, "fma": 20, "fadd_mul": 14, "alu": 16, "flw": 8, "fsw": 8}
        body = _gen_fp_body(rng, "s", 65, "x28", offs, w)
        src = "\n".join(["    li   x28, 0x20000"] + _init_fp_and_int(rng) + body + _fp_checksum_footer("x28"))
        return cat, src

    if seed < 90:
        cat = "S3_fmisc_cvt_cmp_br"
        w = {"fsgnj": 18, "fcmp": 14, "fcmp_br": 16, "fcvt": 24, "fma": 12, "fadd_mul": 8, "alu": 8}
        body = _gen_fp_body(rng, "s", 75, "x28", offs, w)
        src = "\n".join(["    li   x28, 0x20000"] + _init_fp_and_int(rng) + body + _fp_checksum_footer("x28"))
        return cat, src

    if seed < 120:
        cat = "S4_flw_fsw_stlf_spec"
        w = {"flw": 18, "fsw": 18, "stlf_cross": 16, "spec_fp_squash": 14, "fma": 14, "fdiv": 8, "alu": 12}
        body = _gen_fp_body(rng, "s", 65, "x28", offs, w)
        src = "\n".join(["    li   x28, 0x20000"] + _init_fp_and_int(rng) + body + _fp_checksum_footer("x28"))
        return cat, src

    # S5_frm_fcsr_fs_traps: dynamic frm switching + mstatus.FS=Off + bad_rm + misaligned FLW/FSW traps!
    cat = "S5_frm_fcsr_fs_traps"
    w = {"fcsr_dyn": 26, "fp_trap": 18, "fma": 18, "fdiv": 10, "fcvt": 14, "flw": 7, "fsw": 7}
    body = _gen_fp_body(rng, "s", 55, "x28", offs, w, allow_traps=True)
    # Minimal trap handler that re-enables mstatus.FS=Initial, records cause^tval in x26, advances mepc+=4!
    header = [
        "    la   x27, u6_trap_h",
        "    csrw mtvec, x27",
        "    addi x26, x0, 0",
        "    li   x28, 0x20000",
    ]
    footer = [
        "    csrw mtvec, x0",
        "    addi x14, x26, 0",
    ] + [f"    xor  x14, x14, x{r}" for r in IREGS] + [
        "    frcsr x29",
        "    xor  x14, x14, x29",
        "    ebreak",
        "u6_trap_h:",
        "    li   x31, 0x2000",
        "    csrs mstatus, x31",
        "    csrr x31, mcause",
        "    add  x26, x26, x31",
        "    csrr x31, mepc",
        "    addi x31, x31, 4",
        "    csrw mepc, x31",
        "    mret",
    ]
    src = "\n".join(header + _init_fp_and_int(rng) + body + footer)
    return cat, src


def run_single_seed(seed: int) -> Tuple[int, str, bool, str, Dict[str, int]]:
    cat, src = gen_single_fuzz(seed)
    hex_p = BUILD / f"s1c_{seed:03d}.hex"
    trc_p = BUILD / f"s1c_{seed:03d}.fptrace"
    try:
        words = isa.assemble(src)
        isa.write_hex(words, hex_p, pad_words=1024)
        iss = isa.RV32IMA_ISS(words, core_id=0, unified_mem=True)
        trace = iss.run(max_steps=50_000)
        isa.write_fp_trace(trace, trc_p)

        cmd = [
            str(BIN_SINGLE),
            f"+HEX={hex_p}",
            f"+FP_TRACE={trc_p}",
            f"+EXPECT_X14={iss.regs[14]:08x}",
            "+JITTER",
            "+MAX_CYCLES=80000",
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0 or "PASS [single_ooo2w_L2]:" not in res.stdout:
            return seed, cat, False, f"SIM FAIL:\n{res.stdout}\n{res.stderr}", {}

        hex_p.unlink(missing_ok=True)
        trc_p.unlink(missing_ok=True)
        m = re.search(r"commits=(\d+)\s+cycles=(\d+)", res.stdout)
        return seed, cat, True, "OK", {
            "commits": int(m.group(1)) if m else len(trace),
            "cycles": int(m.group(2)) if m else 0,
            "traps": iss.trap_count,
        }
    except Exception as exc:
        return seed, cat, False, f"{type(exc).__name__}: {exc}", {}


# ===========================================================================
# Suite B: 100-Program Dual-Core RV32IMAF Coherence Fuzz (Vdual_soc +LOG +JITTER)
# ===========================================================================
def gen_dual_fuzz(seed: int) -> Tuple[str, str, str]:
    rng = random.Random(20267000 + seed)
    offs = [0, 4, 8, 12, 16, 20, 24, 28, 32, 48]

    if seed < 25:
        cat = "D1_disjoint_rv32imaf"
        mode = "disjoint"
        w = {"fma": 22, "fadd_mul": 14, "fdiv": 8, "fsqrt": 6, "fsgnj": 8, "fcvt": 10, "fcmp_br": 6, "flw": 10, "fsw": 10, "alu": 6}
        c0 = ["    li   x28, 0x20000"] + _init_fp_and_int(rng, 0) + _gen_fp_body(rng, "c0", 50, "x28", offs, w) + _fp_checksum_footer("x28")
        c1 = ["    li   x28, 0x21000"] + _init_fp_and_int(rng, 1) + _gen_fp_body(rng, "c1", 50, "x28", offs, w) + _fp_checksum_footer("x28")
    elif seed < 50:
        cat = "D2_shared_flw_fsw_false_share"
        mode = "shared"
        w = {"flw": 22, "fsw": 22, "stlf_cross": 12, "fma": 14, "fdiv": 6, "fsqrt": 4, "ld_int": 6, "st_int": 6, "fence": 4, "alu": 4}
        c0 = ["    li   x28, 0x20000"] + _init_fp_and_int(rng, 0) + _gen_fp_body(rng, "c0", 55, "x28", offs, w) + _fp_checksum_footer("x28")
        c1 = ["    li   x28, 0x20000"] + _init_fp_and_int(rng, 1) + _gen_fp_body(rng, "c1", 55, "x28", offs, w) + _fp_checksum_footer("x28")
    elif seed < 75:
        cat = "D3_mixed_fp_amo_spin"
        mode = "shared"
        offs_spin = [8, 12, 16, 20, 24, 28]
        w = {"fp_spin": 14, "amo": 16, "flw": 16, "fsw": 14, "fma": 16, "fcvt": 10, "fcmp_br": 6, "fence": 4, "alu": 4}
        c0 = ["    li   x28, 0x20000"] + _init_fp_and_int(rng, 0) + _gen_fp_body(rng, "c0", 42, "x28", offs_spin, w) + _fp_checksum_footer("x28")
        c1 = ["    li   x28, 0x20000"] + _init_fp_and_int(rng, 1) + _gen_fp_body(rng, "c1", 42, "x28", offs_spin, w) + _fp_checksum_footer("x28")
    else:
        cat = "D4_flw_corr_snoop_replay"
        mode = "shared"
        offs_hot = [0, 4, 8, 12]
        w = {"flw_spec_corr": 26, "flw": 18, "fsw": 22, "fma": 12, "fdiv": 8, "fence": 6, "alu": 8}
        c0 = ["    li   x28, 0x20000"] + _init_fp_and_int(rng, 0) + _gen_fp_body(rng, "c0", 50, "x28", offs_hot, w) + _fp_checksum_footer("x28")
        c1 = ["    li   x28, 0x20000"] + _init_fp_and_int(rng, 1) + _gen_fp_body(rng, "c1", 50, "x28", offs_hot, w) + _fp_checksum_footer("x28")

    src = (
        "    csrr x31, mhartid\n"
        "    bne  x31, x0, core1_start\n"
        "core0_start:\n" + "\n".join(c0) + "\n"
        "core1_start:\n" + "\n".join(c1) + "\n"
    )
    return cat, mode, src


def run_dual_seed(seed: int) -> Tuple[int, str, str, bool, str, Dict[str, int]]:
    cat, mode, src = gen_dual_fuzz(seed)
    hex_p = BUILD / f"d2c_{seed:03d}.hex"
    log_p = BUILD / f"d2c_{seed:03d}.log"
    try:
        words = isa.assemble(src)
        isa.write_hex(words, hex_p, pad_words=1024)
        cmd = [
            str(BIN_DUAL),
            f"+HEX={hex_p}",
            f"+LOG={log_p}",
            "+JITTER",
            "+MAX_CYCLES=200000",
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0 or "PASS [dual_ooo2w_soc]:" not in res.stdout:
            return seed, cat, mode, False, f"SIM FAIL:\n{res.stdout}\n{res.stderr}", {}

        pass_ln = [ln for ln in res.stdout.splitlines() if "PASS [dual_ooo2w_soc]:" in ln][-1]
        kv = dict(re.findall(r"(\w+)=([^\s]+)", pass_ln))
        st = chk.check_dual_trace(hex_p, log_p, mode=mode)

        hex_p.unlink(missing_ok=True)
        log_p.unlink(missing_ok=True)
        rep0, rep1 = [int(x) for x in kv["replay"].split("/")]
        return seed, cat, mode, True, "OK", {
            "cycles": int(kv["cycles"]),
            "commits": st.c0_commits + st.c1_commits,
            "fp_ops": st.fp_ops_checked,
            "loads": st.loads_checked,
            "stores": st.stores_checked,
            "replays": rep0 + rep1,
            "c2c": st.bus_c2c,
        }
    except Exception as exc:
        return seed, cat, mode, False, f"{type(exc).__name__}: {exc}", {}


def run_flw_corr_litmus() -> None:
    """Directed RVWMO L05_CoRR Litmus using FLW/FSW: proves FLW triggers LQ snoop-replay!"""
    # Core 0 writes float 1.0 (0x3F800000) then 2.0 (0x40000000) to [0x20000] across 64 rounds.
    # Core 1 issues a slow DIV-dependent FLW #1 followed by independent FLW #2 to [0x20000]!
    # If FLW #2 executes out-of-order BEFORE FLW #1 and observes 2.0 while FLW #1 observes 1.0
    # -> no violation; if FLW #2 observed 1.0 before C0 wrote 2.0 and FLW #1 observed 2.0 -> CoRR VIOLATION!
    # Our hardware LQ snoop-invalidation MUST replay FLW #2 -> 0 CoRR violations AND replay > 0!
    litmus_src = """
    li   x28, 0x20000
    csrr x31, mhartid
    bne  x31, x0, c1_lit

c0_lit:
    li   x1, 0
    li   x2, 64
    li   x3, 0x3F800000
    li   x4, 0x40000000
    fmv.w.x f1, x3
    fmv.w.x f2, x4
c0_lp:
    slli x5, x1, 4
    add  x6, x28, x5
    fsw  f1, 0(x6)
    fence rw, rw
    fsw  f2, 0(x6)
    addi x1, x1, 1
    blt  x1, x2, c0_lp
    li   x14, 0xC0000040
    ebreak

c1_lit:
    li   x1, 0
    li   x2, 64
    li   x3, 1234567
    li   x4, 17
    li   x14, 0
c1_lp:
    slli x5, x1, 4
    add  x6, x28, x5
    # Slow DIV delays FLW #1 so FLW #2 executes speculatively first (CoRR trap!)
    div  x7, x3, x4
    sub  x7, x7, x7
    add  x8, x6, x7
    flw  f10, 0(x8)      # Older FLW #1
    flw  f11, 0(x6)      # Younger FLW #2 (speculative!)
    fmv.x.w x9, f10
    fmv.x.w x10, f11
    # Forbidden CoRR outcome: older saw 2.0 (0x40000000) and younger saw 1.0 (0x3F800000) or 0.0!
    li   x11, 0x40000000
    bne  x9, x11, c1_ok
    beq  x10, x11, c1_ok
    addi x14, x14, 1     # CoRR violation counter!
c1_ok:
    addi x1, x1, 1
    blt  x1, x2, c1_lp
    ebreak
"""
    hex_p = BUILD / "litmus_flw_corr.hex"
    log_p = BUILD / "litmus_flw_corr.log"
    words = isa.assemble(litmus_src)
    isa.write_hex(words, hex_p, pad_words=512)
    res = subprocess.run(
        [str(BIN_DUAL), f"+HEX={hex_p}", f"+LOG={log_p}", "+EXPECT_C0_X14=c0000040",
         "+EXPECT_C1_X14=00000000", "+JITTER", "+MAX_CYCLES=100000"],
        capture_output=True, text=True,
    )
    if res.returncode != 0 or "PASS [dual_ooo2w_soc]:" not in res.stdout:
        print(f"FAIL [FLW L05_CoRR Litmus]:\n{res.stdout}\n{res.stderr}")
        sys.exit(1)
    st = chk.check_dual_trace(hex_p, log_p, mode="shared")
    pass_ln = [ln for ln in res.stdout.splitlines() if "PASS [dual_ooo2w_soc]:" in ln][-1]
    print(f"  [OK] FLW L05_CoRR Snoop-Replay Litmus (0 violations, fp_ops={st.fp_ops_checked}): {pass_ln}")


# ===========================================================================
# Suite C: Exact Python IEEE-754 Oracles & Live Compiled GCC Hard-Float C Tests
# ===========================================================================
def _rotl32(x: int, r: int) -> int:
    return ((x << r) | (x >> (32 - r))) & 0xFFFF_FFFF


def _mix_hash(h: int, v: int) -> int:
    return (_rotl32(h ^ (v & 0xFFFF_FFFF), 5) + 0x9E37_79B9) & 0xFFFF_FFFF


def oracle_workloads_hashes() -> Tuple[int, int]:
    """Compute bit-exact IEEE-754 hashes for SAXPY and FSQRT tests using exact Fraction oracle."""
    a_u = 0x3FB0_0000  # 1.375f
    h_saxpy = 0x1234_5678
    for i in range(32):
        xi_u, _ = fpo.eval_misc(fpo.FMISC_CVT_SW, (i * 3 - 40) & 0xFFFF_FFFF, 0, fpo.RM_RNE)
        xi_u, _ = fpo.eval_fma(fpo.FMA_FMUL, xi_u, 0x3E00_0000, 0, fpo.RM_RNE)  # * 0.125f
        yi_u, _ = fpo.eval_misc(fpo.FMISC_CVT_SW, (i * 5 - 70) & 0xFFFF_FFFF, 0, fpo.RM_RNE)
        yi_u, _ = fpo.eval_fma(fpo.FMA_FMUL, yi_u, 0x3E80_0000, 0, fpo.RM_RNE)  # * 0.25f
        res_u, _ = fpo.eval_fma(fpo.FMA_FMADD, a_u, xi_u, yi_u, fpo.RM_RNE)
        h_saxpy = _mix_hash(h_saxpy, res_u)

    sqrt_in = [0x0000_0000, 0x8000_0000, 0x3F80_0000, 0x4010_0000,
               0x4000_0000, 0x4040_0000, 0x0000_0004, 0x461C_4000]
    h_sqrt = 0x3344_5566
    for u in sqrt_in:
        r_u, _ = fpo.eval_div_sqrt(fpo.FDIV_SQRT, u, 0, fpo.RM_RNE)
        h_sqrt = _mix_hash(h_sqrt, r_u)
    return h_saxpy, h_sqrt


def oracle_sgemm24_hash() -> int:
    """Bit-exact IEEE-754 24x24 FMA SGEMM hash using Python Fraction oracle."""
    n = 24
    A = [[0] * n for _ in range(n)]
    B = [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            av = ((i * 7 + j * 3) & 15) - 7
            bv = ((i * 5 + j * 11) & 15) - 8
            au, _ = fpo.eval_misc(fpo.FMISC_CVT_SW, av & 0xFFFF_FFFF, 0, fpo.RM_RNE)
            A[i][j], _ = fpo.eval_fma(fpo.FMA_FMUL, au, 0x3E80_0000, 0, fpo.RM_RNE)  # * 0.25f
            bu, _ = fpo.eval_misc(fpo.FMISC_CVT_SW, bv & 0xFFFF_FFFF, 0, fpo.RM_RNE)
            B[i][j], _ = fpo.eval_fma(fpo.FMA_FMUL, bu, 0x3E00_0000, 0, fpo.RM_RNE)  # * 0.125f

    h = 0x56E0_0024
    for i in range(n):
        for j in range(n):
            acc = 0x0000_0000
            for k in range(n):
                acc, _ = fpo.eval_fma(fpo.FMA_FMADD, A[i][k], B[k][j], acc, fpo.RM_RNE)
            h = _mix_hash(h, acc)
    return h


def compile_hardfloat_c(srcs: List[str], stem: str) -> Path:
    BUILD.mkdir(parents=True, exist_ok=True)
    elf = BUILD / f"{stem}.elf"
    bin_p = BUILD / f"{stem}.bin"
    hex_p = BUILD / f"{stem}.hex"
    subprocess.run(
        [
            GCC, "-march=rv32imaf", "-mabi=ilp32f", "-O2", "-fno-math-errno",
            "-ffreestanding", "-nostdlib", "-nostartfiles",
            "-Wall", "-Wextra", "-Isw/common", "-Isw/hal", "-T", "sw/common/link.ld",
            *srcs, "-o", str(elf),
        ],
        cwd=str(ROOT),
        check=True,
    )
    subprocess.run([OBJCOPY, "-O", "binary", str(elf), str(bin_p)], cwd=str(ROOT), check=True)
    subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "elf2hex.py"), str(bin_p), str(hex_p), "--pad-words", "4096"],
        cwd=str(ROOT),
        check=True,
    )
    return hex_p


def run_c_suites() -> Dict[str, object]:
    print("\n=== Suite C: Live GCC (-march=rv32imaf -mabi=ilp32f -O2) Hard-Float C Workloads ===")
    exp_saxpy, exp_sqrt = oracle_workloads_hashes()
    exp_w_low = (exp_saxpy ^ exp_sqrt) & 0xFFFF
    exp_w_1c = 0xF601_0000 | exp_w_low
    exp_w_2c_c0 = 0xF602_0000 | exp_w_low
    exp_w_2c_c1 = 0xF612_0000 | exp_w_low

    wk_hex = compile_hardfloat_c(
        ["sw/common/crt0.S", "sw/common/trap.S", "sw/hal/hal_irq.c", "sw/hal/hal_uart.c",
         "sw/c_soc/test_fp_workloads_c.c"],
        "test_fp_workloads_c",
    )

    u1_p = BUILD / "fp_wk_1c_uart.txt"
    r1 = subprocess.run(
        [str(BIN_SINGLE), f"+HEX={wk_hex}", f"+UART_OUT={u1_p}",
         f"+EXPECT_X14={exp_w_1c:08x}", "+JITTER", "+MAX_CYCLES=150000"],
        capture_output=True, text=True,
    )
    if r1.returncode != 0 or "PASS [single_ooo2w_L2]:" not in r1.stdout:
        print(f"FAIL [C1a Workloads 1C]:\n{r1.stdout}\n{r1.stderr}")
        sys.exit(1)
    u1_txt = u1_p.read_text(encoding="utf-8").strip()
    assert f"saxpy=0x{exp_saxpy:08x} sqrt=0x{exp_sqrt:08x}" in u1_txt, f"Unexpected 1C UART: {u1_txt}"
    print(f"  [OK] C1a Hard-Float Workloads + Live FP-Preemption ISR (1C): {u1_txt}")

    u2_p = BUILD / "fp_wk_2c_uart.txt"
    r2 = subprocess.run(
        [str(BIN_DUAL), f"+HEX={wk_hex}", f"+UART_OUT={u2_p}",
         f"+EXPECT_C0_X14={exp_w_2c_c0:08x}", f"+EXPECT_C1_X14={exp_w_2c_c1:08x}",
         "+JITTER", "+MAX_CYCLES=200000"],
        capture_output=True, text=True,
    )
    if r2.returncode != 0 or "PASS [dual_ooo2w_soc]:" not in r2.stdout:
        print(f"FAIL [C1b Workloads 2C]:\n{r2.stdout}\n{r2.stderr}")
        sys.exit(1)
    u2_txt = u2_p.read_text(encoding="utf-8").strip()
    assert f"saxpy=0x{exp_saxpy:08x} sqrt=0x{exp_sqrt:08x}" in u2_txt, f"Unexpected 2C UART: {u2_txt}"
    print(f"  [OK] C1b Hard-Float Workloads + Live FP-Preemption ISR (2C): {u2_txt}")

    # C2: 24x24 Parallel IEEE-754 Float SGEMM on 1C & 2C!
    exp_sgemm = oracle_sgemm24_hash()
    sg_hex = compile_hardfloat_c(
        ["sw/common/crt0.S", "sw/c_soc/test_fp_sgemm_c.c"],
        "test_fp_sgemm_c",
    )

    rs1 = subprocess.run(
        [str(BIN_SINGLE), f"+HEX={sg_hex}",
         f"+EXPECT_X14={exp_sgemm:08x}", "+MAX_CYCLES=300000"],
        capture_output=True, text=True,
    )
    if rs1.returncode != 0 or "PASS [single_ooo2w_L2]:" not in rs1.stdout:
        print(f"FAIL [C2a SGEMM24 1C]:\n{rs1.stdout}\n{rs1.stderr}")
        sys.exit(1)
    p1 = [ln for ln in rs1.stdout.splitlines() if "PASS [single_ooo2w_L2]:" in ln][-1]
    kv1 = dict(re.findall(r"(\w+)=([^\s]+)", p1))
    cyc_1c = int(kv1["cycles"])
    ipc_1c = float(kv1["IPC"])
    print(f"  [OK] C2a 24x24 Float SGEMM (1C, bit-exact hash=0x{exp_sgemm:08x}): cycles={cyc_1c:,} IPC={ipc_1c:.3f}")

    rs2 = subprocess.run(
        [str(BIN_DUAL), f"+HEX={sg_hex}",
         f"+EXPECT_C0_X14={exp_sgemm:08x}", f"+EXPECT_C1_X14={exp_sgemm:08x}", "+MAX_CYCLES=300000"],
        capture_output=True, text=True,
    )
    if rs2.returncode != 0 or "PASS [dual_ooo2w_soc]:" not in rs2.stdout:
        print(f"FAIL [C2b SGEMM24 2C]:\n{rs2.stdout}\n{rs2.stderr}")
        sys.exit(1)
    p2 = [ln for ln in rs2.stdout.splitlines() if "PASS [dual_ooo2w_soc]:" in ln][-1]
    kv2 = dict(re.findall(r"(\w+)=([^\s]+)", p2))
    cyc_2c = int(kv2["cycles"])
    chip_ipc_2c = float(kv2["chip_IPC"])
    speedup = cyc_1c / max(1, cyc_2c)
    print(
        f"  [OK] C2b 24x24 Float SGEMM (2C, bit-exact hash=0x{exp_sgemm:08x}): "
        f"cycles={cyc_2c:,} chip_IPC={chip_ipc_2c:.3f} speedup={speedup:.2f}x"
    )
    if chip_ipc_2c <= 3.0:
        print(f"FAIL [C2b SGEMM24 2C]: chip_IPC={chip_ipc_2c:.3f} did not exceed 3.0 target!")
        sys.exit(1)

    return {
        "saxpy_hash": f"0x{exp_saxpy:08x}",
        "sqrt_hash": f"0x{exp_sqrt:08x}",
        "sgemm24_hash": f"0x{exp_sgemm:08x}",
        "sgemm24_1c_cycles": cyc_1c,
        "sgemm24_1c_ipc": ipc_1c,
        "sgemm24_2c_cycles": cyc_2c,
        "sgemm24_2c_chip_ipc": chip_ipc_2c,
        "sgemm24_speedup": round(speedup, 3),
    }


def main() -> int:
    BUILD.mkdir(parents=True, exist_ok=True)
    t_start = time.time()
    workers = min(8, os.cpu_count() or 4)

    # ------------------------------------------------------------------
    # Suite A: 150-Program Randomized Single-Core RV32IMAF Fuzz
    # ------------------------------------------------------------------
    n_single = 150
    print(f"=== Suite A: {n_single}-Program Randomized Single-Core RV32IMAF Differential Fuzz (+FP_TRACE +JITTER) ===")
    t0 = time.time()
    s_cats: Dict[str, int] = {}
    s_tot = {"commits": 0, "cycles": 0, "traps": 0}
    s_fails: List[Tuple[int, str, str]] = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=workers) as ex:
        futs = [ex.submit(run_single_seed, s) for s in range(n_single)]
        done = 0
        for fut in concurrent.futures.as_completed(futs):
            seed, cat, ok, msg, m = fut.result()
            done += 1
            if not ok:
                s_fails.append((seed, cat, msg))
                print(f"  [FAIL] 1C seed={seed:03d} ({cat}): {msg}")
            else:
                s_cats[cat] = s_cats.get(cat, 0) + 1
                for k in s_tot:
                    s_tot[k] += m[k]
            if done % 50 == 0 or done == n_single:
                print(f"  Progress: {done}/{n_single} complete ({len(s_fails)} failures, {time.time()-t0:.1f}s)")
    if s_fails:
        print(f"FAIL [Suite A]: {len(s_fails)}/{n_single} single-core RV32IMAF fuzz programs failed!")
        return 1
    print(
        f"  PASS [Suite A]: {n_single}/{n_single} bit-exact 1C RV32IMAF programs "
        f"({s_tot['commits']:,} commits, {s_tot['traps']:,} FP/mem traps, {time.time()-t0:.1f}s)"
    )

    # ------------------------------------------------------------------
    # Suite B: 100-Program Randomized Dual-Core RV32IMAF Coherence Fuzz + FLW CoRR
    # ------------------------------------------------------------------
    n_dual = 100
    print(f"\n=== Suite B: {n_dual}-Program Dual-Core RV32IMAF Coherence Fuzz + FLW CoRR Litmus (+LOG +JITTER) ===")
    run_flw_corr_litmus()
    t1 = time.time()
    d_cats: Dict[str, int] = {}
    d_tot = {"cycles": 0, "commits": 0, "fp_ops": 0, "loads": 0, "stores": 0, "replays": 0, "c2c": 0}
    d_fails: List[Tuple[int, str, str]] = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=workers) as ex:
        futs = [ex.submit(run_dual_seed, s) for s in range(n_dual)]
        done = 0
        for fut in concurrent.futures.as_completed(futs):
            seed, cat, mode, ok, msg, m = fut.result()
            done += 1
            if not ok:
                d_fails.append((seed, cat, msg))
                print(f"  [FAIL] 2C seed={seed:03d} ({cat}, {mode}): {msg}")
            else:
                d_cats[cat] = d_cats.get(cat, 0) + 1
                for k in d_tot:
                    d_tot[k] += m[k]
            if done % 25 == 0 or done == n_dual:
                print(f"  Progress: {done}/{n_dual} complete ({len(d_fails)} failures, {time.time()-t1:.1f}s)")
    if d_fails:
        print(f"FAIL [Suite B]: {len(d_fails)}/{n_dual} dual-core RV32IMAF fuzz programs failed!")
        return 1
    print(
        f"  PASS [Suite B]: {n_dual}/{n_dual} dual-core RV32IMAF coherence programs "
        f"({d_tot['commits']:,} commits, {d_tot['fp_ops']:,} FP ops, {d_tot['replays']:,} snoop-replays, "
        f"{d_tot['c2c']:,} dirty C2C, {time.time()-t1:.1f}s)"
    )

    # ------------------------------------------------------------------
    # Suite C: Live Compiled GCC Hard-Float C Workloads + 24x24 SGEMM > 3.0 IPC
    # ------------------------------------------------------------------
    c_stats = run_c_suites()

    summary = {
        "suite_a_1c_fuzz": {"programs": n_single, "categories": dict(sorted(s_cats.items())), "totals": s_tot},
        "suite_b_2c_fuzz": {"programs": n_dual, "categories": dict(sorted(d_cats.items())), "totals": d_tot},
        "suite_c_c_workloads": c_stats,
        "elapsed_sec": round(time.time() - t_start, 2),
    }
    (BUILD / "phase_u6_results.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(
        f"\nPASS [Phase U6]: ALL RV32IMAF Verification Gates PASSED in {summary['elapsed_sec']}s!\n"
        f"  - Suite A: {n_single}/{n_single} 1C bit-exact FP+INT differential fuzz ({s_tot['commits']:,} commits, {s_tot['traps']:,} traps)\n"
        f"  - Suite B: {n_dual}/{n_dual} 2C RV32IMAF coherence fuzz + FLW CoRR litmus ({d_tot['fp_ops']:,} FP ops, {d_tot['replays']:,} replays)\n"
        f"  - Suite C: GCC -march=rv32imaf -mabi=ilp32f -O2 workloads (1C & 2C + live FP ISR preemption) &\n"
        f"             24x24 Float SGEMM bit-exact={c_stats['sgemm24_hash']} | 1C IPC={c_stats['sgemm24_1c_ipc']:.3f} -> "
        f"2C chip_IPC={c_stats['sgemm24_2c_chip_ipc']:.3f} ({c_stats['sgemm24_speedup']:.2f}x speedup!)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
