#!/usr/bin/env python3
"""
Phase 5: 200-Program Randomized Dual-Core Coherence & RVWMO Differential Fuzzer.

Generates and executes 200 randomized dual-core RV32IMA_Zicsr_Zifencei stress programs
on the live Dual-Core 2-Wide Out-of-Order MOESI SoC (Vdual_soc with +JITTER DRAM latency
jitter and always-on coh_checker SWMR/Shadow monitor), and verifies EVERY SINGLE ONE
end-to-end with scripts/dual_core_checker.py!

5 Adversarial Categories (200 programs total):
  1. Disjoint Data-Parallel Bit-Exact RVFI Fuzz (30 programs, seeds 0..29)
     -> Verified bit-exact against TWO independent RV32IMA_ISS models AND linearizability checker!
  2. Shared-Memory Hot-Spot & False-Sharing Hammer (50 programs, seeds 30..79)
     -> Both cores hammer 4 cache lines (sub-word SB/SH/SW, LB/LH/LW, MSHR coalescing, DIV+spec loads!)
  3. Contended Atomic AMO*.W + LR/SC + Spinlock Stress (50 programs, seeds 80..129)
     -> All 9 AMOs (.aq/.rl/.aqrl), bounded LR/SC CAS loops, AMOSWAP spinlocks, mixed loads/stores
  4. L1D Set-Conflict & Dirty-Eviction / WBB Snoop-Intervention Stress (40 programs, seeds 130..169)
     -> 6 tag-aliased addresses (stride 0x800 -> same 2-way L1D set!) forcing constant dirty WBB evictions + C2C!
  5. Producer-Consumer FENCE + FENCE.I + CLINT MMIO + Call/RAS Control-Flow (30 programs, seeds 170..199)
     -> Cross-core FENCE handshakes, FENCE.I clean-all sweeps mid-flight, JAL/JALR calls, CLINT MMIO!
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
import rv32ima_asm as isa  # noqa: E402

BUILD = ROOT / "build" / "phase5_fuzz"
BIN_DUAL = ROOT / "build" / "v_dual_soc" / "Vdual_soc"

REGS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
ALU_R = ["add", "sub", "xor", "or", "and", "slt", "sltu", "mul", "mulh", "mulhsu", "mulhu"]
DIV_R = ["div", "divu", "rem", "remu"]
AMOS = [
    "amoswap.w", "amoadd.w", "amoxor.w", "amoand.w", "amoor.w",
    "amomin.w", "amomax.w", "amominu.w", "amomaxu.w",
]


def _init_core_regs(rng: random.Random, core_tag: int) -> List[str]:
    lines = []
    for r in REGS:
        val = rng.randint(-120, 120) + core_tag * 17
        lines.append(f"    addi x{r}, x0, {val}")
    return lines


def _mix_footer(prefix: str, base_reg: str) -> List[str]:
    lines = ["    addi x14, x0, 0"]
    for r in REGS:
        lines.append(f"    xor  x14, x14, x{r}")
    lines.append(f"    lw   x29, 0({base_reg})")
    lines.append("    add  x14, x14, x29")
    lines.append("    ebreak")
    return lines


def _gen_body(
    rng: random.Random,
    prefix: str,
    n_ops: int,
    base_reg: str,
    offsets: List[int],
    weights: Dict[str, int],
    allow_clint: bool = False,
    allow_fence_i: bool = False,
    allow_call: bool = False,
    set_alias_regs: List[str] | None = None,
) -> List[str]:
    kinds = list(weights.keys())
    wvals = [weights[k] for k in kinds]
    lines: List[str] = []

    for i in range(n_ops):
        kind = rng.choices(kinds, weights=wvals, k=1)[0]
        rd = rng.choice(REGS)
        rs1 = rng.choice(REGS)
        rs2 = rng.choice(REGS)
        breg = rng.choice(set_alias_regs) if set_alias_regs else base_reg
        off = rng.choice(offsets)

        if kind == "alu":
            op = rng.choice(ALU_R)
            lines.append(f"    {op} x{rd}, x{rs1}, x{rs2}")
        elif kind == "imm":
            op = rng.choice(["addi", "xori", "ori", "andi"])
            imm = rng.randint(-128, 127)
            lines.append(f"    {op} x{rd}, x{rs1}, {imm}")
        elif kind == "shift":
            op = rng.choice(["slli", "srli", "srai"])
            sh = rng.randint(0, 31)
            lines.append(f"    {op} x{rd}, x{rs1}, {sh}")
        elif kind == "div":
            op = rng.choice(DIV_R)
            lines.append(f"    {op} x{rd}, x{rs1}, x{rs2}")
        elif kind == "ld":
            op = rng.choice(["lw", "lw", "lh", "lhu", "lb", "lbu"])
            if op in ("lh", "lhu"):
                lo = (off & ~3) + rng.choice([0, 2])
            elif op in ("lb", "lbu"):
                lo = (off & ~3) + rng.randint(0, 3)
            else:
                lo = off & ~3
            lines.append(f"    {op} x{rd}, {lo}({breg})")
        elif kind == "st":
            op = rng.choice(["sw", "sw", "sh", "sb"])
            if op == "sh":
                so = (off & ~3) + rng.choice([0, 2])
            elif op == "sb":
                so = (off & ~3) + rng.randint(0, 3)
            else:
                so = off & ~3
            lines.append(f"    {op} x{rs2}, {so}({breg})")
        elif kind == "amo":
            op = rng.choice(AMOS)
            suf = rng.choice(["", ".aq", ".rl", ".aqrl"])
            ao = off & ~3
            lines.append(f"    addi x29, {breg}, {ao}")
            lines.append(f"    {op}{suf} x{rd}, x{rs2}, (x29)")
        elif kind == "lrsc":
            ao = off & ~3
            lines.append(f"    addi x29, {breg}, {ao}")
            lines.append(f"    lr.w x{rd}, (x29)")
            if rng.random() < 0.75:
                lines.append(f"    addi x30, x{rd}, {rng.randint(1, 7)}")
                lines.append(f"    sc.w x{rs1}, x30, (x29)")
            else:
                # Deliberately break own reservation sometimes!
                lines.append(f"    sw   x{rs2}, 0(x29)")
                lines.append(f"    sc.w x{rs1}, x{rs2}, (x29)")
        elif kind == "spin":
            # Short contended AMOSWAP.W.aq / .rl critical section on lock at 0(x28), data at 4(x28)
            lbl = f"{prefix}_sp_{i}"
            lines.append("    li   x30, 1")
            lines.append(f"{lbl}:")
            lines.append("    amoswap.w.aq x29, x30, (x28)")
            lines.append(f"    bne  x29, x0, {lbl}")
            lines.append(f"    lw   x{rd}, 4(x28)")
            lines.append(f"    addi x{rd}, x{rd}, 1")
            lines.append(f"    sw   x{rd}, 4(x28)")
            lines.append("    amoswap.w.rl x0, x0, (x28)")
        elif kind == "cas":
            # Bounded LR/SC CAS loop (guaranteed forward progress!)
            lbl = f"{prefix}_cas_{i}"
            ao = (off & ~3) if (off & ~3) != 0 else 8
            lines.append(f"    addi x29, {breg}, {ao}")
            lines.append(f"{lbl}:")
            lines.append(f"    lr.w x{rd}, (x29)")
            lines.append(f"    addi x30, x{rd}, 1")
            lines.append(f"    sc.w x31, x30, (x29)")
            lines.append(f"    bne  x31, x0, {lbl}")
        elif kind == "csr":
            c = rng.choice(["mhartid", "mscratch_w", "mscratch_s", "mscratch_c"])
            if c == "mhartid":
                lines.append(f"    csrr x{rd}, mhartid")
            elif c == "mscratch_w":
                lines.append(f"    csrrw x{rd}, mscratch, x{rs1}")
            elif c == "mscratch_s":
                lines.append(f"    csrrsi x{rd}, mscratch, {rng.randint(0, 31)}")
            else:
                lines.append(f"    csrrci x{rd}, mscratch, {rng.randint(0, 31)}")
        elif kind == "fence":
            lines.append("    fence rw, rw")
        elif kind == "fence_i" and allow_fence_i:
            lines.append("    fence.i")
        elif kind == "br":
            lbl = f"{prefix}_br_{i}"
            bop = rng.choice(["beq", "bne", "blt", "bge", "bltu", "bgeu"])
            lines.append(f"    {bop} x{rs1}, x{rs2}, {lbl}")
            lines.append(f"    addi x{rd}, x{rd}, {rng.randint(-15, 15)}")
            lines.append(f"{lbl}:")
        elif kind == "div_spec_ld":
            # Trigger speculative younger load behind slow DIV (stresses LQ snoop-replay!)
            ao = off & ~3
            lines.append("    li   x30, 1234567")
            lines.append(f"    addi x31, x0, {rng.choice([7, 13, 19, 29])}")
            lines.append("    div  x29, x30, x31")
            lines.append("    sub  x29, x29, x29")
            lines.append(f"    add  x29, {breg}, x29")
            lines.append(f"    lw   x{rd}, {ao}(x29)")
            lines.append(f"    lw   x{rs1}, {ao}({breg})")
        elif kind == "clint" and allow_clint:
            # Uncached CLINT MMIO read of mtime_lo / scratch write+read
            lines.append("    li   x29, 0x0200C000")
            lines.append(f"    lw   x{rd}, -8(x29)")
            off_sc = 0 if prefix == "c0" else 4
            lines.append(f"    sw   x{rs1}, {off_sc}(x29)")
            lines.append(f"    lw   x{rs2}, {off_sc}(x29)")
        elif kind == "call" and allow_call:
            lines.append("    jal  x1, helper_fn")

    return lines


def generate_fuzz_program(seed: int) -> Tuple[str, str, str]:
    """Return (category_name, check_mode, asm_source) for seed in 0..199."""
    rng = random.Random(20260000 + seed)

    if seed < 30:
        # Category 1: Disjoint Data-Parallel (bit-exact dual ISS + linearizability!)
        cat = "C1_disjoint_rvfi"
        mode = "disjoint"
        w = {"alu": 20, "imm": 12, "shift": 6, "div": 5, "ld": 16, "st": 14, "amo": 10, "lrsc": 6, "csr": 5, "fence": 3, "br": 5}
        offs = [0, 4, 8, 12, 16, 20, 24, 28, 32, 48]
        c0 = ["    li   x28, 0x20000"] + _init_core_regs(rng, 0) + _gen_body(rng, "c0", 55, "x28", offs, w) + _mix_footer("c0", "x28")
        c1 = ["    li   x28, 0x21000"] + _init_core_regs(rng, 1) + _gen_body(rng, "c1", 55, "x28", offs, w) + _mix_footer("c1", "x28")
        src = (
            "    csrr x31, mhartid\n"
            "    bne  x31, x0, core1_start\n"
            "core0_start:\n" + "\n".join(c0) + "\n"
            "core1_start:\n" + "\n".join(c1) + "\n"
        )
        return cat, mode, src

    if seed < 80:
        # Category 2: Shared-Memory Hot-Spot & False-Sharing Hammer (4 cache lines: 0x20000..0x2003F)
        cat = "C2_shared_false_share"
        mode = "shared"
        w = {"alu": 10, "imm": 6, "div": 4, "ld": 24, "st": 24, "amo": 8, "lrsc": 4, "fence": 5, "br": 5, "div_spec_ld": 10}
        offs = [0, 4, 8, 12, 16, 20, 24, 28, 32, 48]
        c0 = ["    li   x28, 0x20000"] + _init_core_regs(rng, 0) + _gen_body(rng, "c0", 65, "x28", offs, w) + _mix_footer("c0", "x28")
        c1 = ["    li   x28, 0x20000"] + _init_core_regs(rng, 1) + _gen_body(rng, "c1", 65, "x28", offs, w) + _mix_footer("c1", "x28")
        src = (
            "    csrr x31, mhartid\n"
            "    bne  x31, x0, core1_start\n"
            "core0_start:\n" + "\n".join(c0) + "\n"
            "core1_start:\n" + "\n".join(c1) + "\n"
        )
        return cat, mode, src

    if seed < 130:
        # Category 3: Contended Atomics (9x AMO*.W + LR/SC CAS + AMOSWAP Spinlock!)
        cat = "C3_contended_atomics"
        mode = "shared"
        w = {"alu": 8, "ld": 14, "st": 12, "amo": 28, "lrsc": 14, "cas": 10, "spin": 8, "fence": 4, "br": 4}
        # Reserve 0(x28) for spinlock, 4(x28) for spinlock data, 8..28(x28) for AMO/CAS/LD/ST!
        offs = [8, 12, 16, 20, 24, 28]
        c0 = ["    li   x28, 0x20000"] + _init_core_regs(rng, 0) + _gen_body(rng, "c0", 45, "x28", offs, w) + _mix_footer("c0", "x28")
        c1 = ["    li   x28, 0x20000"] + _init_core_regs(rng, 1) + _gen_body(rng, "c1", 45, "x28", offs, w) + _mix_footer("c1", "x28")
        src = (
            "    csrr x31, mhartid\n"
            "    bne  x31, x0, core1_start\n"
            "core0_start:\n" + "\n".join(c0) + "\n"
            "core1_start:\n" + "\n".join(c1) + "\n"
        )
        return cat, mode, src

    if seed < 170:
        # Category 4: L1D Set-Conflict & Dirty-Eviction / WBB Snoop-Intervention Stress!
        # 6 tag-aliased base addresses (stride 0x800 = 128 sets * 16 B -> ALL map to the SAME 2-way L1D set!)
        cat = "C4_set_evict_wbb"
        mode = "shared"
        w = {"alu": 8, "ld": 24, "st": 28, "amo": 14, "lrsc": 6, "fence": 6, "div_spec_ld": 6, "br": 4}
        alias_regs = ["x22", "x23", "x24", "x25", "x26", "x27"]
        set_off = rng.choice([0x00, 0x10, 0x20, 0x30])
        setup_bases = [
            f"    li   x22, {0x20000 + set_off}",
            f"    li   x23, {0x20800 + set_off}",
            f"    li   x24, {0x21000 + set_off}",
            f"    li   x25, {0x21800 + set_off}",
            f"    li   x26, {0x22000 + set_off}",
            f"    li   x27, {0x22800 + set_off}",
        ]
        offs = [0, 4, 8, 12]
        c0 = setup_bases + _init_core_regs(rng, 0) + _gen_body(rng, "c0", 60, "x22", offs, w, set_alias_regs=alias_regs) + _mix_footer("c0", "x22")
        c1 = setup_bases + _init_core_regs(rng, 1) + _gen_body(rng, "c1", 60, "x22", offs, w, set_alias_regs=alias_regs) + _mix_footer("c1", "x22")
        src = (
            "    csrr x31, mhartid\n"
            "    bne  x31, x0, core1_start\n"
            "core0_start:\n" + "\n".join(c0) + "\n"
            "core1_start:\n" + "\n".join(c1) + "\n"
        )
        return cat, mode, src

    # Category 5: Producer-Consumer FENCE + FENCE.I + CLINT MMIO + Call/RAS Control-Flow
    cat = "C5_fencei_clint_ras"
    mode = "shared"
    w = {"alu": 10, "ld": 18, "st": 18, "amo": 12, "lrsc": 6, "fence": 8, "fence_i": 8, "clint": 8, "call": 8, "br": 6}
    offs = [0, 4, 8, 12, 16, 20, 24, 28]
    c0 = ["    li   x28, 0x20000"] + _init_core_regs(rng, 0) + _gen_body(rng, "c0", 50, "x28", offs, w, allow_clint=True, allow_fence_i=True, allow_call=True) + _mix_footer("c0", "x28")
    c1 = ["    li   x28, 0x20000"] + _init_core_regs(rng, 1) + _gen_body(rng, "c1", 50, "x28", offs, w, allow_clint=True, allow_fence_i=True, allow_call=True) + _mix_footer("c1", "x28")
    helper = (
        "helper_fn:\n"
        "    addi x2, x2, 13\n"
        "    xor  x3, x3, x2\n"
        "    ret\n"
    )
    src = (
        "    csrr x31, mhartid\n"
        "    bne  x31, x0, core1_start\n"
        "core0_start:\n" + "\n".join(c0) + "\n"
        "core1_start:\n" + "\n".join(c1) + "\n"
        + helper
    )
    return cat, mode, src


def run_one_seed(seed: int) -> Tuple[int, str, str, bool, str, Dict[str, int]]:
    cat, mode, src = generate_fuzz_program(seed)
    hex_p = BUILD / f"fuzz_{seed:03d}.hex"
    log_p = BUILD / f"fuzz_{seed:03d}.log"

    try:
        words = isa.assemble(src)
        isa.write_hex(words, hex_p, pad_words=512)

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

        pass_ln = ""
        for ln in res.stdout.splitlines():
            if "PASS [dual_ooo2w_soc]:" in ln:
                pass_ln = ln
        kv = dict(re.findall(r"(\w+)=([^\s]+)", pass_ln))

        st = chk.check_dual_trace(hex_p, log_p, mode=mode)

        # Clean up large trace log after passing (keeps build dir tidy; preserve on failure!)
        log_p.unlink(missing_ok=True)
        hex_p.unlink(missing_ok=True)

        rep0, rep1 = [int(x) for x in kv["replay"].split("/")]
        metrics = {
            "cycles": int(kv["cycles"]),
            "commits": st.c0_commits + st.c1_commits,
            "w_events": st.w_events,
            "bus_events": st.bus_events,
            "c2c": st.bus_c2c,
            "loads": st.loads_checked,
            "stlf": st.stlf_loads,
            "stores": st.stores_checked,
            "coalesced": st.coalesced_stores,
            "amos": st.amos_checked,
            "lr": st.lr_checked,
            "sc_ok": st.sc_ok,
            "sc_fail": st.sc_fail,
            "fences": st.fences_checked,
            "fence_i": st.fence_i_checked,
            "replays": rep0 + rep1,
            "mmio": st.mmio_ops,
        }
        return seed, cat, mode, True, "OK", metrics
    except Exception as exc:
        return seed, cat, mode, False, f"{type(exc).__name__}: {exc}", {}


def main() -> int:
    n_progs = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    BUILD.mkdir(parents=True, exist_ok=True)

    print("=== Phase 5a: dual_core_checker Negative Bug-Injection Self-Test ===")
    chk.run_self_test()

    print(f"\n=== Phase 5b: {n_progs}-Program Randomized Dual-Core Coherence & RVWMO Fuzzer (+JITTER) ===")
    t0 = time.time()
    workers = min(8, os.cpu_count() or 4)

    cat_counts: Dict[str, int] = {}
    totals: Dict[str, int] = {
        "cycles": 0, "commits": 0, "w_events": 0, "bus_events": 0, "c2c": 0,
        "loads": 0, "stlf": 0, "stores": 0, "coalesced": 0, "amos": 0,
        "lr": 0, "sc_ok": 0, "sc_fail": 0, "fences": 0, "fence_i": 0,
        "replays": 0, "mmio": 0,
    }
    failures: List[Tuple[int, str, str]] = []

    with concurrent.futures.ProcessPoolExecutor(max_workers=workers) as ex:
        futs = [ex.submit(run_one_seed, s) for s in range(n_progs)]
        done = 0
        for fut in concurrent.futures.as_completed(futs):
            seed, cat, mode, ok, msg, m = fut.result()
            done += 1
            if not ok:
                failures.append((seed, cat, msg))
                print(f"  [FAIL] seed={seed:03d} ({cat}, {mode}): {msg}")
            else:
                cat_counts[cat] = cat_counts.get(cat, 0) + 1
                for k in totals:
                    totals[k] += m[k]
            if done % 40 == 0 or done == n_progs:
                print(f"  Progress: {done}/{n_progs} complete ({len(failures)} failures, {time.time()-t0:.1f}s)")

    elapsed = time.time() - t0
    if failures:
        print(f"\nFAIL [Phase 5]: {len(failures)}/{n_progs} fuzz programs failed!")
        return 1

    summary = {
        "total_programs": n_progs,
        "passed": n_progs,
        "failed": 0,
        "elapsed_sec": round(elapsed, 2),
        "categories": dict(sorted(cat_counts.items())),
        "totals": totals,
    }
    (BUILD / "phase5_fuzz_results.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print("\nCategory Breakdown:")
    for cat, cnt in sorted(cat_counts.items()):
        print(f"  {cat:24s}: {cnt:3d}/{cnt:3d} PASS")
    print(
        f"\nPASS [Phase 5]: {n_progs}/{n_progs} Dual-Core RV32IMA_Zicsr_Zifencei Coherence Fuzz Programs PASSED "
        f"in {elapsed:.1f}s!\n"
        f"  Verified {totals['commits']:,} committed instrs across {totals['cycles']:,} dual-core cycles:\n"
        f"  - {totals['loads']:,} loads ({totals['stlf']:,} STLF), {totals['stores']:,} stores ({totals['coalesced']:,} MSHR-coalesced)\n"
        f"  - {totals['amos']:,} AMO*.W RMWs, {totals['lr']:,} LR.W, {totals['sc_ok']:,} SC-ok / {totals['sc_fail']:,} SC-fail\n"
        f"  - {totals['bus_events']:,} Ch-A bus events ({totals['c2c']:,} dirty C2C), "
        f"{totals['fences']:,} FENCE, {totals['fence_i']:,} FENCE.I, {totals['replays']:,} LQ snoop-replays, "
        f"{totals['mmio']:,} CLINT MMIO ops\n"
        f"  - 0 SWMR violations, 0 Shadow Data errors, 0 RVWMO linearizability errors, 0 hang watchdogs!"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
