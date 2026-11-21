#!/usr/bin/env python3
"""
Phase 6: Multicore Compiled C Benchmark Suite (P1-P6), 1C vs 2C Scaling & Ablation Study.

Compiles all 6 bare-metal multi-threaded C benchmarks with riscv64-unknown-elf-gcc
(-march=rv32ima -mabi=ilp32 -O2) for both 1-Core (NUM_CORES=1) and 2-Core (NUM_CORES=2),
verifies bit-exact architectural equivalence (x14 checksum match across ALL configurations +
0 SWMR/Shadow errors), and executes 4 quantitative hardware studies:

  1. Apples-to-Apples 1-Core (single_ooo2w_soc) vs. 2-Core (dual_ooo2w_soc) Scaling
     on identical L1I(4KB) + L1D(4KB) + Shared L2(32KB 2-Bank) + DRAM(L16) hierarchy
  2. False-Sharing (same 16B cache line) vs. Cache-Line-Padded (16B aligned) Ablation
  3. MOESI (ENABLE_MOESI_O=1) vs. MESI (ENABLE_MOESI_O=0) Coherence Ablation
  4. Shared 32 KB L2 (L2-ON vs. L2-Bypassed) & L1I Next-Line Prefetcher (ON vs. OFF) Ablations

Exports full structured metrics to build/bench_dual.json.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build" / "phase6_bench"

GCC = "/usr/bin/riscv64-unknown-elf-gcc"
OBJCOPY = "/usr/bin/riscv64-unknown-elf-objcopy"
PYTHON = "/home/moonis/auto_job_seeker/.eda_tools/envs/dft/bin/python3"

BIN_1C_L2    = ROOT / "build" / "v_p2_c0" / "Vp2_c0"
BIN_1C_NOL2  = ROOT / "build" / "v_p1_pf_l16" / "Vp1_pf_l16"
BIN_2C_MOESI = ROOT / "build" / "v_dual_soc" / "Vdual_soc"
BIN_2C_MESI  = ROOT / "build" / "v_dual_soc_mesi" / "Vdual_soc_mesi"
BIN_2C_NOPF  = ROOT / "build" / "v_dual_soc_nopf" / "Vdual_soc_nopf"

CFLAGS = [
    "-march=rv32ima", "-mabi=ilp32", "-O2",
    "-ffreestanding", "-nostdlib", "-nostartfiles", "-fno-builtin",
    "-Wall", "-Wextra", "-Werror",
    f"-I{ROOT / 'sw' / 'common'}",
    "-T", str(ROOT / "sw" / "common" / "link.ld"),
]


def compile_bench(c_file: Path, tag: str, defines: List[str]) -> Tuple[Path, int]:
    elf_p = BUILD / f"{tag}.elf"
    bin_p = BUILD / f"{tag}.bin"
    hex_p = BUILD / f"{tag}.hex"
    cmd_cc = [GCC] + CFLAGS + defines + [str(ROOT / "sw" / "common" / "crt0.S"), str(c_file), "-o", str(elf_p)]
    r = subprocess.run(cmd_cc, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"GCC FAIL [{tag}]:\n{r.stdout}\n{r.stderr}")
        sys.exit(1)
    r2 = subprocess.run([OBJCOPY, "-O", "binary", str(elf_p), str(bin_p)], capture_output=True, text=True)
    if r2.returncode != 0:
        print(f"OBJCOPY FAIL [{tag}]:\n{r2.stderr}")
        sys.exit(1)
    bin_bytes = bin_p.stat().st_size
    r3 = subprocess.run(
        [PYTHON, str(ROOT / "scripts" / "elf2hex.py"), str(bin_p), str(hex_p), "--pad-words", "4096"],
        capture_output=True, text=True,
    )
    if r3.returncode != 0:
        print(f"ELF2HEX FAIL [{tag}]:\n{r3.stdout}\n{r3.stderr}")
        sys.exit(1)
    return hex_p, bin_bytes


def run_1c_l2(hex_p: Path, expect_x14: str | None = None) -> Dict[str, object]:
    cmd = [str(BIN_1C_L2), f"+HEX={hex_p}", "+MAX_CYCLES=1500000"]
    if expect_x14:
        cmd.append(f"+EXPECT_X14={expect_x14}")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or "PASS [single_ooo2w_L2]:" not in r.stdout:
        print(f"1C_L2 FAIL [{hex_p.name}]:\n{r.stdout}\n{r.stderr}")
        sys.exit(1)
    ln = [s for s in r.stdout.splitlines() if "PASS [single_ooo2w_L2]:" in s][0]
    kv = dict(re.findall(r"(\w+)=([^\s]+)", ln))
    ic_m = re.search(r"ic=(\d+)/(\d+)\(pf=(\d+)\)", ln)
    dc_m = re.search(r"dc=(\d+)/(\d+)\(wb=(\d+)", ln)
    l2_h, l2_m = int(kv["l2_h"]), int(kv["l2_m"])
    return {
        "commits": int(kv["commits"]),
        "cycles": int(kv["cycles"]),
        "ipc": float(kv["IPC"]),
        "x14": kv["x14"].lower(),
        "ic_hits": int(ic_m.group(1)) if ic_m else 0,
        "ic_misses": int(ic_m.group(2)) if ic_m else 0,
        "ic_pf": int(ic_m.group(3)) if ic_m else 0,
        "dc_hits": int(dc_m.group(1)) if dc_m else 0,
        "dc_misses": int(dc_m.group(2)) if dc_m else 0,
        "l2_hits": l2_h,
        "l2_misses": l2_m,
        "l2_hit_pct": round(100.0 * l2_h / max(1, l2_h + l2_m), 1),
        "l2_wbs": int(kv["l2_wb"]),
        "l2_bc": int(kv["l2_bc"]),
        "dram_rds": int(kv["dram_r"]),
        "dram_wrs": int(kv["dram_w"]),
    }


def run_1c_nol2(hex_p: Path, expect_x14: str | None = None) -> Dict[str, object]:
    cmd = [str(BIN_1C_NOL2), f"+HEX={hex_p}", "+MAX_CYCLES=1500000"]
    if expect_x14:
        cmd.append(f"+EXPECT_X14={expect_x14}")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or "PASS [single_ooo2w_p1]:" not in r.stdout:
        print(f"1C_NOL2 FAIL [{hex_p.name}]:\n{r.stdout}\n{r.stderr}")
        sys.exit(1)
    ln = [s for s in r.stdout.splitlines() if "PASS [single_ooo2w_p1]:" in s][0]
    kv = dict(re.findall(r"(\w+)=([^\s]+)", ln))
    return {
        "commits": int(kv["commits"]),
        "cycles": int(kv["cycles"]),
        "ipc": float(kv["IPC"]),
        "x14": kv["x14"].lower(),
        "ic_misses": int(kv["ic_m"]),
        "dc_misses": int(kv["dc_m"]),
        "dc_wbs": int(kv["dc_wb"]),
        "dram_rds": int(kv["ic_m"]) + int(kv["dc_m"]),
        "dram_wrs": int(kv["dc_wb"]),
    }


def run_2c(sim_bin: Path, hex_p: Path, expect_x14: str) -> Dict[str, object]:
    exp_raw = expect_x14.replace("0x", "")
    cmd = [
        str(sim_bin),
        f"+HEX={hex_p}",
        "+MAX_CYCLES=1500000",
        f"+EXPECT_C0_X14={exp_raw}",
        f"+EXPECT_C1_X14={exp_raw}",
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or "PASS [dual_ooo2w_soc]:" not in r.stdout:
        print(f"2C FAIL [{sim_bin.name} / {hex_p.name}]:\n{r.stdout}\n{r.stderr}")
        sys.exit(1)
    ln = [s for s in r.stdout.splitlines() if "PASS [dual_ooo2w_soc]:" in s][0]
    kv = dict(re.findall(r"(\w+)=([^\s]+)", ln))
    ic_m0, ic_m1 = [int(x) for x in kv["ic_m"].split("/")]
    dc_m0, dc_m1 = [int(x) for x in kv["dc_m"].split("/")]
    rep0, rep1 = [int(x) for x in kv["replay"].split("/")]
    amo0, amo1 = [int(x) for x in kv["amo"].split("/")]
    l2_h, l2_m = int(kv["l2_h"]), int(kv["l2_m"])
    return {
        "cycles": int(kv["cycles"]),
        "c0_ret": int(kv["c0_ret"]),
        "c1_ret": int(kv["c1_ret"]),
        "total_ret": int(kv["c0_ret"]) + int(kv["c1_ret"]),
        "chip_ipc": float(kv["chip_IPC"]),
        "c0_x14": kv["c0_x14"].lower(),
        "c1_x14": kv["c1_x14"].lower(),
        "replay_c0": rep0,
        "replay_c1": rep1,
        "amo_c0": amo0,
        "amo_c1": amo1,
        "ic_m0": ic_m0,
        "ic_m1": ic_m1,
        "ic_misses": ic_m0 + ic_m1,
        "dc_m0": dc_m0,
        "dc_m1": dc_m1,
        "coh_gets": int(kv["coh_gets"]),
        "coh_getm": int(kv["coh_getm"]),
        "coh_upg": int(kv["coh_upg"]),
        "c2c": int(kv["c2c"]),
        "e2m": int(kv["e2m"]),
        "mesi_wb": int(kv["mesi_wb"]),
        "l2_hits": l2_h,
        "l2_misses": l2_m,
        "l2_hit_pct": round(100.0 * l2_h / max(1, l2_h + l2_m), 1),
        "l2_wbs": int(kv["l2_wb"]),
        "l2_bc": int(kv["l2_bc"]),
        "dram_rds": int(kv["dram_r"]),
        "dram_wrs": int(kv["dram_w"]),
    }


def main() -> int:
    BUILD.mkdir(parents=True, exist_ok=True)
    bench_dir = ROOT / "sw" / "c_bench"

    specs = [
        ("P1_parallel_matmul",      bench_dir / "p1_parallel_matmul.c",       []),
        ("P2_parallel_stencil_mlp", bench_dir / "p2_parallel_stencil_mlp.c",  []),
        ("P3_prod_cons_ring",       bench_dir / "p3_prod_cons_ring.c",        []),
        ("P4_parallel_hist_amo",    bench_dir / "p4_parallel_histogram_amo.c", []),
        ("P5_padded_counters",      bench_dir / "p5_false_sharing.c",         ["-DFALSE_SHARE=0"]),
        ("P5_false_shared",         bench_dir / "p5_false_sharing.c",         ["-DFALSE_SHARE=1"]),
        ("P6_parallel_crc_fsm",     bench_dir / "p6_parallel_crc_call_fsm.c", []),
    ]

    print("=== Phase 6: Compiling & Verifying Bare-Metal RV32IMA -O2 C Benchmarks (1C vs 2C) ===")
    results: Dict[str, Dict[str, object]] = {}

    for name, cpath, extra_defs in specs:
        hex_1c, sz1 = compile_bench(cpath, f"{name}_1c", ["-DNUM_CORES=1"] + extra_defs)
        hex_2c, sz2 = compile_bench(cpath, f"{name}_2c", ["-DNUM_CORES=2"] + extra_defs)

        m_1c    = run_1c_l2(hex_1c)
        exp_x14 = str(m_1c["x14"])
        m_nol2  = run_1c_nol2(hex_1c, expect_x14=exp_x14.replace("0x", ""))
        m_moesi = run_2c(BIN_2C_MOESI, hex_2c, expect_x14=exp_x14)
        m_mesi  = run_2c(BIN_2C_MESI,  hex_2c, expect_x14=exp_x14)
        m_nopf  = run_2c(BIN_2C_NOPF,  hex_2c, expect_x14=exp_x14)

        sp_2c  = round(int(m_1c["cycles"]) / int(m_moesi["cycles"]), 3)
        sp_l2  = round(int(m_nol2["cycles"]) / int(m_1c["cycles"]), 3)
        sp_tot = round(int(m_nol2["cycles"]) / int(m_moesi["cycles"]), 3)

        results[name] = {
            "bin_bytes_2c": sz2,
            "checksum_x14": exp_x14,
            "c1_no_l2": m_nol2,
            "c1_l2": m_1c,
            "c2_moesi": m_moesi,
            "c2_mesi": m_mesi,
            "c2_nopf": m_nopf,
            "speedup_2c_vs_1c": sp_2c,
            "speedup_l2_on_1c": sp_l2,
            "speedup_2c_l2_vs_1c_nol2": sp_tot,
        }

    # Verify exact checksum match between P5_padded and P5_false_shared too!
    assert results["P5_padded_counters"]["checksum_x14"] == results["P5_false_shared"]["checksum_x14"], \
        "P5 padded vs false-shared checksum mismatch!"

    # ---- Table 1: 1-Core vs. 2-Core Scaling ----
    print("\n--- Study 1: Apples-to-Apples 1-Core vs. 2-Core Scaling (L1:L2:DRAM = 1:4:16, GCC -O2) ---")
    print(f"{'Benchmark':25s} | {'1C Cyc':>8s} | {'1C IPC':>6s} | {'2C Cyc':>8s} | {'2C IPC':>6s} | {'Speedup':>7s} | {'L2 Hit%':>7s} | {'C2C':>5s} | {'x14 Checksum':>10s}")
    print("-" * 105)
    for name, _, _ in specs:
        r = results[name]
        c1, c2 = r["c1_l2"], r["c2_moesi"]
        print(
            f"{name:25s} | {c1['cycles']:8d} | {c1['ipc']:6.3f} | {c2['cycles']:8d} | {c2['chip_ipc']:6.3f} | "
            f"{r['speedup_2c_vs_1c']:6.2f}x | {c2['l2_hit_pct']:6.1f}% | {c2['c2c']:5d} | {r['checksum_x14']:>10s}"
        )

    # ---- Table 2: False-Sharing vs. Cache-Line-Padded ----
    fs = results["P5_false_shared"]
    pd = results["P5_padded_counters"]
    pad_gain = fs["c2_moesi"]["cycles"] / pd["c2_moesi"]["cycles"]
    print("\n--- Study 2: False-Sharing vs. Cache-Line-Padded Counters (P5) ---")
    print(f"  1-Core Baseline      : {pd['c1_l2']['cycles']:7d} cyc  (IPC={pd['c1_l2']['ipc']:.3f})")
    print(f"  2-Core False-Shared  : {fs['c2_moesi']['cycles']:7d} cyc  (IPC={fs['c2_moesi']['chip_ipc']:.3f}, {fs['speedup_2c_vs_1c']:.2f}x vs 1C, C2C={fs['c2_moesi']['c2c']}, GetM={fs['c2_moesi']['coh_getm']}, Upg={fs['c2_moesi']['coh_upg']})")
    print(f"  2-Core Padded (16B)  : {pd['c2_moesi']['cycles']:7d} cyc  (IPC={pd['c2_moesi']['chip_ipc']:.3f}, {pd['speedup_2c_vs_1c']:.2f}x vs 1C, C2C={pd['c2_moesi']['c2c']}, {pad_gain:.2f}x faster than False-Shared!)")

    # ---- Table 3: MOESI vs. MESI Ablation ----
    print("\n--- Study 3: MOESI (ENABLE_MOESI_O=1) vs. MESI (ENABLE_MOESI_O=0) Ablation ---")
    print(f"{'Benchmark':25s} | {'MOESI Cyc':>9s} | {'MESI Cyc':>9s} | {'MOESI L2 Ops':>12s} | {'MESI L2 Ops':>11s} | {'MESI Extra WB':>13s} | {'L2 Saved':>8s}")
    print("-" * 102)
    for name, _, _ in specs:
        m1 = results[name]["c2_moesi"]
        m0 = results[name]["c2_mesi"]
        l2_1 = m1["l2_hits"] + m1["l2_misses"]
        l2_0 = m0["l2_hits"] + m0["l2_misses"]
        sav = 100.0 * (l2_0 - l2_1) / max(1, l2_0)
        print(
            f"{name:25s} | {m1['cycles']:9d} | {m0['cycles']:9d} | {l2_1:12d} | {l2_0:11d} | "
            f"{m0['mesi_wb']:13d} | {sav:7.1f}%"
        )

    # ---- Table 4: Shared 32 KB L2 & L1I Next-Line Prefetcher Ablations ----
    print("\n--- Study 4a: Shared 32 KB L2 Cache Ablation (1C No-L2 vs. 1C+L2 vs. 2C+L2) ---")
    print(f"{'Benchmark':25s} | {'1C NoL2':>8s} | {'1C+L2':>8s} | {'L2 Gain':>7s} | {'DRAM Rd (NoL2->L2)':>18s} | {'2C+L2':>8s} | {'Total Gain':>10s}")
    print("-" * 100)
    for name, _, _ in specs:
        r = results[name]
        n0, c1, c2 = r["c1_no_l2"], r["c1_l2"], r["c2_moesi"]
        dr = f"{n0['dram_rds']} -> {c1['dram_rds']}"
        print(
            f"{name:25s} | {n0['cycles']:8d} | {c1['cycles']:8d} | {r['speedup_l2_on_1c']:6.2f}x | {dr:>18s} | "
            f"{c2['cycles']:8d} | {r['speedup_2c_l2_vs_1c_nol2']:9.2f}x"
        )

    print("\n--- Study 4b: L1I Next-Line Prefetcher Ablation on 2-Core (ENABLE_PREFETCH=0 vs. 1) ---")
    print(f"{'Benchmark':25s} | {'No-PF Cyc':>9s} | {'PF-ON Cyc':>9s} | {'No-PF L1I Miss':>14s} | {'PF-ON L1I Miss':>14s} | {'Miss Saved':>10s}")
    print("-" * 96)
    for name, _, _ in specs:
        pf0 = results[name]["c2_nopf"]
        pf1 = results[name]["c2_moesi"]
        saved = pf0["ic_misses"] - pf1["ic_misses"]
        pct = 100.0 * saved / max(1, pf0["ic_misses"])
        print(
            f"{name:25s} | {pf0['cycles']:9d} | {pf1['cycles']:9d} | {pf0['ic_misses']:14d} | "
            f"{pf1['ic_misses']:14d} | {saved:4d} ({pct:4.1f}%)"
        )

    out_json = ROOT / "build" / "bench_dual.json"
    out_json.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(f"\nPASS [Phase 6]: All 7 compiled C benchmark variants verified bit-exact across 5 hardware configs! Exported {out_json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
