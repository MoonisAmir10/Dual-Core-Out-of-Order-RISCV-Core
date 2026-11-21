#!/usr/bin/env python3
"""
Phase U8: Full-SoC Flagship Multi-Threaded C Benchmarks (S1-S4) & Non-Blocking Audit!

Compiles and executes all 4 flagship RV32IMAF (-march=rv32imaf -mabi=ilp32f -O2)
system-level workloads on single_ooo2w_soc (1C) and dual_ooo2w_soc (2C + MOESI +
AMBA APB4 + PLIC + I/O-Coherent AXI4 DMA + always-on 3-master coh_checker):

  S1_fp_sgemm_2c           : 24x24 & 32x32 IEEE-754 float SGEMM (1C vs 2C scaling, >3.0 chip IPC)
  S2_dma_double_buf_fp_dsp : 8x 2 KB streaming 8-tap FP FIR + dot-product
                             (Async I/O-Coherent DMA + PLIC ISRs vs. CPU-Copy baseline)
  S3_irq_latency_gpio_uart : Exact CLINT MTIP, PLIC MEIP & Cross-Core IPI RTT latencies,
                             GPIO/UART console, and MIE=0 vs MIE=1 zero-stall OoO IPC audit
  S4_trap_stress_mixed     : Simultaneous 6-class traps (illegal emul, misalign, APB4 fault,
                             ECALL) + live CLINT/PLIC/GPIO/DMA IRQs + RV32IMAF + RV32A!

Exports full structured metrics to build/bench_soc_u8.json.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build" / "phase_u8"

GCC = "/usr/bin/riscv64-unknown-elf-gcc"
OBJCOPY = "/usr/bin/riscv64-unknown-elf-objcopy"
PYTHON = "/home/moonis/auto_job_seeker/.eda_tools/envs/dft/bin/python3"

BIN_1C = ROOT / "build" / "v_p2_c0" / "Vp2_c0"
BIN_2C = ROOT / "build" / "v_dual_soc" / "Vdual_soc"

CFLAGS_FP = [
    "-march=rv32imaf", "-mabi=ilp32f", "-O2",
    "-ffreestanding", "-nostdlib", "-nostartfiles", "-fno-builtin",
    "-Wall", "-Wextra", "-Werror",
    f"-I{ROOT / 'sw' / 'common'}", f"-I{ROOT / 'sw' / 'hal'}",
    "-T", str(ROOT / "sw" / "common" / "link.ld"),
]


def compile_soc(srcs: List[str], stem: str, defines: List[str] | None = None) -> Path:
    BUILD.mkdir(parents=True, exist_ok=True)
    elf = BUILD / f"{stem}.elf"
    bin_p = BUILD / f"{stem}.bin"
    hex_p = BUILD / f"{stem}.hex"
    cmd = [GCC] + CFLAGS_FP + (defines or []) + srcs + ["-o", str(elf)]
    r = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
    if r.returncode != 0:
        print(f"GCC FAIL [{stem}]:\n{r.stdout}\n{r.stderr}")
        sys.exit(1)
    subprocess.run([OBJCOPY, "-O", "binary", str(elf), str(bin_p)], cwd=str(ROOT), check=True)
    subprocess.run(
        [PYTHON, str(ROOT / "scripts" / "elf2hex.py"), str(bin_p), str(hex_p), "--pad-words", "4096"],
        cwd=str(ROOT),
        check=True,
        capture_output=True,
    )
    return hex_p


def run_1c(hex_p: Path, max_cyc: int = 300000) -> Dict[str, object]:
    cmd = [str(BIN_1C), f"+HEX={hex_p}", f"+MAX_CYCLES={max_cyc}"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or "PASS [single_ooo2w_L2]:" not in r.stdout:
        print(f"FAIL [1C {hex_p.name}]:\n{r.stdout}\n{r.stderr}")
        sys.exit(1)
    ln = [s for s in r.stdout.splitlines() if "PASS [single_ooo2w_L2]:" in s][0]
    kv = dict(re.findall(r"(\w+)=([^\s]+)", ln))
    return {
        "cycles": int(kv["cycles"]),
        "commits": int(kv["commits"]),
        "ipc": float(kv["IPC"]),
        "x14": kv["x14"].lower(),
        "l2_h": int(kv["l2_h"]),
        "l2_m": int(kv["l2_m"]),
        "dram_r": int(kv["dram_r"]),
        "dram_w": int(kv["dram_w"]),
    }


def run_2c(hex_p: Path, extra_args: List[str] | None = None, max_cyc: int = 400000) -> Dict[str, object]:
    cmd = [str(BIN_2C), f"+HEX={hex_p}", f"+MAX_CYCLES={max_cyc}"] + (extra_args or [])
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or "PASS [dual_ooo2w_soc]:" not in r.stdout:
        print(f"FAIL [2C {hex_p.name}]:\n{r.stdout}\n{r.stderr}")
        sys.exit(1)
    ln = [s for s in r.stdout.splitlines() if "PASS [dual_ooo2w_soc]:" in s][0]
    kv = dict(re.findall(r"(\w+)=([^\s]+)", ln))
    rpl = kv.get("replay", "0/0").split("/")
    return {
        "cycles": int(kv["cycles"]),
        "c0_ret": int(kv["c0_ret"]),
        "c1_ret": int(kv["c1_ret"]),
        "chip_ipc": float(kv["chip_IPC"]),
        "c0_x14": kv["c0_x14"].lower(),
        "c1_x14": kv["c1_x14"].lower(),
        "c2c": int(kv["c2c"]),
        "dma_r": int(kv["dma_r"]),
        "dma_w": int(kv["dma_w"]),
        "l2_h": int(kv["l2_h"]),
        "l2_m": int(kv["l2_m"]),
        "l2_bc": int(kv["l2_bc"]),
        "dram_r": int(kv["dram_r"]),
        "dram_w": int(kv["dram_w"]),
        "replay_c0": int(rpl[0]),
        "replay_c1": int(rpl[1]),
        "raw_line": ln,
    }


def main() -> int:
    print("=== Phase U8: Full-SoC Flagship RV32IMAF Benchmarks & Non-Blocking Audit ===")
    results: Dict[str, object] = {}

    # ------------------------------------------------------------------------
    # S1: 24x24 & 32x32 IEEE-754 Float SGEMM (1C vs. 2C Scaling)
    # ------------------------------------------------------------------------
    print("\n--- S1: Parallel IEEE-754 Float SGEMM (1C vs. 2C Non-Blocking FP Scaling) ---")
    s1_res: Dict[str, object] = {}
    for n in (24, 32):
        hex_p = compile_soc(["sw/common/crt0.S", "sw/c_soc/s1_fp_sgemm_2c.c"], f"s1_sgemm_{n}", [f"-DSGEMM_N={n}"])
        r1 = run_1c(hex_p, max_cyc=200000)
        r2 = run_2c(hex_p, max_cyc=200000)
        assert r1["x14"] == r2["c0_x14"] == r2["c1_x14"], (
            f"S1_{n} checksum mismatch! 1C={r1['x14']} 2C={r2['c0_x14']}/{r2['c1_x14']}"
        )
        speedup = round(r1["cycles"] / max(1, r2["cycles"]), 2)
        assert r2["chip_ipc"] >= 3.0, f"S1_{n}: chip_IPC={r2['chip_ipc']} < 3.0!"
        s1_res[f"{n}x{n}"] = {
            "1c": r1, "2c": r2, "speedup": speedup, "hash": r1["x14"],
        }
        print(
            f"  [OK] S1_fp_sgemm_{n}x{n}: hash={r1['x14']} | "
            f"1C={r1['cycles']:,d} cyc ({r1['ipc']:.3f} IPC) -> "
            f"2C={r2['cycles']:,d} cyc ({r2['chip_ipc']:.3f} chip IPC, {speedup:.2f}x speedup!)"
        )
    results["S1_fp_sgemm_2c"] = s1_res

    # ------------------------------------------------------------------------
    # S2: Double-Buffered Streaming FP DSP (Async Coherent DMA vs. CPU Copy)
    # ------------------------------------------------------------------------
    print("\n--- S2: Double-Buffered Streaming FP DSP (Async Coherent DMA vs. CPU Copy) ---")
    hal_srcs = ["sw/common/crt0.S", "sw/common/trap.S", "sw/hal/hal_irq.c", "sw/hal/hal_uart.c",
                "sw/c_soc/s2_dma_double_buf_fp_dsp.c"]
    hex_dma = compile_soc(hal_srcs, "s2_dsp_dma", ["-DUSE_DMA=1"])
    hex_cpu = compile_soc(hal_srcs, "s2_dsp_cpu", ["-DUSE_DMA=0"])

    uart_dma = BUILD / "s2_dma.uart"
    uart_cpu = BUILD / "s2_cpu.uart"
    r_dma = run_2c(hex_dma, [f"+UART_OUT={uart_dma}"], max_cyc=400000)
    r_cpu = run_2c(hex_cpu, [f"+UART_OUT={uart_cpu}"], max_cyc=400000)

    assert r_dma["c0_x14"] == r_dma["c1_x14"] == r_cpu["c0_x14"] == r_cpu["c1_x14"], (
        f"S2 bit-exact mismatch! DMA={r_dma['c0_x14']} CPU={r_cpu['c0_x14']}"
    )
    txt_dma = uart_dma.read_text(encoding="utf-8", errors="replace").strip()
    txt_cpu = uart_cpu.read_text(encoding="utf-8", errors="replace").strip()
    m_dma = dict(re.findall(r"(\w+)=([^\s]+)", txt_dma))
    m_cpu = dict(re.findall(r"(\w+)=([^\s]+)", txt_cpu))
    pipe_dma = int(m_dma["pipe_cyc"])
    pipe_cpu = int(m_cpu["pipe_cyc"])
    pipe_speedup = round(pipe_cpu / max(1, pipe_dma), 2)
    assert int(m_dma["ch0_irqs"]) == 8 and int(m_dma["ch1_irqs"]) == 8, f"Expected 8+8 DMA IRQs: {txt_dma}"
    assert r_dma["dma_r"] >= 2048 and r_dma["dma_w"] >= 2048, f"Expected >=2048 DMA beats: {r_dma}"
    assert r_dma["c2c"] > 0, "Expected dirty C2C snoops on zero-flush DMA drain!"

    results["S2_dma_double_buf_fp_dsp"] = {
        "hash": r_dma["c0_x14"],
        "dma_pipe_cyc": pipe_dma,
        "cpu_pipe_cyc": pipe_cpu,
        "pipe_speedup": pipe_speedup,
        "dma_run": r_dma,
        "cpu_run": r_cpu,
        "uart_dma": txt_dma,
        "uart_cpu": txt_cpu,
    }
    print(
        f"  [OK] S2_dsp_CPU : pipe={pipe_cpu:,d} cyc, total={r_cpu['cycles']:,d} cyc "
        f"(chip_IPC={r_cpu['chip_ipc']:.3f}, hash={r_cpu['c0_x14']})"
    )
    print(
        f"  [OK] S2_dsp_DMA : pipe={pipe_dma:,d} cyc, total={r_dma['cycles']:,d} cyc "
        f"(chip_IPC={r_dma['chip_ipc']:.3f}, pipe_gain={pipe_speedup:.2f}x, "
        f"dma_r/w={r_dma['dma_r']}/{r_dma['dma_w']}, c2c={r_dma['c2c']}, PLIC_IRQs=8+8, bit-exact!)"
    )

    # ------------------------------------------------------------------------
    # S3: Hardware Interrupt Dispatch Latency, IPI RTT & Zero-Stall IPC Audit
    # ------------------------------------------------------------------------
    print("\n--- S3: Hardware Interrupt Latency, Cross-Core IPI RTT & Zero-Stall IPC Audit ---")
    hex_s3 = compile_soc(
        ["sw/common/crt0.S", "sw/common/trap.S", "sw/hal/hal_irq.c", "sw/hal/hal_uart.c",
         "sw/c_soc/s3_irq_latency_gpio_uart.c"],
        "s3_irq_lat",
    )
    uart_s3 = BUILD / "s3_irq.uart"
    r_s3 = run_2c(
        hex_s3,
        ["+EXPECT_C0_X14=00083386", "+EXPECT_C1_X14=00183386", "+GPIO_STIM", f"+UART_OUT={uart_s3}"],
        max_cyc=250000,
    )
    txt_s3 = uart_s3.read_text(encoding="utf-8", errors="replace").strip()
    kv3 = dict(re.findall(r"(\w+)=([^\s]+)", txt_s3))
    base_cyc, base_ret = [int(x) for x in kv3["base"].split("/")]
    idle_cyc, idle_ret = [int(x) for x in kv3["idle"].split("/")]
    busy_cyc, busy_ret = [int(x) for x in kv3["busy"].split("/")]
    base_ipc = round(base_ret / max(1, base_cyc), 3)
    idle_ipc = round(idle_ret / max(1, idle_cyc), 3)
    busy_ipc = round(busy_ret / max(1, busy_cyc), 3)
    idle_delta_pct = round(100.0 * (idle_cyc - base_cyc) / max(1, base_cyc), 2)
    assert abs(idle_delta_pct) <= 1.0, f"S3: MIE=1 idle overhead {idle_delta_pct}% > 1%!"
    assert int(kv3["gpio"]) == 3 and int(kv3["t0"]) >= 2, f"S3 unexpected IRQ counts: {txt_s3}"

    results["S3_irq_latency_gpio_uart"] = {
        "uart": txt_s3,
        "base_cyc": base_cyc, "base_ipc": base_ipc,
        "idle_cyc": idle_cyc, "idle_ipc": idle_ipc, "idle_overhead_pct": idle_delta_pct,
        "busy_cyc": busy_cyc, "busy_ipc": busy_ipc,
        "t0_irqs": int(kv3["t0"]), "gpio_irqs": int(kv3["gpio"]),
        "mtip_lat_min_avg_max": kv3["mtip"],
        "plic_lat_min_avg_max": kv3["plic"],
        "ipi_rtt_min_avg_max": kv3["ipi_rtt"],
        "run": r_s3,
    }
    print(f"  [OK] S3_irq_lat : {txt_s3}")
    print(
        f"       Zero-Stall Audit: MIE=0 {base_cyc} cyc ({base_ipc:.3f} IPC) vs. "
        f"MIE=1 idle {idle_cyc} cyc ({idle_ipc:.3f} IPC, delta={idle_delta_pct:+.2f}%) | "
        f"MTIP={kv3['mtip']} cyc | PLIC={kv3['plic']} cyc | IPI_RTT={kv3['ipi_rtt']} cyc"
    )

    # ------------------------------------------------------------------------
    # S4: Full-SoC Mixed Trap + IRQ + Coherent DMA + RV32IMAF + RV32A Stress
    # ------------------------------------------------------------------------
    print("\n--- S4: Full-SoC Mixed Trap + IRQ + Coherent DMA + RV32IMAF + RV32A Stress ---")
    hex_s4 = compile_soc(
        ["sw/common/crt0.S", "sw/common/trap.S", "sw/hal/hal_irq.c", "sw/hal/hal_uart.c",
         "sw/c_soc/s4_trap_stress_mixed.c"],
        "s4_trap_mixed",
    )
    uart_s4 = BUILD / "s4_mixed.uart"
    r_s4 = run_2c(
        hex_s4,
        ["+EXPECT_C0_X14=000840cc", "+EXPECT_C1_X14=001840cc", "+JITTER", "+GPIO_STIM", f"+UART_OUT={uart_s4}"],
        max_cyc=300000,
    )
    txt_s4 = uart_s4.read_text(encoding="utf-8", errors="replace").strip()
    kv4 = dict(re.findall(r"(\w+)=([^\s]+)", txt_s4))
    assert int(kv4["traps"]) == 144, f"Expected 144 precise traps (6*12*2): {txt_s4}"
    assert int(kv4["amo"]) == 192, f"Expected 192 contended AMOs: {txt_s4}"
    assert int(kv4["gpio"]) == 3 and int(kv4["clint_t"]) > 0 and int(kv4["plic_t0"]) > 0, f"S4 IRQs: {txt_s4}"
    assert kv4["dma"] == "12/12" and kv4["fp"] == "OK", f"S4 DMA/FP: {txt_s4}"

    results["S4_trap_stress_mixed"] = {
        "uart": txt_s4,
        "run": r_s4,
    }
    print(f"  [OK] S4_mixed   : {txt_s4}")
    print(
        f"       Coherence & DMA: cycles={r_s4['cycles']:,d} chip_IPC={r_s4['chip_ipc']:.3f} "
        f"dma_r/w={r_s4['dma_r']}/{r_s4['dma_w']} c2c={r_s4['c2c']} (0 SWMR/Shadow errors!)"
    )

    out_p = ROOT / "build" / "bench_soc_u8.json"
    out_p.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(f"\nPASS [Phase U8]: ALL 4 Flagship Full-SoC Benchmarks PASSED! Exported {out_p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
