#!/usr/bin/env python3
"""
Phase U2 End-to-End Verification: Standardized AMBA APB4 Fabric, 32-Bit PMA,
Upgraded APB4 CLINT, APB4 SoC Control/PMU & Precise mcause=5/7 Access Faults!

Tests:
  1. U2-01 (1C): Compiled GCC -O2 C program on single_ooo2w_soc (+JITTER):
     - apb4_soc_ctrl (SOC_ID, CORE_COUNT=1, FEATURE_MASK, byte-strobed SCRATCH, live PMU)
     - apb4_clint (mtime, mtimecmp0, msip0, scratch)
     - 4x Load Access Faults (mcause=5) + 6x Store/RO/AMO Access Faults (mcause=7)
       caught & recovered cleanly by C __attribute__((interrupt("machine"))) handler!
  2. U2-02 (2C): Exact same C binary on dual_ooo2w_soc (+JITTER + ALWAYS-ON SWMR/Shadow):
     - Both Core 0 & Core 1 concurrently hammer the shared AMBA APB4 bridge (20 total
       precise bus-fault traps across 2 cores + concurrent CLINT/SOC_CTRL accesses)!
  3. U2-03 (SYS_HALT): Verifies clean software halt via apb4_soc_ctrl.TOHOST + SYS_HALT
     while mtvec != 0 on BOTH 1C and 2C SoCs!
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import List

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build" / "phase_u2"
BIN_SINGLE = ROOT / "build" / "v_p2_c0" / "Vp2_c0"
BIN_DUAL = ROOT / "build" / "v_dual_soc" / "Vdual_soc"


def run_cmd(cmd: List[str], tag: str) -> str:
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


def compile_c(src: str, stem: str) -> Path:
    BUILD.mkdir(parents=True, exist_ok=True)
    elf = BUILD / f"{stem}.elf"
    bin_p = BUILD / f"{stem}.bin"
    hex_p = BUILD / f"{stem}.hex"
    subprocess.run(
        [
            "/usr/bin/riscv64-unknown-elf-gcc",
            "-march=rv32ima", "-mabi=ilp32", "-O2",
            "-ffreestanding", "-nostdlib", "-nostartfiles",
            "-Wall", "-Wextra", "-Isw/common", "-T", "sw/common/link.ld",
            "sw/common/crt0.S", src,
            "-o", str(elf),
        ],
        cwd=str(ROOT),
        check=True,
    )
    subprocess.run(
        ["/usr/bin/riscv64-unknown-elf-objcopy", "-O", "binary", str(elf), str(bin_p)],
        cwd=str(ROOT),
        check=True,
    )
    subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "elf2hex.py"), str(bin_p), str(hex_p), "--pad-words", "2048"],
        cwd=str(ROOT),
        check=True,
    )
    return hex_p


def main() -> int:
    print("=== Phase U2 End-to-End Verification: AMBA APB4 Fabric, Peripherals & mcause=5/7 Faults ===")
    apb_hex = compile_c("sw/c_bench/test_apb4_soc_c.c", "test_apb4_soc_c")
    halt_hex = compile_c("sw/c_bench/test_apb4_halt_c.c", "test_apb4_halt_c")

    # U2-01: Single-Core APB4 + CLINT + SOC_CTRL + 10x mcause=5/7 C trap recovery (+JITTER)
    out1 = run_cmd(
        [str(BIN_SINGLE), f"+HEX={apb_hex}", "+EXPECT_X14=00001406", "+MAX_CYCLES=50000", "+JITTER"],
        "PASS [single_ooo2w_L2]",
    )
    print(f"  [OK] U2-01_apb4_c_1C (4x mcause=5 + 6x mcause=7 recovered): {out1}")

    # U2-02: Dual-Core Concurrent APB4 Bridge + 20x mcause=5/7 across 2 cores (+JITTER + SWMR)
    out2 = run_cmd(
        [
            str(BIN_DUAL),
            f"+HEX={apb_hex}",
            "+EXPECT_C0_X14=00002406",
            "+EXPECT_C1_X14=00012406",
            "+MAX_CYCLES=50000",
            "+JITTER",
        ],
        "PASS [dual_ooo2w_soc]",
    )
    print(f"  [OK] U2-02_apb4_c_2C (20 concurrent bus-fault traps across 2 cores): {out2}")

    # U2-03: SYS_HALT & TOHOST clean software halt while mtvec != 0 (1C & 2C)
    out3 = run_cmd(
        [str(BIN_SINGLE), f"+HEX={halt_hex}", "+EXPECT_X14=0000cafe", "+MAX_CYCLES=20000", "+JITTER"],
        "PASS [single_ooo2w_L2]",
    )
    print(f"  [OK] U2-03a_sys_halt_1C (mtvec!=0 -> SYS_HALT): {out3}")

    out4 = run_cmd(
        [str(BIN_DUAL), f"+HEX={halt_hex}", "+EXPECT_C0_X14=0000cafe", "+MAX_CYCLES=20000", "+JITTER"],
        "PASS [dual_ooo2w_soc]",
    )
    print(f"  [OK] U2-03b_sys_halt_2C (mtvec!=0 -> SYS_HALT): {out4}")

    print("PASS [test_phase_u2_apb4]: All Phase U2 AMBA APB4 fabric & mcause=5/7 fault suites passed!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
