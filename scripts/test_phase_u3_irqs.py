#!/usr/bin/env python3
"""
Phase U3 End-to-End Verification: Standard RISC-V Dual-Core PLIC, Live MSIP/MTIP/MEIP
Interrupts, sw/common/trap.S Trampoline, C HAL (sw/hal/), Vectored mtvec & Zero-Stall IPC!

Tests:
  1. U3-01a/b (Direct mtvec — 1C & 2C + JITTER + SWMR/Shadow):
     - Periodic MTIP timer C ISRs (Core 0: 600 cyc, Core 1: 450 cyc) concurrent with
       deterministic foreground ALU/MUL compute (proving zero state corruption!).
     - 4x Cross-Core MSIP IPI ping-pong round-trips (Core 0 <-> Core 1).
     - PLIC Priority Ordering & Threshold Masking (Src 5 prio=6 preempts & services
       before masked Src 2 prio=3; lowering threshold immediately services Src 2!).
     - Dual-Core Shared PLIC Atomic Claim Race (6x edge pulses on Src 9 enabled on
       BOTH cores -> strictly 6 total claims, 0 lost, 0 double-claimed!).
  2. U3-02a/b (Vectored mtvec MODE=01 — 1C & 2C + JITTER):
     - Verifies hardware jumps to exact BASE+0 (ECALL), BASE+0x0C (MSIP),
       BASE+0x1C (MTIP), BASE+0x2C (PLIC MEIP)!
  3. U3-03 (Zero-Stall IPC Audit on Dual-Core 16x16 Parallel Matmul):
     - Compares MIE=0 baseline vs MIE=1+mtvec+PLIC armed (idle IRQs) -> verifies
       identical checksum and ~0.0% cycle/IPC overhead!
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path
from typing import List

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build" / "phase_u3"
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


def compile_hal_c(src: str, stem: str, extra_flags: List[str] | None = None) -> Path:
    BUILD.mkdir(parents=True, exist_ok=True)
    elf = BUILD / f"{stem}.elf"
    bin_p = BUILD / f"{stem}.bin"
    hex_p = BUILD / f"{stem}.hex"
    cmd = [
        "/usr/bin/riscv64-unknown-elf-gcc",
        "-march=rv32ima", "-mabi=ilp32", "-O2",
        "-ffreestanding", "-nostdlib", "-nostartfiles",
        "-Wall", "-Wextra", "-Werror",
        "-Isw/common", "-Isw/hal", "-T", "sw/common/link.ld",
        "sw/common/crt0.S", "sw/common/trap.S", "sw/hal/hal_irq.c", src,
        "-o", str(elf),
    ]
    if extra_flags:
        cmd.extend(extra_flags)
    subprocess.run(cmd, cwd=str(ROOT), check=True)
    subprocess.run(
        ["/usr/bin/riscv64-unknown-elf-objcopy", "-O", "binary", str(elf), str(bin_p)],
        cwd=str(ROOT),
        check=True,
    )
    subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "elf2hex.py"), str(bin_p), str(hex_p), "--pad-words", "4096"],
        cwd=str(ROOT),
        check=True,
    )
    return hex_p


def parse_kv(line: str) -> dict[str, str]:
    return dict(re.findall(r"(\w+)=([^\s]+)", line))


def main() -> int:
    print("=== Phase U3 End-to-End Verification: RISC-V Dual-Core PLIC, C ISRs & Zero-Stall IPC ===")

    # 1. Direct-mode full MTIP + MSIP + PLIC MEIP + Claim Race C Suite (1C & 2C)
    irqs_hex = compile_hal_c("sw/c_soc/test_irqs_c.c", "test_irqs_c")

    # Expected checksum: (ncores<<16) | (ticks=5<<12) | (ipi=4<<8) | (prio=2<<4) | (shared=6)
    out1 = run_cmd(
        [str(BIN_SINGLE), f"+HEX={irqs_hex}", "+EXPECT_X14=00015426", "+MAX_CYCLES=80000", "+JITTER"],
        "PASS [single_ooo2w_L2]",
    )
    print(f"  [OK] U3-01a_direct_irqs_1C (MTIP+MSIP+PLIC Prio/Thresh): {out1}")

    out2 = run_cmd(
        [
            str(BIN_DUAL),
            f"+HEX={irqs_hex}",
            "+EXPECT_C0_X14=00025426",
            "+EXPECT_C1_X14=00125426",
            "+MAX_CYCLES=100000",
            "+JITTER",
        ],
        "PASS [dual_ooo2w_soc]",
    )
    print(f"  [OK] U3-01b_direct_irqs_2C (2C MTIP + IPI Ping-Pong + PLIC Race): {out2}")

    # 2. Vectored-mode mtvec (MODE=01) Suite (1C & 2C)
    vec_hex = compile_hal_c("sw/c_soc/test_irqs_vectored_c.c", "test_irqs_vectored_c")

    # Expected: (ncores<<16) | (exc=1<<12) | (msi=1<<8) | (mti=1<<4) | (mei=1)
    out3 = run_cmd(
        [str(BIN_SINGLE), f"+HEX={vec_hex}", "+EXPECT_X14=00011111", "+MAX_CYCLES=50000", "+JITTER"],
        "PASS [single_ooo2w_L2]",
    )
    print(f"  [OK] U3-02a_vectored_mtvec_1C (ECALL+MSI+MTI+MEI): {out3}")

    out4 = run_cmd(
        [
            str(BIN_DUAL),
            f"+HEX={vec_hex}",
            "+EXPECT_C0_X14=00021111",
            "+EXPECT_C1_X14=00121111",
            "+MAX_CYCLES=50000",
            "+JITTER",
        ],
        "PASS [dual_ooo2w_soc]",
    )
    print(f"  [OK] U3-02b_vectored_mtvec_2C (2C Vectored ECALL+MSI+MTI+MEI): {out4}")

    # 3. Zero-Stall IPC Audit: Dual-Core 16x16 Parallel Matmul (MIE=0 vs MIE=1 Armed Idle)
    ipc0_hex = compile_hal_c("sw/c_soc/test_irq_ipc_audit_c.c", "test_irq_ipc_off", ["-DIRQ_ARMED=0"])
    ipc1_hex = compile_hal_c("sw/c_soc/test_irq_ipc_audit_c.c", "test_irq_ipc_on", ["-DIRQ_ARMED=1"])

    def run_audit(hex_p: Path) -> tuple[dict[str, str], dict[int, int]]:
        cmd = [
            str(BIN_DUAL), f"+HEX={hex_p}", "+MAX_CYCLES=100000",
            "+DUMP_MEM_BASE=00020000", "+DUMP_MEM_WORDS=3",
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0 or "PASS [dual_ooo2w_soc]:" not in res.stdout:
            print(f"FAIL running: {' '.join(cmd)}\n{res.stdout}\n{res.stderr}")
            sys.exit(1)
        pass_ln = ""
        mem_vals: dict[int, int] = {}
        for ln in res.stdout.splitlines():
            if "PASS [dual_ooo2w_soc]:" in ln:
                pass_ln = ln
            m = re.match(r"MEM\[0x([0-9a-fA-F]+)\]\s*=\s*0x([0-9a-fA-F]+)", ln.strip())
            if m:
                mem_vals[int(m.group(1), 16)] = int(m.group(2), 16)
        return parse_kv(pass_ln), mem_vals

    kv_off, mem_off = run_audit(ipc0_hex)
    kv_on, mem_on = run_audit(ipc1_hex)
    if kv_off["c0_x14"] != kv_on["c0_x14"] or kv_off["c1_x14"] != kv_on["c1_x14"]:
        print(f"FAIL [U3-03]: Checksum mismatch! off={kv_off} on={kv_on}")
        return 1

    dt0_off = mem_off.get(0x20000, 0)
    dt1_off = mem_off.get(0x20004, 0)
    dt0_on = mem_on.get(0x20000, 0)
    dt1_on = mem_on.get(0x20004, 0)
    kern_off = max(dt0_off, dt1_off)
    kern_on = max(dt0_on, dt1_on)
    overhead_pct = ((kern_on - kern_off) / max(1, kern_off)) * 100.0
    print(f"  [OK] U3-03_zero_stall_ipc_audit (16x16 2C Matmul, x14={kv_off['c0_x14']}):")
    print(f"       MIE=0 Baseline : kernel_cyc=({dt0_off}/{dt1_off}) total={kv_off['cycles']} chip_IPC={kv_off['chip_IPC']}")
    print(f"       MIE=1 Armed    : kernel_cyc=({dt0_on}/{dt1_on}) total={kv_on['cycles']} chip_IPC={kv_on['chip_IPC']} (kernel delta={overhead_pct:+.2f}%)")
    if kern_off == 0 or overhead_pct > 1.0:
        print(f"FAIL [U3-03]: Unexpected kernel cycle overhead {overhead_pct:.2f}% > 1.0%!")
        return 1

    print("PASS [test_phase_u3_irqs]: All Phase U3 PLIC, C ISR, Vectored mtvec & Zero-Stall IPC gates PASSED!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
