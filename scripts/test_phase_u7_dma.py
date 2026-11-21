#!/usr/bin/env python3
"""
Phase U7 End-to-End Verification: AMBA AXI4 2-Channel I/O-Coherent DMA Engine
on Dual-Core RV32IMAF MOESI SoC!

Compiles sw/c_soc/test_dma_coh_c.c with GCC -march=rv32imaf -mabi=ilp32f -O2 -Werror
and runs on Vdual_soc (+JITTER + always-on 3-master coh_checker SWMR/Shadow monitor):
  1. Verifies exact dual-core checksums (C0=0x00070041, C1=0x00170041)
  2. Verifies UART console summary confirming 4KB zero-flush dirty-L1D FP DMA,
     concurrent unaligned 1D + 2D strided dual-channel DMA, PMA DECERR recovery,
     and 24 speculative-load vs. DMA-write iterations
  3. Verifies hardware PMU counters:
       - dma_r > 0 and dma_w > 0 (AXI4 coherent DMA traffic active!)
       - c2c > 0 (DMA snooped dirty M lines directly out of Core 0 L1D!)
       - replay_c1 > 0 (DMA invalidations triggered REAL hardware LQ snoop-replay!)
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build" / "phase_u7"
BIN_DUAL = ROOT / "build" / "v_dual_soc" / "Vdual_soc"


def compile_dma_c(src: str, stem: str) -> Path:
    BUILD.mkdir(parents=True, exist_ok=True)
    elf = BUILD / f"{stem}.elf"
    bin_p = BUILD / f"{stem}.bin"
    hex_p = BUILD / f"{stem}.hex"
    cmd = [
        "/usr/bin/riscv64-unknown-elf-gcc",
        "-march=rv32imaf", "-mabi=ilp32f", "-O2",
        "-ffreestanding", "-nostdlib", "-nostartfiles",
        "-Wall", "-Wextra", "-Werror",
        "-Isw/common", "-Isw/hal", "-T", "sw/common/link.ld",
        "sw/common/crt0.S", "sw/common/trap.S",
        "sw/hal/hal_irq.c", "sw/hal/hal_uart.c",
        src, "-o", str(elf),
    ]
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


def main() -> int:
    print("=== Phase U7 End-to-End Verification: Dual-Core RV32IMAF I/O-Coherent AXI4 DMA ===")

    hex_p = compile_dma_c("sw/c_soc/test_dma_coh_c.c", "test_dma_coh_c")
    uart_log = BUILD / "uart_u7.log"

    cmd = [
        str(BIN_DUAL),
        f"+HEX={hex_p}",
        "+EXPECT_C0_X14=00070041",
        "+EXPECT_C1_X14=00170041",
        "+MAX_CYCLES=250000",
        "+JITTER",
        f"+UART_OUT={uart_log}",
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0 or "PASS [dual_ooo2w_soc]" not in res.stdout:
        print(f"FAIL running: {' '.join(cmd)}")
        print(res.stdout)
        print(res.stderr)
        sys.exit(1)

    pass_ln = ""
    for ln in res.stdout.splitlines():
        if "PASS [dual_ooo2w_soc]" in ln:
            pass_ln = ln
            break
    print(f"  [OK] U7-01_dma_coh_2C: {pass_ln}")

    uart_txt = uart_log.read_text(encoding="utf-8", errors="replace")
    exp_uart = "SOC_U7_DMA_DONE: ch0_irqs=4 ch1_irqs=1 decerr=OK 4kb_fp=OK unalign=OK 2d=OK rpl=24\n"
    if exp_uart not in uart_txt:
        print(f"FAIL [U7-01]: Missing expected UART output {exp_uart!r}")
        print(f"Actual UART output:\n{uart_txt!r}")
        sys.exit(1)

    kv = dict(re.findall(r"(\w+)=([^\s]+)", pass_ln))
    dma_r = int(kv.get("dma_r", "0"))
    dma_w = int(kv.get("dma_w", "0"))
    c2c = int(kv.get("c2c", "0"))
    rpl_parts = kv.get("replay", "0/0").split("/")
    rpl_c0, rpl_c1 = int(rpl_parts[0]), int(rpl_parts[1])

    assert dma_r >= 256, f"Expected >=256 coherent DMA reads (4KB+tests), got {dma_r}"
    assert dma_w >= 256, f"Expected >=256 coherent DMA writes (4KB+tests), got {dma_w}"
    assert c2c >= 128, f"Expected >=128 dirty C2C snoops from Core 0 L1D, got {c2c}"
    assert rpl_c1 > 0, (
        f"Expected Core 1 hardware LQ snoop-replay > 0 during DMA overwrite proof, got {rpl_c1}!"
    )

    print(
        f"       Coherent DMA Proof Verified: dma_r={dma_r} dma_w={dma_w} "
        f"c2c_dirty={c2c} replay_c0/c1={rpl_c0}/{rpl_c1} uart={uart_txt.strip()!r}"
    )
    print("PASS [test_phase_u7_dma]: All Phase U7 I/O-Coherent AXI4 DMA & Snoop-Replay gates PASSED!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
