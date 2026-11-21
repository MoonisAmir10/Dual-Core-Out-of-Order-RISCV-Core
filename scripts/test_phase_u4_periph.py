#!/usr/bin/env python3
"""
Phase U4 End-to-End Verification: Expandable AMBA APB4 Peripheral Suite
(32-Pin GPIO, 16-FIFO 8N1 UART + hal_uart_printf(), 2-Ch Timer/PWM) & C HAL!

Validates BOTH 1-Core (single_ooo2w_soc) and 2-Core (dual_ooo2w_soc + MOESI + JITTER):
  1. Compiles sw/c_soc/test_periph_hal_c.c with sw/hal/{hal_irq.c, hal_uart.c}
     under -Wall -Wextra -Werror (zero libc dependencies).
  2. Runs with external +GPIO_STIM pin stimulus harness and +UART_OUT console logger.
  3. Verifies:
       - Exact x14 exit checksums (1C: 0x00013413; 2C: C0=0x00023413, C1=0x00123413)
       - Exact UART console output strings from GPIO PLIC ISRs, real 8N1 loopback "OK!",
         and final hal_uart_printf() summary with %u/%d/%02x/%04X/%s/%%!
       - Physical top-level SoC pins: gpio_oe == 0x0000FF00, gpio_out == 0x00008700,
         pwm_out[0] periodic edges & duty cycles, pwm_out[1] one-shot pulse & auto-stop!
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build" / "phase_u4"
BIN_SINGLE = ROOT / "build" / "v_p2_c0" / "Vp2_c0"
BIN_DUAL = ROOT / "build" / "v_dual_soc" / "Vdual_soc"


def compile_periph_c(src: str, stem: str) -> Path:
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


def run_soc_test(cmd: List[str], pass_tag: str) -> Tuple[str, Dict[str, str]]:
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0 or pass_tag not in res.stdout:
        print(f"FAIL running: {' '.join(cmd)}")
        print(res.stdout)
        print(res.stderr)
        sys.exit(1)
    pass_ln = ""
    periph_kv: Dict[str, str] = {}
    for ln in res.stdout.splitlines():
        if pass_tag in ln:
            pass_ln = ln
        if ln.startswith("PERIPH_STATS:"):
            periph_kv = dict(re.findall(r"(\w+)=([^\s]+)", ln))
    return pass_ln, periph_kv


def verify_outputs(tag: str, ncores: int, gpio_core: int, uart_file: Path, pkv: Dict[str, str]) -> None:
    uart_txt = uart_file.read_text(encoding="utf-8", errors="replace")

    expected_fragments = [
        f"BOOT cores={ncores}\n",
        f"GPIO_ISR core={gpio_core} st=0x01 in=0x01\n",
        f"GPIO_ISR core={gpio_core} st=0x02 in=0x03\n",
        f"GPIO_ISR core={gpio_core} st=0x04 in=0x06\n",
        "OK!",
        f"SOC_U4_DONE: cores={ncores} gpio=3 t0=4 t1=1 rx=OK! neg=-42 hex=0xCAFE 100%\n",
    ]
    for frag in expected_fragments:
        if frag not in uart_txt:
            print(f"FAIL [{tag}]: Missing expected UART output fragment {frag!r}")
            print(f"--- Actual UART console ({uart_file}) ---\n{uart_txt!r}")
            sys.exit(1)

    if not pkv:
        print(f"FAIL [{tag}]: Missing PERIPH_STATS line in simulation output!")
        sys.exit(1)

    gpio_out = int(pkv["gpio_out"], 16)
    gpio_oe = int(pkv["gpio_oe"], 16)
    pwm0_hi = int(pkv["pwm0_hi"])
    pwm0_edges = int(pkv["pwm0_edges"])
    pwm1_hi = int(pkv["pwm1_hi"])
    pwm1_edges = int(pkv["pwm1_edges"])
    uart_bytes = int(pkv["uart_bytes"])

    assert gpio_oe == 0x0000FF00, f"{tag}: gpio_oe exp=0x0000ff00 got=0x{gpio_oe:08x}"
    assert (gpio_out & 0xFFFF) == 0x8700, f"{tag}: gpio_out exp=0x00008700 got=0x{gpio_out:08x}"
    assert pwm0_edges >= 4, f"{tag}: pwm0_edges={pwm0_edges} < 4"
    # Mathematical 50%-duty invariant: every completed 500-cyc period has exact 250 hi cyc,
    # and the final period stopped by timer0_isr contributes 1..250 hi cyc!
    assert (pwm0_edges - 1) * 250 <= pwm0_hi <= pwm0_edges * 250, (
        f"{tag}: pwm0_hi={pwm0_hi} violates exact 50% duty for pwm0_edges={pwm0_edges}!"
    )
    assert pwm1_edges == 1, f"{tag}: pwm1_edges={pwm1_edges} != 1 (one-shot should fire once!)"
    assert pwm1_hi == 300, f"{tag}: pwm1_hi={pwm1_hi} != exact 300-cycle one-shot pulse!"
    assert uart_bytes == len(uart_txt), f"{tag}: uart_bytes={uart_bytes} != len={len(uart_txt)}"

    print(
        f"       Pins & Console Verified: gpio_out=0x{gpio_out:08x} gpio_oe=0x{gpio_oe:08x} "
        f"pwm0=({pwm0_hi} hi / {pwm0_edges} edges) pwm1=({pwm1_hi} hi / {pwm1_edges} edge) "
        f"uart={uart_bytes} bytes"
    )


def main() -> int:
    print("=== Phase U4 End-to-End Verification: AMBA APB4 GPIO, 8N1 UART, Timer/PWM & C HAL ===")

    hex_p = compile_periph_c("sw/c_soc/test_periph_hal_c.c", "test_periph_hal_c")

    # 1. Single-Core SoC (1C + JITTER + GPIO_STIM + UART_OUT)
    uart_1c = BUILD / "uart_1c.log"
    out_1c, pkv_1c = run_soc_test(
        [
            str(BIN_SINGLE),
            f"+HEX={hex_p}",
            "+EXPECT_X14=00013413",
            "+MAX_CYCLES=80000",
            "+JITTER",
            "+GPIO_STIM",
            f"+UART_OUT={uart_1c}",
        ],
        "PASS [single_ooo2w_L2]",
    )
    print(f"  [OK] U4-01a_periph_hal_1C: {out_1c}")
    verify_outputs("U4-01a_1C", ncores=1, gpio_core=0, uart_file=uart_1c, pkv=pkv_1c)

    # 2. Dual-Core MOESI SoC (2C + JITTER + GPIO_STIM + UART_OUT + SWMR/Shadow)
    uart_2c = BUILD / "uart_2c.log"
    out_2c, pkv_2c = run_soc_test(
        [
            str(BIN_DUAL),
            f"+HEX={hex_p}",
            "+EXPECT_C0_X14=00023413",
            "+EXPECT_C1_X14=00123413",
            "+MAX_CYCLES=80000",
            "+JITTER",
            "+GPIO_STIM",
            f"+UART_OUT={uart_2c}",
        ],
        "PASS [dual_ooo2w_soc]",
    )
    print(f"  [OK] U4-01b_periph_hal_2C: {out_2c}")
    verify_outputs("U4-01b_2C", ncores=2, gpio_core=1, uart_file=uart_2c, pkv=pkv_2c)

    print("PASS [test_phase_u4_periph]: All Phase U4 GPIO, 8N1 UART, Timer/PWM & C HAL gates PASSED!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
