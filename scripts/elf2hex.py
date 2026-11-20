#!/usr/bin/env python3
"""
Convert a raw little-endian binary (from riscv64-unknown-elf-objcopy -O binary)
into a 32-bit word-per-line Verilog $readmemh .hex image (and optional 128-bit
cache-line-per-line unified memory image).
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description="Convert flat RISC-V binary to Verilog .hex")
    ap.add_argument("bin_in", type=Path, help="Input flat binary (.bin)")
    ap.add_argument("hex_out", type=Path, help="Output 32-bit word-per-line .hex")
    ap.add_argument("--pad-words", type=int, default=1024, help="Minimum word count (padded with 0)")
    ap.add_argument("--line128-out", type=Path, help="Optional 128-bit line-per-line .hex for unified DRAM")
    args = ap.parse_args()

    raw = args.bin_in.read_bytes()
    if len(raw) % 4 != 0:
        raw += b"\x00" * (4 - (len(raw) % 4))

    words = list(struct.unpack(f"<{len(raw) // 4}I", raw))
    orig_words = len(words)
    if args.pad_words > len(words):
        words.extend([0] * (args.pad_words - len(words)))

    args.hex_out.parent.mkdir(parents=True, exist_ok=True)
    args.hex_out.write_text("\n".join(f"{w:08x}" for w in words) + "\n", encoding="utf-8")

    if args.line128_out is not None:
        while len(words) % 4 != 0:
            words.append(0)
        lines = []
        for i in range(0, len(words), 4):
            w0, w1, w2, w3 = words[i : i + 4]
            lines.append(f"{w3:08x}{w2:08x}{w1:08x}{w0:08x}")
        args.line128_out.parent.mkdir(parents=True, exist_ok=True)
        args.line128_out.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"[elf2hex] {args.bin_in.name}: {orig_words} words ({len(raw)} B) -> {args.hex_out} (padded {len(words)} words)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
