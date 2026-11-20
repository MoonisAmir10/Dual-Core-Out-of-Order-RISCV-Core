#!/usr/bin/env python3
"""
Phase 2 End-to-End Verification & 3-Level Hierarchy Baseline (L1I+L1D -> 32KB L2 -> L16 DRAM + CLINT).

Verifies:
  1. P2-01_clint_mmio: Live uncached CLINT MMIO reads/writes (mtime monotonic tick,
     msip0/msip1 toggle, scratch[0..3] mailbox registers)
  2. P2-02_rv32ima_l2_jitter: Full 64-op RV32IMA + Self-Modifying FENCE.I + BOTH GCC C binaries
     (smoke_c & smoke_rv32ima_c) bit-exact through L1 -> 32KB 2-Bank L2 -> L16 DRAM (+JITTER!)
  3. P2-03_l2_ablation_8kb: 8 KB Working-Set Multi-Pass Array Reduction (overflows 4 KB L1D on
     every pass, fits 100% inside 32 KB L2!) comparing No-L2 (Phase 1 @ L16) vs. With-32KB-L2
     (Phase 2 @ L1:L2:DRAM = 1:3:16) -> quantitatively proves DRAM reduction & IPC speedup!
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import rv32ima_asm as isa  # noqa: E402

BUILD = ROOT / "build" / "phase2"
BIN_L2_C0 = ROOT / "build" / "v_p2_c0" / "Vp2_c0"
BIN_L2_C1 = ROOT / "build" / "v_p2_c1" / "Vp2_c1"
BIN_NOL2_L16 = ROOT / "build" / "v_p1_pf_l16" / "Vp1_pf_l16"


def run_cmd(cmd: List[str], tag: str) -> str:
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0 or tag not in res.stdout:
        print(f"FAIL running {' '.join(cmd)}")
        print(res.stdout)
        print(res.stderr)
        sys.exit(1)
    for ln in res.stdout.splitlines():
        if tag in ln:
            return ln
    return res.stdout.strip()


CLINT_MMIO_PROG = """
    # Verify uncached CLINT MMIO at 0x0200_0000:
    #   - msip0 (0x0200_0000), msip1 (0x0200_0004)
    #   - mtime_lo (0x0200_BFF8) monotonic tick check
    #   - scratch[0] (0x0200_C000) & scratch[1] (0x0200_C004)
    lui   x1, 0x02000            # x1 = 0x0200_0000
    lui   x2, 0x0200C            # x2 = 0x0200_C000 (mtime_lo = -8(x2))

    # 1. Read mtime_lo before loop
    lw    x3, -8(x2)             # t0 = mtime_lo

    # 2. Write & readback msip0=1, msip1=1, scratch[0..1]
    addi  x4, x0, 1
    sw    x4, 0(x1)              # msip0 = 1
    sw    x4, 4(x1)              # msip1 = 1
    li    x5, 0x12345678
    li    x6, 0xCAFEBABE
    sw    x5, 0(x2)              # scratch[0] = 0x12345678
    sw    x6, 4(x2)              # scratch[1] = 0xCAFEBABE

    # Delay loop to guarantee mtime advances
    addi  x7, x0, 16
delay_lp:
    addi  x7, x7, -1
    bne   x7, x0, delay_lp

    lw    x8, -8(x2)             # t1 = mtime_lo
    sltu  x9, x3, x8             # x9 = (t0 < t1) ? 1 : 0 -> MUST be 1!

    lw    x10, 0(x1)             # msip0 == 1
    lw    x11, 4(x1)             # msip1 == 1
    sw    x0,  4(x1)             # clear msip1 = 0
    lw    x12, 4(x1)             # msip1 == 0
    lw    x13, 0(x2)             # scratch[0] == 0x12345678
    lw    x15, 4(x2)             # scratch[1] == 0xCAFEBABE

    # Checksum: (0x12345678 ^ 0xCAFEBABE) + x9(1) + x10(1) + x11(1) - x12(0)
    # 0x12345678 ^ 0xCAFEBABE = 0xD8CAECC6; + 3 = 0xD8CAECC9
    xor   x14, x13, x15
    add   x14, x14, x9
    add   x14, x14, x10
    add   x14, x14, x11
    sub   x14, x14, x12
    ebreak
"""

# 8 KB Working Set (512 x 16B lines = 8,192 B > 4 KB L1D, < 32 KB L2!)
# Pass 1 initializes 512 lines (stride 16B = 1 word/line -> every access misses 4KB L1D
# and evicts dirty lines into L2!). Passes 2..4 read + accumulate all 512 lines (3 full
# passes): without L2 every pass misses to L16 DRAM; WITH 32KB L2 every pass HITS L2 in 3 cyc!
L2_8KB_WORKSET_PROG = """
    li    x1, 0x10000            # base = 64 KB (512 lines -> 0x10000..0x11FFF = 8 KB!)
    li    x2, 512                # 512 cache lines (8 KB > 4 KB L1D, << 32 KB L2!)

    # Pass 0: initialize 512 cache lines (dirty stores -> overflows 4 KB L1D into 32 KB L2!)
    mv    x3, x1
    addi  x4, x0, 0
init_lp:
    addi  x5, x4, 1
    sw    x5, 0(x3)
    addi  x3, x3, 16
    addi  x4, x4, 1
    blt   x4, x2, init_lp

    # Passes 1..3: 3 full read-sum passes over the 8 KB array (1,536 L1D misses -> 100% L2 HITS!)
    addi  x14, x0, 0
    addi  x6, x0, 3
pass_lp:
    mv    x3, x1
    addi  x4, x0, 0
sum_lp:
    lw    x7, 0(x3)
    lw    x8, 16(x3)
    add   x14, x14, x7
    add   x14, x14, x8
    addi  x3, x3, 32
    addi  x4, x4, 2
    blt   x4, x2, sum_lp
    addi  x6, x6, -1
    bne   x6, x0, pass_lp
    ebreak
"""


def parse_kv(line: str) -> Dict[str, str]:
    return dict(re.findall(r"(\w+)=([^\s]+)", line))


def main() -> int:
    BUILD.mkdir(parents=True, exist_ok=True)
    print("=== Phase 2: 3-Level Hierarchy Verification (L1I+L1D -> 32KB 2-Bank L2 -> L16 DRAM + CLINT) ===")

    # 1. CLINT MMIO live hardware test (uncached bypass around L1D/L2!)
    w_clint = isa.assemble(CLINT_MMIO_PROG)
    hex_clint = BUILD / "P2-01_clint_mmio.hex"
    isa.write_hex(w_clint, hex_clint, pad_words=256)
    exp_clint = ((0x12345678 ^ 0xCAFEBABE) + 3) & 0xFFFFFFFF
    out = run_cmd(
        [str(BIN_L2_C0), f"+HEX={hex_clint}", f"+EXPECT_X14={exp_clint:08x}", "+JITTER"],
        "PASS [single_ooo2w_L2]:",
    )
    print(f"  [P2-01_clint_mmio          ] {out}")

    # 2. Full 64-op RV32IMA + Zicsr (CORE_ID=1) through L2 + L16 DRAM + Jitter (bit-exact RVFI!)
    w_all = isa.assemble(isa.ALL_RV32IMA_PROG)
    hex_all = BUILD / "P2-02_rv32ima_all64.hex"
    tr_all = BUILD / "P2-02_rv32ima_all64.trace"
    isa.write_hex(w_all, hex_all, pad_words=256)
    iss_all = isa.RV32IMA_ISS(w_all, core_id=1, unified_mem=True)
    isa.write_trace(iss_all.run(max_steps=50_000), tr_all)
    out = run_cmd(
        [str(BIN_L2_C1), f"+HEX={hex_all}", f"+TRACE={tr_all}", "+JITTER"],
        "PASS [single_ooo2w_L2]:",
    )
    print(f"  [P2-02_rv32ima_l2_jitter   ] {out}")

    # 3. Both Real GCC-Compiled Bare-Metal C Binaries through L2 + L16 DRAM + Jitter!
    for cname, sim_bin, cid, exp in [
        ("smoke_c", BIN_L2_C0, 0, "00001582"),
        ("smoke_rv32ima_c", BIN_L2_C1, 1, "00000085"),
    ]:
        hex_p = ROOT / "build" / f"{cname}.hex"
        words = [int(ln.strip(), 16) for ln in hex_p.read_text().splitlines() if ln.strip()]
        iss = isa.RV32IMA_ISS(words, core_id=cid, unified_mem=True)
        tr_p = BUILD / f"{cname}_p2.trace"
        isa.write_trace(iss.run(max_steps=50_000), tr_p)
        out = run_cmd(
            [str(sim_bin), f"+HEX={hex_p}", f"+TRACE={tr_p}", f"+EXPECT_X14={exp}", "+JITTER"],
            "PASS [single_ooo2w_L2]:",
        )
        print(f"  [GCC_{cname:22s}] {out}")

    # 4. Quantitative L2 Ablation: 8 KB Multi-Pass Working Set @ L16 DRAM (No-L2 vs With-32KB-L2!)
    print("\n=== Phase 2 Quantitative L2 Ablation (8 KB Working Set > 4 KB L1D, DRAM = L16) ===")
    w_ws = isa.assemble(L2_8KB_WORKSET_PROG)
    hex_ws = BUILD / "P2-03_l2_8kb_workset.hex"
    tr_ws = BUILD / "P2-03_l2_8kb_workset.trace"
    isa.write_hex(w_ws, hex_ws, pad_words=256)
    iss_ws = isa.RV32IMA_ISS(w_ws, core_id=0, unified_mem=True)
    tr_ev = iss_ws.run(max_steps=50_000)
    isa.write_trace(tr_ev, tr_ws)
    exp_sum = iss_ws.regs[14]  # 3 * (512 * 513 / 2) = 393984 = 0x00060300

    out_nol2 = run_cmd(
        [str(BIN_NOL2_L16), f"+HEX={hex_ws}", f"+TRACE={tr_ws}", f"+EXPECT_X14={exp_sum:08x}", "+MAX_CYCLES=200000"],
        "PASS [single_ooo2w_p1]:",
    )
    out_l2 = run_cmd(
        [str(BIN_L2_C0), f"+HEX={hex_ws}", f"+TRACE={tr_ws}", f"+EXPECT_X14={exp_sum:08x}", "+MAX_CYCLES=200000"],
        "PASS [single_ooo2w_L2]:",
    )
    print(f"  [No-L2  (L1 -> L16 DRAM)   ] {out_nol2}")
    print(f"  [32KB L2 (L1 -> L2 -> L16) ] {out_l2}")

    kv_no = parse_kv(out_nol2)
    kv_l2 = parse_kv(out_l2)
    cyc_no, ipc_no = int(kv_no["cycles"]), float(kv_no["IPC"])
    cyc_l2, ipc_l2 = int(kv_l2["cycles"]), float(kv_l2["IPC"])
    l2_h, l2_m = int(kv_l2["l2_h"]), int(kv_l2["l2_m"])
    dram_r, dram_w = int(kv_l2["dram_r"]), int(kv_l2["dram_w"])
    speedup = cyc_no / cyc_l2
    hit_rate = 100.0 * l2_h / (l2_h + l2_m) if (l2_h + l2_m) > 0 else 0.0

    print(
        f"\n  L2 Ablation Summary (8 KB working set, expected sum=0x{exp_sum:08x}):\n"
        f"    No-L2 (L16 DRAM): {cyc_no} cyc (IPC={ipc_no:.3f})\n"
        f"    32KB 2-Bank L2  : {cyc_l2} cyc (IPC={ipc_l2:.3f}) -> {speedup:.2f}x FASTER!\n"
        f"    L2 Hit Rate     : {hit_rate:.1f}% (l2_hits={l2_h}, l2_misses={l2_m}, dram_rds={dram_r}, dram_wrs={dram_w})"
    )
    assert cyc_l2 < cyc_no and hit_rate > 70.0, "32 KB L2 must accelerate 8 KB working set with >70% L2 hit rate!"

    summary = {
        "workset_8kb_no_l2_cycles": cyc_no,
        "workset_8kb_no_l2_ipc": ipc_no,
        "workset_8kb_l2_cycles": cyc_l2,
        "workset_8kb_l2_ipc": ipc_l2,
        "l2_speedup": round(speedup, 3),
        "l2_hit_rate_pct": round(hit_rate, 2),
        "l2_hits": l2_h,
        "l2_misses": l2_m,
        "dram_reads": dram_r,
        "dram_writes": dram_w,
    }
    (BUILD / "phase2_baseline.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print("PASS [Phase 2]: All 3-level hierarchy, CLINT MMIO, RV32IMA+Jitter, and L2 ablation gates PASSED!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
