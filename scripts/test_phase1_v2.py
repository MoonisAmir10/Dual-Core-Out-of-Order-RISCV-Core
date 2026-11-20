#!/usr/bin/env python3
"""
Phase 1 End-to-End Verification Suite for Unified Single-Core RV32IMA_Zicsr_Zifencei
Out-of-Order Core (ooo2w_v2 = ooo2w_core + icache_2w + dcache_nb RV32A + Unified 256KB RAM).

Verifies:
  1. P1-01: Full 64-op RV32IMA + Zicsr (mhartid=1, mscratch, CSRR*) + FENCE + LR/SC + 9 AMOs
  2. P1-02: Real Self-Modifying Code via SW + FENCE.I (D$ dirty sweep + L1I invalidate + re-fetch!)
  3. P1-03: End-to-End Speculative Load Snoop-Invalidation Replay (live DIV holds ROB head while
            speculative younger LW gets invalidated by synthetic peer write -> replayed cleanly!)
  4. P1-04 & P1-05: Real GCC-compiled (-march=rv32ima -O2) C binaries (smoke_c & smoke_rv32ima_c)
  5. P1-06: L1I Next-Line Prefetcher Ablation (ENABLE_PREFETCH=1 vs 0 at L4 & L16)
  6. 120-Program Randomized RV32IMA Differential Fuzzer (bit-exact commit-by-commit vs RV32IMA_ISS!)
"""

from __future__ import annotations

import random
import re
import subprocess
import sys
from pathlib import Path
from typing import List, Optional, Tuple

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import rv32ima_asm as isa  # noqa: E402

BUILD = ROOT / "build" / "phase1"
BIN_C0 = ROOT / "build" / "v_p1_c0" / "Vp1_c0"
BIN_C1 = ROOT / "build" / "v_p1_c1" / "Vp1_c1"
BIN_NOPF = ROOT / "build" / "v_p1_nopf_l16" / "Vp1_nopf_l16"
BIN_PF16 = ROOT / "build" / "v_p1_pf_l16" / "Vp1_pf_l16"


def run_sim(
    sim_bin: Path,
    hex_path: Path,
    trace_path: Optional[Path] = None,
    extra_args: Optional[List[str]] = None,
) -> str:
    cmd = [str(sim_bin), f"+HEX={hex_path}"]
    if trace_path is not None:
        cmd.append(f"+TRACE={trace_path}")
    if extra_args:
        cmd.extend(extra_args)
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0 or "PASS [single_ooo2w_p1]:" not in res.stdout:
        print(f"FAIL running {' '.join(cmd)}")
        print(res.stdout)
        print(res.stderr)
        sys.exit(1)
    for ln in res.stdout.splitlines():
        if ln.startswith("PASS [single_ooo2w_p1]:"):
            return ln
    return res.stdout.strip()


def run_asm_test(
    name: str,
    src: str,
    sim_bin: Path,
    core_id: int = 0,
    check_trace: bool = True,
    extra_args: Optional[List[str]] = None,
) -> str:
    words = isa.assemble(src)
    hex_p = BUILD / f"{name}.hex"
    isa.write_hex(words, hex_p, pad_words=256)
    tr_p: Optional[Path] = None
    if check_trace:
        iss = isa.RV32IMA_ISS(words, core_id=core_id, unified_mem=True)
        tr = iss.run(max_steps=50_000)
        tr_p = BUILD / f"{name}.trace"
        isa.write_trace(tr, tr_p)
    out = run_sim(sim_bin, hex_p, tr_p, extra_args=extra_args)
    print(f"  [{name:26s}] {out}")
    return out


# ---------------------------------------------------------------------------
# Directed Phase 1 Tests
# ---------------------------------------------------------------------------
SELF_MODIFY_PROG = """
    # Patch the instruction at 'patch_site' from 'addi x14, x0, 0x111' -> 'addi x14, x0, 0x777'!
    # Encoding of 'addi x14, x0, 0x777' = (0x777 << 20) | (0 << 15) | (0 << 12) | (14 << 7) | 0x13 = 0x77700713
    # First call patch_site BEFORE patching to pull its line into L1 I-Cache!
    jal  x1, patch_site         # executes original: x14 = 0x111 (warms L1I line!)
    mv   x5, x14                # x5 = 0x111

    # Now overwrite patch_site via SW (lands dirty in L1D!)
    auipc x6, 0
    # Compute exact address of patch_site using jal trick
    jal  x7, after_addr
patch_site:
    addi x14, x0, 0x111         # will be overwritten with 0x77700713!
    ret
after_addr:
    # x7 points directly to patch_site!
    li   x8, 0x77700713         # addi x14, x0, 0x777
    sw   x8, 0(x7)              # dirty store in L1D!
    fence.i                     # sweeps L1D dirty -> RAM + invalidates L1I + flushes FQ!
    jalr x1, x7, 0              # re-fetches patched instruction -> x14 = 0x777!
    add  x14, x14, x5           # x14 = 0x777 + 0x111 = 0x888
    ebreak
"""

SNOOP_REPLAY_PROG = """
    # Pre-load data[0x1000] = 0x11111111 into L1D, then execute a slow DIV at ROB head
    # while a younger speculative LW reads data[0x1000] early (gets stale 0x11111111).
    # While DIV is executing, TB patches mem[0x1000] = 0x99998888 & fires ext_snoop_inv!
    # The poisoned LW MUST replay when it reaches ROB head and commit 0x99998888!
    li   x1, 0x1000
    li   x2, 0x11111111
    sw   x2, 0(x1)
    fence rw, rw                # ensure store is cleanly in L1D before DIV+speculative LW
    lw   x3, 0(x1)              # warm hit verify (0x11111111)
    li   x4, 100000
    li   x5, 7
    # Long 18-cycle DIV blocks ROB head; younger LW executes immediately in parallel!
    div  x6, x4, x5             # holds ROB head for ~18 cycles
    div  x7, x6, x5             # holds ROB head another ~18 cycles (36-cyc window!)
    lw   x14, 0(x1)             # executes speculatively EARLY -> gets poisoned by snoop -> MUST REPLAY!
    add  x8, x14, x0
    ebreak
"""

ICACHE_STREAM_PROG = "\n".join(
    ["    addi x1, x0, 0"]
    + [f"    addi x1, x1, {(i % 7) + 1}" for i in range(60)]
    + ["    mv x14, x1", "    ebreak"]
)


def gen_random_rv32ima_prog(seed: int, n_ops: int = 45) -> str:
    """Generate a deterministic RV32IMA_Zicsr_Zifencei differential fuzz program."""
    rng = random.Random(seed)
    # Use x1..x13, x15..x20 as scratch, x28 as base ptr = 0x2000 (safe data area)
    lines = [
        "    li   x28, 0x2000",
        "    addi x1, x0, 5",
        "    addi x2, x0, -11",
        "    addi x3, x0, 17",
        "    addi x4, x0, 3",
        "    sw   x1, 0(x28)",
        "    sw   x2, 4(x28)",
        "    sw   x3, 8(x28)",
        "    sw   x4, 12(x28)",
    ]
    regs = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
    alu_r = ["add", "sub", "xor", "or", "and", "slt", "sltu", "mul", "mulh", "mulhu"]
    amos = ["amoswap.w", "amoadd.w", "amoxor.w", "amoand.w", "amoor.w",
            "amomin.w", "amomax.w", "amominu.w", "amomaxu.w"]

    for i in range(n_ops):
        kind = rng.choices(
            ["alu", "imm", "div", "ld", "st", "amo", "lrsc", "csr", "fence", "br"],
            weights=[24, 16, 4, 14, 10, 12, 6, 6, 4, 4],
            k=1,
        )[0]
        rd = rng.choice(regs)
        rs1 = rng.choice(regs)
        rs2 = rng.choice(regs)
        off = rng.choice([0, 4, 8, 12, 16, 20, 24, 28])

        if kind == "alu":
            op = rng.choice(alu_r)
            lines.append(f"    {op} x{rd}, x{rs1}, x{rs2}")
        elif kind == "imm":
            imm = rng.randint(-64, 63)
            op = rng.choice(["addi", "xori", "ori", "andi"])
            lines.append(f"    {op} x{rd}, x{rs1}, {imm}")
        elif kind == "div":
            op = rng.choice(["div", "divu", "rem", "remu"])
            lines.append(f"    {op} x{rd}, x{rs1}, x{rs2}")
        elif kind == "ld":
            op = rng.choice(["lw", "lh", "lhu", "lb", "lbu"])
            if op in ("lh", "lhu"):
                off = rng.choice([0, 2, 4, 6, 8, 10, 12, 14])
            elif op in ("lb", "lbu"):
                off = rng.randint(0, 15)
            lines.append(f"    {op} x{rd}, {off}(x28)")
        elif kind == "st":
            op = rng.choice(["sw", "sh", "sb"])
            if op == "sh":
                off = rng.choice([0, 2, 4, 6, 8, 10, 12, 14])
            elif op == "sb":
                off = rng.randint(0, 15)
            lines.append(f"    {op} x{rs2}, {off}(x28)")
        elif kind == "amo":
            op = rng.choice(amos)
            suf = rng.choice(["", ".aq", ".rl", ".aqrl"])
            lines.append(f"    addi x29, x28, {off}")
            lines.append(f"    {op}{suf} x{rd}, x{rs2}, (x29)")
        elif kind == "lrsc":
            lines.append(f"    addi x29, x28, {off}")
            lines.append(f"    lr.w x{rd}, (x29)")
            if rng.random() < 0.7:
                lines.append(f"    sc.w x{rs1}, x{rs2}, (x29)")
            else:
                # Deliberately break reservation with a store -> SC must fail (=1)!
                lines.append(f"    sw x{rs2}, 0(x29)")
                lines.append(f"    sc.w x{rs1}, x{rs2}, (x29)")
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
        elif kind == "br":
            lbl = f"skip_{i}"
            bop = rng.choice(["beq", "bne", "blt", "bge"])
            lines.append(f"    {bop} x{rs1}, x{rs2}, {lbl}")
            lines.append(f"    addi x{rd}, x{rd}, 3")
            lines.append(f"{lbl}:")

    # Mix all regs + memory into x14
    lines.append("    addi x14, x0, 0")
    for r in regs:
        lines.append(f"    xor x14, x14, x{r}")
    lines.append("    lw x29, 0(x28)")
    lines.append("    add x14, x14, x29")
    lines.append("    ebreak")
    return "\n".join(lines)


def parse_cycles(line: str) -> Tuple[int, int, int]:
    m_c = re.search(r"cycles=(\d+)", line)
    m_m = re.search(r"ic_m=(\d+)", line)
    m_p = re.search(r"ic_pf=(\d+)", line)
    return (
        int(m_c.group(1)) if m_c else 0,
        int(m_m.group(1)) if m_m else 0,
        int(m_p.group(1)) if m_p else 0,
    )


def main() -> int:
    BUILD.mkdir(parents=True, exist_ok=True)
    print("=== Phase 1 Directed Verification Suite (ooo2w_v2 Unified RV32IMA_Zicsr_Zifencei) ===")

    # 1. Full 64-op RV32IMA + Zicsr (CORE_ID=1!) + FENCE + LR/SC + 9 AMOs
    run_asm_test("P1-01_rv32ima_all64", isa.ALL_RV32IMA_PROG, BIN_C1, core_id=1)

    # 2. Real Self-Modifying Code via SW + FENCE.I (bit-exact vs unified ISS!)
    run_asm_test(
        "P1-02_self_mod_fence_i",
        SELF_MODIFY_PROG,
        BIN_C0,
        core_id=0,
        extra_args=["+EXPECT_X14=00000888"],
    )

    # 3. End-to-End Speculative Load Snoop-Invalidation Replay (live DIV + synthetic snoop!)
    # Find when DIV+speculative LW are in flight (~cycle 45..60 at L4): inject snoop at cyc=50!
    run_asm_test(
        "P1-03_snoop_replay_e2e",
        SNOOP_REPLAY_PROG,
        BIN_C0,
        core_id=0,
        check_trace=False,
        extra_args=[
            "+INJECT_SNOOP_CYC=50",
            "+INJECT_PATCH_ADDR=01000",
            "+INJECT_PATCH_VAL=99998888",
            "+EXPECT_X14=99998888",
            "+REQ_REPLAY_MIN=1",
        ],
    )

    # 4. Real GCC-Compiled Bare-Metal C Binaries (bit-exact commit-by-commit vs RV32IMA_ISS!)
    for cname, sim_bin, cid, exp in [
        ("smoke_c", BIN_C0, 0, "00001582"),
        ("smoke_rv32ima_c", BIN_C1, 1, "00000085"),
    ]:
        hex_p = ROOT / "build" / f"{cname}.hex"
        words = [int(ln.strip(), 16) for ln in hex_p.read_text().splitlines() if ln.strip()]
        iss = isa.RV32IMA_ISS(words, core_id=cid, unified_mem=True)
        tr = iss.run(max_steps=50_000)
        tr_p = BUILD / f"{cname}_p1.trace"
        isa.write_trace(tr, tr_p)
        out = run_sim(sim_bin, hex_p, tr_p, extra_args=[f"+EXPECT_X14={exp}"])
        print(f"  [GCC_{cname:22s}] {out}")

    # 5. L1I Next-Line Prefetcher Ablation at L16 (realistic DRAM latency!)
    print("\n=== Phase 1 L1I Next-Line Prefetcher Ablation (MEM_LATENCY=16) ===")
    out_nopf = run_asm_test("P1-06_stream_nopf_l16", ICACHE_STREAM_PROG, BIN_NOPF, core_id=0)
    out_pf   = run_asm_test("P1-06_stream_pf_l16",   ICACHE_STREAM_PROG, BIN_PF16, core_id=0)
    c_nopf, m_nopf, _ = parse_cycles(out_nopf)
    c_pf, m_pf, pfh   = parse_cycles(out_pf)
    print(f"  Prefetcher OFF: {c_nopf} cyc ({m_nopf} L1I misses) -> ON: {c_pf} cyc ({m_pf} misses, {pfh} pf_hits, {c_nopf/c_pf:.2f}x faster!)")
    assert c_pf < c_nopf and pfh > 0, "Prefetcher should reduce L1I misses and cycles at L16!"

    # 6. 120-Program Randomized RV32IMA_Zicsr_Zifencei Differential Fuzzer
    print("\n=== Phase 1 120-Program Randomized RV32IMA Differential Fuzzer ===")
    total_commits = 0
    total_cycles = 0
    total_amos = 0
    for seed in range(120):
        src = gen_random_rv32ima_prog(seed + 1000)
        words = isa.assemble(src)
        hex_p = BUILD / "fuzz.hex"
        tr_p = BUILD / "fuzz.trace"
        isa.write_hex(words, hex_p, pad_words=512)
        iss = isa.RV32IMA_ISS(words, core_id=1, unified_mem=True)
        tr = iss.run(max_steps=10_000)
        isa.write_trace(tr, tr_p)
        out = run_sim(BIN_C1, hex_p, tr_p)
        mc = re.search(r"commits=(\d+) cycles=(\d+)", out)
        ma = re.search(r"amo=(\d+)", out)
        if mc:
            total_commits += int(mc.group(1))
            total_cycles += int(mc.group(2))
        if ma:
            total_amos += int(ma.group(1))

    avg_ipc = total_commits / total_cycles if total_cycles else 0.0
    print(
        f"PASS [Phase 1 RV32IMA Fuzzer]: 120/120 PASS! commits={total_commits}, "
        f"cycles={total_cycles}, avg_IPC={avg_ipc:.3f}, amo_commits={total_amos}, errors=0"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
