#!/usr/bin/env python3
"""
Phase 4: Dual-Core RVWMO Memory Consistency Litmus & Atomic Synchronization Suite.

Executes all 10 canonical RVWMO litmus & atomic synchronization tests on the live
Dual-Core 2-Wide Out-of-Order MOESI SoC (dual_ooo2w_soc) with DRAM latency jitter
(+JITTER) and always-on coh_checker (SWMR, Golden Shadow Memory & deadlocks):

  L01_MP               : Message Passing with FENCE (1,000 rounds)
  L02_MP_aqrl          : Message Passing with Release/Acquire AMOs (1,000 rounds)
  L03_SB               : Store Buffering with FENCE (1,000 rounds)
  L04_LB               : Load Buffering with FENCE (1,000 rounds)
  L05_CoRR             : Same-Address Read-Read Coherence, NO FENCE (1,000 rounds)
  L05_CoRR_NO_REPLAY   : Negative Ablation (ENABLE_SNOOP_REPLAY=0 -> MUST VIOLATE!)
  L06_CoWR             : Same-Address Write-Read Coherence, NO FENCE (1,000 rounds)
  L07_CoWW             : Same-Address Write-Write Coherence, NO FENCE (1,000 rounds)
  L08_R_MCA            : R (Read-Write Multi-Copy Atomicity) with FENCE (1,000 rounds)
  L09_2plus2W          : 2+2W with FENCE (1,000 rounds)
  L10_Atomic_Mutex_3k  : 1,000x Spinlock + 1,000x LR/SC CAS + 1,000x AMOADD stress
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import rv32ima_asm as isa  # noqa: E402

BUILD = ROOT / "build" / "phase4_litmus"
BIN_DUAL = ROOT / "build" / "v_dual_soc" / "Vdual_soc"
BIN_NOREPLAY = ROOT / "build" / "v_dual_soc_noreplay" / "Vdual_soc_noreplay"

ROUNDS = 1000


def make_litmus_prog(
    name: str,
    rounds: int,
    c0_body: str,
    c1_body: str,
    grade_bin_asm: str,
    forbidden_bin: int,
    c1_pre_ack: str = "",
    c0_init_extra: str = "",
) -> str:
    """
    Build a self-contained dual-core synchronized RV32IMA litmus kernel.
    Memory layout (64-byte spacing -> zero unintended false sharing!):
      0x20000 (s4): X
      0x20040 (s5): Y
      0x20080 (s6): RES0 (Core 0 per-round output)
      0x200C0 (s7): RES1 (Core 1 per-round output)
      0x20100 (s8): BAR0 (Core 0 -> Core 1 round start / 0xFFFF exit)
      0x20140 (s9): BAR1 (Core 1 -> Core 0 start ack)
      0x20180 (s10): DONE1 (Core 1 -> Core 0 round done)
      0x20200 (s11): HIST[0..3] (4 outcome counters)
    Core 0 halts with x14 = forbidden_count; Core 1 halts with x14 = rounds.
    """
    return f"""
    # === Litmus Test: {name} ({rounds} synchronized dual-core OoO rounds) ===
    li    s0, 0x20000
    mv    s4, s0
    addi  s5, s0, 0x040
    addi  s6, s0, 0x080
    addi  s7, s0, 0x0C0
    addi  s8, s0, 0x100
    addi  s9, s0, 0x140
    addi  s10, s0, 0x180
    addi  s11, s0, 0x200

    li    s2, {rounds}
    li    x14, 0
    csrr  t0, mhartid
    bne   t0, zero, core1_entry

# ------------------ CORE 0 (Driver + Grader) ------------------
core0_entry:
    sw    zero, 0(s8)
    sw    zero, 0(s9)
    sw    zero, 0(s10)
    sw    zero, 0(s11)
    sw    zero, 4(s11)
    sw    zero, 8(s11)
    sw    zero, 12(s11)
    fence rw, rw
    li    s1, 1                # round k = 1..N
    li    s3, 0x13579BDF       # LCG seed for Core 0 phase jitter

c0_round:
    # Reset shared variables X, Y, RES0, RES1
    sw    zero, 0(s4)
    sw    zero, 0(s5)
    sw    zero, 0(s6)
    sw    zero, 0(s7)
{c0_init_extra}
    fence rw, rw
    # Signal Core 1 to start round k
    sw    s1, 0(s8)
    fence rw, rw
c0_wait_bar1:
    lw    t0, 0(s9)
    bne   t0, s1, c0_wait_bar1
    fence rw, rw

    # Pseudo-random phase jitter (0..15 ALU iterations) so cores sweep all alignments!
    li    t1, 1664525
    mul   s3, s3, t1
    addi  s3, s3, 1013
    srli  t0, s3, 28           # 0..15
    beq   t0, zero, c0_jit_done
c0_jit_lp:
    addi  t0, t0, -1
    bne   t0, zero, c0_jit_lp
c0_jit_done:

    # --- Core 0 Litmus Body ---
{c0_body}

    # Wait for Core 1 round completion
    fence rw, rw
c0_wait_done1:
    lw    t0, 0(s10)
    bne   t0, s1, c0_wait_done1
    fence rw, rw

    # --- Grade Outcome Bin into t6 (0..3) ---
{grade_bin_asm}
    slli  t5, t6, 2
    add   t5, s11, t5
    lw    t4, 0(t5)
    addi  t4, t4, 1
    sw    t4, 0(t5)
    li    t4, {forbidden_bin}
    bne   t6, t4, c0_ok
    addi  x14, x14, 1
c0_ok:
    addi  s1, s1, 1
    bge   s2, s1, c0_round

    # Signal exit sentinel (0xFFFF) and halt Core 0
    fence rw, rw
    li    t0, 0xFFFF
    sw    t0, 0(s8)
    fence rw, rw
    ebreak

# ------------------ CORE 1 (Concurrent Responder) ------------------
core1_entry:
    li    s1, 1
    li    s3, 0x2468ACE1       # Different LCG seed for Core 1 phase jitter
    li    t6, 0xFFFF

c1_round:
c1_wait_bar0:
    lw    t0, 0(s8)
    beq   t0, t6, c1_exit
    bne   t0, s1, c1_wait_bar0
    fence rw, rw
{c1_pre_ack}
    # Acknowledge start barrier
    sw    s1, 0(s9)
    fence rw, rw

    # Pseudo-random phase jitter (0..15 ALU iterations)
    li    t1, 1664525
    mul   s3, s3, t1
    addi  s3, s3, 1999
    srli  t0, s3, 28
    beq   t0, zero, c1_jit_done
c1_jit_lp:
    addi  t0, t0, -1
    bne   t0, zero, c1_jit_lp
c1_jit_done:

    # --- Core 1 Litmus Body ---
{c1_body}

    # Signal round k done to Core 0
    fence rw, rw
    sw    s1, 0(s10)
    fence rw, rw
    addi  s1, s1, 1
    j     c1_round

c1_exit:
    mv    x14, s2
    ebreak
"""


def make_mutex_stress_prog(iters_per_core: int = 500) -> str:
    """
    L10: 3,000-Op Dual-Core Contended Atomic Synchronization Stress:
      Phase A: 2 x 500 = 1,000 critical sections under AMOSWAP.W TTAS Spinlock
      Phase B: 2 x 500 = 1,000 updates via LR.W / SC.W CAS Loop
      Phase C: 2 x 500 = 1,000 updates via AMOADD.W
    Final verification: CNT_SPIN == 1000, CNT_CAS == 1000, CNT_AMO == 1000
    Both cores halt with x14 = CNT_SPIN + CNT_CAS + CNT_AMO = 3000 (0x00000BB8)!
    """
    return f"""
    # === L10: Dual-Core Contended Spinlock + LR/SC CAS + AMOADD Stress ({iters_per_core*6} total) ===
    li    s0, 0x20000
    mv    s4, s0               # 0x20000: SPINLOCK
    addi  s5, s0, 0x040        # 0x20040: CNT_SPIN
    addi  s6, s0, 0x080        # 0x20080: CNT_CAS
    addi  s7, s0, 0x0C0        # 0x200C0: CNT_AMO
    addi  s8, s0, 0x100        # 0x20100: READY_BAR
    addi  s9, s0, 0x140        # 0x20140: DONE_BAR
    li    s2, {iters_per_core}
    csrr  s1, mhartid

    bne   s1, zero, m_wait_init
    sw    zero, 0(s4)
    sw    zero, 0(s5)
    sw    zero, 0(s6)
    sw    zero, 0(s7)
    sw    zero, 0(s9)
    fence rw, rw
    li    t0, 1
    sw    t0, 0(s8)
    fence rw, rw
    j     m_phase_a

m_wait_init:
    lw    t0, 0(s8)
    beq   t0, zero, m_wait_init
    fence rw, rw

# ---- Phase A: TTAS AMOSWAP.W Spinlock protecting non-atomic RMW (2 x {iters_per_core} = {iters_per_core*2}) ----
m_phase_a:
    mv    t3, s2
    li    t1, 1
spin_outer:
spin_acq:
    lw    t0, 0(s4)
    bne   t0, zero, spin_acq
    amoswap.w.aq t0, t1, (s4)
    bne   t0, zero, spin_acq
    # Critical section: non-atomic LW -> ADDI -> SW
    lw    t2, 0(s5)
    addi  t2, t2, 1
    sw    t2, 0(s5)
    # Release lock
    amoswap.w.rl zero, zero, (s4)
    addi  t3, t3, -1
    bne   t3, zero, spin_outer

# ---- Phase B: Lock-Free LR.W / SC.W CAS Loop (2 x {iters_per_core} = {iters_per_core*2}) ----
    mv    t3, s2
cas_outer:
cas_retry:
    lr.w  t0, (s6)
    addi  t1, t0, 1
    sc.w  t2, t1, (s6)
    bne   t2, zero, cas_retry
    addi  t3, t3, -1
    bne   t3, zero, cas_outer

# ---- Phase C: Contended AMOADD.W Hammer (2 x {iters_per_core} = {iters_per_core*2}) ----
    mv    t3, s2
    li    t1, 1
amo_lp:
    amoadd.w zero, t1, (s7)
    addi  t3, t3, -1
    bne   t3, zero, amo_lp

# ---- Final Barrier & Verify All 3 Counters ----
    fence rw, rw
    li    t1, 1
    amoadd.w.aqrl zero, t1, (s9)
wait_both:
    lw    t0, 0(s9)
    li    t1, 2
    bne   t0, t1, wait_both
    fence rw, rw

    lw    a0, 0(s5)
    lw    a1, 0(s6)
    lw    a2, 0(s7)
    # Store into 0x20200..0x20208 for TB memory dump
    addi  t4, s0, 0x200
    bne   s1, zero, skip_dump_wr
    sw    a0, 0(t4)
    sw    a1, 4(t4)
    sw    a2, 8(t4)
    sw    zero, 12(t4)
skip_dump_wr:
    add   x14, a0, a1
    add   x14, x14, a2
    ebreak
"""


def run_sim(sim_bin: Path, hex_path: Path, extra_args: List[str]) -> Tuple[str, Dict[str, str], List[int]]:
    cmd = [
        str(sim_bin),
        f"+HEX={hex_path}",
        "+JITTER",
        "+MAX_CYCLES=1500000",
        "+DUMP_MEM_BASE=00020200",
        "+DUMP_MEM_WORDS=4",
    ] + extra_args
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0 or "PASS [dual_ooo2w_soc]:" not in res.stdout:
        print(f"FAIL running {' '.join(cmd)}")
        print(res.stdout)
        print(res.stderr)
        sys.exit(1)
    pass_line = ""
    mem_vals: List[int] = []
    for ln in res.stdout.splitlines():
        if ln.startswith("MEM["):
            m = re.search(r"=0x([0-9a-fA-F]+)", ln)
            if m:
                mem_vals.append(int(m.group(1), 16))
        if "PASS [dual_ooo2w_soc]:" in ln:
            pass_line = ln
    kv = dict(re.findall(r"(\w+)=([^\s]+)", pass_line))
    return pass_line, kv, mem_vals


def main() -> int:
    BUILD.mkdir(parents=True, exist_ok=True)
    print("=== Phase 4: Dual-Core RVWMO Memory Consistency Litmus & Atomic Mutex Suite ===")

    tests = [
        (
            "L01_MP",
            make_litmus_prog(
                "L01_MP (Message Passing + FENCE)",
                ROUNDS,
                c0_body="""
    li    t0, 1
    sw    t0, 0(s4)            # [X] = 1
    fence rw, rw
    sw    t0, 0(s5)            # [Y] = 1
""",
                c1_body="""
    lw    a0, 0(s5)            # r1 = [Y]
    fence rw, rw
    lw    a1, 0(s4)            # r2 = [X]
    slli  a0, a0, 1
    or    a0, a0, a1
    sw    a0, 0(s7)
""",
                grade_bin_asm="    lw    t6, 0(s7)",
                forbidden_bin=2,  # (r1=1, r2=0)
            ),
        ),
        (
            "L02_MP_aqrl",
            make_litmus_prog(
                "L02_MP_aqrl (Message Passing + Release/Acquire AMOs)",
                ROUNDS,
                c0_body="""
    li    t0, 1
    sw    t0, 0(s4)            # [X] = 1
    amoswap.w.rl zero, t0, (s5)# [Y] = 1 (release)
""",
                c1_body="""
    amoor.w.aq a0, zero, (s5)  # r1 = [Y] (acquire)
    lw    a1, 0(s4)            # r2 = [X]
    slli  a0, a0, 1
    or    a0, a0, a1
    sw    a0, 0(s7)
""",
                grade_bin_asm="    lw    t6, 0(s7)",
                forbidden_bin=2,  # (r1=1, r2=0)
            ),
        ),
        (
            "L03_SB",
            make_litmus_prog(
                "L03_SB (Store Buffering + FENCE)",
                ROUNDS,
                c0_body="""
    li    t0, 1
    sw    t0, 0(s4)            # [X] = 1
    fence rw, rw
    lw    a0, 0(s5)            # r0 = [Y]
    sw    a0, 0(s6)
""",
                c1_body="""
    li    t0, 1
    sw    t0, 0(s5)            # [Y] = 1
    fence rw, rw
    lw    a1, 0(s4)            # r1 = [X]
    sw    a1, 0(s7)
""",
                grade_bin_asm="""
    lw    t0, 0(s6)
    lw    t1, 0(s7)
    slli  t0, t0, 1
    or    t6, t0, t1
""",
                forbidden_bin=0,  # (r0=0, r1=0)
            ),
        ),
        (
            "L04_LB",
            make_litmus_prog(
                "L04_LB (Load Buffering + FENCE)",
                ROUNDS,
                c0_body="""
    lw    a0, 0(s4)            # r0 = [X]
    fence rw, rw
    li    t0, 1
    sw    t0, 0(s5)            # [Y] = 1
    sw    a0, 0(s6)
""",
                c1_body="""
    lw    a1, 0(s5)            # r1 = [Y]
    fence rw, rw
    li    t0, 1
    sw    t0, 0(s4)            # [X] = 1
    sw    a1, 0(s7)
""",
                grade_bin_asm="""
    lw    t0, 0(s6)
    lw    t1, 0(s7)
    slli  t0, t0, 1
    or    t6, t0, t1
""",
                forbidden_bin=3,  # (r0=1, r1=1)
            ),
        ),
        (
            "L05_CoRR",
            make_litmus_prog(
                "L05_CoRR (Same-Address Read-Read Coherence, NO FENCE, OoO Speculative Load!)",
                ROUNDS,
                c0_body="""
    # Sweep store timing right across Core 1's 34-cycle DIV-stalled speculative load window!
    li    t0, 1
    sw    t0, 0(s4)            # [X] = 1 (invalidates Core 1's S copy mid-flight!)
""",
                c1_pre_ack="""
    # Pre-warm [X] in Core 1 L1D in S state so younger load hits in 1 cycle!
    lw    t5, 0(s4)
    fence rw, rw
""",
                c1_body="""
    # Older load's address waits on 32-cycle DIV; younger load uses s4 directly ->
    # executes SPECULATIVELY ~30 cycles BEFORE older load!
    li    t0, 0x1234567
    li    t1, 17
    div   t2, t0, t1
    sub   t2, t2, t2           # 0 (data-dependent on 32-cyc DIV!)
    add   t3, s4, t2           # &X delayed ~34 cycles!
    lw    a0, 0(t3)            # OLDER LOAD: blocked waiting for DIV
    lw    a1, 0(s4)            # YOUNGER LOAD: speculative 1-cyc hit -> reads 0 before C0 writes 1!
    slli  a0, a0, 1
    or    a0, a0, a1
    sw    a0, 0(s7)
""",
                grade_bin_asm="    lw    t6, 0(s7)",
                forbidden_bin=2,  # (a0=1, a1=0)
            ),
        ),
        (
            "L06_CoWR",
            make_litmus_prog(
                "L06_CoWR (Same-Address Write-Read Coherence, NO FENCE)",
                ROUNDS,
                c0_body="""
    li    t0, 1
    sw    t0, 0(s4)            # [X] = 1
    lw    a0, 0(s4)            # r0 = [X] -> must be 1 (STLF) or 2 (C1), NEVER 0!
    sw    a0, 0(s6)
""",
                c1_body="""
    li    t0, 2
    sw    t0, 0(s4)            # [X] = 2
""",
                grade_bin_asm="""
    lw    t6, 0(s6)
    andi  t6, t6, 3
""",
                forbidden_bin=0,  # r0 == 0
            ),
        ),
        (
            "L07_CoWW",
            make_litmus_prog(
                "L07_CoWW (Same-Address Write-Write Coherence, NO FENCE, OoO Observer)",
                ROUNDS,
                c0_body="""
    li    t0, 1
    li    t1, 2
    sw    t0, 0(s4)            # [X] = 1
    sw    t1, 0(s4)            # [X] = 2
""",
                c1_pre_ack="""
    lw    t5, 0(s4)
    fence rw, rw
""",
                c1_body="""
    li    t0, 0x1234567
    li    t1, 17
    div   t2, t0, t1
    sub   t2, t2, t2
    add   t3, s4, t2
    lw    a0, 0(t3)            # older load
    lw    a1, 0(s4)            # younger speculative load
    # Bin: 0 if a0==a1, 1 if a0<a1, 2 if a0>a1 (FORBIDDEN!)
    li    t4, 0
    beq   a0, a1, coww_st
    li    t4, 1
    blt   a0, a1, coww_st
    li    t4, 2
coww_st:
    sw    t4, 0(s7)
""",
                grade_bin_asm="    lw    t6, 0(s7)",
                forbidden_bin=2,  # a0 > a1 (observed C0 stores backwards!)
            ),
        ),
        (
            "L08_R_MCA",
            make_litmus_prog(
                "L08_R_MCA (R: Read-Write Multi-Copy Atomicity + FENCE)",
                ROUNDS,
                c0_body="""
    li    t0, 1
    sw    t0, 0(s4)            # [X] = 1
    fence rw, rw
    sw    t0, 0(s5)            # [Y] = 1
""",
                c1_body="""
    li    t0, 2
    sw    t0, 0(s5)            # [Y] = 2
    fence rw, rw
    lw    a0, 0(s4)            # r1 = [X]
    sw    a0, 0(s7)
""",
                grade_bin_asm="""
    lw    t0, 0(s5)            # final [Y] in {1, 2}
    addi  t0, t0, -1           # 0 if Y==1, 1 if Y==2
    lw    t1, 0(s7)            # r1 in {0, 1}
    slli  t0, t0, 1
    or    t6, t0, t1           # bin 2 = (Y==2 && r1==0) -> FORBIDDEN!
""",
                forbidden_bin=2,
            ),
        ),
        (
            "L09_2plus2W",
            make_litmus_prog(
                "L09_2plus2W (2+2W + FENCE)",
                ROUNDS,
                c0_body="""
    li    t0, 1
    li    t1, 2
    sw    t0, 0(s4)            # [X] = 1
    fence rw, rw
    sw    t1, 0(s5)            # [Y] = 2
""",
                c1_body="""
    li    t0, 1
    li    t1, 2
    sw    t0, 0(s5)            # [Y] = 1
    fence rw, rw
    sw    t1, 0(s4)            # [X] = 2
""",
                grade_bin_asm="""
    lw    t0, 0(s4)            # final [X] in {1, 2}
    addi  t0, t0, -1           # 0 if X==1, 1 if X==2
    lw    t1, 0(s5)            # final [Y] in {1, 2}
    addi  t1, t1, -1           # 0 if Y==1, 1 if Y==2
    slli  t0, t0, 1
    or    t6, t0, t1           # bin 0 = (X==1 && Y==1) -> FORBIDDEN!
""",
                forbidden_bin=0,
            ),
        ),
    ]

    results = {}
    corr_hex = None

    for tname, src in tests:
        words = isa.assemble(src)
        hex_p = BUILD / f"{tname}.hex"
        isa.write_hex(words, hex_p, pad_words=512)
        if tname == "L05_CoRR":
            corr_hex = hex_p

        pass_ln, kv, hist = run_sim(
            BIN_DUAL,
            hex_p,
            [f"+EXPECT_C0_X14=00000000", f"+EXPECT_C1_X14={ROUNDS:08x}"],
        )
        forb = int(kv["c0_x14"], 16)
        rep = kv["replay"]
        cyc = int(kv["cycles"])
        c2c = int(kv["c2c"])
        print(
            f"  [PASS] {tname:18s} rounds={ROUNDS} forbidden={forb} hist={hist} "
            f"replay={rep} c2c={c2c} cyc={cyc}"
        )
        assert sum(hist) == ROUNDS, f"{tname}: histogram sum {sum(hist)} != {ROUNDS}"
        results[tname] = {
            "rounds": ROUNDS,
            "forbidden": forb,
            "hist": hist,
            "replay": rep,
            "c2c": c2c,
            "cycles": cyc,
        }

    # --- L10: Contended Dual-Core Atomic Synchronization Stress (3,000 total atomics) ---
    mutex_words = isa.assemble(make_mutex_stress_prog(iters_per_core=500))
    mutex_hex = BUILD / "L10_Atomic_Mutex_3k.hex"
    isa.write_hex(mutex_words, mutex_hex, pad_words=512)
    _, kv_m, cnts = run_sim(
        BIN_DUAL,
        mutex_hex,
        ["+EXPECT_C0_X14=00000bb8", "+EXPECT_C1_X14=00000bb8"],
    )
    assert cnts[:3] == [1000, 1000, 1000], f"L10 counter mismatch: {cnts}"
    print(
        f"  [PASS] {'L10_Atomic_Mutex':18s} spin={cnts[0]} cas={cnts[1]} amo={cnts[2]} "
        f"(total=3000) amo_hw={kv_m['amo']} c2c={kv_m['c2c']} cyc={kv_m['cycles']}"
    )
    results["L10_Atomic_Mutex"] = {
        "spin": cnts[0],
        "cas": cnts[1],
        "amo": cnts[2],
        "c2c": int(kv_m["c2c"]),
        "cycles": int(kv_m["cycles"]),
    }

    # --- Negative Ablation Proof: L05_CoRR with ENABLE_SNOOP_REPLAY=0 MUST VIOLATE! ---
    print("\n=== Phase 4 Negative Ablation: Speculative LQ Snoop-Replay OFF vs ON (L05_CoRR) ===")
    assert corr_hex is not None
    _, kv_nr, hist_nr = run_sim(
        BIN_NOREPLAY,
        corr_hex,
        [f"+EXPECT_C1_X14={ROUNDS:08x}"],
    )
    forb_nr = int(kv_nr["c0_x14"], 16)
    rep_on_c0, rep_on_c1 = [int(x) for x in results["L05_CoRR"]["replay"].split("/")]
    print(f"  ENABLE_SNOOP_REPLAY=0 : forbidden={forb_nr}/{ROUNDS} hist={hist_nr} replay={kv_nr['replay']}  <-- VIOLATION REPRODUCED!")
    print(
        f"  ENABLE_SNOOP_REPLAY=1 : forbidden=0/{ROUNDS} hist={results['L05_CoRR']['hist']} "
        f"replay={results['L05_CoRR']['replay']}  <-- 100% FIXED BY HARDWARE REPLAY!"
    )
    assert forb_nr > 0, "Expected reproducible CoRR violation when ENABLE_SNOOP_REPLAY=0!"
    assert rep_on_c1 > 0, "Expected >0 hardware LQ snoop-replays on Core 1 when ENABLE_SNOOP_REPLAY=1!"

    results["ablation_snoop_replay"] = {
        "replay_off_forbidden": forb_nr,
        "replay_off_hist": hist_nr,
        "replay_on_forbidden": 0,
        "replay_on_c1_replays": rep_on_c1,
        "replay_on_hist": results["L05_CoRR"]["hist"],
    }

    # --- Bonus Dual-Core CLINT MMIO 2-to-1 RR Arbiter & Trace Log Sanity Check ---
    clint_src = """
    # Both cores concurrently stress the 2-to-1 RR uncached CLINT MMIO arbiter:
    #   0x0200_0000: msip0, 0x0200_0004: msip1
    #   0x0200_BFF8: mtime_lo (strictly increasing every cycle)
    #   0x0200_C000: scratch[0], 0x0200_C004: scratch[1]
    li    s0, 0x02000000       # CLINT MSIP base
    li    s3, 0x0200C000       # CLINT MTIME(-8) / SCRATCH(+0,+4) base
    csrr  s1, mhartid
    li    s2, 32
    li    x14, 0
    bne   s1, zero, clint_c1

clint_c0:
    # Core 0 writes MSIP0=1, SCRATCH0=0x11223344, loops reading strictly monotonic MTIME_LO
    li    t0, 1
    sw    t0, 0(s0)            # MSIP0 = 1
    li    t1, 0x11223344
    sw    t1, 0(s3)            # SCRATCH0 = 0x11223344
    lw    t2, -8(s3)           # initial MTIME_LO (0x0200_BFF8)
clint_c0_lp:
    lw    t3, -8(s3)           # MTIME_LO must strictly advance (t3 > t2)
    bgeu  t2, t3, clint_c0_err
    mv    t2, t3
    lw    t4, 0(s3)            # readback SCRATCH0
    bne   t4, t1, clint_c0_err
    addi  s2, s2, -1
    bne   s2, zero, clint_c0_lp
    lw    t5, 0(s0)            # readback MSIP0 == 1
    add   x14, t4, t5          # 0x11223345
    ebreak
clint_c0_err:
    li    x14, 0xDEAD0000
    ebreak

clint_c1:
    # Core 1 writes MSIP1=1, SCRATCH1=0x55667788, loops reading strictly monotonic MTIME_LO
    li    t0, 1
    sw    t0, 4(s0)            # MSIP1 = 1
    li    t1, 0x55667788
    sw    t1, 4(s3)            # SCRATCH1 = 0x55667788
    lw    t2, -8(s3)           # initial MTIME_LO (0x0200_BFF8)
clint_c1_lp:
    lw    t3, -8(s3)           # MTIME_LO must strictly advance (t3 > t2)
    bgeu  t2, t3, clint_c1_err
    mv    t2, t3
    lw    t4, 4(s3)            # readback SCRATCH1
    bne   t4, t1, clint_c1_err
    addi  s2, s2, -1
    bne   s2, zero, clint_c1_lp
    lw    t5, 4(s0)            # readback MSIP1 == 1
    add   x14, t4, t5          # 0x55667789
    ebreak
clint_c1_err:
    li    x14, 0xDEAD0001
    ebreak
"""
    clint_hex = BUILD / "clint_dual_mmio.hex"
    clint_log = BUILD / "clint_dual_mmio.log"
    isa.write_hex(isa.assemble(clint_src), clint_hex, pad_words=256)
    _, kv_cl, _ = run_sim(
        BIN_DUAL,
        clint_hex,
        [
            "+EXPECT_C0_X14=11223345",
            "+EXPECT_C1_X14=55667789",
            f"+LOG={clint_log}",
        ],
    )
    log_lines = clint_log.read_text(encoding="utf-8").splitlines()
    c_lines = sum(1 for ln in log_lines if ln.startswith("C "))
    assert c_lines > 100, f"Expected >100 RVFI commit lines in trace log, got {c_lines}"
    print(
        f"  [PASS] {'CLINT_Dual_MMIO':18s} c0_x14={kv_cl['c0_x14']} c1_x14={kv_cl['c1_x14']} "
        f"rvfi_commits={c_lines} cyc={kv_cl['cycles']}"
    )
    results["CLINT_Dual_MMIO"] = {
        "c0_x14": kv_cl["c0_x14"],
        "c1_x14": kv_cl["c1_x14"],
        "rvfi_commits": c_lines,
        "cycles": int(kv_cl["cycles"]),
    }

    (BUILD / "phase4_litmus_results.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(
        f"\nPASS [Phase 4]: 10/10 RVWMO Litmus & Atomic Mutex tests PASSED "
        f"({9*ROUNDS:,} litmus rounds + 3,000 contended atomics, 0 forbidden outcomes, "
        f"0 SWMR/Shadow errors; negative ablation proved {forb_nr} -> 0 violations via {rep_on_c1} hardware replays)!"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
