#!/usr/bin/env python3
"""
Phase U1 End-to-End Verification Suite: Precise M-Mode Traps, Full Trap CSRs,
Misaligned & Illegal Detection, ECALL/EBREAK/MRET, Speculative-Fault Squash Proof,
Precise Store-Drain-Before-Trap Proof, and Dual-Core Concurrent Traps!

Runs on:
  - Single-Core 3-Level Hierarchy SoC (Vp2_c0): Bit-exact RVFI comparison against
    Golden RV32IMA_ISS (+TRACE) AND ooo_trace_mon2w (0 invariant errors!)
  - Dual-Core MOESI SoC (Vdual_soc): Concurrent dual-core traps + shared atomic
    traffic with ALWAYS-ON SWMR + 128-bit Golden Shadow checker + DRAM +JITTER!
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import List

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import rv32ima_asm as isa  # noqa: E402

BUILD = ROOT / "build" / "phase_u1"
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


def run_single_trap_test(name: str, asm_src: str, expect_traps: int, jitter: bool = True) -> None:
    BUILD.mkdir(parents=True, exist_ok=True)
    words = isa.assemble(asm_src)
    hex_path = BUILD / f"{name}.hex"
    trace_path = BUILD / f"{name}.trace"
    isa.write_hex(words, hex_path, pad_words=2048)

    iss = isa.RV32IMA_ISS(words, core_id=0, unified_mem=True)
    trace = iss.run(max_steps=50_000)
    isa.write_trace(trace, trace_path)

    if iss.trap_count != expect_traps:
        print(f"FAIL [{name}]: Golden ISS trap_count={iss.trap_count}, expected {expect_traps}")
        sys.exit(1)

    exp_x14 = iss.regs[14]
    cmd = [
        str(BIN_SINGLE),
        f"+HEX={hex_path}",
        f"+TRACE={trace_path}",
        f"+EXPECT_X14={exp_x14:08x}",
        "+MAX_CYCLES=50000",
    ]
    if jitter:
        cmd.append("+JITTER")
    out = run_cmd(cmd, "PASS [single_ooo2w_L2]")
    print(f"  [OK] {name} (traps={iss.trap_count}, x14=0x{exp_x14:08x}): {out}")


# ---------------------------------------------------------------------------
# U1-01: Illegal Instructions + Illegal CSR Accesses (mcause=2, mtval=instr)
# ---------------------------------------------------------------------------
TEST_U1_01_ILLEGAL = """
    la    t0, trap_handler
    csrw  mtvec, t0
    addi  s0, zero, 0           # trap count
    addi  s1, zero, 0           # checksum accumulator
    li    a5, 0x12345678        # canary register that illegal ops must NOT clobber!

    # 1. All-ones illegal opcode (0xFFFFFFFF)
    .word 0xFFFFFFFF
    # 2. All-zeros illegal opcode (0x00000000)
    .word 0x00000000
    # 3. Illegal LOAD funct3=3 targeting a5 (must NOT overwrite a5!)
    #    enc_i(0, rs1=0, f3=3, rd=15, op=0x03) = 0x00003783
    .word 0x00003783
    # 4. Illegal LR.W with rs2!=0 targeting a5 (f5=00010, rs2=1, rs1=0, f3=2, rd=15, op=0x2F)
    .word 0x101027AF
    # 5. Illegal write to read-only CSR mhartid (csrrw a5, mhartid, t0 -> must trap, NOT clobber a5!)
    csrrw a5, mhartid, t0
    # 6. Illegal access to unimplemented CSR 0x777 (csrrs a5, 0x777, zero -> must trap!)
    csrrs a5, 0x777, zero

    # Verify a5 canary intact + s0 == 6 traps!
    li    t1, 0x12345678
    bne   a5, t1, fail_exit
    addi  t2, zero, 6
    bne   s0, t2, fail_exit

    # Final checksum in x14 (a4)
    add   x14, s1, s0
    csrw  mtvec, zero
    ebreak

fail_exit:
    li    x14, 0xBAD00001
    csrw  mtvec, zero
    ebreak

trap_handler:
    csrr  t3, mcause
    csrr  t4, mtval
    csrr  t5, mepc
    addi  t6, zero, 2
    bne   t3, t6, trap_bad
    # Mix mcause ^ mtval ^ mepc into checksum s1
    xor   t6, t4, t5
    add   s1, s1, t6
    addi  s0, s0, 1
    addi  t5, t5, 4
    csrw  mepc, t5
    mret
trap_bad:
    li    x14, 0xBAD00002
    csrw  mtvec, zero
    ebreak
"""

# ---------------------------------------------------------------------------
# U1-02: Misaligned LH/LHU/LW/SH/SW/LR/SC/AMO + Misaligned JALR (mcause=4,6,0)
# ---------------------------------------------------------------------------
TEST_U1_02_MISALIGNED = """
    la    t0, trap_handler
    csrw  mtvec, t0

    # Initialize sentinel memory at 0x10000..0x1000F
    li    s2, 0x10000
    li    t1, 0xCAFEBABE
    li    t2, 0xDEADBEEF
    sw    t1, 0(s2)
    sw    t2, 4(s2)
    sw    t1, 8(s2)
    sw    t2, 12(s2)

    addi  s0, zero, 0           # trap count (expect 11)
    addi  s1, zero, 0           # weighted cause+tval checksum
    li    a5, 0x55AA33CC        # canary rd (must NEVER be clobbered by faulting ops!)

    # Misaligned loads (mcause = 4)
    lh    a5, 1(s2)             # 1: tval = 0x10001
    lhu   a5, 3(s2)             # 2: tval = 0x10003
    lw    a5, 1(s2)             # 3: tval = 0x10001
    lw    a5, 2(s2)             # 4: tval = 0x10002
    lw    a5, 3(s2)             # 5: tval = 0x10003

    # Misaligned stores (mcause = 6) -> MUST NOT modify memory!
    li    t3, 0xBAD1BAD1
    sh    t3, 1(s2)             # 6: tval = 0x10001
    sw    t3, 2(s2)             # 7: tval = 0x10002

    # Misaligned RV32A atomics (LR -> mcause=4, SC/AMO -> mcause=6)
    addi  s3, s2, 2
    lr.w     a5, (s3)           # 8: mcause = 4, tval = 0x10002
    addi  s4, s2, 1
    sc.w     a5, t3, (s4)       # 9: mcause = 6, tval = 0x10001
    addi  s5, s2, 3
    amoadd.w a5, t3, (s5)       # 10: mcause = 6, tval = 0x10003

    # Misaligned JALR target (mcause = 0, tval = (s2 + 2) & ~1 = 0x10002; a5 must NOT get PC+4!)
    jalr  a5, s2, 3             # 11: target = 0x10002 -> mcause = 0!

    # Verify a5 canary untouched
    li    t4, 0x55AA33CC
    bne   a5, t4, fail_m
    # Verify trap count == 11
    addi  t4, zero, 11
    bne   s0, t4, fail_m
    # Verify ALL 4 sentinel memory words are 100% untouched!
    lw    t4, 0(s2)
    bne   t4, t1, fail_m
    lw    t4, 4(s2)
    bne   t4, t2, fail_m
    lw    t4, 8(s2)
    bne   t4, t1, fail_m
    lw    t4, 12(s2)
    bne   t4, t2, fail_m

    add   x14, s1, s0
    csrw  mtvec, zero
    ebreak

fail_m:
    li    x14, 0xBAD00003
    csrw  mtvec, zero
    ebreak

trap_handler:
    csrr  t4, mcause
    csrr  t5, mtval
    csrr  t6, mepc
    # Accumulate (mcause << 20) ^ mtval into s1
    slli  a0, t4, 20
    xor   a0, a0, t5
    add   s1, s1, a0
    addi  s0, s0, 1
    addi  t6, t6, 4
    csrw  mepc, t6
    mret
"""

# ---------------------------------------------------------------------------
# U1-03: ECALL Syscalls (mcause=11) + EBREAK Breakpoint (mcause=3) + mstatus MIE/MPIE
# ---------------------------------------------------------------------------
TEST_U1_03_ECALL_MSTATUS = """
    la    t0, trap_handler
    csrw  mtvec, t0
    addi  s0, zero, 0           # trap counter

    # Set mstatus.MIE = 1 (bit 3 = 0x8) before ECALL to prove MIE->MPIE save & MRET restore!
    csrsi mstatus, 8
    csrr  s2, mstatus           # s2 has MIE=1

    # Syscall 1: a7=1 -> returns a0 + a1 in a0
    addi  a7, zero, 1
    addi  a0, zero, 40
    addi  a1, zero, 2
    ecall                       # trap 1: returns a0 = 42, MRET restores MIE=1!
    mv    s3, a0                # s3 = 42
    csrr  s4, mstatus           # MIE must be 1 again, MPIE must be 1 (0x3888)!

    # Syscall 2: a7=2 -> returns (a0 * a1) in a0
    addi  a7, zero, 2
    addi  a0, zero, 7
    addi  a1, zero, 9
    ecall                       # trap 2: returns a0 = 63
    mv    s5, a0                # s5 = 63

    # Breakpoint trap (mtvec != 0 -> mcause=3, mtval=ebreak_pc!)
    ebreak                      # trap 3!

    # Verify results: s3=42, s5=63, s0=3, (s4 & 0x88)==0x88
    addi  t1, zero, 42
    bne   s3, t1, fail_e
    addi  t1, zero, 63
    bne   s5, t1, fail_e
    addi  t1, zero, 3
    bne   s0, t1, fail_e
    andi  t1, s4, 0x88
    addi  t2, zero, 0x88
    bne   t1, t2, fail_e

    # Checksum: s3 + s5 + s0 + (s4 & 0xFF) + s6(mstatus inside handler)
    andi  t1, s4, 0xFF
    add   x14, s3, s5
    add   x14, x14, s0
    add   x14, x14, t1
    add   x14, x14, s6
    csrw  mtvec, zero           # clear mtvec so final EBREAK halts cleanly!
    ebreak

fail_e:
    li    x14, 0xBAD00004
    csrw  mtvec, zero
    ebreak

trap_handler:
    csrr  t3, mcause
    csrr  t4, mepc
    csrr  t5, mstatus
    andi  s6, t5, 0xFF          # inside trap: MIE must be 0, MPIE must be 1 -> 0x80!
    addi  s0, s0, 1
    addi  t6, zero, 11
    beq   t3, t6, handle_ecall
    addi  t6, zero, 3
    beq   t3, t6, handle_ebreak
    j     fail_e

handle_ecall:
    addi  t6, zero, 1
    beq   a7, t6, sys_add
    addi  t6, zero, 2
    beq   a7, t6, sys_mul
    j     fail_e
sys_add:
    add   a0, a0, a1
    j     ret_trap
sys_mul:
    mul   a0, a0, a1
    j     ret_trap

handle_ebreak:
    # Verify mtval == mepc on EBREAK!
    csrr  t6, mtval
    bne   t6, t4, fail_e

ret_trap:
    addi  t4, t4, 4
    csrw  mepc, t4
    mret
"""

# ---------------------------------------------------------------------------
# U1-04: OoO Speculative-Fault Squash Proof!
#   A slow DIV resolves a mispredicted taken branch; along the wrong-path
#   fall-through sit an illegal instr, misaligned LW, misaligned SW, misaligned
#   JALR, and ECALL. ALL MUST BE SQUASHED WITH 0 TRAPS TAKEN!
# ---------------------------------------------------------------------------
TEST_U1_04_SPEC_SQUASH = """
    la    t0, trap_handler
    csrw  mtvec, t0
    li    s2, 0x10000
    li    t1, 0x77778888
    sw    t1, 0(s2)

    addi  s0, zero, 0           # trap count (MUST remain 0!)
    addi  s1, zero, 0           # loop counter
    addi  s3, zero, 8           # 8 rounds
    li    s4, 1234567
    addi  s5, zero, 13
    addi  x14, zero, 0

loop_spec:
    # Slow DIV keeps the branch in RS_BR for ~33 cycles while 2-wide front-end
    # speculatively fetches & dispatches the entire wrong-path fault minefield!
    div   t2, s4, s5            # ~94966 != 0
    add   x14, x14, t2
    bne   t2, zero, safe_target # ALWAYS TAKEN (mispredicted on initial BHT=weakly-NT!)

    # --- WRONG-PATH FAULT MINEFIELD (must NEVER take a trap!) ---
    .word 0xFFFFFFFF            # speculative illegal instruction!
    lw    t3, 1(s2)             # speculative misaligned load!
    sw    t2, 2(s2)             # speculative misaligned store!
    jalr  t4, s2, 3             # speculative misaligned JALR!
    ecall                       # speculative ECALL!

safe_target:
    # Second pattern: alternating direction branch to force repeated mispredictions!
    andi  t5, s1, 1
    div   t6, s4, s5
    sub   t6, t6, t2            # t6 = 0
    beq   t6, zero, safe_target2
    lw    t3, 3(s2)             # speculative misaligned load!
    sh    t2, 1(s2)             # speculative misaligned store!
    .word 0x00000000            # speculative illegal instruction!

safe_target2:
    addi  s1, s1, 1
    blt   s1, s3, loop_spec

    # Verify ZERO traps taken and sentinel memory untouched!
    bne   s0, zero, fail_s
    lw    t4, 0(s2)
    bne   t4, t1, fail_s

    csrw  mtvec, zero
    ebreak

fail_s:
    li    x14, 0xBAD00005
    csrw  mtvec, zero
    ebreak

trap_handler:
    addi  s0, s0, 1             # Should NEVER be reached!
    csrr  t6, mepc
    addi  t6, t6, 4
    csrw  mepc, t6
    mret
"""

# ---------------------------------------------------------------------------
# U1-05: Precise Store-Drain-Before-Trap & Younger-Store Squash Proof!
#   4 older stores (cold L1D miss lines!) -> misaligned load -> 2 younger poison stores.
#   Inside trap handler (BEFORE MRET!), read all 4 lines and prove older stores
#   committed cleanly AND younger poison stores were completely squashed!
# ---------------------------------------------------------------------------
TEST_U1_05_STORE_DRAIN = """
    la    t0, trap_handler
    csrw  mtvec, t0
    li    s2, 0x12000           # Cold cache lines (stride 64 B so all 4 miss L1D!)
    li    a0, 0x11110001
    li    a1, 0x22220002
    li    a2, 0x33330003
    li    a3, 0x44440004
    li    t5, 0xDEAD0001
    li    t6, 0xDEAD0002
    addi  s0, zero, 0

    # 4 older stores preceding a faulting misaligned load
    sw    a0, 0(s2)
    sw    a1, 64(s2)
    sw    a2, 128(s2)
    sw    a3, 192(s2)
    lw    t1, 1(s2)             # FAULT! mcause=4, mtval=0x12001 -> older 4 stores MUST drain first!
    # 2 younger poison stores (if executed/not squashed, would overwrite 0(s2) and 64(s2)!)
    sw    t5, 0(s2)
    sw    t6, 64(s2)

    # After trap handler skips past the 2 poison stores (mepc += 12!), verify s0 == 1 & x14 ok
    addi  t1, zero, 1
    bne   s0, t1, fail_sd
    csrw  mtvec, zero
    ebreak

fail_sd:
    li    x14, 0xBAD00006
    csrw  mtvec, zero
    ebreak

trap_handler:
    # Verify mcause == 4 & mtval == 0x12001
    csrr  t1, mcause
    addi  t2, zero, 4
    bne   t1, t2, fail_sd
    csrr  t1, mtval
    addi  t2, s2, 1
    bne   t1, t2, fail_sd

    # Read back ALL 4 older stores RIGHT INSIDE the trap handler!
    lw    t1, 0(s2)
    bne   t1, a0, fail_sd
    lw    t2, 64(s2)
    bne   t2, a1, fail_sd
    lw    t3, 128(s2)
    bne   t3, a2, fail_sd
    lw    t4, 192(s2)
    bne   t4, a3, fail_sd

    # Compute exact checksum from the 4 drained values
    add   x14, t1, t2
    add   x14, x14, t3
    add   x14, x14, t4
    addi  s0, s0, 1

    # Skip faulting lw (+4) AND the 2 poison stores (+8) -> mepc += 12
    csrr  t1, mepc
    addi  t1, t1, 12
    csrw  mepc, t1
    mret
"""

# ---------------------------------------------------------------------------
# U1-06: Dual-Core Concurrent Traps + Coherent Shared Atomics (+JITTER!)
#   Core 0 and Core 1 run simultaneously, each taking 6 distinct traps (illegal,
#   misaligned load/store/AMO/JALR, ECALL) + speculative fault squash + shared
#   AMOADD.W counter updates!
# ---------------------------------------------------------------------------
TEST_U1_06_DUAL_CORE = """
    csrr  s0, mhartid           # s0 = 0 (Core 0) or 1 (Core 1)
    la    t0, trap_handler
    csrw  mtvec, t0

    # Per-core private scratch buffer: Core 0 = 0x18000, Core 1 = 0x18100
    li    s2, 0x18000
    slli  t1, s0, 8
    add   s2, s2, t1
    csrw  mscratch, s2          # store per-core base in mscratch!

    # Sentinel word at 0(s2) = 0xA0000000 + core_id
    li    t2, 0xA0000000
    add   t2, t2, s0
    sw    t2, 0(s2)

    # Shared atomic counter at 0x19000
    li    s3, 0x19000
    addi  s1, zero, 0           # per-core trap count
    addi  s4, zero, 0           # per-core trap cause sum

    # 8 rounds of: shared atomic increment + 6 traps + speculative fault squash!
    addi  s5, zero, 0
    addi  s6, zero, 8
    li    a2, 99991
    addi  a3, zero, 7
round_lp:
    addi  t1, zero, 1
    amoadd.w zero, t1, (s3)     # coherent shared atomic increment

    # 1. Illegal instr (mcause = 2)
    .word 0xFFFFFFFF
    # 2. Misaligned LW (mcause = 4)
    lw    a5, 1(s2)
    # 3. Misaligned SW (mcause = 6) -> must NOT corrupt 0(s2)!
    sw    s5, 2(s2)
    # 4. Misaligned AMOADD (mcause = 6)
    addi  t3, s2, 2
    amoadd.w a5, t1, (t3)
    # 5. Misaligned JALR (mcause = 0)
    jalr  a5, s2, 3
    # 6. ECALL (mcause = 11)
    ecall

    # Speculative fault squash behind slow DIV!
    div   t4, a2, a3
    bne   t4, zero, ok_spec
    .word 0x00000000
    sw    t4, 1(s2)
ok_spec:
    addi  s5, s5, 1
    blt   s5, s6, round_lp

    # Verify per-core sentinel untouched & exact trap count = 8 * 6 = 48
    # Cause sum per round = 2 + 4 + 6 + 6 + 0 + 11 = 29; 8 rounds = 232 (0xE8)
    lw    t3, 0(s2)
    bne   t3, t2, fail_d
    addi  t4, zero, 48
    bne   s1, t4, fail_d
    addi  t4, zero, 232
    bne   s4, t4, fail_d

    # Wait until shared counter 0x19000 reaches 16 (both cores completed all 8 rounds!)
wait_both:
    lw    t5, 0(s3)
    addi  t6, zero, 16
    blt   t5, t6, wait_both

    # Final x14 = (core_id << 16) | (trap_cnt << 8) | cause_sum = 0x000030E8 (C0) / 0x000130E8 (C1)
    slli  x14, s0, 16
    slli  t1, s1, 8
    or    x14, x14, t1
    or    x14, x14, s4
    csrw  mtvec, zero
    ebreak

fail_d:
    li    x14, 0xBAD00009
    csrw  mtvec, zero
    ebreak

trap_handler:
    csrr  t5, mcause
    add   s4, s4, t5
    addi  s1, s1, 1
    csrr  t6, mepc
    addi  t6, t6, 4
    csrw  mepc, t6
    mret
"""


def main() -> int:
    print("=== Phase U1 End-to-End Verification: Precise M-Mode Traps & OoO Exception Safety ===")
    run_single_trap_test("u1_01_illegal_csr", TEST_U1_01_ILLEGAL, expect_traps=6)
    run_single_trap_test("u1_02_misaligned_mem_amo_jalr", TEST_U1_02_MISALIGNED, expect_traps=11)
    run_single_trap_test("u1_03_ecall_ebreak_mstatus", TEST_U1_03_ECALL_MSTATUS, expect_traps=3)
    run_single_trap_test("u1_04_ooo_spec_fault_squash", TEST_U1_04_SPEC_SQUASH, expect_traps=0)
    run_single_trap_test("u1_05_precise_store_drain", TEST_U1_05_STORE_DRAIN, expect_traps=1)

    # U1-06: Dual-core concurrent traps + coherent shared atomics + DRAM +JITTER!
    words = isa.assemble(TEST_U1_06_DUAL_CORE)
    dual_hex = BUILD / "u1_06_dual_core_traps.hex"
    isa.write_hex(words, dual_hex, pad_words=2048)
    out = run_cmd(
        [
            str(BIN_DUAL),
            f"+HEX={dual_hex}",
            "+EXPECT_C0_X14=000030e8",
            "+EXPECT_C1_X14=000130e8",
            "+MAX_CYCLES=50000",
            "+JITTER",
        ],
        "PASS [dual_ooo2w_soc]",
    )
    print(f"  [OK] u1_06_dual_core_traps (96 total traps across 2 cores + JITTER): {out}")

    # U1-07: Real GCC -O2 compiled C __attribute__((interrupt("machine"))) trap handler!
    elf_c = BUILD / "test_traps_c.elf"
    bin_c = BUILD / "test_traps_c.bin"
    hex_c = BUILD / "test_traps_c.hex"
    trc_c = BUILD / "test_traps_c.trace"
    subprocess.run(
        [
            "/usr/bin/riscv64-unknown-elf-gcc",
            "-march=rv32ima", "-mabi=ilp32", "-O2",
            "-ffreestanding", "-nostdlib", "-nostartfiles",
            "-Wall", "-Wextra", "-Isw/common", "-T", "sw/common/link.ld",
            "sw/common/crt0.S", "sw/c_bench/test_traps_c.c",
            "-o", str(elf_c),
        ],
        cwd=str(ROOT),
        check=True,
    )
    subprocess.run(
        ["/usr/bin/riscv64-unknown-elf-objcopy", "-O", "binary", str(elf_c), str(bin_c)],
        cwd=str(ROOT),
        check=True,
    )
    subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "elf2hex.py"), str(bin_c), str(hex_c), "--pad-words", "2048"],
        cwd=str(ROOT),
        check=True,
    )
    c_words = [int(ln.strip(), 16) for ln in hex_c.read_text(encoding="utf-8").splitlines() if ln.strip()]
    c_iss = isa.RV32IMA_ISS(c_words, core_id=0, unified_mem=True)
    c_trace = c_iss.run(max_steps=50_000)
    isa.write_trace(c_trace, trc_c)
    assert c_iss.trap_count == 4 and c_iss.regs[14] == 0x0000_0417, (
        f"C ISS mismatch: traps={c_iss.trap_count}, x14=0x{c_iss.regs[14]:08x}"
    )
    out_c1 = run_cmd(
        [
            str(BIN_SINGLE),
            f"+HEX={hex_c}",
            f"+TRACE={trc_c}",
            "+EXPECT_X14=00000417",
            "+MAX_CYCLES=50000",
            "+JITTER",
        ],
        "PASS [single_ooo2w_L2]",
    )
    print(f"  [OK] u1_07a_gcc_c_trap_handler_1C (traps=4, x14=0x00000417): {out_c1}")
    out_c2 = run_cmd(
        [
            str(BIN_DUAL),
            f"+HEX={hex_c}",
            "+EXPECT_C0_X14=00000417",
            "+EXPECT_C1_X14=00010417",
            "+MAX_CYCLES=50000",
            "+JITTER",
        ],
        "PASS [dual_ooo2w_soc]",
    )
    print(f"  [OK] u1_07b_gcc_c_trap_handler_2C (8 traps across 2 cores + JITTER): {out_c2}")

    print("PASS [test_phase_u1_traps]: All 7/7 Phase U1 trap & OoO exception safety suites passed!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
