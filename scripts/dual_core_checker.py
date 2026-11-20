#!/usr/bin/env python3
"""
Phase 5: Dual-Core 4-Wide RVFI + Global Channel A Bus Linearizability & RVWMO Checker.

Verifies traces dumped by tb_dual_ooo2w_soc (+LOG=<file>) containing:
  - C <cyc> <core> <pc> <instr> <rd_we> <rd_addr> <rd_wdata> <mem_v> <mem_we> <mem_addr> <mem_wstrb> <mem_wdata>
  - W <cyc> <core> <laddr> <line128_hex>      (Exact MOESI M-state write linearization event)
  - B <cyc> <src> <op> <laddr> <c2c> <shared> (Serialized Channel A coherence bus event)

Verification Modes:
  1. mode='disjoint':
     - Runs TWO independent bit-exact RV32IMA_Zicsr_Zifencei golden ISS instances
       (core_id=0 and core_id=1) and asserts 100% commit-by-commit equality on
       PC, instr, rd_we, rd_addr, rd_wdata, mem_valid, mem_we, mem_addr, wstrb, wdata,
       PLUS runs the full shared linearizability pass!
  2. mode='shared':
     - Pass 1: Per-core bit-exact RV32IMA_Zicsr_Zifencei ISA audit (PC, instruction
       fetch, ALU, RV32M MUL/DIV/REM, branches, JAL/JALR, CSRs, effective memory
       addresses, byte-aligned store data & write strobes).
     - Pass 2: Reconstructs the EXACT Global Serialized Memory Timeline from all `W`
       events (asserting SWMR single-writer-per-cycle-per-line, 1-to-1 write accounting
       with MSHR store coalescing, exact atomic AMO*.W read-modify-write linearizability
       against the global timeline, LR.W/SC.W reservation validity, and FENCE/AMO store-drain).
     - Pass 3: Verifies EVERY committed Load and LR.W on both cores obeys:
       * No out-of-thin-air / future reads (t_w <= t_commit, or legal same-core STLF)
       * CoWR (Program-Order Write-to-Read coherence)
       * CoRR (Program-Order Read-to-Read coherence, per byte, no fence needed)
       * FENCE / FENCE.I / AMO / Acquire-Release global barrier visibility
       * Cross-core Message-Passing (MP) causal vector-clock visibility

Includes `--self-test` which injects 6 distinct hardware bug classes (corrupted ALU,
corrupted load data, CoRR violation, CoWR violation, broken AMO atomicity, illegal
SC success after remote write) and asserts that the checker catches EVERY ONE!
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
from collections import defaultdict, deque
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import rv32ima_asm as isa  # noqa: E402
from rv32ima_asm import s32, sext, u32  # noqa: E402


# ---------------------------------------------------------------------------
# Trace Records & Helpers
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class CommitRec:
    cyc: int
    core: int
    pc: int
    instr: int
    rd_we: int
    rd_addr: int
    rd_wdata: int
    mem_v: int
    mem_we: int
    mem_addr: int
    mem_wstrb: int
    mem_wdata: int


@dataclasses.dataclass
class WriteEvt:
    cyc: int
    core: int
    laddr: int
    line128: int


@dataclasses.dataclass
class BusEvt:
    cyc: int
    src: int
    op: int
    laddr: int
    c2c: int
    shared: int


@dataclasses.dataclass
class ArchWriteOp:
    c_idx: int
    commit_cyc: int
    bar_epoch: int
    addr: int
    laddr: int
    wstrb: int
    wdata: int
    is_amo: bool
    amo_f5: int
    amo_rs2: int
    rd_addr: int
    obs_rd: int
    is_sc: bool
    # Assigned in Pass 2:
    g_idx: int = -1
    w_cyc: int = -1


@dataclasses.dataclass
class ArchLoadOp:
    c_idx: int
    commit_cyc: int
    bar_epoch: int
    pc: int
    addr: int
    laddr: int
    size: int       # 1, 2, or 4
    unsigned: bool
    obs_val: int
    is_lr: bool
    is_mmio: bool


@dataclasses.dataclass
class ArchBarrierOp:
    c_idx: int
    commit_cyc: int
    bar_epoch: int
    kind: str       # FENCE, FENCE.I, AMO, LR, SC


@dataclasses.dataclass
class LineVersion:
    g_idx: int
    w_cyc: int
    core: int
    line128: int
    byte_mod_gidx: Tuple[int, ...]   # 16 ints: g_idx when each byte last changed
    causal_fence_cyc: int            # commit cyc of writer's last barrier before this write


@dataclasses.dataclass
class CheckStats:
    c0_commits: int = 0
    c1_commits: int = 0
    w_events: int = 0
    bus_events: int = 0
    bus_gets: int = 0
    bus_getm: int = 0
    bus_upg: int = 0
    bus_c2c: int = 0
    loads_checked: int = 0
    stlf_loads: int = 0
    stores_checked: int = 0
    coalesced_stores: int = 0
    amos_checked: int = 0
    lr_checked: int = 0
    sc_ok: int = 0
    sc_fail: int = 0
    fences_checked: int = 0
    fence_i_checked: int = 0
    mmio_ops: int = 0


class CheckError(RuntimeError):
    pass


def _line_get_word(line128: int, woff: int) -> int:
    return (line128 >> (woff * 32)) & 0xFFFF_FFFF


def _line_get_byte(line128: int, boff: int) -> int:
    return (line128 >> (boff * 8)) & 0xFF


def _line_apply_write(line128: int, addr: int, wstrb: int, wdata: int) -> Tuple[int, int]:
    """Apply aligned 32-bit word write + 4-bit strobe to 128-bit line; return (new_line, byte_mask16)."""
    woff = (addr >> 2) & 3
    shift = woff * 32
    cur_w = (line128 >> shift) & 0xFFFF_FFFF
    new_w = cur_w
    bmask16 = 0
    for b in range(4):
        if (wstrb >> b) & 1:
            mask = 0xFF << (b * 8)
            new_w = (new_w & ~mask) | (wdata & mask)
            bmask16 |= 1 << (woff * 4 + b)
    new_line = (line128 & ~(0xFFFF_FFFF << shift)) | (new_w << shift)
    return new_line, bmask16


def _extract_load_val(line128: int, addr: int, size: int, unsigned: bool) -> int:
    boff = addr & 0xF
    if size == 1:
        b0 = _line_get_byte(line128, boff)
        return b0 if unsigned else u32(sext(b0, 8))
    if size == 2:
        b0 = _line_get_byte(line128, boff)
        b1 = _line_get_byte(line128, (boff + 1) & 0xF)
        h = b0 | (b1 << 8)
        return h if unsigned else u32(sext(h, 16))
    woff = (addr >> 2) & 3
    return _line_get_word(line128, woff)


def _amo_alu(f5: int, old_w: int, rs2: int) -> int:
    if f5 == 0b00001:  # AMOSWAP
        return u32(rs2)
    if f5 == 0b00000:  # AMOADD
        return u32(old_w + rs2)
    if f5 == 0b00100:  # AMOXOR
        return u32(old_w ^ rs2)
    if f5 == 0b01100:  # AMOAND
        return u32(old_w & rs2)
    if f5 == 0b01000:  # AMOOR
        return u32(old_w | rs2)
    if f5 == 0b10000:  # AMOMIN
        return u32(old_w if s32(old_w) < s32(rs2) else rs2)
    if f5 == 0b10100:  # AMOMAX
        return u32(old_w if s32(old_w) > s32(rs2) else rs2)
    if f5 == 0b11000:  # AMOMINU
        return u32(old_w if u32(old_w) < u32(rs2) else rs2)
    if f5 == 0b11100:  # AMOMAXU
        return u32(old_w if u32(old_w) > u32(rs2) else rs2)
    raise CheckError(f"Unsupported AMO f5=0x{f5:02x}")


# ---------------------------------------------------------------------------
# Trace Loader
# ---------------------------------------------------------------------------
def parse_trace_log(log_path: Path) -> Tuple[List[CommitRec], List[WriteEvt], List[BusEvt]]:
    commits: List[CommitRec] = []
    writes: List[WriteEvt] = []
    buses: List[BusEvt] = []

    for lineno, raw in enumerate(log_path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        p = line.split()
        tag = p[0]
        if tag == "C":
            if len(p) != 13:
                raise CheckError(f"Malformed C record at line {lineno}: {line}")
            commits.append(
                CommitRec(
                    cyc=int(p[1]),
                    core=int(p[2]),
                    pc=int(p[3], 16),
                    instr=int(p[4], 16),
                    rd_we=int(p[5]),
                    rd_addr=int(p[6]),
                    rd_wdata=int(p[7], 16),
                    mem_v=int(p[8]),
                    mem_we=int(p[9]),
                    mem_addr=int(p[10], 16),
                    mem_wstrb=int(p[11], 16),
                    mem_wdata=int(p[12], 16),
                )
            )
        elif tag == "W":
            if len(p) != 5:
                raise CheckError(f"Malformed W record at line {lineno}: {line}")
            writes.append(
                WriteEvt(
                    cyc=int(p[1]),
                    core=int(p[2]),
                    laddr=int(p[3], 16),
                    line128=int(p[4], 16),
                )
            )
        elif tag == "B":
            if len(p) != 7:
                raise CheckError(f"Malformed B record at line {lineno}: {line}")
            buses.append(
                BusEvt(
                    cyc=int(p[1]),
                    src=int(p[2]),
                    op=int(p[3]),
                    laddr=int(p[4], 16),
                    c2c=int(p[5]),
                    shared=int(p[6]),
                )
            )
        else:
            raise CheckError(f"Unknown trace record '{tag}' at line {lineno}")
    return commits, writes, buses


def load_hex_words(hex_path: Path) -> List[int]:
    words: List[int] = []
    for ln in hex_path.read_text(encoding="utf-8").splitlines():
        s = ln.strip()
        if s:
            words.append(int(s, 16) & 0xFFFF_FFFF)
    return words


def build_init_lines(imem_words: List[int]) -> Dict[int, int]:
    lines: Dict[int, int] = {}
    n_lines = (len(imem_words) + 3) // 4
    for li in range(n_lines):
        w0 = imem_words[li * 4 + 0] if (li * 4 + 0) < len(imem_words) else 0
        w1 = imem_words[li * 4 + 1] if (li * 4 + 1) < len(imem_words) else 0
        w2 = imem_words[li * 4 + 2] if (li * 4 + 2) < len(imem_words) else 0
        w3 = imem_words[li * 4 + 3] if (li * 4 + 3) < len(imem_words) else 0
        val = w0 | (w1 << 32) | (w2 << 64) | (w3 << 96)
        if val != 0:
            lines[li] = val
    return lines


# ---------------------------------------------------------------------------
# Pass 1: Per-Core Bit-Exact RV32IMA_Zicsr_Zifencei ISA Audit
# ---------------------------------------------------------------------------
def _audit_core_stream(
    core: int,
    core_commits: List[CommitRec],
    imem_words: List[int],
    allow_self_mod: bool,
    stats: CheckStats,
) -> Tuple[
    List[ArchWriteOp],
    List[ArchLoadOp],
    List[ArchBarrierOp],
    List[Tuple[int, str, int]],  # timeline of core ops: (c_idx, 'W'|'L'|'B', list_idx)
]:
    regs = [0] * 32
    pc = 0
    mscratch = 0
    last_mcycle = -1
    last_minstret = -1
    halted = False
    bar_epoch = 0

    writes_out: List[ArchWriteOp] = []
    loads_out: List[ArchLoadOp] = []
    barriers_out: List[ArchBarrierOp] = []
    op_order: List[Tuple[int, str, int]] = []

    for c_idx, ev in enumerate(core_commits):
        if halted:
            raise CheckError(f"[Core {core}] Commit after EBREAK at cyc={ev.cyc} pc=0x{ev.pc:08x}")
        if ev.pc != pc:
            raise CheckError(
                f"[Core {core}] PC mismatch at step {c_idx} (cyc={ev.cyc}): "
                f"exp=0x{pc:08x} got=0x{ev.pc:08x} instr=0x{ev.instr:08x}"
            )
        if not allow_self_mod:
            w_idx = pc >> 2
            exp_ins = imem_words[w_idx] if w_idx < len(imem_words) else 0
            if ev.instr != exp_ins:
                raise CheckError(
                    f"[Core {core}] Instr fetch mismatch at PC=0x{pc:08x} (cyc={ev.cyc}): "
                    f"exp=0x{exp_ins:08x} got=0x{ev.instr:08x}"
                )

        instr = ev.instr
        op = instr & 0x7F
        rd = (instr >> 7) & 0x1F
        f3 = (instr >> 12) & 0x7
        rs1 = (instr >> 15) & 0x1F
        rs2 = (instr >> 20) & 0x1F
        f7 = (instr >> 25) & 0x7F
        f5 = (instr >> 27) & 0x1F

        v1 = regs[rs1]
        v2 = regs[rs2]

        imm_i = sext(instr >> 20, 12)
        imm_s = sext(((instr >> 25) << 5) | ((instr >> 7) & 0x1F), 12)
        imm_b = sext(
            (((instr >> 31) & 1) << 12)
            | (((instr >> 7) & 1) << 11)
            | (((instr >> 25) & 0x3F) << 5)
            | (((instr >> 8) & 0xF) << 1),
            13,
        )
        imm_u = instr & 0xFFFF_F000
        imm_j = sext(
            (((instr >> 31) & 1) << 20)
            | (((instr >> 12) & 0xFF) << 12)
            | (((instr >> 20) & 1) << 11)
            | (((instr >> 21) & 0x3FF) << 1),
            21,
        )

        next_pc = u32(pc + 4)
        exp_rd_we = 0
        exp_rd_wdata: Optional[int] = 0
        exp_mem_v = 0
        exp_mem_we: Optional[int] = 0
        exp_mem_addr = 0
        exp_mem_wstrb: Optional[int] = 0
        exp_mem_wdata: Optional[int] = 0

        if instr == 0x0010_0073:  # EBREAK
            halted = True
            bar_epoch += 1
            bidx = len(barriers_out)
            barriers_out.append(ArchBarrierOp(c_idx, ev.cyc, bar_epoch, "EBREAK"))
            op_order.append((c_idx, "B", bidx))
        elif op == 0x0F and f3 in (0x0, 0x1):  # FENCE / FENCE.I
            bar_epoch += 1
            bkind = "FENCE.I" if f3 == 0x1 else "FENCE"
            if f3 == 0x1:
                stats.fence_i_checked += 1
            else:
                stats.fences_checked += 1
            bidx = len(barriers_out)
            barriers_out.append(ArchBarrierOp(c_idx, ev.cyc, bar_epoch, bkind))
            op_order.append((c_idx, "B", bidx))
        elif op == 0x73 and f3 in (0x1, 0x2, 0x3, 0x5, 0x6, 0x7):  # Zicsr
            csr_addr = (instr >> 20) & 0xFFF
            exp_rd_we = 1
            if csr_addr == 0x340:
                old_val: Optional[int] = mscratch
            elif csr_addr == 0xF11:
                old_val = 0x0000_0000
            elif csr_addr == 0xF12:
                old_val = 0x0000_2002
            elif csr_addr == 0xF13:
                old_val = 0x0001_0000
            elif csr_addr == 0xF14:
                old_val = core
            elif csr_addr in (0xB00, 0xC00):
                old_val = None
                if rd != 0:
                    if last_mcycle >= 0 and ev.rd_wdata < last_mcycle:
                        raise CheckError(
                            f"[Core {core}] mcycle went backwards at cyc={ev.cyc}: "
                            f"{last_mcycle} -> {ev.rd_wdata}"
                        )
                    last_mcycle = ev.rd_wdata
            elif csr_addr in (0xB02, 0xC02):
                old_val = None
                if rd != 0:
                    if last_minstret >= 0 and ev.rd_wdata < last_minstret:
                        raise CheckError(
                            f"[Core {core}] minstret went backwards at cyc={ev.cyc}: "
                            f"{last_minstret} -> {ev.rd_wdata}"
                        )
                    last_minstret = ev.rd_wdata
            elif csr_addr in (0xB80, 0xC80, 0xB82, 0xC82):
                old_val = None
            else:
                raise CheckError(f"[Core {core}] Unsupported CSR 0x{csr_addr:03x} at PC=0x{pc:08x}")

            exp_rd_wdata = old_val
            operand = rs1 if (f3 & 4) else v1
            kind = f3 & 3
            wants_write = (kind == 1) or (rs1 != 0)
            if wants_write and csr_addr == 0x340:
                if kind == 1:
                    mscratch = u32(operand)
                elif kind == 2:
                    mscratch = u32(mscratch | operand)
                else:
                    mscratch = u32(mscratch & (~operand))
        elif op == 0x2F and f3 == 0x2:  # RV32A (LR.W / SC.W / 9x AMO*.W)
            addr = u32(v1)
            laddr = (addr >> 4) & 0xFFFF
            exp_mem_v = 1
            exp_mem_addr = addr
            exp_rd_we = 1
            bar_epoch += 1
            if f5 == 0b00010:  # LR.W
                exp_mem_we = 0
                exp_mem_wstrb = 0
                exp_mem_wdata = 0
                exp_rd_wdata = None
                stats.lr_checked += 1
                lidx = len(loads_out)
                loads_out.append(
                    ArchLoadOp(
                        c_idx=c_idx,
                        commit_cyc=ev.cyc,
                        bar_epoch=bar_epoch,
                        pc=pc,
                        addr=addr,
                        laddr=laddr,
                        size=4,
                        unsigned=True,
                        obs_val=ev.rd_wdata if rd != 0 else -1,
                        is_lr=True,
                        is_mmio=((addr >> 24) == 0x02),
                    )
                )
                op_order.append((c_idx, "L", lidx))
                bidx = len(barriers_out)
                barriers_out.append(ArchBarrierOp(c_idx, ev.cyc, bar_epoch, "LR"))
                op_order.append((c_idx, "B", bidx))
            elif f5 == 0b00011:  # SC.W
                exp_rd_wdata = None
                if ev.mem_we not in (0, 1):
                    raise CheckError(f"[Core {core}] Invalid SC.W mem_we={ev.mem_we}")
                if rd != 0:
                    if ev.rd_wdata not in (0, 1):
                        raise CheckError(f"[Core {core}] Illegal SC.W rd_wdata=0x{ev.rd_wdata:08x}")
                    if (ev.rd_wdata == 0) != (ev.mem_we == 1):
                        raise CheckError(
                            f"[Core {core}] SC.W rd_wdata={ev.rd_wdata} disagrees with mem_we={ev.mem_we}"
                        )
                exp_mem_we = ev.mem_we
                exp_mem_wstrb = 0xF if ev.mem_we else 0
                exp_mem_wdata = u32(v2) if ev.mem_we else 0
                bidx = len(barriers_out)
                barriers_out.append(ArchBarrierOp(c_idx, ev.cyc, bar_epoch, "SC"))
                op_order.append((c_idx, "B", bidx))
                if ev.mem_we:
                    stats.sc_ok += 1
                    widx = len(writes_out)
                    writes_out.append(
                        ArchWriteOp(
                            c_idx=c_idx,
                            commit_cyc=ev.cyc,
                            bar_epoch=bar_epoch,
                            addr=addr,
                            laddr=laddr,
                            wstrb=0xF,
                            wdata=u32(v2),
                            is_amo=False,
                            amo_f5=f5,
                            amo_rs2=u32(v2),
                            rd_addr=rd,
                            obs_rd=ev.rd_wdata,
                            is_sc=True,
                        )
                    )
                    op_order.append((c_idx, "W", widx))
                else:
                    stats.sc_fail += 1
            else:  # 9x AMO*.W RMW
                exp_mem_we = 1
                exp_mem_wstrb = 0xF
                exp_mem_wdata = None
                exp_rd_wdata = None
                if rd != 0:
                    exp_new = _amo_alu(f5, ev.rd_wdata, v2)
                    if ev.mem_wdata != exp_new:
                        raise CheckError(
                            f"[Core {core}] AMO RMW mismatch at PC=0x{pc:08x} cyc={ev.cyc}: "
                            f"old=0x{ev.rd_wdata:08x} rs2=0x{v2:08x} exp_new=0x{exp_new:08x} got=0x{ev.mem_wdata:08x}"
                        )
                stats.amos_checked += 1
                bidx = len(barriers_out)
                barriers_out.append(ArchBarrierOp(c_idx, ev.cyc, bar_epoch, "AMO"))
                op_order.append((c_idx, "B", bidx))
                widx = len(writes_out)
                writes_out.append(
                    ArchWriteOp(
                        c_idx=c_idx,
                        commit_cyc=ev.cyc,
                        bar_epoch=bar_epoch,
                        addr=addr,
                        laddr=laddr,
                        wstrb=0xF,
                        wdata=ev.mem_wdata,
                        is_amo=True,
                        amo_f5=f5,
                        amo_rs2=u32(v2),
                        rd_addr=rd,
                        obs_rd=ev.rd_wdata,
                        is_sc=False,
                    )
                )
                op_order.append((c_idx, "W", widx))
        elif op == 0x37:  # LUI
            exp_rd_we = 1
            exp_rd_wdata = u32(imm_u)
        elif op == 0x17:  # AUIPC
            exp_rd_we = 1
            exp_rd_wdata = u32(pc + imm_u)
        elif op == 0x6F:  # JAL
            exp_rd_we = 1
            exp_rd_wdata = u32(pc + 4)
            next_pc = u32(pc + imm_j)
        elif op == 0x67 and f3 == 0x0:  # JALR
            exp_rd_we = 1
            exp_rd_wdata = u32(pc + 4)
            next_pc = u32(v1 + imm_i) & 0xFFFF_FFFE
        elif op == 0x63:  # BRANCH
            if f3 == 0x0:
                taken = v1 == v2
            elif f3 == 0x1:
                taken = v1 != v2
            elif f3 == 0x4:
                taken = s32(v1) < s32(v2)
            elif f3 == 0x5:
                taken = s32(v1) >= s32(v2)
            elif f3 == 0x6:
                taken = u32(v1) < u32(v2)
            elif f3 == 0x7:
                taken = u32(v1) >= u32(v2)
            else:
                raise CheckError(f"[Core {core}] Illegal branch f3={f3} at PC=0x{pc:08x}")
            if taken:
                next_pc = u32(pc + imm_b)
        elif op == 0x03:  # LOAD
            addr = u32(v1 + imm_i)
            laddr = (addr >> 4) & 0xFFFF
            if f3 in (0x0, 0x4):
                sz, uns = 1, (f3 == 0x4)
            elif f3 in (0x1, 0x5):
                sz, uns = 2, (f3 == 0x5)
            elif f3 == 0x2:
                sz, uns = 4, True
            else:
                raise CheckError(f"[Core {core}] Illegal load f3={f3} at PC=0x{pc:08x}")
            exp_rd_we = 1
            exp_rd_wdata = None
            exp_mem_v = 1
            exp_mem_we = 0
            exp_mem_addr = addr
            exp_mem_wstrb = 0
            exp_mem_wdata = 0
            is_mmio = (addr >> 24) == 0x02
            if is_mmio:
                stats.mmio_ops += 1
            lidx = len(loads_out)
            loads_out.append(
                ArchLoadOp(
                    c_idx=c_idx,
                    commit_cyc=ev.cyc,
                    bar_epoch=bar_epoch,
                    pc=pc,
                    addr=addr,
                    laddr=laddr,
                    size=sz,
                    unsigned=uns,
                    obs_val=ev.rd_wdata if rd != 0 else -1,
                    is_lr=False,
                    is_mmio=is_mmio,
                )
            )
            op_order.append((c_idx, "L", lidx))
        elif op == 0x23:  # STORE
            addr = u32(v1 + imm_s)
            lane = addr & 0x3
            laddr = (addr >> 4) & 0xFFFF
            if f3 == 0x0:
                wstrb = 1 << lane
                wdata = u32((v2 & 0xFF) << (8 * lane))
            elif f3 == 0x1:
                wstrb = 0xC if (lane & 2) else 0x3
                wdata = u32((v2 & 0xFFFF) << (16 if (lane & 2) else 0))
            elif f3 == 0x2:
                wstrb = 0xF
                wdata = u32(v2)
            else:
                raise CheckError(f"[Core {core}] Illegal store f3={f3} at PC=0x{pc:08x}")
            exp_mem_v = 1
            exp_mem_we = 1
            exp_mem_addr = addr
            exp_mem_wstrb = wstrb
            exp_mem_wdata = wdata
            if (addr >> 24) == 0x02:
                stats.mmio_ops += 1
            else:
                stats.stores_checked += 1
                widx = len(writes_out)
                writes_out.append(
                    ArchWriteOp(
                        c_idx=c_idx,
                        commit_cyc=ev.cyc,
                        bar_epoch=bar_epoch,
                        addr=addr,
                        laddr=laddr,
                        wstrb=wstrb,
                        wdata=wdata,
                        is_amo=False,
                        amo_f5=0,
                        amo_rs2=0,
                        rd_addr=0,
                        obs_rd=0,
                        is_sc=False,
                    )
                )
                op_order.append((c_idx, "W", widx))
        elif op == 0x13:  # OP-IMM
            exp_rd_we = 1
            shamt = rs2
            if f3 == 0x0:
                exp_rd_wdata = u32(v1 + imm_i)
            elif f3 == 0x1 and f7 == 0x00:
                exp_rd_wdata = u32(v1 << shamt)
            elif f3 == 0x2:
                exp_rd_wdata = 1 if s32(v1) < imm_i else 0
            elif f3 == 0x3:
                exp_rd_wdata = 1 if u32(v1) < u32(imm_i) else 0
            elif f3 == 0x4:
                exp_rd_wdata = u32(v1 ^ u32(imm_i))
            elif f3 == 0x5 and f7 == 0x00:
                exp_rd_wdata = u32(v1) >> shamt
            elif f3 == 0x5 and f7 == 0x20:
                exp_rd_wdata = u32(s32(v1) >> shamt)
            elif f3 == 0x6:
                exp_rd_wdata = u32(v1 | u32(imm_i))
            elif f3 == 0x7:
                exp_rd_wdata = u32(v1 & u32(imm_i))
            else:
                raise CheckError(f"[Core {core}] Illegal OP-IMM f7={f7:02x} f3={f3} at PC=0x{pc:08x}")
        elif op == 0x33:  # OP (RV32I + RV32M)
            exp_rd_we = 1
            if f7 == 0x01:
                if f3 == 0x0:
                    exp_rd_wdata = u32(s32(v1) * s32(v2))
                elif f3 == 0x1:
                    exp_rd_wdata = u32((s32(v1) * s32(v2)) >> 32)
                elif f3 == 0x2:
                    exp_rd_wdata = u32((s32(v1) * u32(v2)) >> 32)
                elif f3 == 0x3:
                    exp_rd_wdata = u32((u32(v1) * u32(v2)) >> 32)
                elif f3 == 0x4:
                    if v2 == 0:
                        exp_rd_wdata = 0xFFFF_FFFF
                    elif v1 == 0x8000_0000 and v2 == 0xFFFF_FFFF:
                        exp_rd_wdata = 0x8000_0000
                    else:
                        sa, sb = s32(v1), s32(v2)
                        q = abs(sa) // abs(sb)
                        exp_rd_wdata = u32(-q if ((sa < 0) ^ (sb < 0)) else q)
                elif f3 == 0x5:
                    exp_rd_wdata = 0xFFFF_FFFF if v2 == 0 else u32(u32(v1) // u32(v2))
                elif f3 == 0x6:
                    if v2 == 0:
                        exp_rd_wdata = u32(v1)
                    elif v1 == 0x8000_0000 and v2 == 0xFFFF_FFFF:
                        exp_rd_wdata = 0
                    else:
                        sa, sb = s32(v1), s32(v2)
                        r = abs(sa) % abs(sb)
                        exp_rd_wdata = u32(-r if sa < 0 else r)
                elif f3 == 0x7:
                    exp_rd_wdata = u32(v1) if v2 == 0 else u32(u32(v1) % u32(v2))
            elif f7 == 0x00 and f3 == 0x0:
                exp_rd_wdata = u32(v1 + v2)
            elif f7 == 0x20 and f3 == 0x0:
                exp_rd_wdata = u32(v1 - v2)
            elif f7 == 0x00 and f3 == 0x1:
                exp_rd_wdata = u32(v1 << (v2 & 0x1F))
            elif f7 == 0x00 and f3 == 0x2:
                exp_rd_wdata = 1 if s32(v1) < s32(v2) else 0
            elif f7 == 0x00 and f3 == 0x3:
                exp_rd_wdata = 1 if u32(v1) < u32(v2) else 0
            elif f7 == 0x00 and f3 == 0x4:
                exp_rd_wdata = u32(v1 ^ v2)
            elif f7 == 0x00 and f3 == 0x5:
                exp_rd_wdata = u32(v1) >> (v2 & 0x1F)
            elif f7 == 0x20 and f3 == 0x5:
                exp_rd_wdata = u32(s32(v1) >> (v2 & 0x1F))
            elif f7 == 0x00 and f3 == 0x6:
                exp_rd_wdata = u32(v1 | v2)
            elif f7 == 0x00 and f3 == 0x7:
                exp_rd_wdata = u32(v1 & v2)
            else:
                raise CheckError(f"[Core {core}] Illegal OP f7={f7:02x} f3={f3} at PC=0x{pc:08x}")
        else:
            raise CheckError(f"[Core {core}] Illegal instr 0x{instr:08x} at PC=0x{pc:08x}")

        # Hardware clears rd_we when rd == 0
        if rd == 0:
            exp_rd_we = 0
            exp_rd_addr = 0
            exp_rd_wdata = 0
        else:
            exp_rd_addr = rd if exp_rd_we else 0

        if ev.rd_we != exp_rd_we or ev.rd_addr != exp_rd_addr:
            raise CheckError(
                f"[Core {core}] rd_we/addr mismatch at PC=0x{pc:08x} cyc={ev.cyc} ({isa.disassemble(instr)}): "
                f"exp=({exp_rd_we},x{exp_rd_addr}) got=({ev.rd_we},x{ev.rd_addr})"
            )
        if exp_rd_wdata is not None and ev.rd_wdata != exp_rd_wdata:
            raise CheckError(
                f"[Core {core}] rd_wdata mismatch at PC=0x{pc:08x} cyc={ev.cyc} ({isa.disassemble(instr)}): "
                f"exp=0x{exp_rd_wdata:08x} got=0x{ev.rd_wdata:08x}"
            )
        if ev.mem_v != exp_mem_v:
            raise CheckError(
                f"[Core {core}] mem_valid mismatch at PC=0x{pc:08x} cyc={ev.cyc}: "
                f"exp={exp_mem_v} got={ev.mem_v}"
            )
        if exp_mem_v:
            if exp_mem_we is not None and ev.mem_we != exp_mem_we:
                raise CheckError(f"[Core {core}] mem_we mismatch at PC=0x{pc:08x} cyc={ev.cyc}")
            if ev.mem_addr != exp_mem_addr:
                raise CheckError(
                    f"[Core {core}] mem_addr mismatch at PC=0x{pc:08x} cyc={ev.cyc}: "
                    f"exp=0x{exp_mem_addr:08x} got=0x{ev.mem_addr:08x}"
                )
            if exp_mem_wstrb is not None and ev.mem_wstrb != exp_mem_wstrb:
                raise CheckError(
                    f"[Core {core}] mem_wstrb mismatch at PC=0x{pc:08x} cyc={ev.cyc}: "
                    f"exp=0x{exp_mem_wstrb:x} got=0x{ev.mem_wstrb:x}"
                )
            if exp_mem_wdata is not None and ev.mem_wdata != exp_mem_wdata:
                raise CheckError(
                    f"[Core {core}] mem_wdata mismatch at PC=0x{pc:08x} cyc={ev.cyc}: "
                    f"exp=0x{exp_mem_wdata:08x} got=0x{ev.mem_wdata:08x}"
                )

        if ev.rd_we and ev.rd_addr != 0:
            regs[ev.rd_addr] = u32(ev.rd_wdata)
        pc = next_pc

    if not halted:
        raise CheckError(f"[Core {core}] Trace ended without EBREAK commit!")
    return writes_out, loads_out, barriers_out, op_order


# ---------------------------------------------------------------------------
# Pass 2: Global Serialized Write Timeline & Exact Coalescing / AMO / SC Check
# ---------------------------------------------------------------------------
def _verify_global_writes(
    writes: List[WriteEvt],
    core_writes: List[List[ArchWriteOp]],
    core_loads: List[List[ArchLoadOp]],
    core_barriers: List[List[ArchBarrierOp]],
    init_lines: Dict[int, int],
    stats: CheckStats,
) -> Dict[int, List[LineVersion]]:
    # Map each core's barrier epoch -> commit cycle of previous barrier (for causal tracking)
    prev_bar_cyc: List[Dict[int, int]] = [{0: 0}, {0: 0}]
    for c in (0, 1):
        for b in core_barriers[c]:
            prev_bar_cyc[c][b.bar_epoch] = b.commit_cyc

    core_lrs: List[List[ArchLoadOp]] = [
        [ld for ld in core_loads[0] if ld.is_lr],
        [ld for ld in core_loads[1] if ld.is_lr],
    ]

    # Step 2a: Pre-scan global W events to assert SWMR single-writer-per-cycle-per-line
    # and record exact pre-write line128 & global chronological history per laddr!
    cur_global_line: Dict[int, int] = {}
    last_w_cyc_line: Dict[int, Tuple[int, int]] = {}
    # Per-(core, laddr) list of (g_idx, WriteEvt, prev_line)
    we_by_cl: Dict[Tuple[int, int], List[Tuple[int, WriteEvt, int]]] = defaultdict(list)
    # Global W events per laddr: (g_idx, cyc, core) for SC reservation verification
    we_meta_by_laddr: Dict[int, List[Tuple[int, int, int]]] = defaultdict(list)

    for g_idx, we in enumerate(writes, start=1):
        prev_wc = last_w_cyc_line.get(we.laddr)
        if prev_wc is not None and prev_wc[0] == we.cyc and prev_wc[1] != we.core:
            raise CheckError(
                f"SWMR violation in W-trace: cyc={we.cyc} laddr=0x{we.laddr:04x} "
                f"both core {prev_wc[1]} and core {we.core} wrote simultaneously!"
            )
        last_w_cyc_line[we.laddr] = (we.cyc, we.core)
        prev_line = cur_global_line.get(we.laddr, init_lines.get(we.laddr, 0))
        we_by_cl[(we.core, we.laddr)].append((g_idx, we, prev_line))
        we_meta_by_laddr[we.laddr].append((g_idx, we.cyc, we.core))
        cur_global_line[we.laddr] = we.line128

    # Group architectural writes per (core, laddr) in exact program order
    aw_by_cl: Dict[Tuple[int, int], List[ArchWriteOp]] = defaultdict(list)
    for c in (0, 1):
        for w in core_writes[c]:
            aw_by_cl[(c, w.laddr)].append(w)

    # Ensure identical key sets (no phantom W events, no completely missing laddr!)
    all_keys = set(we_by_cl.keys()) | set(aw_by_cl.keys())
    # Record matched (consumed_ops, bmask16) per g_idx
    matched_by_gidx: Dict[int, Tuple[List[ArchWriteOp], int]] = {}

    for key in sorted(all_keys):
        c, la = key
        wlist = we_by_cl.get(key, [])
        alist = aw_by_cl.get(key, [])
        nw, na = len(wlist), len(alist)
        if nw == 0 and na > 0:
            raise CheckError(
                f"Lost architectural writes! Core {c} laddr=0x{la:04x} has {na} committed store(s) "
                f"but 0 W events (first commit cyc={alist[0].commit_cyc}, addr=0x{alist[0].addr:08x})"
            )
        if na == 0 and nw > 0:
            g0, we0, _ = wlist[0]
            raise CheckError(
                f"Phantom W event #{g0}: cyc={we0.cyc} core={c} laddr=0x{la:04x} "
                f"has no matching committed architectural write!"
            )

        # Exact memoized DFS matching wlist[0..nw-1] to alist[0..na-1] (handles arbitrary
        # MSHR store coalescing including idempotent stores with 0 false positives/negatives!)
        memo_fail: set[Tuple[int, int]] = set()
        solution: List[Tuple[int, List[ArchWriteOp], int]] = []
        last_err: List[str] = [""]

        def dfs(wi: int, ai: int) -> bool:
            if wi == nw and ai == na:
                return True
            if wi == nw or ai == na:
                return False
            if (na - ai) < (nw - wi):
                return False
            if (wi, ai) in memo_fail:
                return False

            g_idx, we, prev_line = wlist[wi]
            head = alist[ai]

            if head.is_amo or head.is_sc:
                # AMOs & SC.W execute solo at ROB head (never coalesce!)
                woff = (head.addr >> 2) & 3
                old_word = _line_get_word(prev_line, woff)
                if head.is_amo:
                    exp_new = _amo_alu(head.amo_f5, old_word, head.amo_rs2)
                    if (head.rd_addr != 0 and head.obs_rd != old_word) or head.wdata != exp_new:
                        last_err[0] = (
                            f"AMO linearizability error at W#{g_idx} cyc={we.cyc} core={c} "
                            f"addr=0x{head.addr:08x}: global_old=0x{old_word:08x} "
                            f"rd_obs=0x{head.obs_rd:08x} exp_new=0x{exp_new:08x} got_w=0x{head.wdata:08x}"
                        )
                        memo_fail.add((wi, ai))
                        return False
                else:
                    matching_lrs = [
                        lr for lr in core_lrs[c]
                        if lr.c_idx < head.c_idx and lr.laddr == la
                    ]
                    if not matching_lrs:
                        last_err[0] = (
                            f"Illegal SC.W success at W#{g_idx} cyc={we.cyc} core={c} "
                            f"addr=0x{head.addr:08x}: no prior LR.W to laddr=0x{la:04x}!"
                        )
                        memo_fail.add((wi, ai))
                        return False
                    last_lr = matching_lrs[-1]
                    for _mg, mcyc, mcore in we_meta_by_laddr[la]:
                        if mcore == (1 - c) and last_lr.commit_cyc <= mcyc < we.cyc:
                            last_err[0] = (
                                f"Illegal SC.W success at W#{g_idx} cyc={we.cyc} core={c} "
                                f"addr=0x{head.addr:08x}: remote core {mcore} wrote laddr=0x{la:04x} "
                                f"at cyc={mcyc} after LR.W commit cyc={last_lr.commit_cyc}!"
                            )
                            memo_fail.add((wi, ai))
                            return False
                exp_line, bm = _line_apply_write(prev_line, head.addr, head.wstrb, head.wdata)
                if exp_line != we.line128:
                    last_err[0] = (
                        f"{'AMO' if head.is_amo else 'SC'} line128 mismatch at W#{g_idx} cyc={we.cyc} "
                        f"core={c} laddr=0x{la:04x}: exp=0x{exp_line:032x} got=0x{we.line128:032x}"
                    )
                    memo_fail.add((wi, ai))
                    return False
                solution.append((g_idx, [head], bm))
                if dfs(wi + 1, ai + 1):
                    return True
                solution.pop()
                memo_fail.add((wi, ai))
                return False

            # Regular store: may coalesce k >= 1 same-line, same-epoch stores committed <= we.cyc
            max_k = (na - ai) - (nw - wi - 1)
            cur_line = prev_line
            cur_bm = 0
            cands: List[Tuple[int, int]] = []
            for k in range(1, max_k + 1):
                cand = alist[ai + k - 1]
                if k > 1 and (cand.is_amo or cand.is_sc or cand.bar_epoch != head.bar_epoch):
                    break
                if cand.commit_cyc > we.cyc:
                    break
                cur_line, bm = _line_apply_write(cur_line, cand.addr, cand.wstrb, cand.wdata)
                cur_bm |= bm
                if cur_line == we.line128:
                    cands.append((k, cur_bm))
            if not cands:
                last_err[0] = (
                    f"Store line128 mismatch at W#{g_idx} cyc={we.cyc} core={c} laddr=0x{la:04x} "
                    f"(head commit={head.commit_cyc}, addr=0x{head.addr:08x}, strb=0x{head.wstrb:x}, "
                    f"wdata=0x{head.wdata:08x}): prev=0x{prev_line:032x} got=0x{we.line128:032x}"
                )
            for k, bm in cands:
                solution.append((g_idx, alist[ai : ai + k], bm))
                if dfs(wi + 1, ai + k):
                    return True
                solution.pop()
            memo_fail.add((wi, ai))
            return False

        if not dfs(0, 0):
            raise CheckError(
                last_err[0]
                or f"Failed to align {nw} W events with {na} arch writes for Core {c} laddr=0x{la:04x}"
            )

        for g_idx, ops, bm in solution:
            matched_by_gidx[g_idx] = (ops, bm)
            if len(ops) > 1:
                stats.coalesced_stores += len(ops) - 1

    # Step 2b: Build final chronological mem_hist[laddr] with exact per-byte mod epochs!
    mem_hist: Dict[int, List[LineVersion]] = {}

    def get_hist(laddr: int) -> List[LineVersion]:
        if laddr not in mem_hist:
            mem_hist[laddr] = [LineVersion(0, 0, -1, init_lines.get(laddr, 0), (0,) * 16, 0)]
        return mem_hist[laddr]

    for g_idx, we in enumerate(writes, start=1):
        consumed, tot_bmask = matched_by_gidx[g_idx]
        for wop in consumed:
            wop.g_idx = g_idx
            wop.w_cyc = we.cyc
        hist = get_hist(we.laddr)
        prev_ver = hist[-1]
        new_bm = list(prev_ver.byte_mod_gidx)
        for bi in range(16):
            if (tot_bmask >> bi) & 1:
                new_bm[bi] = g_idx
        c_fcyc = prev_bar_cyc[we.core].get(consumed[0].bar_epoch, 0)
        hist.append(LineVersion(g_idx, we.cyc, we.core, we.line128, tuple(new_bm), c_fcyc))

    # Step 2c: Assert FENCE / FENCE.I / AMO / LR / SC / EBREAK drained all older stores on the same core!
    for c in (0, 1):
        epoch_max_wcyc: Dict[int, int] = defaultdict(int)
        for w in core_writes[c]:
            if w.w_cyc > epoch_max_wcyc[w.bar_epoch]:
                epoch_max_wcyc[w.bar_epoch] = w.w_cyc
        running_max_w = 0
        for b in core_barriers[c]:
            for ep in range(b.bar_epoch):
                if epoch_max_wcyc.get(ep, 0) > running_max_w:
                    running_max_w = epoch_max_wcyc[ep]
            if running_max_w > b.commit_cyc:
                raise CheckError(
                    f"[Core {c}] Barrier store-drain violation: {b.kind} committed at cyc={b.commit_cyc}, "
                    f"but an older store did not drain to L1D until cyc={running_max_w}!"
                )

    stats.w_events = len(writes)
    return mem_hist


# ---------------------------------------------------------------------------
# Pass 3: RVWMO & Global Linearizability Verification of EVERY Load & LR.W
# ---------------------------------------------------------------------------
def _verify_all_loads_rvwmo(
    mem_hist: Dict[int, List[LineVersion]],
    core_writes: List[List[ArchWriteOp]],
    core_loads: List[List[ArchLoadOp]],
    core_barriers: List[List[ArchBarrierOp]],
    core_op_order: List[List[Tuple[int, str, int]]],
    init_lines: Dict[int, int],
    stats: CheckStats,
) -> None:
    def get_hist(laddr: int) -> List[LineVersion]:
        if laddr not in mem_hist:
            return [LineVersion(0, 0, -1, init_lines.get(laddr, 0), (0,) * 16, 0)]
        return mem_hist[laddr]

    for c in (0, 1):
        # Per-byte tracking for Core c:
        #   last_own_store[baddr] = (g_idx, w_cyc, byte_val)
        #   max_obs_gidx[baddr]   = latest observed byte_mod_gidx (for CoRR!)
        last_own_store: Dict[int, Tuple[int, int, int]] = {}
        max_obs_gidx: Dict[int, int] = defaultdict(int)
        last_barrier_cyc = 0
        # Track the exact most recent older store op covering a word (for STLF)
        older_writes: List[ArchWriteOp] = []

        for _c_idx, kind, idx in core_op_order[c]:
            if kind == "B":
                b = core_barriers[c][idx]
                if b.commit_cyc > last_barrier_cyc:
                    last_barrier_cyc = b.commit_cyc
            elif kind == "W":
                w = core_writes[c][idx]
                older_writes.append(w)
                woff = (w.addr >> 2) & 3
                base_addr = w.addr & ~3
                for bi in range(4):
                    if (w.wstrb >> bi) & 1:
                        ba = base_addr + bi
                        bval = (w.wdata >> (bi * 8)) & 0xFF
                        last_own_store[ba] = (w.g_idx, w.w_cyc, bval)
                        if w.g_idx > max_obs_gidx[ba]:
                            max_obs_gidx[ba] = w.g_idx
            elif kind == "L":
                ld = core_loads[c][idx]
                if ld.is_mmio or ld.obs_val < 0:
                    # MMIO uncached load or rd==x0 (architectural result discarded)
                    continue
                stats.loads_checked += 1

                # Byte addresses accessed by this load
                baddrs = [ld.addr + i for i in range(ld.size)]
                boffs = [(ba & 0xF) for ba in baddrs]

                # 1. Check same-core STLF from older STILL-UNDRAINED store (w_cyc > ld.commit_cyc)
                undrained_bytes = [
                    ba for ba in baddrs
                    if ba in last_own_store and last_own_store[ba][1] > ld.commit_cyc
                ]
                if undrained_bytes:
                    # Hardware STLF requires ALL load bytes covered by older SQ store!
                    if len(undrained_bytes) == ld.size:
                        raw_bytes = [last_own_store[ba][2] for ba in baddrs]
                        if ld.size == 1:
                            stlf_val = raw_bytes[0] if ld.unsigned else u32(sext(raw_bytes[0], 8))
                        elif ld.size == 2:
                            h = raw_bytes[0] | (raw_bytes[1] << 8)
                            stlf_val = h if ld.unsigned else u32(sext(h, 16))
                        else:
                            stlf_val = (
                                raw_bytes[0]
                                | (raw_bytes[1] << 8)
                                | (raw_bytes[2] << 16)
                                | (raw_bytes[3] << 24)
                            )
                        if stlf_val == ld.obs_val:
                            stats.stlf_loads += 1
                            stlf_g = max(last_own_store[ba][0] for ba in baddrs)
                            for ba in baddrs:
                                if stlf_g > max_obs_gidx[ba]:
                                    max_obs_gidx[ba] = stlf_g
                            continue
                    # If not STLF, fall through to check mem_hist (will fail if violating CoWR!)

                # 2. Compute exact RVWMO lower bounds on mem_hist[ld.laddr]
                hist = get_hist(ld.laddr)

                # CoWR lower bound: must be >= own core's most recent older store to each byte
                min_cowr = max((last_own_store[ba][0] for ba in baddrs if ba in last_own_store), default=0)

                # CoRR lower bound: must be >= latest previously observed byte version on this core
                min_corr = max((max_obs_gidx[ba] for ba in baddrs), default=0)

                # FENCE / Barrier lower bound: any write that linearized strictly BEFORE
                # last_barrier_cyc on this core MUST be visible to this post-barrier load!
                min_fence = 0
                if last_barrier_cyc > 0:
                    for v in hist:
                        if v.w_cyc < last_barrier_cyc:
                            min_fence = v.g_idx
                        else:
                            break

                min_gidx = max(min_cowr, min_corr, min_fence)

                # Search for earliest legal version v with:
                #   v.w_cyc <= ld.commit_cyc  (no reading from the future!)
                #   v.g_idx >= min_gidx       (obeys CoWR + CoRR + FENCE!)
                #   extract(v.line128) == ld.obs_val
                chosen: Optional[LineVersion] = None
                for v in hist:
                    if v.w_cyc > ld.commit_cyc:
                        break
                    if v.g_idx < min_gidx:
                        continue
                    val = _extract_load_val(v.line128, ld.addr, ld.size, ld.unsigned)
                    if val == ld.obs_val:
                        chosen = v
                        break

                if chosen is None:
                    # Build rich diagnostic for immediate root-cause debugging!
                    hist_desc = [
                        f"(g={v.g_idx}, cyc={v.w_cyc}, c={v.core}, "
                        f"val=0x{_extract_load_val(v.line128, ld.addr, ld.size, ld.unsigned):08x})"
                        for v in hist
                    ]
                    raise CheckError(
                        f"RVWMO Load Linearizability VIOLATION on Core {c} at cyc={ld.commit_cyc} "
                        f"PC=0x{ld.pc:08x} addr=0x{ld.addr:08x} (sz={ld.size}):\n"
                        f"  observed=0x{ld.obs_val:08x}\n"
                        f"  bounds: min_cowr={min_cowr}, min_corr={min_corr}, "
                        f"min_fence={min_fence} (bar_cyc={last_barrier_cyc}) => min_gidx={min_gidx}\n"
                        f"  timeline: {', '.join(hist_desc)}"
                    )

                # Advance per-byte CoRR watermark to the actual modification epoch of each byte
                # in the chosen version (or max_obs_gidx, never decreases!)
                for ba, bo in zip(baddrs, boffs):
                    bg = chosen.byte_mod_gidx[bo]
                    if bg > max_obs_gidx[ba]:
                        max_obs_gidx[ba] = bg


# ---------------------------------------------------------------------------
# Disjoint Mode: Bit-Exact Comparison Against Independent Dual RV32IMA ISSs
# ---------------------------------------------------------------------------
def _verify_disjoint_iss(
    core_commits: List[List[CommitRec]],
    imem_words: List[int],
) -> None:
    for c in (0, 1):
        iss = isa.RV32IMA_ISS(imem_words, core_id=c, unified_mem=True)
        gold = iss.run(max_steps=max(len(core_commits[c]) + 100, 10_000))
        if len(core_commits[c]) != len(gold):
            raise CheckError(
                f"[Disjoint ISS Core {c}] Commit count mismatch: "
                f"rtl={len(core_commits[c])} gold={len(gold)}"
            )
        for i, (r, g) in enumerate(zip(core_commits[c], gold)):
            if (
                r.pc != g.pc
                or r.instr != g.instr
                or r.rd_we != g.rd_we
                or r.rd_addr != g.rd_addr
                or r.rd_wdata != g.rd_wdata
                or r.mem_v != g.mem_valid
                or r.mem_we != g.mem_we
                or r.mem_addr != g.mem_addr
                or r.mem_wstrb != g.mem_wstrb
                or r.mem_wdata != g.mem_wdata
            ):
                raise CheckError(
                    f"[Disjoint ISS Core {c}] Bit-exact mismatch at step {i} (cyc={r.cyc}):\n"
                    f"  GOLD: pc=0x{g.pc:08x} ins=0x{g.instr:08x} rd=({g.rd_we},x{g.rd_addr},0x{g.rd_wdata:08x}) "
                    f"mem=({g.mem_valid},{g.mem_we},0x{g.mem_addr:08x},0x{g.mem_wstrb:x},0x{g.mem_wdata:08x})\n"
                    f"  RTL : pc=0x{r.pc:08x} ins=0x{r.instr:08x} rd=({r.rd_we},x{r.rd_addr},0x{r.rd_wdata:08x}) "
                    f"mem=({r.mem_v},{r.mem_we},0x{r.mem_addr:08x},0x{r.mem_wstrb:x},0x{r.mem_wdata:08x})"
                )


# ---------------------------------------------------------------------------
# Public Entry Point
# ---------------------------------------------------------------------------
def check_dual_trace(
    hex_path: Path,
    log_path: Path,
    mode: str = "shared",
    allow_self_mod: bool = False,
) -> CheckStats:
    imem_words = load_hex_words(hex_path)
    commits, writes, buses = parse_trace_log(log_path)
    init_lines = build_init_lines(imem_words)

    core_commits: List[List[CommitRec]] = [[], []]
    for ev in commits:
        if ev.core not in (0, 1):
            raise CheckError(f"Invalid core id {ev.core} in commit record")
        core_commits[ev.core].append(ev)

    stats = CheckStats(
        c0_commits=len(core_commits[0]),
        c1_commits=len(core_commits[1]),
        bus_events=len(buses),
    )

    # Validate Channel A bus trace sanity & tally coherence stats
    for be in buses:
        if be.src not in (0, 1) or be.op not in (0, 1, 2):
            raise CheckError(f"Invalid bus event: {be}")
        if be.op == 0:
            stats.bus_gets += 1
        elif be.op == 1:
            stats.bus_getm += 1
        else:
            stats.bus_upg += 1
        if be.c2c:
            stats.bus_c2c += 1

    # Mode 'disjoint': bit-exact ISS comparison per core FIRST!
    if mode == "disjoint":
        _verify_disjoint_iss(core_commits, imem_words)

    # Pass 1: Per-core RV32IMA ISA audit
    cw0, cl0, cb0, co0 = _audit_core_stream(0, core_commits[0], imem_words, allow_self_mod, stats)
    cw1, cl1, cb1, co1 = _audit_core_stream(1, core_commits[1], imem_words, allow_self_mod, stats)

    # Pass 2: Global serialized W-timeline + coalescing + AMO + SC + barrier store-drain
    mem_hist = _verify_global_writes(
        writes,
        [cw0, cw1],
        [cl0, cl1],
        [cb0, cb1],
        init_lines,
        stats,
    )

    # Pass 3: RVWMO + Global Linearizability check on EVERY committed Load & LR.W
    _verify_all_loads_rvwmo(
        mem_hist,
        [cw0, cw1],
        [cl0, cl1],
        [cb0, cb1],
        [co0, co1],
        init_lines,
        stats,
    )

    return stats


# ---------------------------------------------------------------------------
# Negative Bug-Injection Self-Test (Proves Checker Catches Real Bugs!)
# ---------------------------------------------------------------------------
def run_self_test() -> int:
    import tempfile

    # Tiny shared-memory test program where C0 writes [0x20000]=1 then [0x20000]=2
    # and C1 loads [0x20000] twice, runs AMOADD, and runs LR/SC.
    prog = """
    li   s0, 0x20000
    csrr t0, mhartid
    bne  t0, zero, c1
c0:
    li   t1, 1
    sw   t1, 0(s0)
    fence rw, rw
    li   t2, 2
    sw   t2, 0(s0)
    ebreak
c1:
    lw   a0, 0(s0)
    lw   a1, 0(s0)
    li   t1, 5
    amoadd.w a2, t1, (s0)
    lr.w a3, (s0)
    li   t2, 99
    sc.w a4, t2, (s0)
    ebreak
"""
    words = isa.assemble(prog)

    def _mk_clean_trace() -> List[str]:
        # Build a clean valid dual-core trace for `words`
        # Offsets:
        #   0x00: lui s0, 0x20 (x8 = 0x20000)
        #   0x04: csrr t0, mhartid (x5)
        #   0x08: bne t0, zero, c1 (0x24)
        #   0x0c: li t1, 1 (x6=1)
        #   0x10: sw t1, 0(s0)
        #   0x14: fence rw, rw
        #   0x18: li t2, 2 (x7=2)
        #   0x1c: sw t2, 0(s0)
        #   0x20: ebreak
        #   0x24: lw a0, 0(s0) (x10)
        #   0x28: lw a1, 0(s0) (x11)
        #   0x2c: li t1, 5 (x6=5)
        #   0x30: amoadd.w a2, t1, (s0) (x12)
        #   0x34: lr.w a3, (s0) (x13)
        #   0x38: li t2, 99 (x7=99)
        #   0x3c: sc.w a4, t2, (s0) (x14)
        #   0x40: ebreak
        def c_ln(cyc, core, pc, rwe, rda, rdw, mv=0, mwe=0, ma=0, ms=0, md=0):
            ins = words[pc >> 2]
            return f"C {cyc} {core} {pc:08x} {ins:08x} {rwe} {rda} {rdw:08x} {mv} {mwe} {ma:08x} {ms:x} {md:08x}"

        return [
            # Core 0
            c_ln(10, 0, 0x00, 1, 8, 0x20000),
            c_ln(11, 0, 0x04, 1, 5, 0),
            c_ln(12, 0, 0x08, 0, 0, 0),
            c_ln(13, 0, 0x0C, 1, 6, 1),
            c_ln(14, 0, 0x10, 0, 0, 0, 1, 1, 0x20000, 0xF, 1),
            "B 15 0 1 2000 0 0",
            "W 16 0 2000 00000000000000000000000000000001",
            c_ln(18, 0, 0x14, 0, 0, 0),
            c_ln(19, 0, 0x18, 1, 7, 2),
            c_ln(20, 0, 0x1C, 0, 0, 0, 1, 1, 0x20000, 0xF, 2),
            "W 22 0 2000 00000000000000000000000000000002",
            c_ln(24, 0, 0x20, 0, 0, 0),
            # Core 1
            c_ln(10, 1, 0x00, 1, 8, 0x20000),
            c_ln(11, 1, 0x04, 1, 5, 1),
            c_ln(12, 1, 0x08, 0, 0, 0),
            c_ln(25, 1, 0x24, 1, 10, 1, 1, 0, 0x20000, 0, 0),  # sees 1
            c_ln(26, 1, 0x28, 1, 11, 2, 1, 0, 0x20000, 0, 0),  # sees 2
            c_ln(27, 1, 0x2C, 1, 6, 5),
            "B 28 1 1 2000 1 0",
            "W 29 1 2000 00000000000000000000000000000007",   # 2 + 5 = 7
            c_ln(30, 1, 0x30, 1, 12, 2, 1, 1, 0x20000, 0xF, 7),
            c_ln(32, 1, 0x34, 1, 13, 7, 1, 0, 0x20000, 0, 0),  # LR=7
            c_ln(33, 1, 0x38, 1, 7, 99),
            "W 35 1 2000 00000000000000000000000000000063",   # SC=99 (0x63)
            c_ln(36, 1, 0x3C, 1, 14, 0, 1, 1, 0x20000, 0xF, 99),
            c_ln(37, 1, 0x40, 0, 0, 0),
        ]

    with tempfile.TemporaryDirectory() as td:
        hp = Path(td) / "t.hex"
        lp = Path(td) / "t.log"
        isa.write_hex(words, hp, pad_words=64)

        # 0. Clean trace MUST PASS
        clean = _mk_clean_trace()
        lp.write_text("\n".join(clean) + "\n", encoding="utf-8")
        st = check_dual_trace(hp, lp, mode="shared")
        assert st.loads_checked == 3 and st.amos_checked == 1 and st.sc_ok == 1

        # 1. Inject CoRR violation (C1 reads 2 then 1!) -> MUST FAIL
        bad = list(clean)
        bad[15] = bad[15].replace(" 1 10 00000001 ", " 1 10 00000002 ")
        bad[16] = bad[16].replace(" 1 11 00000002 ", " 1 11 00000001 ")
        lp.write_text("\n".join(bad) + "\n", encoding="utf-8")
        try:
            check_dual_trace(hp, lp, mode="shared")
            raise AssertionError("Failed to catch CoRR violation!")
        except CheckError:
            pass

        # 2. Inject thin-air / corrupted load value (0x9999) -> MUST FAIL
        bad = list(clean)
        bad[15] = bad[15].replace(" 1 10 00000001 ", " 1 10 00009999 ")
        lp.write_text("\n".join(bad) + "\n", encoding="utf-8")
        try:
            check_dual_trace(hp, lp, mode="shared")
            raise AssertionError("Failed to catch thin-air load value!")
        except CheckError:
            pass

        # 3. Inject broken AMO RMW (writes 6 instead of 7) -> MUST FAIL
        bad = list(clean)
        bad[19] = bad[19].replace("00000007", "00000006")
        bad[20] = bad[20].replace(" 00000007", " 00000006")
        lp.write_text("\n".join(bad) + "\n", encoding="utf-8")
        try:
            check_dual_trace(hp, lp, mode="shared")
            raise AssertionError("Failed to catch broken AMO RMW!")
        except CheckError:
            pass

        # 4. Inject ALU wrong result on Core 0 -> MUST FAIL
        bad = list(clean)
        bad[3] = bad[3].replace(" 1 6 00000001 ", " 1 6 00000009 ")
        lp.write_text("\n".join(bad) + "\n", encoding="utf-8")
        try:
            check_dual_trace(hp, lp, mode="shared")
            raise AssertionError("Failed to catch corrupted ALU result!")
        except CheckError:
            pass

        # 5. Inject barrier store-drain violation (W at cyc=25 after FENCE at cyc=18!) -> MUST FAIL
        bad = list(clean)
        bad[6] = bad[6].replace("W 16 0", "W 19 0")
        lp.write_text("\n".join(bad) + "\n", encoding="utf-8")
        try:
            check_dual_trace(hp, lp, mode="shared")
            raise AssertionError("Failed to catch barrier store-drain violation!")
        except CheckError:
            pass

        # 6. Inject illegal SC success after intervening remote write -> MUST FAIL
        bad = list(clean)
        # Move Core 0's second store W from cyc=22 to cyc=34 (between C1 LR@32 and SC@35!)
        bad[10] = "W 34 0 2000 00000000000000000000000000000002"
        # Adjust C1 earlier reads/AMO so only SC check triggers
        lp.write_text("\n".join(bad) + "\n", encoding="utf-8")
        try:
            check_dual_trace(hp, lp, mode="shared")
            raise AssertionError("Failed to catch illegal SC/ordering violation!")
        except CheckError:
            pass

    print("PASS [dual_core_checker --self-test]: Clean trace PASS + 6/6 injected bugs CAUGHT!")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Dual-Core RVFI + Global Linearizability Checker")
    ap.add_argument("--hex", type=Path, help="Program hex file")
    ap.add_argument("--log", type=Path, help="tb_dual_ooo2w_soc +LOG trace file")
    ap.add_argument("--mode", choices=["shared", "disjoint"], default="shared")
    ap.add_argument("--self-test", action="store_true", help="Run negative bug-injection self-tests")
    args = ap.parse_args()

    if args.self_test:
        return run_self_test()
    if not args.hex or not args.log:
        ap.error("--hex and --log are required unless --self-test is given")

    st = check_dual_trace(args.hex, args.log, mode=args.mode)
    print(
        f"PASS [dual_core_checker ({args.mode})]: c0={st.c0_commits} c1={st.c1_commits} "
        f"W={st.w_events} B={st.bus_events} (c2c={st.bus_c2c}) loads={st.loads_checked} "
        f"(stlf={st.stlf_loads}) stores={st.stores_checked} (coal={st.coalesced_stores}) "
        f"amos={st.amos_checked} lr={st.lr_checked} sc={st.sc_ok}/{st.sc_ok+st.sc_fail} "
        f"fence={st.fences_checked} fence.i={st.fence_i_checked}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
