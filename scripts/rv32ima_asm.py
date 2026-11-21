#!/usr/bin/env python3
"""
RV32IMA_Zicsr_Zifencei Assembler, Disassembler, and Bit-Accurate Golden ISS.

Supports:
  - Full RV32I (all 37 user instructions + EBREAK)
  - Full RV32M (MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU)
  - Full RV32A (LR.W, SC.W, AMOSWAP/ADD/XOR/AND/OR/MIN/MAX/MINU/MAXU.W + .aq/.rl)
  - Memory Ordering (FENCE, FENCE.TSO, FENCE.I)
  - Zicsr (CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI + csrr/csrw/csrs/csrc/rdcycle/rdinstret)
  - Common pseudo-instructions (NOP, MV, LI, J, JR, RET, NEG, NOT, SEQZ, SNEZ, CALL)

Usage:
  python3 scripts/rv32ima_asm.py --self-test
  python3 scripts/rv32ima_asm.py prog.S --hex out.hex --trace out.trace --core-id 0
"""

from __future__ import annotations

import argparse
import dataclasses
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

_SCRIPT_DIR = str(Path(__file__).resolve().parent)
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
import ieee754_oracle as fpo  # noqa: E402


# ---------------------------------------------------------------------------
# Bit Helpers & Register / CSR Maps
# ---------------------------------------------------------------------------
def u32(x: int) -> int:
    return x & 0xFFFF_FFFF


def s32(x: int) -> int:
    x &= 0xFFFF_FFFF
    return x - 0x1_0000_0000 if (x & 0x8000_0000) else x


def sext(val: int, bits: int) -> int:
    mask = (1 << bits) - 1
    val &= mask
    sign = 1 << (bits - 1)
    return val - (1 << bits) if (val & sign) else val


ABI_REGS: Dict[str, int] = {
    "zero": 0, "ra": 1, "sp": 2, "gp": 3, "tp": 4,
    "t0": 5, "t1": 6, "t2": 7,
    "s0": 8, "fp": 8, "s1": 9,
    "a0": 10, "a1": 11, "a2": 12, "a3": 13, "a4": 14, "a5": 15, "a6": 16, "a7": 17,
    "s2": 18, "s3": 19, "s4": 20, "s5": 21, "s6": 22, "s7": 23,
    "s8": 24, "s9": 25, "s10": 26, "s11": 27,
    "t3": 28, "t4": 29, "t5": 30, "t6": 31,
}
for _i in range(32):
    ABI_REGS[f"x{_i}"] = _i

FP_ABI_REGS: Dict[str, int] = {
    "ft0": 0, "ft1": 1, "ft2": 2, "ft3": 3, "ft4": 4, "ft5": 5, "ft6": 6, "ft7": 7,
    "fs0": 8, "fs1": 9,
    "fa0": 10, "fa1": 11, "fa2": 12, "fa3": 13, "fa4": 14, "fa5": 15, "fa6": 16, "fa7": 17,
    "fs2": 18, "fs3": 19, "fs4": 20, "fs5": 21, "fs6": 22, "fs7": 23,
    "fs8": 24, "fs9": 25, "fs10": 26, "fs11": 27,
    "ft8": 28, "ft9": 29, "ft10": 30, "ft11": 31,
}
for _i in range(32):
    FP_ABI_REGS[f"f{_i}"] = _i

FP_RM_MAP: Dict[str, int] = {
    "rne": 0, "rtz": 1, "rdn": 2, "rup": 3, "rmm": 4, "dyn": 7,
}
FP_RM_REV: Dict[int, str] = {v: k for k, v in FP_RM_MAP.items()}

CSR_NAMES: Dict[str, int] = {
    "fflags":    0x001,
    "frm":       0x002,
    "fcsr":      0x003,
    "mstatus":   0x300,
    "misa":      0x301,
    "mie":       0x304,
    "mtvec":     0x305,
    "mscratch":  0x340,
    "mepc":      0x341,
    "mcause":    0x342,
    "mtval":     0x343,
    "mip":       0x344,
    "mcycle":    0xB00,
    "minstret":  0xB02,
    "mcycleh":   0xB80,
    "minstreth": 0xB82,
    "cycle":     0xC00,
    "instret":   0xC02,
    "cycleh":    0xC80,
    "instreth":  0xC82,
    "mvendorid": 0xF11,
    "marchid":   0xF12,
    "mimpid":    0xF13,
    "mhartid":   0xF14,
}
CSR_REV: Dict[int, str] = {v: k for k, v in CSR_NAMES.items()}


def parse_reg(tok: str) -> int:
    key = tok.strip().lower()
    if key not in ABI_REGS:
        raise ValueError(f"Unknown register: '{tok}'")
    return ABI_REGS[key]


def parse_freg(tok: str) -> int:
    key = tok.strip().lower()
    if key not in FP_ABI_REGS:
        raise ValueError(f"Unknown FP register: '{tok}'")
    return FP_ABI_REGS[key]


def parse_rm(tok: str) -> int:
    key = tok.strip().lower()
    if key in FP_RM_MAP:
        return FP_RM_MAP[key]
    v = parse_int(tok)
    if not (0 <= v <= 7):
        raise ValueError(f"Rounding mode out of range: '{tok}'")
    return v


def parse_int(tok: str) -> int:
    return int(tok.strip(), 0)


def parse_csr(tok: str) -> int:
    key = tok.strip().lower()
    if key in CSR_NAMES:
        return CSR_NAMES[key]
    val = parse_int(tok)
    if not (0 <= val <= 0xFFF):
        raise ValueError(f"CSR address out of range: '{tok}'")
    return val


def parse_fence_mask(tok: str) -> int:
    m = 0
    for ch in tok.strip().lower():
        if ch == "i":
            m |= 8
        elif ch == "o":
            m |= 4
        elif ch == "r":
            m |= 2
        elif ch == "w":
            m |= 1
        else:
            raise ValueError(f"Invalid fence ordering spec: '{tok}'")
    return m


# ---------------------------------------------------------------------------
# Encoders (Official RISC-V RV32IMA_Zicsr_Zifencei Spec)
# ---------------------------------------------------------------------------
def enc_r(funct7: int, rs2: int, rs1: int, funct3: int, rd: int, opcode: int = 0x33) -> int:
    return (
        ((funct7 & 0x7F) << 25)
        | ((rs2 & 0x1F) << 20)
        | ((rs1 & 0x1F) << 15)
        | ((funct3 & 0x7) << 12)
        | ((rd & 0x1F) << 7)
        | (opcode & 0x7F)
    )


def enc_i(imm: int, rs1: int, funct3: int, rd: int, opcode: int) -> int:
    if not (-2048 <= imm <= 2047) and not (0 <= imm <= 4095):
        raise ValueError(f"I-type immediate out of range: {imm}")
    return (
        ((imm & 0xFFF) << 20)
        | ((rs1 & 0x1F) << 15)
        | ((funct3 & 0x7) << 12)
        | ((rd & 0x1F) << 7)
        | (opcode & 0x7F)
    )


def enc_s(imm: int, rs2: int, rs1: int, funct3: int, opcode: int = 0x23) -> int:
    if not (-2048 <= imm <= 2047):
        raise ValueError(f"S-type immediate out of range: {imm}")
    imm12 = imm & 0xFFF
    return (
        (((imm12 >> 5) & 0x7F) << 25)
        | ((rs2 & 0x1F) << 20)
        | ((rs1 & 0x1F) << 15)
        | ((funct3 & 0x7) << 12)
        | ((imm12 & 0x1F) << 7)
        | (opcode & 0x7F)
    )


def enc_b(offset: int, rs2: int, rs1: int, funct3: int, opcode: int = 0x63) -> int:
    if (offset & 1) != 0 or not (-4096 <= offset <= 4094):
        raise ValueError(f"B-type branch offset out of range or misaligned: {offset}")
    imm = offset & 0x1FFF
    b12 = (imm >> 12) & 1
    b11 = (imm >> 11) & 1
    b10_5 = (imm >> 5) & 0x3F
    b4_1 = (imm >> 1) & 0xF
    return (
        (b12 << 31)
        | (b10_5 << 25)
        | ((rs2 & 0x1F) << 20)
        | ((rs1 & 0x1F) << 15)
        | ((funct3 & 0x7) << 12)
        | (b4_1 << 8)
        | (b11 << 7)
        | (opcode & 0x7F)
    )


def enc_u(imm20: int, rd: int, opcode: int) -> int:
    if not (-0x80000 <= imm20 <= 0xFFFFF):
        raise ValueError(f"U-type 20-bit immediate out of range: {imm20}")
    return (((imm20 & 0xFFFFF) << 12) | ((rd & 0x1F) << 7) | (opcode & 0x7F))


def enc_j(offset: int, rd: int, opcode: int = 0x6F) -> int:
    if (offset & 1) != 0 or not (-(1 << 20) <= offset <= (1 << 20) - 2):
        raise ValueError(f"J-type jump offset out of range or misaligned: {offset}")
    imm = offset & 0x1F_FFFF
    b20 = (imm >> 20) & 1
    b10_1 = (imm >> 1) & 0x3FF
    b11 = (imm >> 11) & 1
    b19_12 = (imm >> 12) & 0xFF
    return (
        (b20 << 31)
        | (b10_1 << 21)
        | (b11 << 20)
        | (b19_12 << 12)
        | ((rd & 0x1F) << 7)
        | (opcode & 0x7F)
    )


def enc_amo(funct5: int, aq: int, rl: int, rs2: int, rs1: int, rd: int) -> int:
    f7 = ((funct5 & 0x1F) << 2) | ((aq & 1) << 1) | (rl & 1)
    return enc_r(f7, rs2, rs1, 0x2, rd, opcode=0x2F)


def enc_r4(rs3: int, fmt: int, rs2: int, rs1: int, rm: int, rd: int, opcode: int) -> int:
    return (
        ((rs3 & 0x1F) << 27)
        | ((fmt & 0x3) << 25)
        | ((rs2 & 0x1F) << 20)
        | ((rs1 & 0x1F) << 15)
        | ((rm & 0x7) << 12)
        | ((rd & 0x1F) << 7)
        | (opcode & 0x7F)
    )


FP_R4_OPS: Dict[str, int] = {
    "fmadd.s":  0x43,
    "fmsub.s":  0x47,
    "fnmsub.s": 0x4B,
    "fnmadd.s": 0x4F,
}
FP_BIN_RM_OPS: Dict[str, int] = {
    "fadd.s": 0x00,
    "fsub.s": 0x04,
    "fmul.s": 0x08,
    "fdiv.s": 0x0C,
}
FP_SGNJ_OPS: Dict[str, int] = {
    "fsgnj.s":  0x0,
    "fsgnjn.s": 0x1,
    "fsgnjx.s": 0x2,
}
FP_MINMAX_OPS: Dict[str, int] = {
    "fmin.s": 0x0,
    "fmax.s": 0x1,
}
FP_CMP_OPS: Dict[str, int] = {
    "fle.s": 0x0,
    "flt.s": 0x1,
    "feq.s": 0x2,
}


R_OPS: Dict[str, Tuple[int, int]] = {
    # RV32I
    "add":    (0x00, 0x0),
    "sub":    (0x20, 0x0),
    "sll":    (0x00, 0x1),
    "slt":    (0x00, 0x2),
    "sltu":   (0x00, 0x3),
    "xor":    (0x00, 0x4),
    "srl":    (0x00, 0x5),
    "sra":    (0x20, 0x5),
    "or":     (0x00, 0x6),
    "and":    (0x00, 0x7),
    # RV32M
    "mul":    (0x01, 0x0),
    "mulh":   (0x01, 0x1),
    "mulhsu": (0x01, 0x2),
    "mulhu":  (0x01, 0x3),
    "div":    (0x01, 0x4),
    "divu":   (0x01, 0x5),
    "rem":    (0x01, 0x6),
    "remu":   (0x01, 0x7),
}

I_ALU_OPS: Dict[str, int] = {
    "addi":  0x0,
    "slti":  0x2,
    "sltiu": 0x3,
    "xori":  0x4,
    "ori":   0x6,
    "andi":  0x7,
}

I_SHIFT_OPS: Dict[str, Tuple[int, int]] = {
    "slli": (0x00, 0x1),
    "srli": (0x00, 0x5),
    "srai": (0x20, 0x5),
}

LOAD_OPS: Dict[str, int] = {
    "lb":  0x0,
    "lh":  0x1,
    "lw":  0x2,
    "lbu": 0x4,
    "lhu": 0x5,
}

STORE_OPS: Dict[str, int] = {
    "sb": 0x0,
    "sh": 0x1,
    "sw": 0x2,
}

BRANCH_OPS: Dict[str, int] = {
    "beq":  0x0,
    "bne":  0x1,
    "blt":  0x4,
    "bge":  0x5,
    "bltu": 0x6,
    "bgeu": 0x7,
}

AMO_F5: Dict[str, int] = {
    "amoadd.w":  0b00000,
    "amoswap.w": 0b00001,
    "lr.w":      0b00010,
    "sc.w":      0b00011,
    "amoxor.w":  0b00100,
    "amoor.w":   0b01000,
    "amoand.w":  0b01100,
    "amomin.w":  0b10000,
    "amomax.w":  0b10100,
    "amominu.w": 0b11000,
    "amomaxu.w": 0b11100,
}
AMO_REV: Dict[int, str] = {v: k for k, v in AMO_F5.items()}

CSR_OPS: Dict[str, Tuple[int, bool]] = {
    "csrrw":  (0x1, False),
    "csrrs":  (0x2, False),
    "csrrc":  (0x3, False),
    "csrrwi": (0x5, True),
    "csrrsi": (0x6, True),
    "csrrci": (0x7, True),
}

MEM_ARG_RE = re.compile(r"^([^(]*)\(([^)]+)\)$")


def _split_operands(arg_str: str) -> List[str]:
    if not arg_str.strip():
        return []
    return [tok.strip() for tok in arg_str.split(",") if tok.strip()]


def _parse_amo_mnem(mnem: str) -> Optional[Tuple[str, int, int]]:
    """Parse RV32A mnemonic with optional .aq, .rl, or .aqrl suffix."""
    base = mnem
    aq = 0
    rl = 0
    if base.endswith(".aqrl"):
        aq = 1
        rl = 1
        base = base[:-5]
    elif base.endswith(".aq"):
        aq = 1
        base = base[:-3]
    elif base.endswith(".rl"):
        rl = 1
        base = base[:-3]
    if base in AMO_F5:
        return (base, aq, rl)
    return None


def _parse_mem_reg(tok: str) -> int:
    """Parse (rs1) or 0(rs1) or rs1 for RV32A address operand."""
    m = MEM_ARG_RE.match(tok.strip())
    if m:
        off_str = m.group(1).strip()
        if off_str and parse_int(off_str) != 0:
            raise ValueError(f"RV32A address offset must be 0, got '{tok}'")
        return parse_reg(m.group(2))
    return parse_reg(tok)


def _expand_pseudo(mnem: str, ops: List[str]) -> List[Tuple[str, List[str]]]:
    if mnem == "nop":
        return [("addi", ["x0", "x0", "0"])]
    if mnem == "mv":
        return [("addi", [ops[0], ops[1], "0"])]
    if mnem == "neg":
        return [("sub", [ops[0], "x0", ops[1]])]
    if mnem == "not":
        return [("xori", [ops[0], ops[1], "-1"])]
    if mnem == "seqz":
        return [("sltiu", [ops[0], ops[1], "1"])]
    if mnem == "snez":
        return [("sltu", [ops[0], "x0", ops[1]])]
    if mnem == "j":
        return [("jal", ["x0", ops[0]])]
    if mnem == "jr":
        return [("jalr", ["x0", ops[0], "0"])]
    if mnem == "ret":
        return [("jalr", ["x0", "x1", "0"])]
    if mnem == "jal" and len(ops) == 1:
        return [("jal", ["x1", ops[0]])]
    if mnem == "csrr":
        return [("csrrs", [ops[0], ops[1], "x0"])]
    if mnem == "csrw":
        return [("csrrw", ["x0", ops[0], ops[1]])]
    if mnem == "csrs":
        return [("csrrs", ["x0", ops[0], ops[1]])]
    if mnem == "csrc":
        return [("csrrc", ["x0", ops[0], ops[1]])]
    if mnem == "csrwi":
        return [("csrrwi", ["x0", ops[0], ops[1]])]
    if mnem == "csrsi":
        return [("csrrsi", ["x0", ops[0], ops[1]])]
    if mnem == "csrci":
        return [("csrrci", ["x0", ops[0], ops[1]])]
    if mnem == "rdcycle":
        return [("csrrs", [ops[0], "cycle", "x0"])]
    if mnem == "rdinstret":
        return [("csrrs", [ops[0], "instret", "x0"])]
    if mnem == "fmv.s":
        return [("fsgnj.s", [ops[0], ops[1], ops[1]])]
    if mnem == "fabs.s":
        return [("fsgnjx.s", [ops[0], ops[1], ops[1]])]
    if mnem == "fneg.s":
        return [("fsgnjn.s", [ops[0], ops[1], ops[1]])]
    if mnem == "frcsr":
        return [("csrrs", [ops[0], "fcsr", "x0"])]
    if mnem == "fscsr":
        return [("csrrw", [ops[0], "fcsr", ops[1]])] if len(ops) == 2 else [("csrrw", ["x0", "fcsr", ops[0]])]
    if mnem == "frrm":
        return [("csrrs", [ops[0], "frm", "x0"])]
    if mnem == "fsrm":
        return [("csrrw", [ops[0], "frm", ops[1]])] if len(ops) == 2 else [("csrrw", ["x0", "frm", ops[0]])]
    if mnem == "fsrmi":
        return [("csrrwi", [ops[0], "frm", ops[1]])] if len(ops) == 2 else [("csrrwi", ["x0", "frm", ops[0]])]
    if mnem == "frflags":
        return [("csrrs", [ops[0], "fflags", "x0"])]
    if mnem == "fsflags":
        return [("csrrw", [ops[0], "fflags", ops[1]])] if len(ops) == 2 else [("csrrw", ["x0", "fflags", ops[0]])]
    if mnem == "fsflagsi":
        return [("csrrwi", [ops[0], "fflags", ops[1]])] if len(ops) == 2 else [("csrrwi", ["x0", "fflags", ops[0]])]
    if mnem == "la":
        return [("la_hi", [ops[0], ops[1]]), ("la_lo", [ops[0], ops[1]])]
    if mnem == "li":
        rd = ops[0]
        val = s32(parse_int(ops[1]))
        if -2048 <= val <= 2047:
            return [("addi", [rd, "x0", str(val)])]
        lo12 = sext(val & 0xFFF, 12)
        hi20 = ((val - lo12) >> 12) & 0xFFFFF
        if lo12 == 0:
            return [("lui", [rd, str(hi20)])]
        return [("lui", [rd, str(hi20)]), ("addi", [rd, rd, str(lo12)])]
    return [(mnem, ops)]


def assemble(source: str) -> List[int]:
    """Two-pass RV32IMA_Zicsr_Zifencei assembler with label resolution."""
    labels: Dict[str, int] = {}
    parsed: List[Tuple[int, str, List[str]]] = []
    pc = 0

    for lineno, raw in enumerate(source.splitlines(), start=1):
        line = re.split(r"//|#|;", raw, maxsplit=1)[0].strip()
        if not line:
            continue

        while ":" in line:
            lbl, rest = line.split(":", 1)
            lbl = lbl.strip()
            if not lbl:
                raise ValueError(f"Line {lineno}: empty label")
            if lbl in labels:
                raise ValueError(f"Line {lineno}: duplicate label '{lbl}'")
            labels[lbl] = pc
            line = rest.strip()
        if not line:
            continue

        parts = line.split(None, 1)
        mnem = parts[0].lower()
        ops = _split_operands(parts[1]) if len(parts) > 1 else []
        for emnem, eops in _expand_pseudo(mnem, ops):
            parsed.append((pc, emnem, eops))
            pc += 4

    words: List[int] = []
    for cur_pc, mnem, ops in parsed:
        if mnem == ".word":
            words.append(u32(parse_int(ops[0])))
        elif mnem == "ecall":
            words.append(0x0000_0073)
        elif mnem == "ebreak":
            words.append(0x0010_0073)
        elif mnem == "mret":
            words.append(0x3020_0073)
        elif mnem == "la_hi":
            rd = parse_reg(ops[0])
            target = labels[ops[1]] if ops[1] in labels else parse_int(ops[1])
            delta = target - cur_pc
            lo12 = sext(delta & 0xFFF, 12)
            hi20 = ((delta - lo12) >> 12) & 0xFFFFF
            words.append(enc_u(hi20, rd, 0x17))  # auipc rd, hi20
        elif mnem == "la_lo":
            rd = parse_reg(ops[0])
            target = labels[ops[1]] if ops[1] in labels else parse_int(ops[1])
            delta = target - (cur_pc - 4)
            lo12 = sext(delta & 0xFFF, 12)
            words.append(enc_i(lo12, rd, 0x0, rd, 0x13))  # addi rd, rd, lo12
        elif mnem == "fence":
            if len(ops) == 0:
                pred, succ = 0xF, 0xF
            elif len(ops) == 2:
                pred, succ = parse_fence_mask(ops[0]), parse_fence_mask(ops[1])
            else:
                raise ValueError(f"FENCE expects 0 or 2 operands at PC=0x{cur_pc:x}")
            imm = ((pred & 0xF) << 4) | (succ & 0xF)
            words.append(enc_i(imm, 0, 0x0, 0, 0x0F))
        elif mnem == "fence.tso":
            words.append(0x8330_000F)
        elif mnem == "fence.i":
            words.append(0x0000_100F)
        elif mnem in CSR_OPS:
            f3, is_imm = CSR_OPS[mnem]
            rd = parse_reg(ops[0])
            csr = parse_csr(ops[1])
            if is_imm:
                uimm = parse_int(ops[2])
                if not (0 <= uimm <= 31):
                    raise ValueError(f"CSR uimm out of range: {uimm}")
                src = uimm
            else:
                src = parse_reg(ops[2])
            words.append(enc_i(csr, src, f3, rd, 0x73))
        elif _parse_amo_mnem(mnem) is not None:
            base, aq, rl = _parse_amo_mnem(mnem)  # type: ignore[misc]
            f5 = AMO_F5[base]
            rd = parse_reg(ops[0])
            if base == "lr.w":
                rs1 = _parse_mem_reg(ops[1])
                words.append(enc_amo(f5, aq, rl, 0, rs1, rd))
            else:
                rs2 = parse_reg(ops[1])
                rs1 = _parse_mem_reg(ops[2])
                words.append(enc_amo(f5, aq, rl, rs2, rs1, rd))
        elif mnem in R_OPS:
            f7, f3 = R_OPS[mnem]
            rd, rs1, rs2 = parse_reg(ops[0]), parse_reg(ops[1]), parse_reg(ops[2])
            words.append(enc_r(f7, rs2, rs1, f3, rd))
        elif mnem in I_ALU_OPS:
            f3 = I_ALU_OPS[mnem]
            rd, rs1, imm = parse_reg(ops[0]), parse_reg(ops[1]), parse_int(ops[2])
            words.append(enc_i(imm, rs1, f3, rd, 0x13))
        elif mnem in I_SHIFT_OPS:
            f7, f3 = I_SHIFT_OPS[mnem]
            rd, rs1, shamt = parse_reg(ops[0]), parse_reg(ops[1]), parse_int(ops[2])
            if not (0 <= shamt <= 31):
                raise ValueError(f"Shift amount out of range: {shamt}")
            words.append(enc_r(f7, shamt, rs1, f3, rd, opcode=0x13))
        elif mnem in LOAD_OPS:
            f3 = LOAD_OPS[mnem]
            rd = parse_reg(ops[0])
            m = MEM_ARG_RE.match(ops[1])
            if m:
                off = parse_int(m.group(1)) if m.group(1).strip() else 0
                rs1 = parse_reg(m.group(2))
            else:
                rs1, off = parse_reg(ops[1]), parse_int(ops[2])
            words.append(enc_i(off, rs1, f3, rd, 0x03))
        elif mnem in STORE_OPS:
            f3 = STORE_OPS[mnem]
            rs2 = parse_reg(ops[0])
            m = MEM_ARG_RE.match(ops[1])
            if m:
                off = parse_int(m.group(1)) if m.group(1).strip() else 0
                rs1 = parse_reg(m.group(2))
            else:
                rs1, off = parse_reg(ops[1]), parse_int(ops[2])
            words.append(enc_s(off, rs2, rs1, f3))
        elif mnem in BRANCH_OPS:
            f3 = BRANCH_OPS[mnem]
            rs1, rs2 = parse_reg(ops[0]), parse_reg(ops[1])
            target = ops[2]
            offset = (labels[target] - cur_pc) if target in labels else parse_int(target)
            words.append(enc_b(offset, rs2, rs1, f3))
        elif mnem == "lui":
            words.append(enc_u(parse_int(ops[1]), parse_reg(ops[0]), 0x37))
        elif mnem == "auipc":
            words.append(enc_u(parse_int(ops[1]), parse_reg(ops[0]), 0x17))
        elif mnem == "jal":
            rd = parse_reg(ops[0])
            target = ops[1]
            offset = (labels[target] - cur_pc) if target in labels else parse_int(target)
            words.append(enc_j(offset, rd))
        elif mnem == "jalr":
            rd = parse_reg(ops[0])
            if len(ops) == 2:
                m = MEM_ARG_RE.match(ops[1])
                if m:
                    off = parse_int(m.group(1)) if m.group(1).strip() else 0
                    rs1 = parse_reg(m.group(2))
                else:
                    rs1, off = parse_reg(ops[1]), 0
            else:
                rs1, off = parse_reg(ops[1]), parse_int(ops[2])
            words.append(enc_i(off, rs1, 0x0, rd, 0x67))
        # --- RV32F Single-Precision Floating-Point Instructions (All 26 Ops) ---
        elif mnem == "flw":
            fd = parse_freg(ops[0])
            m = MEM_ARG_RE.match(ops[1])
            if m:
                off = parse_int(m.group(1)) if m.group(1).strip() else 0
                rs1 = parse_reg(m.group(2))
            else:
                rs1, off = parse_reg(ops[1]), parse_int(ops[2])
            words.append(enc_i(off, rs1, 0x2, fd, 0x07))
        elif mnem == "fsw":
            fs2 = parse_freg(ops[0])
            m = MEM_ARG_RE.match(ops[1])
            if m:
                off = parse_int(m.group(1)) if m.group(1).strip() else 0
                rs1 = parse_reg(m.group(2))
            else:
                rs1, off = parse_reg(ops[1]), parse_int(ops[2])
            words.append(enc_s(off, fs2, rs1, 0x2, opcode=0x27))
        elif mnem in FP_R4_OPS:
            fd, fs1, fs2, fs3 = parse_freg(ops[0]), parse_freg(ops[1]), parse_freg(ops[2]), parse_freg(ops[3])
            rm = parse_rm(ops[4]) if len(ops) > 4 else 7
            words.append(enc_r4(fs3, 0, fs2, fs1, rm, fd, FP_R4_OPS[mnem]))
        elif mnem in FP_BIN_RM_OPS:
            fd, fs1, fs2 = parse_freg(ops[0]), parse_freg(ops[1]), parse_freg(ops[2])
            rm = parse_rm(ops[3]) if len(ops) > 3 else 7
            words.append(enc_r(FP_BIN_RM_OPS[mnem], fs2, fs1, rm, fd, opcode=0x53))
        elif mnem == "fsqrt.s":
            fd, fs1 = parse_freg(ops[0]), parse_freg(ops[1])
            rm = parse_rm(ops[2]) if len(ops) > 2 else 7
            words.append(enc_r(0x2C, 0, fs1, rm, fd, opcode=0x53))
        elif mnem in FP_SGNJ_OPS:
            fd, fs1, fs2 = parse_freg(ops[0]), parse_freg(ops[1]), parse_freg(ops[2])
            words.append(enc_r(0x10, fs2, fs1, FP_SGNJ_OPS[mnem], fd, opcode=0x53))
        elif mnem in FP_MINMAX_OPS:
            fd, fs1, fs2 = parse_freg(ops[0]), parse_freg(ops[1]), parse_freg(ops[2])
            words.append(enc_r(0x14, fs2, fs1, FP_MINMAX_OPS[mnem], fd, opcode=0x53))
        elif mnem in FP_CMP_OPS:
            rd, fs1, fs2 = parse_reg(ops[0]), parse_freg(ops[1]), parse_freg(ops[2])
            words.append(enc_r(0x50, fs2, fs1, FP_CMP_OPS[mnem], rd, opcode=0x53))
        elif mnem in ("fcvt.w.s", "fcvt.wu.s"):
            rd, fs1 = parse_reg(ops[0]), parse_freg(ops[1])
            rm = parse_rm(ops[2]) if len(ops) > 2 else 7
            words.append(enc_r(0x60, 0 if mnem == "fcvt.w.s" else 1, fs1, rm, rd, opcode=0x53))
        elif mnem in ("fcvt.s.w", "fcvt.s.wu"):
            fd, rs1 = parse_freg(ops[0]), parse_reg(ops[1])
            rm = parse_rm(ops[2]) if len(ops) > 2 else 7
            words.append(enc_r(0x68, 0 if mnem == "fcvt.s.w" else 1, rs1, rm, fd, opcode=0x53))
        elif mnem in ("fmv.x.w", "fmv.x.s"):
            rd, fs1 = parse_reg(ops[0]), parse_freg(ops[1])
            words.append(enc_r(0x70, 0, fs1, 0x0, rd, opcode=0x53))
        elif mnem == "fclass.s":
            rd, fs1 = parse_reg(ops[0]), parse_freg(ops[1])
            words.append(enc_r(0x70, 0, fs1, 0x1, rd, opcode=0x53))
        elif mnem in ("fmv.w.x", "fmv.s.x"):
            fd, rs1 = parse_freg(ops[0]), parse_reg(ops[1])
            words.append(enc_r(0x78, 0, rs1, 0x0, fd, opcode=0x53))
        else:
            raise ValueError(f"Unsupported mnemonic at PC=0x{cur_pc:x}: '{mnem}' {ops}")

    return words


def _fmt_fence_bits(bits: int) -> str:
    s = ""
    if bits & 8:
        s += "i"
    if bits & 4:
        s += "o"
    if bits & 2:
        s += "r"
    if bits & 1:
        s += "w"
    return s or "0"


def disassemble(word: int) -> str:
    """Decode a 32-bit RV32IMA_Zicsr_Zifencei instruction word."""
    op = word & 0x7F
    rd = (word >> 7) & 0x1F
    f3 = (word >> 12) & 0x7
    rs1 = (word >> 15) & 0x1F
    rs2 = (word >> 20) & 0x1F
    f7 = (word >> 25) & 0x7F
    f5 = (word >> 27) & 0x1F
    imm_i = sext(word >> 20, 12)
    imm_s = sext(((word >> 25) << 5) | ((word >> 7) & 0x1F), 12)
    imm_b = sext(
        (((word >> 31) & 1) << 12)
        | (((word >> 7) & 1) << 11)
        | (((word >> 25) & 0x3F) << 5)
        | (((word >> 8) & 0xF) << 1),
        13,
    )
    imm_u = (word >> 12) & 0xFFFFF
    imm_j = sext(
        (((word >> 31) & 1) << 20)
        | (((word >> 12) & 0xFF) << 12)
        | (((word >> 20) & 1) << 11)
        | (((word >> 21) & 0x3FF) << 1),
        21,
    )

    if word == 0x0000_0073:
        return "ecall"
    if word == 0x0010_0073:
        return "ebreak"
    if word == 0x3020_0073:
        return "mret"
    if op == 0x0F:
        if f3 == 0x0:
            if (word >> 28) == 0x8:
                return "fence.tso"
            pred = (word >> 24) & 0xF
            succ = (word >> 20) & 0xF
            return f"fence {_fmt_fence_bits(pred)}, {_fmt_fence_bits(succ)}"
        if f3 == 0x1:
            return "fence.i"
    elif op == 0x2F and f3 == 0x2 and f5 in AMO_REV:
        base = AMO_REV[f5]
        aq = (word >> 26) & 1
        rl = (word >> 25) & 1
        suf = ".aqrl" if (aq and rl) else (".aq" if aq else (".rl" if rl else ""))
        if base == "lr.w" and rs2 == 0:
            return f"lr.w{suf} x{rd}, (x{rs1})"
        if base != "lr.w":
            return f"{base}{suf} x{rd}, x{rs2}, (x{rs1})"
    elif op == 0x73:
        csr = (word >> 20) & 0xFFF
        cname = CSR_REV.get(csr, f"0x{csr:03x}")
        for name, (cf3, is_imm) in CSR_OPS.items():
            if cf3 == f3:
                return f"{name} x{rd}, {cname}, {rs1 if is_imm else f'x{rs1}'}"
    elif op == 0x33:
        for name, (rf7, rf3) in R_OPS.items():
            if rf7 == f7 and rf3 == f3:
                return f"{name} x{rd}, x{rs1}, x{rs2}"
    elif op == 0x13:
        if f3 == 0x1 and f7 == 0x00:
            return f"slli x{rd}, x{rs1}, {rs2}"
        if f3 == 0x5 and f7 == 0x00:
            return f"srli x{rd}, x{rs1}, {rs2}"
        if f3 == 0x5 and f7 == 0x20:
            return f"srai x{rd}, x{rs1}, {rs2}"
        for name, if3 in I_ALU_OPS.items():
            if if3 == f3:
                return f"{name} x{rd}, x{rs1}, {imm_i}"
    elif op == 0x03:
        for name, lf3 in LOAD_OPS.items():
            if lf3 == f3:
                return f"{name} x{rd}, {imm_i}(x{rs1})"
    elif op == 0x23:
        for name, sf3 in STORE_OPS.items():
            if sf3 == f3:
                return f"{name} x{rs2}, {imm_s}(x{rs1})"
    elif op == 0x63:
        for name, bf3 in BRANCH_OPS.items():
            if bf3 == f3:
                return f"{name} x{rs1}, x{rs2}, {imm_b}"
    elif op == 0x37:
        return f"lui x{rd}, 0x{imm_u:x}"
    elif op == 0x17:
        return f"auipc x{rd}, 0x{imm_u:x}"
    elif op == 0x6F:
        return f"jal x{rd}, {imm_j}"
    elif op == 0x67 and f3 == 0x0:
        return f"jalr x{rd}, x{rs1}, {imm_i}"
    elif op == 0x07 and f3 == 0x2:
        return f"flw f{rd}, {imm_i}(x{rs1})"
    elif op == 0x27 and f3 == 0x2:
        return f"fsw f{rs2}, {imm_s}(x{rs1})"
    elif op in (0x43, 0x47, 0x4B, 0x4F) and ((word >> 25) & 3) == 0:
        rs3 = (word >> 27) & 0x1F
        name = {0x43: "fmadd.s", 0x47: "fmsub.s", 0x4B: "fnmsub.s", 0x4F: "fnmadd.s"}[op]
        rms = FP_RM_REV.get(f3, str(f3))
        return f"{name} f{rd}, f{rs1}, f{rs2}, f{rs3}, {rms}"
    elif op == 0x53:
        rms = FP_RM_REV.get(f3, str(f3))
        if f7 == 0x00:
            return f"fadd.s f{rd}, f{rs1}, f{rs2}, {rms}"
        if f7 == 0x04:
            return f"fsub.s f{rd}, f{rs1}, f{rs2}, {rms}"
        if f7 == 0x08:
            return f"fmul.s f{rd}, f{rs1}, f{rs2}, {rms}"
        if f7 == 0x0C:
            return f"fdiv.s f{rd}, f{rs1}, f{rs2}, {rms}"
        if f7 == 0x2C and rs2 == 0:
            return f"fsqrt.s f{rd}, f{rs1}, {rms}"
        if f7 == 0x10 and f3 in (0, 1, 2):
            return f"{['fsgnj.s', 'fsgnjn.s', 'fsgnjx.s'][f3]} f{rd}, f{rs1}, f{rs2}"
        if f7 == 0x14 and f3 in (0, 1):
            return f"{['fmin.s', 'fmax.s'][f3]} f{rd}, f{rs1}, f{rs2}"
        if f7 == 0x50 and f3 in (0, 1, 2):
            return f"{['fle.s', 'flt.s', 'feq.s'][f3]} x{rd}, f{rs1}, f{rs2}"
        if f7 == 0x60 and rs2 in (0, 1):
            return f"{'fcvt.w.s' if rs2 == 0 else 'fcvt.wu.s'} x{rd}, f{rs1}, {rms}"
        if f7 == 0x68 and rs2 in (0, 1):
            return f"{'fcvt.s.w' if rs2 == 0 else 'fcvt.s.wu'} f{rd}, x{rs1}, {rms}"
        if f7 == 0x70 and rs2 == 0 and f3 == 0:
            return f"fmv.x.w x{rd}, f{rs1}"
        if f7 == 0x70 and rs2 == 0 and f3 == 1:
            return f"fclass.s x{rd}, f{rs1}"
        if f7 == 0x78 and rs2 == 0 and f3 == 0:
            return f"fmv.w.x f{rd}, x{rs1}"
    return f".word 0x{word:08x}"


# ---------------------------------------------------------------------------
# Golden RV32IMA_Zicsr_Zifencei Instruction Set Simulator (ISS)
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class CommitEvent:
    step: int
    pc: int
    instr: int
    rd_we: int
    rd_addr: int
    rd_wdata: int
    mem_valid: int
    mem_we: int
    mem_addr: int
    mem_wstrb: int
    mem_wdata: int
    halted: int
    rd_is_fp: int = 0
    fflags: int = 0

    def format_line(self) -> str:
        return (
            f"{self.step:06d} {self.pc:08x} {self.instr:08x} "
            f"{self.rd_we} {self.rd_addr:02d} {self.rd_wdata:08x} "
            f"{self.mem_valid} {self.mem_we} {self.mem_addr:08x} "
            f"{self.mem_wstrb:01x} {self.mem_wdata:08x} {self.halted}"
        )

    def format_fp_line(self) -> str:
        return f"{self.format_line()} {self.rd_is_fp} {self.fflags:02x}"


class RV32IMA_ISS:
    """Bit-accurate golden RV32IMA_Zicsr_Zifencei single-core architectural ISS."""

    def __init__(
        self,
        imem: List[int],
        mem_bytes: int = 262144,
        core_id: int = 0,
        unified_mem: bool = False,
    ) -> None:
        self.imem = list(imem)
        self.mem_size = mem_bytes
        self.unified_mem = unified_mem
        self.dmem = bytearray(mem_bytes)
        if unified_mem:
            for i, w in enumerate(imem):
                base = i * 4
                if base + 3 < mem_bytes:
                    self.dmem[base] = w & 0xFF
                    self.dmem[base + 1] = (w >> 8) & 0xFF
                    self.dmem[base + 2] = (w >> 16) & 0xFF
                    self.dmem[base + 3] = (w >> 24) & 0xFF
        self.regs = [0] * 32
        self.fregs = [0] * 32
        self.pc = 0
        self.steps = 0
        self.halted = False
        self.core_id = u32(core_id)
        self.resv_valid = False
        self.resv_line = 0
        # CSR & M-mode trap state
        self.mstatus_mie = 0
        self.mstatus_mpie = 0
        self.mstatus_fs = 1
        self.fflags = 0
        self.frm = 0
        self.mie = 0
        self.mtvec = 0
        self.mscratch = 0
        self.mepc = 0
        self.mcause = 0
        self.mtval = 0
        self.mip = 0
        self.misa = 0x4000_1121
        self.mcycle = 0
        self.minstret = 0
        self.trap_count = 0

    def _mstatus_val(self) -> int:
        return (
            ((self.mstatus_fs & 3) << 13)
            | (3 << 11)
            | ((self.mstatus_mpie & 1) << 7)
            | ((self.mstatus_mie & 1) << 3)
        )

    def _take_trap(self, epc: int, cause: int, tval: int, is_irq: bool = False) -> None:
        if (self.mtvec & ~3) == 0:
            raise ValueError(
                f"Unhandled trap cause={cause} tval=0x{u32(tval):08x} at PC=0x{epc:08x} (mtvec=0)"
            )
        self.mepc = epc & ~3
        self.mcause = ((1 << 31) if is_irq else 0) | (cause & 0xF)
        self.mtval = u32(tval)
        self.mstatus_mpie = self.mstatus_mie
        self.mstatus_mie = 0
        base = self.mtvec & ~3
        self.pc = u32(base + ((cause & 0xF) << 2)) if (is_irq and (self.mtvec & 1)) else base
        self.mcycle += 1
        self.trap_count += 1

    def _read_byte(self, addr: int) -> int:
        return self.dmem[addr % self.mem_size]

    def _write_byte(self, addr: int, val: int) -> None:
        self.dmem[addr % self.mem_size] = val & 0xFF

    def read_word(self, addr: int) -> int:
        a = addr & ~3
        return (
            self._read_byte(a)
            | (self._read_byte(a + 1) << 8)
            | (self._read_byte(a + 2) << 16)
            | (self._read_byte(a + 3) << 24)
        )

    def write_word(self, addr: int, val: int) -> None:
        a = addr & ~3
        v = u32(val)
        self._write_byte(a, v & 0xFF)
        self._write_byte(a + 1, (v >> 8) & 0xFF)
        self._write_byte(a + 2, (v >> 16) & 0xFF)
        self._write_byte(a + 3, (v >> 24) & 0xFF)

    def _fetch_instr(self, pc: int) -> int:
        if self.unified_mem:
            return self.read_word(pc)
        idx = pc >> 2
        if idx < 0 or idx >= len(self.imem):
            raise RuntimeError(f"PC out of instruction memory bounds: 0x{pc:08x}")
        return u32(self.imem[idx])

    def _csr_read(self, csr_addr: int) -> Optional[int]:
        if csr_addr == 0x001:
            return self.fflags if self.mstatus_fs != 0 else None
        if csr_addr == 0x002:
            return self.frm if self.mstatus_fs != 0 else None
        if csr_addr == 0x003:
            return (((self.frm & 7) << 5) | (self.fflags & 0x1F)) if self.mstatus_fs != 0 else None
        if csr_addr == 0x300:
            return self._mstatus_val()
        if csr_addr == 0x301:
            return self.misa
        if csr_addr == 0x304:
            return self.mie
        if csr_addr == 0x305:
            return self.mtvec
        if csr_addr == 0x340:
            return self.mscratch
        if csr_addr == 0x341:
            return self.mepc
        if csr_addr == 0x342:
            return self.mcause
        if csr_addr == 0x343:
            return self.mtval
        if csr_addr == 0x344:
            return self.mip
        if csr_addr in (0xB00, 0xC00):
            return self.mcycle & 0xFFFF_FFFF
        if csr_addr in (0xB80, 0xC80):
            return (self.mcycle >> 32) & 0xFFFF_FFFF
        if csr_addr in (0xB02, 0xC02):
            return self.minstret & 0xFFFF_FFFF
        if csr_addr in (0xB82, 0xC82):
            return (self.minstret >> 32) & 0xFFFF_FFFF
        if csr_addr == 0xF11:
            return 0x0000_0000
        if csr_addr == 0xF12:
            return 0x0000_2002
        if csr_addr == 0xF13:
            return 0x0001_0000
        if csr_addr == 0xF14:
            return self.core_id
        return None

    def _csr_write(self, csr_addr: int, val: int) -> bool:
        if (csr_addr >> 10) == 0x3:
            return False
        v = u32(val)
        if csr_addr == 0x001:
            if self.mstatus_fs == 0:
                return False
            self.fflags = v & 0x1F
            self.mstatus_fs = 3
        elif csr_addr == 0x002:
            if self.mstatus_fs == 0:
                return False
            self.frm = v & 0x7
            self.mstatus_fs = 3
        elif csr_addr == 0x003:
            if self.mstatus_fs == 0:
                return False
            self.fflags = v & 0x1F
            self.frm = (v >> 5) & 0x7
            self.mstatus_fs = 3
        elif csr_addr == 0x300:
            self.mstatus_mie = (v >> 3) & 1
            self.mstatus_mpie = (v >> 7) & 1
            self.mstatus_fs = (v >> 13) & 3
        elif csr_addr == 0x301:
            pass  # WARL-ignored
        elif csr_addr == 0x304:
            self.mie = v & ((1 << 11) | (1 << 7) | (1 << 3))
        elif csr_addr == 0x305:
            self.mtvec = v & 0xFFFF_FFFD
        elif csr_addr == 0x340:
            self.mscratch = v
        elif csr_addr == 0x341:
            self.mepc = v & ~3
        elif csr_addr == 0x342:
            self.mcause = v
        elif csr_addr == 0x343:
            self.mtval = v
        elif csr_addr == 0x344:
            pass  # WARL-ignored
        elif csr_addr == 0xB00:
            self.mcycle = (self.mcycle & 0xFFFF_FFFF_0000_0000) | v
        elif csr_addr == 0xB80:
            self.mcycle = (self.mcycle & 0x0000_0000_FFFF_FFFF) | (v << 32)
        elif csr_addr == 0xB02:
            self.minstret = (self.minstret & 0xFFFF_FFFF_0000_0000) | v
        elif csr_addr == 0xB82:
            self.minstret = (self.minstret & 0x0000_0000_FFFF_FFFF) | (v << 32)
        else:
            return False
        return True

    def step(self) -> Optional[CommitEvent]:
        if self.halted:
            raise RuntimeError("ISS already halted")

        cur_pc = self.pc
        instr = self._fetch_instr(cur_pc)
        op = instr & 0x7F
        rd = (instr >> 7) & 0x1F
        f3 = (instr >> 12) & 0x7
        rs1 = (instr >> 15) & 0x1F
        rs2 = (instr >> 20) & 0x1F
        f7 = (instr >> 25) & 0x7F
        f5 = (instr >> 27) & 0x1F

        v1 = self.regs[rs1]
        v2 = self.regs[rs2]

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

        next_pc = u32(cur_pc + 4)
        rd_we = 0
        rd_is_fp = 0
        rd_wdata = 0
        inst_fflags = 0
        mem_valid = 0
        mem_we = 0
        mem_addr = 0
        mem_wstrb = 0
        mem_wdata = 0

        if instr == 0x0010_0073:  # EBREAK
            if (self.mtvec & ~3) != 0:
                self._take_trap(cur_pc, 3, cur_pc)
                return None
            self.halted = True
        elif instr == 0x0000_0073:  # ECALL
            self._take_trap(cur_pc, 11, 0)
            return None
        elif instr == 0x3020_0073:  # MRET
            self.mstatus_mie = self.mstatus_mpie
            self.mstatus_mpie = 1
            next_pc = self.mepc
        elif op == 0x0F and f3 in (0x0, 0x1):  # FENCE / FENCE.I
            pass
        elif op == 0x73 and f3 in (0x1, 0x2, 0x3, 0x5, 0x6, 0x7):  # Zicsr
            csr_addr = (instr >> 20) & 0xFFF
            old_val = self._csr_read(csr_addr)
            operand = rs1 if (f3 & 4) else v1
            kind = f3 & 3
            wants_write = (kind == 1) or (rs1 != 0)
            if old_val is None or (wants_write and (csr_addr >> 10) == 0x3):
                self._take_trap(cur_pc, 2, instr)
                return None
            rd_we = 1
            rd_wdata = old_val
            if wants_write:
                if kind == 1:
                    new_val = operand
                elif kind == 2:
                    new_val = old_val | operand
                else:
                    new_val = old_val & (~operand)
                if not self._csr_write(csr_addr, new_val):
                    self._take_trap(cur_pc, 2, instr)
                    return None
        elif op == 0x2F and f3 == 0x2:  # RV32A
            addr = u32(v1)
            if f5 == 0b00010 and rs2 != 0:
                self._take_trap(cur_pc, 2, instr)
                return None
            if f5 not in AMO_REV:
                self._take_trap(cur_pc, 2, instr)
                return None
            if (addr & 3) != 0:
                self._take_trap(cur_pc, 4 if f5 == 0b00010 else 6, addr)
                return None
            mem_valid = 1
            mem_addr = addr
            rd_we = 1
            if f5 == 0b00010:  # LR.W
                rd_wdata = self.read_word(addr)
                self.resv_valid = True
                self.resv_line = addr & ~0xF
            elif f5 == 0b00011:  # SC.W
                if self.resv_valid and self.resv_line == (addr & ~0xF):
                    self.write_word(addr, v2)
                    mem_we = 1
                    mem_wstrb = 0xF
                    mem_wdata = u32(v2)
                    rd_wdata = 0
                else:
                    mem_we = 0
                    rd_wdata = 1
                self.resv_valid = False
            else:
                old = self.read_word(addr)
                if f5 == 0b00001:    # AMOSWAP
                    nv = v2
                elif f5 == 0b00000:  # AMOADD
                    nv = u32(old + v2)
                elif f5 == 0b00100:  # AMOXOR
                    nv = u32(old ^ v2)
                elif f5 == 0b01100:  # AMOAND
                    nv = u32(old & v2)
                elif f5 == 0b01000:  # AMOOR
                    nv = u32(old | v2)
                elif f5 == 0b10000:  # AMOMIN
                    nv = old if s32(old) < s32(v2) else v2
                elif f5 == 0b10100:  # AMOMAX
                    nv = old if s32(old) > s32(v2) else v2
                elif f5 == 0b11000:  # AMOMINU
                    nv = old if u32(old) < u32(v2) else v2
                elif f5 == 0b11100:  # AMOMAXU
                    nv = old if u32(old) > u32(v2) else v2
                else:
                    self._take_trap(cur_pc, 2, instr)
                    return None
                self.write_word(addr, nv)
                if self.resv_valid and self.resv_line == (addr & ~0xF):
                    self.resv_valid = False
                mem_we = 1
                mem_wstrb = 0xF
                mem_wdata = u32(nv)
                rd_wdata = old
        elif op == 0x37:  # LUI
            rd_we = 1
            rd_wdata = u32(imm_u)
        elif op == 0x17:  # AUIPC
            rd_we = 1
            rd_wdata = u32(cur_pc + imm_u)
        elif op == 0x6F:  # JAL
            tgt = u32(cur_pc + imm_j)
            if (tgt & 3) != 0:
                self._take_trap(cur_pc, 0, tgt)
                return None
            rd_we = 1
            rd_wdata = u32(cur_pc + 4)
            next_pc = tgt
        elif op == 0x67 and f3 == 0x0:  # JALR
            tgt = u32(v1 + imm_i) & 0xFFFF_FFFE
            if (tgt & 3) != 0:
                self._take_trap(cur_pc, 0, tgt)
                return None
            rd_we = 1
            rd_wdata = u32(cur_pc + 4)
            next_pc = tgt
        elif op == 0x63:  # BRANCH
            taken = False
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
                self._take_trap(cur_pc, 2, instr)
                return None
            if taken:
                tgt = u32(cur_pc + imm_b)
                if (tgt & 3) != 0:
                    self._take_trap(cur_pc, 0, tgt)
                    return None
                next_pc = tgt
        elif op == 0x03:  # LOAD
            addr = u32(v1 + imm_i)
            if f3 not in (0x0, 0x1, 0x2, 0x4, 0x5):
                self._take_trap(cur_pc, 2, instr)
                return None
            if (f3 in (0x1, 0x5) and (addr & 1) != 0) or (f3 == 0x2 and (addr & 3) != 0):
                self._take_trap(cur_pc, 4, addr)
                return None
            mem_valid = 1
            mem_we = 0
            mem_addr = addr
            rd_we = 1
            if f3 == 0x0:
                rd_wdata = u32(sext(self._read_byte(addr), 8))
            elif f3 == 0x1:
                h = self._read_byte(addr) | (self._read_byte(addr + 1) << 8)
                rd_wdata = u32(sext(h, 16))
            elif f3 == 0x2:
                rd_wdata = self.read_word(addr)
            elif f3 == 0x4:
                rd_wdata = self._read_byte(addr)
            elif f3 == 0x5:
                rd_wdata = self._read_byte(addr) | (self._read_byte(addr + 1) << 8)
        elif op == 0x23:  # STORE
            addr = u32(v1 + imm_s)
            lane = addr & 0x3
            if f3 not in (0x0, 0x1, 0x2):
                self._take_trap(cur_pc, 2, instr)
                return None
            if (f3 == 0x1 and (addr & 1) != 0) or (f3 == 0x2 and (addr & 3) != 0):
                self._take_trap(cur_pc, 6, addr)
                return None
            mem_valid = 1
            mem_we = 1
            mem_addr = addr
            if self.resv_valid and self.resv_line == (addr & ~0xF):
                self.resv_valid = False
            if f3 == 0x0:
                self._write_byte(addr, v2 & 0xFF)
                mem_wstrb = 1 << lane
                mem_wdata = u32((v2 & 0xFF) << (8 * lane))
            elif f3 == 0x1:
                self._write_byte(addr, v2 & 0xFF)
                self._write_byte(addr + 1, (v2 >> 8) & 0xFF)
                mem_wstrb = 0xC if (lane & 2) else 0x3
                mem_wdata = u32((v2 & 0xFFFF) << (16 if (lane & 2) else 0))
            elif f3 == 0x2:
                self.write_word(addr, v2)
                mem_wstrb = 0xF
                mem_wdata = u32(v2)
        elif op == 0x13:  # OP-IMM
            rd_we = 1
            shamt = rs2
            if f3 == 0x0:
                rd_wdata = u32(v1 + imm_i)
            elif f3 == 0x1 and f7 == 0x00:
                rd_wdata = u32(v1 << shamt)
            elif f3 == 0x2:
                rd_wdata = 1 if s32(v1) < imm_i else 0
            elif f3 == 0x3:
                rd_wdata = 1 if u32(v1) < u32(imm_i) else 0
            elif f3 == 0x4:
                rd_wdata = u32(v1 ^ u32(imm_i))
            elif f3 == 0x5 and f7 == 0x00:
                rd_wdata = u32(v1) >> shamt
            elif f3 == 0x5 and f7 == 0x20:
                rd_wdata = u32(s32(v1) >> shamt)
            elif f3 == 0x6:
                rd_wdata = u32(v1 | u32(imm_i))
            elif f3 == 0x7:
                rd_wdata = u32(v1 & u32(imm_i))
            else:
                self._take_trap(cur_pc, 2, instr)
                return None
        elif op == 0x33:  # OP (RV32I + RV32M)
            rd_we = 1
            if f7 == 0x01:
                if f3 == 0x0:
                    rd_wdata = u32(s32(v1) * s32(v2))
                elif f3 == 0x1:
                    rd_wdata = u32((s32(v1) * s32(v2)) >> 32)
                elif f3 == 0x2:
                    rd_wdata = u32((s32(v1) * u32(v2)) >> 32)
                elif f3 == 0x3:
                    rd_wdata = u32((u32(v1) * u32(v2)) >> 32)
                elif f3 == 0x4:
                    if v2 == 0:
                        rd_wdata = 0xFFFF_FFFF
                    elif v1 == 0x8000_0000 and v2 == 0xFFFF_FFFF:
                        rd_wdata = 0x8000_0000
                    else:
                        sa, sb = s32(v1), s32(v2)
                        q = abs(sa) // abs(sb)
                        rd_wdata = u32(-q if ((sa < 0) ^ (sb < 0)) else q)
                elif f3 == 0x5:
                    rd_wdata = 0xFFFF_FFFF if v2 == 0 else u32(u32(v1) // u32(v2))
                elif f3 == 0x6:
                    if v2 == 0:
                        rd_wdata = u32(v1)
                    elif v1 == 0x8000_0000 and v2 == 0xFFFF_FFFF:
                        rd_wdata = 0
                    else:
                        sa, sb = s32(v1), s32(v2)
                        r = abs(sa) % abs(sb)
                        rd_wdata = u32(-r if sa < 0 else r)
                elif f3 == 0x7:
                    rd_wdata = u32(v1) if v2 == 0 else u32(u32(v1) % u32(v2))
            elif f7 == 0x00 and f3 == 0x0:
                rd_wdata = u32(v1 + v2)
            elif f7 == 0x20 and f3 == 0x0:
                rd_wdata = u32(v1 - v2)
            elif f7 == 0x00 and f3 == 0x1:
                rd_wdata = u32(v1 << (v2 & 0x1F))
            elif f7 == 0x00 and f3 == 0x2:
                rd_wdata = 1 if s32(v1) < s32(v2) else 0
            elif f7 == 0x00 and f3 == 0x3:
                rd_wdata = 1 if u32(v1) < u32(v2) else 0
            elif f7 == 0x00 and f3 == 0x4:
                rd_wdata = u32(v1 ^ v2)
            elif f7 == 0x00 and f3 == 0x5:
                rd_wdata = u32(v1) >> (v2 & 0x1F)
            elif f7 == 0x20 and f3 == 0x5:
                rd_wdata = u32(s32(v1) >> (v2 & 0x1F))
            elif f7 == 0x00 and f3 == 0x6:
                rd_wdata = u32(v1 | v2)
            elif f7 == 0x00 and f3 == 0x7:
                rd_wdata = u32(v1 & v2)
            else:
                self._take_trap(cur_pc, 2, instr)
                return None
        elif op == 0x07:  # LOAD-FP (FLW)
            if self.mstatus_fs == 0 or f3 != 0x2:
                self._take_trap(cur_pc, 2, instr)
                return None
            addr = u32(v1 + imm_i)
            if (addr & 3) != 0:
                self._take_trap(cur_pc, 4, addr)
                return None
            mem_valid = 1
            mem_we = 0
            mem_addr = addr
            rd_we = 1
            rd_is_fp = 1
            rd_wdata = self.read_word(addr)
        elif op == 0x27:  # STORE-FP (FSW)
            if self.mstatus_fs == 0 or f3 != 0x2:
                self._take_trap(cur_pc, 2, instr)
                return None
            addr = u32(v1 + imm_s)
            if (addr & 3) != 0:
                self._take_trap(cur_pc, 6, addr)
                return None
            fval2 = self.fregs[rs2]
            mem_valid = 1
            mem_we = 1
            mem_addr = addr
            mem_wstrb = 0xF
            mem_wdata = u32(fval2)
            if self.resv_valid and self.resv_line == (addr & ~0xF):
                self.resv_valid = False
            self.write_word(addr, fval2)
        elif op in (0x43, 0x47, 0x4B, 0x4F):  # FMADD/FMSUB/FNMSUB/FNMADD.S
            fmt = (instr >> 25) & 3
            eff_rm = self.frm if f3 == 7 else f3
            if self.mstatus_fs == 0 or fmt != 0 or f3 in (5, 6) or eff_rm > 4:
                self._take_trap(cur_pc, 2, instr)
                return None
            rs3 = (instr >> 27) & 0x1F
            fma_op = {0x43: fpo.FMA_FMADD, 0x47: fpo.FMA_FMSUB, 0x4B: fpo.FMA_FNMSUB, 0x4F: fpo.FMA_FNMADD}[op]
            rd_wdata, inst_fflags = fpo.eval_fma(fma_op, self.fregs[rs1], self.fregs[rs2], self.fregs[rs3], eff_rm)
            rd_we = 1
            rd_is_fp = 1
        elif op == 0x53:  # OP-FP
            if self.mstatus_fs == 0:
                self._take_trap(cur_pc, 2, instr)
                return None
            eff_rm = self.frm if f3 == 7 else f3
            rm_bad = (f3 in (5, 6)) or (eff_rm > 4)
            fv1 = self.fregs[rs1]
            fv2 = self.fregs[rs2]
            if f7 == 0x00:  # FADD.S
                if rm_bad:
                    self._take_trap(cur_pc, 2, instr); return None
                rd_wdata, inst_fflags = fpo.eval_fma(fpo.FMA_FADD, fv1, fv2, 0, eff_rm)
                rd_we, rd_is_fp = 1, 1
            elif f7 == 0x04:  # FSUB.S
                if rm_bad:
                    self._take_trap(cur_pc, 2, instr); return None
                rd_wdata, inst_fflags = fpo.eval_fma(fpo.FMA_FSUB, fv1, fv2, 0, eff_rm)
                rd_we, rd_is_fp = 1, 1
            elif f7 == 0x08:  # FMUL.S
                if rm_bad:
                    self._take_trap(cur_pc, 2, instr); return None
                rd_wdata, inst_fflags = fpo.eval_fma(fpo.FMA_FMUL, fv1, fv2, 0, eff_rm)
                rd_we, rd_is_fp = 1, 1
            elif f7 == 0x0C:  # FDIV.S
                if rm_bad:
                    self._take_trap(cur_pc, 2, instr); return None
                rd_wdata, inst_fflags = fpo.eval_div_sqrt(fpo.FDIV_DIV, fv1, fv2, eff_rm)
                rd_we, rd_is_fp = 1, 1
            elif f7 == 0x2C:  # FSQRT.S
                if rs2 != 0 or rm_bad:
                    self._take_trap(cur_pc, 2, instr); return None
                rd_wdata, inst_fflags = fpo.eval_div_sqrt(fpo.FDIV_SQRT, fv1, 0, eff_rm)
                rd_we, rd_is_fp = 1, 1
            elif f7 == 0x10 and f3 in (0, 1, 2):  # FSGNJ/N/X.S
                mop = [fpo.FMISC_SGNJ, fpo.FMISC_SGNJN, fpo.FMISC_SGNJX][f3]
                rd_wdata, inst_fflags = fpo.eval_misc(mop, fv1, fv2, 0)
                rd_we, rd_is_fp = 1, 1
            elif f7 == 0x14 and f3 in (0, 1):  # FMIN/FMAX.S
                mop = fpo.FMISC_MIN if f3 == 0 else fpo.FMISC_MAX
                rd_wdata, inst_fflags = fpo.eval_misc(mop, fv1, fv2, 0)
                rd_we, rd_is_fp = 1, 1
            elif f7 == 0x50 and f3 in (0, 1, 2):  # FLE/FLT/FEQ.S
                mop = [fpo.FMISC_LE, fpo.FMISC_LT, fpo.FMISC_EQ][f3]
                rd_wdata, inst_fflags = fpo.eval_misc(mop, fv1, fv2, 0)
                rd_we, rd_is_fp = 1, 0
            elif f7 == 0x60 and rs2 in (0, 1):  # FCVT.W/WU.S
                if rm_bad:
                    self._take_trap(cur_pc, 2, instr); return None
                mop = fpo.FMISC_CVT_WS if rs2 == 0 else fpo.FMISC_CVT_WUS
                rd_wdata, inst_fflags = fpo.eval_misc(mop, fv1, 0, eff_rm)
                rd_we, rd_is_fp = 1, 0
            elif f7 == 0x68 and rs2 in (0, 1):  # FCVT.S.W/WU
                if rm_bad:
                    self._take_trap(cur_pc, 2, instr); return None
                mop = fpo.FMISC_CVT_SW if rs2 == 0 else fpo.FMISC_CVT_SWU
                rd_wdata, inst_fflags = fpo.eval_misc(mop, v1, 0, eff_rm)
                rd_we, rd_is_fp = 1, 1
            elif f7 == 0x70 and rs2 == 0 and f3 in (0, 1):  # FMV.X.W / FCLASS.S
                mop = fpo.FMISC_MV_XW if f3 == 0 else fpo.FMISC_CLASS
                rd_wdata, inst_fflags = fpo.eval_misc(mop, fv1, 0, 0)
                rd_we, rd_is_fp = 1, 0
            elif f7 == 0x78 and rs2 == 0 and f3 == 0:  # FMV.W.X
                rd_wdata, inst_fflags = fpo.eval_misc(fpo.FMISC_MV_WX, v1, 0, 0)
                rd_we, rd_is_fp = 1, 1
            else:
                self._take_trap(cur_pc, 2, instr)
                return None
        else:
            self._take_trap(cur_pc, 2, instr)
            return None

        if inst_fflags != 0:
            self.fflags = (self.fflags | inst_fflags) & 0x1F
            if self.mstatus_fs != 0:
                self.mstatus_fs = 3
        if rd_is_fp and rd_we:
            self.fregs[rd] = u32(rd_wdata)
            if self.mstatus_fs != 0:
                self.mstatus_fs = 3
        elif rd_we and rd != 0:
            self.regs[rd] = u32(rd_wdata)
        else:
            if not rd_is_fp and rd == 0:
                rd_we = 0
                rd_wdata = 0

        self.pc = next_pc
        self.mcycle += 1
        self.minstret += 1
        ev = CommitEvent(
            step=self.steps,
            pc=cur_pc,
            instr=instr,
            rd_we=rd_we,
            rd_addr=rd if rd_we else 0,
            rd_wdata=rd_wdata if rd_we else 0,
            mem_valid=mem_valid,
            mem_we=mem_we,
            mem_addr=mem_addr,
            mem_wstrb=mem_wstrb,
            mem_wdata=mem_wdata,
            halted=1 if self.halted else 0,
            rd_is_fp=rd_is_fp if rd_we else 0,
            fflags=inst_fflags,
        )
        self.steps += 1
        return ev

    def run(self, max_steps: int = 200_000) -> List[CommitEvent]:
        trace: List[CommitEvent] = []
        iters = 0
        while not self.halted and iters < max_steps:
            ev = self.step()
            iters += 1
            if ev is not None:
                trace.append(ev)
        if not self.halted:
            raise RuntimeError(f"ISS did not reach EBREAK within {max_steps} steps")
        return trace


def write_hex(words: List[int], path: Path, pad_words: int = 0) -> None:
    lines = [f"{w:08x}" for w in words]
    if pad_words > len(lines):
        lines.extend(["00000013"] * (pad_words - len(lines)))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_trace(trace: List[CommitEvent], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(ev.format_line() for ev in trace) + "\n", encoding="utf-8")


def write_fp_trace(trace: List[CommitEvent], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(ev.format_fp_line() for ev in trace) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# Built-In Full RV32IMA_Zicsr_Zifencei Self-Test (64+ Instructions!)
# ---------------------------------------------------------------------------
ALL_RV32IMA_PROG = """
    # 1. RV32I + RV32M Core Spot Checks
    lui   x1, 0x12345
    auipc x2, 0x00001
    addi  x3, x0, -7
    addi  x4, x0, 2
    mul   x5, x3, x4            # -14
    div   x6, x3, x4            # -3
    rem   x7, x3, x4            # -1

    # 2. Zicsr (mhartid, mscratch all 6 ops)
    csrr   x8, mhartid          # x8 = core_id (1 in self-test)
    li     x9, 0x000000F0
    csrrw  x10, mscratch, x9    # old=0, mscratch=0xF0
    csrrsi x11, mscratch, 0x05  # old=0xF0, mscratch=0xF5
    csrrci x12, mscratch, 0x10  # old=0xF5, mscratch=0xE5
    csrrs  x13, mscratch, x0    # pure read = 0xE5

    # 3. FENCE & FENCE.I
    fence  rw, rw
    fence.tso
    fence.i

    # 4. Full RV32A (LR.W / SC.W success + fail, and all 9 AMO*.W ops)
    addi   x20, x0, 0x100       # base addr = 0x100
    addi   x21, x0, 10
    sw     x21, 0(x20)          # mem[0x100] = 10

    lr.w.aq x22, (x20)          # x22 = 10, reserve line 0x100
    addi    x23, x22, 5         # 15
    sc.w.rl x24, x23, (x20)     # succeeds -> x24 = 0, mem[0x100] = 15
    sc.w    x25, x21, (x20)     # no reservation -> fails! x25 = 1, mem stays 15

    addi    x26, x0, 7
    amoadd.w.aqrl  x27, x26, (x20) # old=15, mem=22
    amoswap.w      x28, x3,  (x20) # old=22, mem=-7 (0xfffffff9)
    amomin.w       x29, x4,  (x20) # old=-7, min(-7, 2) = -7
    amomax.w       x30, x4,  (x20) # old=-7, max(-7, 2) = 2
    amominu.w      x31, x3,  (x20) # old=2, minu(2, 0xfffffff9) = 2
    amomaxu.w      x15, x3,  (x20) # old=2, maxu(2, 0xfffffff9) = -7 (0xfffffff9)
    amoand.w       x16, x9,  (x20) # old=0xfffffff9, mem = 0xf0
    amoor.w        x17, x4,  (x20) # old=0xf0, mem = 0xf2
    amoxor.w       x18, x26, (x20) # old=0xf2, mem = 0xf5
    lw             x19, 0(x20)     # x19 = 0xf5

    # Combine checksum into x14: x8(1) + x13(0xe5) + x24(0) + x25(1) + x19(0xf5) = 0x1dc
    add    x14, x8, x13
    add    x14, x14, x24
    add    x14, x14, x25
    add    x14, x14, x19
    ebreak
"""


def run_self_test(out_hex: Optional[Path] = None, out_trace: Optional[Path] = None, pad_words: int = 1024) -> None:
    words = assemble(ALL_RV32IMA_PROG)
    for w in words:
        s = disassemble(w)
        assert not s.startswith(".word"), f"Failed to disassemble 0x{w:08x}"

    iss = RV32IMA_ISS(words, core_id=1)
    trace = iss.run(max_steps=500)

    assert iss.regs[5] == u32(-14), f"MUL got {iss.regs[5]}"
    assert iss.regs[6] == u32(-3), f"DIV got {iss.regs[6]}"
    assert iss.regs[8] == 1, f"mhartid got {iss.regs[8]}"
    assert iss.regs[13] == 0xE5, f"mscratch got 0x{iss.regs[13]:x}"
    assert iss.regs[22] == 10, f"LR.W got {iss.regs[22]}"
    assert iss.regs[24] == 0, f"SC.W success expected 0, got {iss.regs[24]}"
    assert iss.regs[25] == 1, f"SC.W fail expected 1, got {iss.regs[25]}"
    assert iss.regs[27] == 15, f"AMOADD old expected 15, got {iss.regs[27]}"
    assert iss.regs[28] == 22, f"AMOSWAP old expected 22, got {iss.regs[28]}"
    assert iss.regs[29] == u32(-7), f"AMOMIN old expected -7, got 0x{iss.regs[29]:x}"
    assert iss.regs[30] == u32(-7), f"AMOMAX old expected -7, got 0x{iss.regs[30]:x}"
    assert iss.regs[31] == 2, f"AMOMINU old expected 2, got {iss.regs[31]}"
    assert iss.regs[15] == 2, f"AMOMAXU old expected 2, got {iss.regs[15]}"
    assert iss.regs[16] == u32(-7), f"AMOAND old expected -7, got 0x{iss.regs[16]:x}"
    assert iss.regs[17] == 0xF0, f"AMOOR old expected 0xf0, got 0x{iss.regs[17]:x}"
    assert iss.regs[18] == 0xF2, f"AMOXOR old expected 0xf2, got 0x{iss.regs[18]:x}"
    assert iss.regs[19] == 0xF5, f"Final mem expected 0xf5, got 0x{iss.regs[19]:x}"
    assert iss.regs[14] == 0x1DC, f"Checksum x14 expected 0x1dc, got 0x{iss.regs[14]:x}"

    if out_hex is not None:
        write_hex(words, out_hex, pad_words=pad_words)
    if out_trace is not None:
        write_trace(trace, out_trace)

    print(
        f"PASS [rv32ima_asm.py]: assembled {len(words)} words, simulated {len(trace)} dynamic "
        f"RV32IMA_Zicsr_Zifencei instructions (incl. LR/SC + 9 AMOs + CSR + FENCE), x14=0x{iss.regs[14]:08x}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="RV32IMA_Zicsr_Zifencei Assembler & Golden ISS")
    ap.add_argument("input", nargs="?", type=Path, help="Input .S assembly file or .hex file (with --run-hex)")
    ap.add_argument("--hex", type=Path, help="Output .hex image")
    ap.add_argument("--trace", type=Path, help="Output golden commit trace")
    ap.add_argument("--pad", type=int, default=1024, help="Pad .hex to N words")
    ap.add_argument("--core-id", type=int, default=0, help="mhartid value for ISS")
    ap.add_argument("--unified", action="store_true", help="Run ISS with unified code+data memory")
    ap.add_argument("--run-hex", action="store_true", help="Interpret input file as word-per-line .hex")
    ap.add_argument("--max-steps", type=int, default=200_000, help="ISS max dynamic steps")
    ap.add_argument("--self-test", action="store_true", help="Run built-in RV32IMA_Zicsr_Zifencei self-test")
    args = ap.parse_args()

    if args.self_test:
        run_self_test(args.hex, args.trace, pad_words=args.pad)
        return 0

    if not args.input:
        ap.error("Provide an input file or --self-test")

    if args.run_hex:
        words = [int(ln.strip(), 16) for ln in args.input.read_text(encoding="utf-8").splitlines() if ln.strip()]
    else:
        words = assemble(args.input.read_text(encoding="utf-8"))
        if args.hex:
            write_hex(words, args.hex, pad_words=args.pad)

    if args.trace or args.run_hex:
        iss = RV32IMA_ISS(words, core_id=args.core_id, unified_mem=args.unified)
        trace = iss.run(max_steps=args.max_steps)
        if args.trace:
            write_trace(trace, args.trace)
        print(
            f"[rv32ima_asm] {args.input.name}: {len(words)} words, {len(trace)} steps, "
            f"x14=0x{iss.regs[14]:08x}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
