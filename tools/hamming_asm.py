# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Hamming .hasm assembler + disassembler.

Assembles Hamming protocol-core assembly into a 512 x 32-bit SRAM 0 image
(two 16-bit instruction slots per word, first instruction in the low half).
`python -m tools.hamming_asm prog.hasm` writes `prog.bin` (little-endian
words) and `prog.hex`; `--disassemble` turns a `.bin`/`.hex` image back into
canonical `.hasm` source.  `assemble()`/`disassemble()` are exact inverses:
assemble(disassemble(words)) is bit-identical for every 16-bit slot pattern.

Source grammar
--------------
    ; comment (also //)
    label:                       defines a code symbol (instruction slot)
    add r1, r2                   ALU binary op (mov add sub and or xor cmp not)
    shl r1                       ALU unary op  (shl shr inc dec)
    ldi8.h r0, 0x1f              load imm8 into r0[15:8] (.l = low byte)
    pin_out 0b0101, 0b1111       core_out masked update (mask, val)
    pin_oe  mask, val            core_oe masked update
    pin_in r1, 3                 read synced input pin 0..15
    lut_read r1, 5               read LUT output bit 0..15
    wait_phase r1 | wait_phase 100   stall to anchor+delta (reg or imm8)
    sync_event 0x3, r2           wait EV_FLAGS[3:0]&mask, timeout = r2 cycles
    set_anchor 100               anchor <= timer + imm8
    read_timer r1, lo|hi         timer[15:0] / {missed,11'b0,timer[19:16]}
    bcu_tx r1 / bcu_rx r1 / bcu_ctrl 0x55 / bcu_stat r1 / bcu_crc_rd r1, lo|hi
    ld_dmem_w r1, [r0+2]         SRAM1 word at r0+imm2 (imm2 0..3)
    st_dmem_w / ld_dmem_b / st_dmem_b / ld_imem r1, [r0+2]
    csr_rd r1, 0x08 / csr_wr r1, EV_SET    (CSR names accepted)
    jmp label / jz / jnz / call  (target = label or 0..1023)
    jev 0x1, r3                  pc <= r3 if (EV_FLAGS[3:0] & 0x1)
    ret / halt / nop

Directives
----------
    .word expr [, expr ...]      emit raw 16-bit slot(s)
    .org expr                    set emission slot (0..1023, forward only)
    .lut name = (bool expr)      compile to a 16-bit truth-table word in the
                                 tables region (appended after code); the
                                 name becomes a symbol = its 16-bit entry index
    .bcu key=val, ...            annotation record (kept in the AST, no bytes)
    .constraint ...              timing annotation (kept in the AST)
    .protocol name [@clk 60MHz]  program annotation; @clk sets @us/@baud base
    .clk 60000000|60MHz          set the @us/@baud clock base directly

Numeric expressions accept decimal / 0x / 0b literals, labels, `+`, `-`,
`*`, `//`, parentheses, `@us(n)` (cycles for n microseconds at the program
clock) and `@baud(n)` (cycles per bit at n baud).  Boolean `.lut`
expressions accept identifiers (max 4 distinct inputs, bound in order of
first appearance to index bits 0..3), `~`, `&`, `|`, `^`, parentheses and
the constants 0/1.
"""

import argparse
import ast
import re
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

from . import hamming_isa as isa


class AsmError(Exception):
    """Assembly error with source line context."""

    def __init__(self, msg, lineno=None):
        self.lineno = lineno
        super().__init__(f"line {lineno}: {msg}" if lineno else msg)


# ---------------------------------------------------------------------------
# Lexing helpers
# ---------------------------------------------------------------------------

_LABEL_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*:")
_REG_RE = re.compile(r"^[rR]([0-3])$")
_MEM_RE = re.compile(r"^\[\s*([rR][0-3])\s*(?:\+\s*(.*?))?\s*\]$")
_NUM_SUFFIX = {"khz": 1_000, "mhz": 1_000_000, "ghz": 1_000_000_000}


def _strip_comment(line):
    """Cut a line at the first `;` or `//` comment marker."""
    best = len(line)
    for marker in (";", "//"):
        pos = line.find(marker)
        if 0 <= pos < best:
            best = pos
    return line[:best]


def _split_operands(text):
    """Split on commas that are not inside () or []."""
    parts, depth, cur = [], 0, []
    for ch in text:
        if ch in "([":
            depth += 1
        elif ch in ")]":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    parts.append("".join(cur).strip())
    return [p for p in parts if p != ""]


def _parse_reg(tok, lineno):
    m = _REG_RE.match(tok.strip())
    if not m:
        raise AsmError(f"expected register r0-r3, got {tok!r}", lineno)
    return int(m.group(1))


def _eval_num(tok, lineno):
    """Parse a plain numeric literal (no symbols)."""
    t = tok.strip().replace("_", "")
    m = re.match(r"^([0-9]+(?:\.[0-9]+)?)\s*(kHz|MHz|GHz|khz|mhz|ghz)$", t)
    if m:
        return int(round(float(m.group(1)) * _NUM_SUFFIX[m.group(2).lower()]))
    try:
        return int(t, 0)
    except ValueError:
        pass
    try:
        return int(t, 10)
    except ValueError:
        raise AsmError(f"bad numeric literal {tok!r}", lineno)


class _ExprValidator(ast.NodeVisitor):
    """Whitelist AST walker for arithmetic operand expressions."""

    _OPS = (ast.BinOp, ast.UnaryOp, ast.Expression, ast.Load,
            ast.Add, ast.Sub, ast.Mult, ast.FloorDiv, ast.UAdd, ast.USub,
            ast.Name, ast.Constant)

    def generic_visit(self, node):
        if not isinstance(node, self._OPS):
            raise AsmError(f"disallowed expression element {type(node).__name__}")
        super().generic_visit(node)


def _eval_numeric(expr, symbols, lineno, clk_hz):
    """Evaluate an operand expression to an int or float (internal)."""

    def _func(m):
        name, arg = m.group(1).lower(), m.group(2)
        val = _eval_numeric(arg, symbols, lineno, clk_hz)
        if name == "us":
            return str(round(clk_hz * val / 1_000_000))
        if name == "baud":
            if val <= 0:
                raise AsmError("@baud requires a positive rate", lineno)
            return str(round(clk_hz / val))
        raise AsmError(f"unknown function @{name}", lineno)

    text = re.sub(r"@(\w+)\s*\(\s*([^()]*)\s*\)", _func, expr.strip())
    try:
        tree = ast.parse(text, mode="eval")
    except SyntaxError as e:
        raise AsmError(f"bad expression {expr!r}: {e}", lineno)
    try:
        _ExprValidator().visit(tree)
        env = dict(symbols)
        return eval(compile(tree, "<hasm>", "eval"),
                    {"__builtins__": {}}, env)
    except AsmError:
        raise
    except NameError as e:
        raise AsmError(f"undefined symbol in {expr!r}: {e}", lineno)
    except Exception as e:  # ZeroDivisionError, TypeError, ...
        raise AsmError(f"cannot evaluate {expr!r}: {e}", lineno)


def eval_expr(expr, symbols, lineno, clk_hz=isa.CLK_HZ):
    """Evaluate an operand expression to an int.

    Terms: integer literals, label/lut symbols, @us(n), @baud(n); combined
    with + - * // and parentheses.  Float arguments are allowed inside
    @us/@baud; the final value must be integral.
    """
    value = _eval_numeric(expr, symbols, lineno, clk_hz)
    if isinstance(value, float):
        if not value.is_integer():
            raise AsmError(f"expression {expr!r} is not integral", lineno)
        value = int(value)
    return int(value)


def _check_range(value, bits, what, lineno):
    if not 0 <= value < (1 << bits):
        raise AsmError(f"{what}={value} does not fit in {bits} bits", lineno)
    return value


# ---------------------------------------------------------------------------
# .lut boolean-expression compiler
# ---------------------------------------------------------------------------

def _lut_names_in_order(node, out):
    for child in ast.iter_child_nodes(node):
        if isinstance(child, ast.Name):
            if child.id not in out:
                out.append(child.id)
        else:
            _lut_names_in_order(child, out)


class _BoolValidator(ast.NodeVisitor):
    _OPS = (ast.Expression, ast.BinOp, ast.UnaryOp, ast.Load,
            ast.BitAnd, ast.BitOr, ast.BitXor, ast.Invert,
            ast.Name, ast.Constant)

    def generic_visit(self, node):
        if not isinstance(node, self._OPS):
            raise AsmError(
                f"disallowed .lut expression element {type(node).__name__}")
        super().generic_visit(node)

    def visit_Constant(self, node):
        if not isinstance(node.value, int) or node.value not in (0, 1):
            raise AsmError(".lut constants must be 0 or 1")


def compile_lut(expr_src, lineno=None):
    """Compile a .lut boolean expression to (truth_table, input_names).

    The k-th distinct identifier binds to index bit k (max 4 inputs).
    Returns the 16-bit truth table plus the ordered input name list.
    """
    try:
        tree = ast.parse(expr_src.strip(), mode="eval")
    except SyntaxError as e:
        raise AsmError(f"bad .lut expression {expr_src!r}: {e}", lineno)
    _BoolValidator().visit(tree)
    names = []
    _lut_names_in_order(tree, names)
    if len(names) > 4:
        raise AsmError(
            f".lut expression uses {len(names)} inputs {names}; max 4", lineno)
    code = compile(tree, "<lut>", "eval")
    tt = 0
    for idx in range(16):
        env = {name: (idx >> k) & 1 for k, name in enumerate(names)}
        try:
            bit = int(eval(code, {"__builtins__": {}}, env)) & 1
        except Exception as e:
            raise AsmError(f".lut eval failed: {e}", lineno)
        tt |= bit << idx
    return tt, names


# ---------------------------------------------------------------------------
# Parser: source text -> item AST
# ---------------------------------------------------------------------------

@dataclass
class Program:
    words: list = field(default_factory=list)   # 512 x 32-bit
    used_slots: int = 0                          # 16-bit slots emitted
    labels: dict = field(default_factory=dict)
    luts: dict = field(default_factory=dict)     # name -> 16-bit entry index
    lut_tt: dict = field(default_factory=dict)   # name -> truth table word
    lut_inputs: dict = field(default_factory=dict)
    annotations: list = field(default_factory=list)  # (kind, data, lineno)
    items: list = field(default_factory=list)
    clk_hz: int = isa.CLK_HZ

    @property
    def used_bytes(self):
        return ((self.used_slots + 1) // 2) * 4


def parse_source(src):
    """Parse .hasm source into the item AST (no layout/encoding yet)."""
    items = []
    for lineno, raw in enumerate(src.splitlines(), 1):
        line = _strip_comment(raw).strip()
        if not line:
            continue
        # One or more leading `name:` labels, then the rest of the line.
        while True:
            m = _LABEL_RE.match(line)
            if not m:
                break
            items.append(("label", lineno, m.group(1)))
            line = line[m.end():].strip()
            if not line:
                break
        if not line:
            continue
        if line.startswith("."):
            parts = line.split(None, 1)
            directive = parts[0].lower()
            rest = parts[1] if len(parts) > 1 else ""
            items.append(("directive", lineno, directive, rest))
        else:
            items.append(("instr", lineno, line))
    return items


# ---------------------------------------------------------------------------
# Instruction operand -> encoding
# ---------------------------------------------------------------------------

_SEL_NAMES = {"lo": 0, "hi": 1, "0": 0, "1": 1,
              "timer_lo": 0, "timer_hi": 1}


def _csr_addr(tok, symbols, lineno, clk_hz):
    t = tok.strip()
    if t.upper() in isa.CSR:
        return isa.CSR[t.upper()]
    return _check_range(eval_expr(t, symbols, lineno, clk_hz), 8,
                        "csr_addr", lineno)


def encode_instruction(text, symbols, lineno, clk_hz=isa.CLK_HZ):
    """Encode one instruction source line to a 16-bit word."""
    head, _, rest = text.partition(" ")
    mnemonic, _, suffix = head.lower().partition(".")
    ops = _split_operands(rest) if rest.strip() else []

    def need(n):
        if len(ops) != n:
            raise AsmError(
                f"{mnemonic} expects {n} operand(s), got {len(ops)}", lineno)

    def ev(tok, bits, what):
        return _check_range(eval_expr(tok, symbols, lineno, clk_hz),
                            bits, what, lineno)

    if mnemonic in isa.ALU_BINARY:
        need(2)
        return isa.enc_alu(isa.ALU_OPS[mnemonic],
                           _parse_reg(ops[0], lineno), _parse_reg(ops[1], lineno))
    if mnemonic in isa.ALU_UNARY:
        need(1)
        return isa.enc_alu(isa.ALU_OPS[mnemonic], _parse_reg(ops[0], lineno), 0)
    if mnemonic == "ldi8":
        need(2)
        half = {"": 0, "l": 0, "h": 1}.get(suffix)
        if half is None:
            raise AsmError(f"ldi8 suffix must be .l or .h, got .{suffix}", lineno)
        return isa.enc_ldi8(_parse_reg(ops[0], lineno), half,
                            ev(ops[1], 8, "imm8"))
    if mnemonic == "pin_out":
        need(2)
        return isa.enc_pin_out(ev(ops[0], 4, "mask4"), ev(ops[1], 4, "val4"))
    if mnemonic == "pin_oe":
        need(2)
        return isa.enc_pin_oe(ev(ops[0], 4, "mask4"), ev(ops[1], 4, "val4"))
    if mnemonic == "pin_in":
        need(2)
        return isa.enc_pin_in(_parse_reg(ops[0], lineno), ev(ops[1], 4, "pin"))
    if mnemonic == "lut_read":
        need(2)
        return isa.enc_lut_read(_parse_reg(ops[0], lineno), ev(ops[1], 4, "lut"))
    if mnemonic == "wait_phase":
        need(1)
        if _REG_RE.match(ops[0]):
            return isa.enc_wait_rs(_parse_reg(ops[0], lineno))
        return isa.enc_wait_imm(ev(ops[0], 8, "imm8"))
    if mnemonic == "sync_event":
        need(2)
        return isa.enc_sync_event(ev(ops[0], 4, "mask4"),
                                  _parse_reg(ops[1], lineno))
    if mnemonic == "set_anchor":
        need(1)
        return isa.enc_set_anchor(ev(ops[0], 8, "imm8"))
    if mnemonic == "read_timer":
        need(2)
        sel = _SEL_NAMES.get(ops[1].strip().lower())
        if sel is None:
            raise AsmError("read_timer sel must be lo or hi", lineno)
        return isa.enc_read_timer(_parse_reg(ops[0], lineno), sel)
    if mnemonic == "bcu_tx":
        need(1)
        return isa.enc_bcu_tx(_parse_reg(ops[0], lineno))
    if mnemonic == "bcu_rx":
        need(1)
        return isa.enc_bcu_rx(_parse_reg(ops[0], lineno))
    if mnemonic == "bcu_ctrl":
        need(1)
        return isa.enc_bcu_ctrl(ev(ops[0], 8, "imm8"))
    if mnemonic == "bcu_stat":
        need(1)
        return isa.enc_bcu_stat(_parse_reg(ops[0], lineno))
    if mnemonic == "bcu_crc_rd":
        need(2)
        sel = _SEL_NAMES.get(ops[1].strip().lower())
        if sel is None:
            raise AsmError("bcu_crc_rd sel must be lo or hi", lineno)
        return isa.enc_bcu_crc_rd(_parse_reg(ops[0], lineno), sel)
    if mnemonic in isa.MEM_OPS:
        need(2)
        rt = _parse_reg(ops[0], lineno)
        m = _MEM_RE.match(ops[1])
        if not m:
            raise AsmError(
                f"{mnemonic} expects [rN+imm] addressing, got {ops[1]!r}", lineno)
        ra = _parse_reg(m.group(1), lineno)
        imm2 = ev(m.group(2), 2, "imm2") if m.group(2) is not None else 0
        return isa.enc_mem(isa.MEM_OPS[mnemonic], ra, imm2, rt)
    if mnemonic == "csr_rd":
        need(2)
        return isa.enc_csr_rd(_parse_reg(ops[0], lineno),
                              _csr_addr(ops[1], symbols, lineno, clk_hz))
    if mnemonic == "csr_wr":
        need(2)
        return isa.enc_csr_wr(_parse_reg(ops[0], lineno),
                              _csr_addr(ops[1], symbols, lineno, clk_hz))
    if mnemonic == "jmp":
        need(1)
        return isa.enc_jmp(ev(ops[0], isa.PC_W, "branch target"))
    if mnemonic in ("jz", "jnz", "call"):
        need(1)
        return isa.enc_br(isa.BR_OPS[mnemonic],
                          ev(ops[0], isa.PC_W, "branch target"))
    if mnemonic == "jev":
        need(2)
        return isa.enc_jev(ev(ops[0], 4, "mask4"), _parse_reg(ops[1], lineno))
    if mnemonic in isa.MISC_OPS:
        need(0)
        return isa.enc_misc(isa.MISC_OPS[mnemonic])
    raise AsmError(f"unknown mnemonic {mnemonic!r}", lineno)


# ---------------------------------------------------------------------------
# Two-pass assembly
# ---------------------------------------------------------------------------

def _parse_bcu_annotation(rest, lineno):
    rec = {}
    for pair in _split_operands(rest):
        if "=" not in pair:
            raise AsmError(f".bcu expects key=val pairs, got {pair!r}", lineno)
        k, _, v = pair.partition("=")
        rec[k.strip()] = v.strip()
    return rec


def _parse_clock(rest, lineno):
    m = re.search(r"@clk\s+(\S+)", rest)
    return _eval_num(m.group(1), lineno) if m else _eval_num(rest, lineno)


def assemble_program(src):
    """Assemble .hasm source into a Program (512-word image + AST + symbols)."""
    prog = Program()
    items = parse_source(src)
    prog.items = items

    # ---- pass 1: layout + label/lut symbol tables ----
    pos = 0
    symbols = {}
    emissions = []   # (slot, kind, lineno, payload)
    lut_list = []    # (name, expr_src, lineno)

    def set_pos(p, lineno):
        nonlocal pos
        if not 0 <= p <= (1 << isa.PC_W):
            raise AsmError(f".org slot {p} out of range", lineno)
        pos = p

    for item in items:
        kind, lineno = item[0], item[1]
        if kind == "label":
            name = item[2]
            if name in symbols:
                raise AsmError(f"duplicate label {name!r}", lineno)
            symbols[name] = pos
        elif kind == "instr":
            emissions.append((pos, "instr", lineno, item[2]))
            pos += 1
        elif kind == "directive":
            directive, rest = item[2], item[3]
            if directive == ".org":
                set_pos(eval_expr(rest, symbols, lineno, prog.clk_hz), lineno)
            elif directive == ".word":
                exprs = _split_operands(rest)
                if not exprs:
                    raise AsmError(".word needs at least one expression", lineno)
                for e in exprs:
                    emissions.append((pos, "word", lineno, e))
                    pos += 1
            elif directive == ".lut":
                m = re.match(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.+)$", rest)
                if not m:
                    raise AsmError(".lut syntax: .lut name = (bool expr)", lineno)
                lut_list.append((m.group(1), m.group(2).strip(), lineno))
            elif directive == ".bcu":
                prog.annotations.append(
                    ("bcu", _parse_bcu_annotation(rest, lineno), lineno))
            elif directive == ".constraint":
                prog.annotations.append(("constraint", rest.strip(), lineno))
            elif directive == ".protocol":
                prog.annotations.append(("protocol", rest.strip(), lineno))
                clk = re.search(r"@clk\s+(\S+)", rest)
                if clk:
                    prog.clk_hz = _eval_num(clk.group(1), lineno)
            elif directive == ".clk":
                prog.clk_hz = _eval_num(rest, lineno)
            else:
                raise AsmError(f"unknown directive {directive!r}", lineno)

    # ---- tables region: .lut words appended right after the code ----
    tables_base = pos
    for k, (name, expr_src, lineno) in enumerate(lut_list):
        if name in symbols:
            raise AsmError(f"duplicate symbol {name!r}", lineno)
        tt, names = compile_lut(expr_src, lineno)
        prog.lut_tt[name] = tt
        prog.lut_inputs[name] = names
        prog.luts[name] = tables_base + k
        symbols[name] = tables_base + k

    prog.labels = dict(symbols)

    # ---- pass 2: encode into 16-bit slots ----
    slots = {}

    def put(slot, value, lineno):
        if slot >= (1 << isa.PC_W):
            raise AsmError(
                f"image exceeds {isa.IMAGE_BYTES_MAX} bytes "
                f"(slot {slot} >= {1 << isa.PC_W})", lineno)
        if slot in slots:
            raise AsmError(f"overlapping emission at slot {slot}", lineno)
        slots[slot] = value & 0xFFFF

    for slot, ekind, lineno, payload in emissions:
        if ekind == "instr":
            put(slot, encode_instruction(payload, symbols, lineno, prog.clk_hz),
                lineno)
        else:
            v = eval_expr(payload, symbols, lineno, prog.clk_hz)
            if not 0 <= v <= 0xFFFF:
                raise AsmError(f".word value {v} out of 16-bit range", lineno)
            put(slot, v, lineno)
    for k, (name, expr_src, lineno) in enumerate(lut_list):
        put(tables_base + k, prog.lut_tt[name], lineno)

    prog.used_slots = max(slots, default=-1) + 1
    if prog.used_slots * 2 > isa.IMAGE_BYTES_MAX:
        raise AsmError(
            f"image is {prog.used_slots * 2} bytes > {isa.IMAGE_BYTES_MAX} "
            "byte SRAM 0 budget")
    words = [0] * isa.SRAM0_WORDS
    for slot, value in slots.items():
        w = slot >> 1
        words[w] |= value << (16 * (slot & 1))
    prog.words = words
    return prog


def assemble(src):
    """Assemble .hasm source -> list of 512 32-bit SRAM 0 words."""
    return assemble_program(src).words


def to_bin_bytes(words):
    """Pack 32-bit words to little-endian bytes for .bin images."""
    return struct.pack(f"<{len(words)}I", *[w & 0xFFFFFFFF for w in words])


def to_hex_text(words):
    """One 32-bit little-endian-word hex value per line."""
    return "".join(f"{w & 0xFFFFFFFF:08x}\n" for w in words)


def from_bin_bytes(data):
    if len(data) % 4:
        raise AsmError(f".bin length {len(data)} is not a multiple of 4")
    return list(struct.unpack(f"<{len(data) // 4}I", data))


def from_hex_text(text):
    words = []
    for lineno, line in enumerate(text.splitlines(), 1):
        t = line.strip()
        if not t or t.startswith((";", "#", "//")):
            continue
        try:
            words.append(int(t, 16))
        except ValueError:
            raise AsmError(f"bad hex word {t!r}", lineno)
    return words


# ---------------------------------------------------------------------------
# Disassembler
# ---------------------------------------------------------------------------

def disasm_slot(value):
    """Canonical source text for one 16-bit slot (.word fallback)."""
    value &= 0xFFFF
    dec = isa.decode(value)
    if dec is None:
        return f".word 0x{value:04X}"
    mnem, ops = dec
    if mnem in isa.ALU_BINARY:
        return f"{mnem} r{ops['rd']}, r{ops['rs']}"
    if mnem in isa.ALU_UNARY:
        return f"{mnem} r{ops['rd']}"
    if mnem == "ldi8":
        return f"ldi8.{'lh'[ops['half']]} r{ops['rd']}, 0x{ops['imm8']:02X}"
    if mnem in ("pin_out", "pin_oe"):
        return f"{mnem} 0x{ops['mask4']:X}, 0x{ops['val4']:X}"
    if mnem in ("pin_in", "lut_read"):
        key = "pin" if mnem == "pin_in" else "lut"
        return f"{mnem} r{ops['rd']}, 0x{ops[key]:X}"
    if mnem == "wait_phase":
        if "rs" in ops:
            return f"wait_phase r{ops['rs']}"
        return f"wait_phase 0x{ops['imm8']:02X}"
    if mnem == "sync_event":
        return f"sync_event 0x{ops['mask4']:X}, r{ops['rt']}"
    if mnem == "set_anchor":
        return f"set_anchor 0x{ops['imm8']:02X}"
    if mnem == "read_timer":
        return f"read_timer r{ops['rd']}, {'lo' if ops['sel'] == 0 else 'hi'}"
    if mnem == "bcu_tx":
        return f"bcu_tx r{ops['rs']}"
    if mnem == "bcu_rx":
        return f"bcu_rx r{ops['rd']}"
    if mnem == "bcu_ctrl":
        return f"bcu_ctrl 0x{ops['imm8']:02X}"
    if mnem == "bcu_stat":
        return f"bcu_stat r{ops['rd']}"
    if mnem == "bcu_crc_rd":
        return f"bcu_crc_rd r{ops['rd']}, {'lo' if ops['sel'] == 0 else 'hi'}"
    if mnem in isa.MEM_OPS:
        return (f"{mnem} r{ops['rt']}, "
                f"[r{ops['ra']}+{ops['imm2']}]")
    if mnem == "csr_rd":
        return f"csr_rd r{ops['rd']}, 0x{ops['addr']:02X}"
    if mnem == "csr_wr":
        return f"csr_wr r{ops['rs']}, 0x{ops['addr']:02X}"
    if mnem == "jmp":
        return f"jmp 0x{ops['pc10']:03X}"
    if mnem in ("jz", "jnz", "call"):
        return f"{mnem} 0x{ops['pc10']:03X}"
    if mnem == "jev":
        return f"jev 0x{ops['mask4']:X}, r{ops['rs']}"
    return mnem  # ret / halt / nop


def disassemble(words):
    """List of canonical source lines, one per 16-bit slot (2 per word)."""
    lines = []
    for w in words:
        lines.append(disasm_slot(w & 0xFFFF))
        lines.append(disasm_slot((w >> 16) & 0xFFFF))
    return lines


def disassemble_to_source(words):
    return "\n".join(disassemble(words)) + "\n"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="tools.hamming_asm",
        description="Hamming .hasm assembler / disassembler")
    ap.add_argument("input", help=".hasm source (or .bin/.hex with --disassemble)")
    ap.add_argument("-o", "--output", help="output path (bin for assemble, "
                                         "hasm for disassemble)")
    ap.add_argument("--hex", dest="hex_out", help=".hex output path "
                                                "(default: <input>.hex)")
    ap.add_argument("--bin", dest="bin_out", help=".bin output path "
                                                "(default: <input>.bin)")
    ap.add_argument("--disassemble", action="store_true",
                    help="read a .bin/.hex image and emit .hasm source")
    args = ap.parse_args(argv)
    inp = Path(args.input)

    if args.disassemble:
        if inp.suffix.lower() == ".bin":
            words = from_bin_bytes(inp.read_bytes())
        else:
            words = from_hex_text(inp.read_text())
        src = disassemble_to_source(words)
        out = Path(args.output) if args.output else inp.with_suffix(".hasm")
        out.write_text(src)
        print(f"{out}: {len(words)} words disassembled")
        return 0

    prog = assemble_program(inp.read_text())
    bin_path = Path(args.bin_out or args.output or inp.with_suffix(".bin"))
    hex_path = Path(args.hex_out or inp.with_suffix(".hex"))
    bin_path.write_bytes(to_bin_bytes(prog.words))
    hex_path.write_text(to_hex_text(prog.words))
    print(f"{bin_path}: {len(prog.words)} words "
          f"({prog.used_slots} slots / {prog.used_bytes} B used)")
    print(f"{hex_path}: written")
    return 0


if __name__ == "__main__":
    sys.exit(main())
