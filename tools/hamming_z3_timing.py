#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Z3 static timing verifier for Hamming protocol-core firmware.

The verifier answers properties of the form

    starting at label/address START, every reachable execution that reaches
    label/address END does so within BUDGET core clock cycles,

for all initial register/memory/event/pin/BCU inputs accepted by the optional
assumptions.  A violation witness contains the path prefix that either crosses
the budget before END or reaches END too late.  Reaching HALT without reaching
END is also a violation of the completion property.

The tool intentionally reuses ``tools.hamming_asm`` and ``tools.hamming_isa``;
it does not reimplement the assembler or instruction decoder.  It can read the
``.hasm`` source directly (preferred, because labels survive) or the assembled
``.hex``/``.bin`` image (numeric addresses must then be used).

----------------------------------------------------------------------------
Timing model provenance
----------------------------------------------------------------------------

The timing rules are deliberately tied to the RTL rather than intuition:

* ``src/hamming_core.v`` lines 118-125 make a cache hit or instruction-memory
  response combinationally ``fetch_valid``; lines 234-236 gate execution with
  that validity; lines 376-378 retire a normal instruction at the clock edge.
  A hit therefore costs exactly one core cycle.
* ``src/hamming_core.v`` lines 250-255 issue an instruction fetch request only
  on a miss, and lines 327-330 track the outstanding request.  The fill and
  execution happen in the response cycle (lines 121-129), after which the
  direct-mapped line is usable by following fetches.
* ``src/hamming_icache.v`` lines 31-48 implement an eight-line direct-mapped
  cache indexed by SRAM0 word address bits [2:0], with tag bits [8:3].  The
  analyzer models the same occupancy exactly for every concrete symbolic-
  execution path.
* ``src/hamming_core.v`` lines 460-468 implement WAIT_PHASE with a signed
  20-bit modular comparison: negative means wait, zero is exactly on time,
  positive is a sticky miss without a stall.  Lines 335-339 release the wait
  when the modular difference becomes non-negative.  Lines 469-477 implement
  SYNC_EVENT, and lines 340-347 implement its timeout/event release and
  re-anchor.  ``src/hamming_csr.v`` line 298 increments the global timer once
  per clock.
* ``src/hamming_core.v`` lines 348-375 hold data-memory instructions until
  the SRAM completion condition.  ``src/hamming_sram_arb.v`` lines 8-15,
  74-80 and 123-137 describe registered macro I/O, strict round-robin service
  and the read-return pipeline; ``src/project.v`` lines 183-207 instantiate
  five SRAM0 clients and lines 218-245 instantiate six SRAM1 clients.
* Pin endpoint constants are optional because a label-to-label firmware check
  normally starts and ends inside the core.  When a pin-to-pin budget is
  supplied, ``src/hamming_lut_matrix.v`` lines 73-110 justify a two-cycle
  unfiltered input synchronizer and lines 240-257 justify a one-cycle output
  router leaf.  The glitch filter is configuration-dependent and is not
  included automatically.

# ORCHESTRATOR-CHECK: KTD2 and the generated ISA contract say SRAM0 fetch
# misses are 3..7 cycles and SRAM1 reads add 3..8 cycles.  Reading the
# production RTL pipelines literally, including the fact that ``rvalid`` is a
# registered output of ``hamming_sram_arb`` and is therefore consumed by the
# core on the following edge, gives retired-instruction costs of 5..9 cycles
# for an SRAM0 fetch miss, 4..9 extra cycles for an SRAM1 read, 1..6 extra
# cycles for an SRAM1 write, and 5..9 extra cycles for LD_IMEM.  The
# ``exact-rtl`` profile below uses those conservative production-RTL bounds.
# The ``isa-contract`` profile preserves the checked-in single-source latency
# numbers for comparison.  This off-by-one interpretation should be reviewed
# against the intended U2/U3 contract; using the larger envelope is safe for a
# worst-case prover but can be more pessimistic than the project contract.

----------------------------------------------------------------------------
Soundness and approximation policy
----------------------------------------------------------------------------

The symbolic executor is exact for the core instruction/flag arithmetic it
supports, exact for the affine anchor/timer algebra, exact for concrete static
control flow (including the two-entry call stack), and exact for path-sensitive
cache occupancy.  The following deliberate *over-approximations* are used where
chip-level inputs are outside firmware control:

1. SRAM service latency is an independent integer choice within the bounded
   round-robin envelope for each request.  Correlations of the round-robin
   pointer are not modeled, so some witness latency vectors may be pessimistic;
   no silicon latency behavior is removed.
2. Pins, LUT outputs, BCU status/RX/CRC values, and (in the default ``volatile``
   event model) EV_FLAGS observations are fresh nondeterministic inputs at each
   use.  This is conservative for timing.  ``--event-model internal`` restricts
   observations to core-written state for small closed-system tests.
3. Host SRAM0 writes/cache invalidations are assumed absent during the checked
   fragment.  Modeling an invalidation at an arbitrary cycle would require a
   hybrid concurrent-host model; such a proof obligation is reported as an
   explicit assumption rather than silently ignored.
4. State merging introduces a fresh Boolean selector and merges all state
   components with ITE.  Every original path remains representable, while the
   merge can also admit spurious combinations.  Thus a PROVED result is sound
   with respect to this documented abstraction, and a VIOLATION witness is a
   concrete witness *within that abstraction*.  Arbitration variables in the
   witness identify the required service envelope.
5. Indirect JEV targets that remain symbolic are unsupported rather than
   guessed.  Unsupported behavior, solver ``unknown``, unsatisfiable/vacuous
   assumptions, or resource exhaustion never produce a PASS.

Batch spec format (JSON)::

    {
      "defaults": {"profile": "exact-rtl", "core": 0},
      "checks": [
        {
          "firmware": "../firmware/can20a.hasm",
          "start": "pb_loop", "end": "pb_tm", "budget": 60,
          "assume": ["r2 >= 1", "r2 <= 32"],
          "core": 3,
          "prewarm": [["pb_loop", "pb_tm"]],
          "input_latency": 2, "output_latency": 1,
          "smt2": "can_pb_cex.smt2"
        }
      ]
    }

Firmware paths in a batch file are resolved relative to the batch file.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import sys
from collections import OrderedDict, defaultdict
from dataclasses import dataclass, field, replace
from pathlib import Path

# Support both ``python -m tools.hamming_z3_timing`` and the required direct
# command ``python tools/hamming_z3_timing.py ...``.
try:  # pragma: no cover - import style depends on invocation
    from . import hamming_asm
    from . import hamming_isa as isa
except ImportError:  # pragma: no cover
    _ROOT = Path(__file__).resolve().parents[1]
    if str(_ROOT) not in sys.path:
        sys.path.insert(0, str(_ROOT))
    from tools import hamming_asm
    from tools import hamming_isa as isa

try:  # z3 is intentionally the only new dependency.
    import z3  # type: ignore
    _Z3_IMPORT_ERROR = None
except ImportError as _z3_error:  # pragma: no cover - exercised without z3
    z3 = None  # type: ignore
    _Z3_IMPORT_ERROR = _z3_error


TIMER_MODULUS = 1 << isa.TIMER_W
TIMER_HALF = 1 << (isa.TIMER_W - 1)


class TimingError(Exception):
    """User-facing timing verifier error."""


def require_z3():
    if z3 is None:
        raise TimingError(
            "z3-solver is required for timing verification. Install it with "
            "`pip install z3-solver` (or the project uv environment) and rerun."
        ) from _Z3_IMPORT_ERROR
    return z3


# ---------------------------------------------------------------------------
# Firmware loading and instruction utilities
# ---------------------------------------------------------------------------

@dataclass
class FirmwareImage:
    words: list
    labels: dict = field(default_factory=dict)
    program: object = None
    source_path: str = ""

    def decode_slot(self, pc):
        pc &= isa.PC_MASK
        word32 = self.words[(pc >> 1) & 0x1FF]
        word = (word32 >> (16 * (pc & 1))) & 0xFFFF
        dec = isa.decode(word)
        if dec is None:
            return None, {}, word
        return dec[0], dec[1], word


def load_firmware(pathlike):
    """Load .hasm (with labels) or assembled .hex/.bin as a FirmwareImage."""
    path = Path(pathlike)
    suffix = path.suffix.lower()
    if suffix == ".hasm":
        prog = hamming_asm.assemble_program(path.read_text())
        return FirmwareImage(list(prog.words), dict(prog.labels), prog, str(path))
    if suffix == ".bin":
        words = hamming_asm.from_bin_bytes(path.read_bytes())
    elif suffix in (".hex", ".h", ".mem"):
        words = hamming_asm.from_hex_text(path.read_text())
    else:
        # Source is the safest default: all shipped firmware uses .hasm, but a
        # no-extension copy should still assemble rather than silently decode.
        try:
            prog = hamming_asm.assemble_program(path.read_text())
            return FirmwareImage(list(prog.words), dict(prog.labels), prog, str(path))
        except Exception as e:
            raise TimingError(
                f"{path}: unsupported firmware format (use .hasm/.hex/.bin): {e}")
    if len(words) > isa.SRAM0_WORDS:
        raise TimingError(f"{path}: image has {len(words)} words > {isa.SRAM0_WORDS}")
    words = (list(words) + [0] * isa.SRAM0_WORDS)[:isa.SRAM0_WORDS]
    return FirmwareImage(words, {}, None, str(path))


def image_from_source(src, name="<string>"):
    prog = hamming_asm.assemble_program(src)
    return FirmwareImage(list(prog.words), dict(prog.labels), prog, name)


def resolve_address(image, token, what="address"):
    if isinstance(token, int):
        value = token
    else:
        text = str(token).strip()
        try:
            value = int(text, 0)
        except ValueError:
            if text not in image.labels:
                raise TimingError(f"unknown {what} {text!r}; not numeric and not a label")
            value = image.labels[text]
    if not 0 <= value <= isa.PC_MASK:
        raise TimingError(f"{what} {token!r} resolves to {value}, outside 0..{isa.PC_MASK}")
    return value


# ---------------------------------------------------------------------------
# Basic-block control-flow graph
# ---------------------------------------------------------------------------

@dataclass
class BasicBlock:
    start: int
    pcs: list
    successors: list

    @property
    def end(self):
        return self.pcs[-1]


def _static_successors(mnem, ops, pc):
    """Conservative static successors for CFG display.

    The executor below implements CALL/RET precisely.  For a static basic-
    block graph, CALL includes both the callee and the return/fallthrough edge;
    that overapproximates reachability but keeps the graph useful without a
    separate interprocedural expansion.
    """
    fall = (pc + 1) & isa.PC_MASK
    if mnem == "jmp":
        return [ops["pc10"] & isa.PC_MASK]
    if mnem in ("jz", "jnz"):
        return sorted({ops["pc10"] & isa.PC_MASK, fall})
    if mnem == "call":
        return sorted({ops["pc10"] & isa.PC_MASK, fall})
    if mnem == "jev":
        # The register target is data-dependent.  Only the fallthrough edge is
        # statically certain; indirect targets are diagnosed in the executor.
        return [fall]
    if mnem in ("ret", "halt"):
        return []
    return [fall]


def build_cfg(image, entry=0):
    """Build the reachable basic-block CFG for a concrete entry address.

    Returns ``(blocks, block_by_start)``.  The verifier itself works at
    instruction granularity; this CFG is provided for inspection, tooling and
    tests, and as the structural representation required by U9.
    """
    entry = resolve_address(image, entry, "CFG entry") if not isinstance(entry, int) else entry & isa.PC_MASK
    leaders = {entry}
    for pc in range(1 << isa.PC_W):
        mnem, ops, _ = image.decode_slot(pc)
        succ = _static_successors(mnem, ops, pc)
        if len(succ) != 1 or succ[0] != ((pc + 1) & isa.PC_MASK):
            leaders.update(succ)
            if mnem not in ("jmp", "ret", "halt"):
                leaders.add((pc + 1) & isa.PC_MASK)

    blocks = []
    by_start = {}
    work = [entry]
    seen_starts = set()
    while work:
        start = work.pop() & isa.PC_MASK
        if start in seen_starts:
            continue
        seen_starts.add(start)
        pcs = []
        pc = start
        visited_in_block = set()
        while True:
            if pc in visited_in_block:
                break
            visited_in_block.add(pc)
            pcs.append(pc)
            mnem, ops, _ = image.decode_slot(pc)
            succ = _static_successors(mnem, ops, pc)
            fall = (pc + 1) & isa.PC_MASK
            control = mnem in ("jmp", "jz", "jnz", "jev", "call", "ret", "halt")
            if control or fall in leaders or fall == start or len(pcs) >= (1 << isa.PC_W):
                break
            pc = fall
        last = pcs[-1]
        mnem, ops, _ = image.decode_slot(last)
        succ = _static_successors(mnem, ops, last)
        block = BasicBlock(start, pcs, succ)
        blocks.append(block)
        by_start[start] = block
        for s in succ:
            if s not in seen_starts:
                work.append(s)
    return blocks, by_start


def format_cfg(image, entry=0):
    blocks, _ = build_cfg(image, entry)
    inv_labels = defaultdict(list)
    for name, pc in image.labels.items():
        inv_labels[pc].append(name)
    lines = []
    for b in sorted(blocks, key=lambda x: x.start):
        label = f" ({', '.join(sorted(inv_labels[b.start]))})" if inv_labels[b.start] else ""
        succ = ",".join(f"{s:03x}" for s in b.successors) or "-"
        lines.append(f"bb_{b.start:03x}{label}: pcs {b.pcs[0]:03x}..{b.end:03x} -> {succ}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Timing profiles
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TimingProfile:
    name: str
    imem_miss_min: int
    imem_miss_max: int
    dmem_read_min: int
    dmem_read_max: int
    dmem_write_min: int
    dmem_write_max: int
    ld_imem_min: int
    ld_imem_max: int
    description: str


# See the long provenance note at the top.  These ranges are the latency
# envelope of a retired instruction/operation, not merely the internal SRAM
# rvalid pipeline depth.
TIMING_PROFILES = {
    # Production-conservative RTL derivation:
    #   SRAM0 miss total = 1 cycle first request alignment + G grant delay
    #   + 4 cycles registered response/consume, G=0..4 for five clients.
    #   SRAM1 read extra = G + 4, G=0..5 for six clients.
    #   SRAM1 write extra = G + 1, because MEM_DWRITE retires one edge after
    #   acceptance (hamming_core.v lines 348-350).
    #   LD_IMEM extra = G + 5 because MEM_IREAD first captures rvalid data and
    #   commits it one edge later (hamming_core.v lines 366-375).
    "exact-rtl": TimingProfile(
        "exact-rtl", 5, 9, 4, 9, 1, 6, 5, 9,
        "production RTL registered-rvalid worst envelopes"),
    # Checked-in project contract values from tools/hamming_isa.py lines
    # 63-66.  Kept for cross-checking and for deciding whether the RTL or the
    # contract needs adjustment.
    # ORCHESTRATOR-CHECK: these are one cycle shorter than the conservative
    # production read/fetch interpretation above in several places.
    "isa-contract": TimingProfile(
        "isa-contract", isa.IMEM_MISS_MIN, isa.IMEM_MISS_MAX,
        isa.DMEM_RD_MIN, isa.DMEM_RD_MAX,
        isa.DMEM_WR_MIN, isa.DMEM_WR_MAX,
        isa.IMEM_MISS_MIN, isa.IMEM_MISS_MAX,
        "single-source ISA/KTD2 latency contract"),
    # Fixed Golden-ISS regression defaults.  Useful for exact small-unit tests
    # and for comparing a witness with tools/hamming_iss.py.
    "iss-default": TimingProfile(
        "iss-default", isa.IMEM_MISS_DFLT, isa.IMEM_MISS_DFLT,
        isa.DMEM_RD_DFLT, isa.DMEM_RD_DFLT,
        isa.DMEM_WR_DFLT, isa.DMEM_WR_DFLT,
        isa.IMEM_MISS_DFLT, isa.IMEM_MISS_DFLT,
        "fixed Golden ISS default latencies"),
}


@dataclass
class TimingConfig:
    profile: str = "exact-rtl"
    core_id: int = 0
    event_model: str = "volatile"       # volatile | or | internal
    merge_states: bool = True
    max_steps: int = 0                  # 0 = enough for budget+1 retired instrs
    max_states: int = 512
    solver_timeout_ms: int = 10000
    optimize_timeout_ms: int = 10000
    compute_max: bool = True
    input_latency: int = 0              # pin-to-pin endpoint adjustment only
    output_latency: int = 0
    trace_limit: int = 300

    def timing_profile(self):
        try:
            return TIMING_PROFILES[self.profile]
        except KeyError:
            choices = ", ".join(sorted(TIMING_PROFILES))
            raise TimingError(f"unknown timing profile {self.profile!r}; choices: {choices}")


# ---------------------------------------------------------------------------
# Symbolic execution state and traces
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class InputUse:
    name: str
    expr: object
    description: str


@dataclass(frozen=True)
class Observation:
    space: str
    address: object
    value: object
    description: str


@dataclass(frozen=True)
class StepRecord:
    pc: int
    mnemonic: str
    cumulative_cost: object
    detail: str
    inputs: tuple = ()
    observations: tuple = ()


@dataclass(frozen=True)
class MergeRecord:
    selector: object
    first: tuple
    second: tuple


@dataclass
class ExecState:
    pc: int
    regs: tuple
    z: object
    c: object
    eq: object
    anchor_off: object       # Int: initial_timer + anchor_off == anchor (mod 2^20)
    deadline_missed: object
    core_out: object
    core_oe: object
    ev_state: object
    csr: object
    dmem: object
    cache: tuple             # 8 entries: concrete SRAM0 word address or None
    call_depth: int
    call0: int
    call1: int
    cost: object             # Int elapsed cycles since the START cycle boundary
    steps: int
    path: object
    trace: tuple
    halted: bool = False
    unsupported: str = ""

    def key(self):
        return (self.pc, self.cache, self.call_depth, self.call0, self.call1,
                self.steps, self.halted, self.unsupported)


@dataclass
class Witness:
    kind: str
    cost: object
    inputs: OrderedDict
    trace_lines: list
    note: str = ""


@dataclass
class CheckResult:
    status: str              # PROVED | VIOLATION | UNKNOWN | ERROR
    start: int
    end: int
    budget: int
    core_budget: int
    max_cost: object = None
    max_total_cost: object = None
    witness: Witness = None
    message: str = ""
    stats: dict = field(default_factory=dict)
    assumptions: list = field(default_factory=list)
    smt2_path: str = ""

    @property
    def proved(self):
        return self.status == "PROVED"

    def to_dict(self):
        return {
            "status": self.status,
            "start": self.start,
            "end": self.end,
            "budget": self.budget,
            "core_budget": self.core_budget,
            "max_cost": self.max_cost,
            "max_total_cost": self.max_total_cost,
            "message": self.message,
            "stats": dict(self.stats),
            "assumptions": list(self.assumptions),
            "smt2_path": self.smt2_path,
            "witness": None if self.witness is None else {
                "kind": self.witness.kind,
                "cost": self.witness.cost,
                "inputs": dict(self.witness.inputs),
                "trace": list(self.witness.trace_lines),
                "note": self.witness.note,
            },
        }


# ---------------------------------------------------------------------------
# Constraint language
# ---------------------------------------------------------------------------

class ConstraintCompiler:
    """Small safe parser for assumptions such as ``1 <= r0 <= 3``.

    Names: r0..r3, timer0, ev0, core_out0, core_oe0, z0, c0, eq0,
    deadline_missed0, anchor_delta0, plus all firmware labels.  Arrays:
    ``dmem[byte_addr]`` (8-bit), ``csr[addr]`` (16-bit), and ``imem[slot]``
    (16-bit).  Comparisons on bit-vectors are unsigned, matching the machine.
    """

    def __init__(self, symbols, arrays, labels):
        self.symbols = dict(symbols)
        self.arrays = dict(arrays)
        self.labels = dict(labels)

    def compile(self, text):
        try:
            tree = ast.parse(text, mode="eval")
        except SyntaxError as e:
            raise TimingError(f"bad assumption {text!r}: {e}")
        return self._visit(tree.body, text)

    def _visit(self, node, text):
        require_z3()
        if isinstance(node, ast.Constant):
            if isinstance(node.value, bool):
                return z3.BoolVal(node.value)
            if isinstance(node.value, int):
                return node.value
            raise TimingError(f"unsupported constant in assumption {text!r}")
        if isinstance(node, ast.Name):
            if node.id in self.symbols:
                return self.symbols[node.id]
            if node.id in self.labels:
                return self.labels[node.id]
            raise TimingError(f"unknown name {node.id!r} in assumption {text!r}")
        if isinstance(node, ast.BoolOp):
            vals = [self._bool(self._visit(v, text), text) for v in node.values]
            return z3.And(*vals) if isinstance(node.op, ast.And) else z3.Or(*vals)
        if isinstance(node, ast.UnaryOp):
            val = self._visit(node.operand, text)
            if isinstance(node.op, ast.Not):
                return z3.Not(self._bool(val, text))
            if isinstance(node.op, ast.USub):
                return -val
            if isinstance(node.op, ast.UAdd):
                return val
            if isinstance(node.op, ast.Invert):
                return ~self._as_bv(val, text=text)
            raise TimingError(f"unsupported unary operator in assumption {text!r}")
        if isinstance(node, ast.BinOp):
            return self._binop(node, text)
        if isinstance(node, ast.Compare):
            return self._compare(node, text)
        if isinstance(node, ast.Subscript):
            return self._subscript(node, text)
        if isinstance(node, ast.ParenExpr if hasattr(ast, "ParenExpr") else ast.Expr):
            return self._visit(node.value, text)
        raise TimingError(f"unsupported syntax {type(node).__name__} in assumption {text!r}")

    def _sort_is_bool(self, v):
        return z3.is_expr(v) and v.sort() == z3.BoolSort()

    def _sort_is_bv(self, v):
        return z3.is_expr(v) and z3.is_bv(v)

    def _bool(self, v, text):
        if not self._sort_is_bool(v):
            raise TimingError(f"expected Boolean expression in assumption {text!r}")
        return v

    def _as_bv(self, v, width=None, text="<constraint>"):
        if self._sort_is_bv(v):
            if width is not None and v.size() != width:
                raise TimingError(f"bit-vector width mismatch in assumption {text!r}")
            return v
        if isinstance(v, int):
            if width is None:
                width = 16
            return z3.BitVecVal(v, width)
        raise TimingError(f"expected bit-vector/integer value in assumption {text!r}")

    def _coerce_pair(self, a, b, text):
        if self._sort_is_bv(a) or self._sort_is_bv(b):
            width = a.size() if self._sort_is_bv(a) else b.size()
            aa = self._as_bv(a, width, text)
            bb = self._as_bv(b, width, text)
            return aa, bb, "bv"
        if isinstance(a, int) and isinstance(b, int):
            return a, b, "int"
        return a, b, "intexpr"

    def _binop(self, node, text):
        a0 = self._visit(node.left, text)
        b0 = self._visit(node.right, text)
        a, b, kind = self._coerce_pair(a0, b0, text)
        op = node.op
        if kind == "bv":
            if isinstance(op, ast.Add):
                return a + b
            if isinstance(op, ast.Sub):
                return a - b
            if isinstance(op, ast.Mult):
                return a * b
            if isinstance(op, ast.FloorDiv):
                return z3.UDiv(a, b)
            if isinstance(op, ast.Mod):
                return z3.URem(a, b)
            if isinstance(op, ast.BitAnd):
                return a & b
            if isinstance(op, ast.BitOr):
                return a | b
            if isinstance(op, ast.BitXor):
                return a ^ b
            if isinstance(op, ast.LShift):
                return a << b
            if isinstance(op, ast.RShift):
                return z3.LShR(a, b)
        else:
            if isinstance(op, ast.Add):
                return a + b
            if isinstance(op, ast.Sub):
                return a - b
            if isinstance(op, ast.Mult):
                return a * b
            if isinstance(op, ast.FloorDiv):
                return a // b
            if isinstance(op, ast.Mod):
                return a % b
        raise TimingError(f"unsupported arithmetic operator in assumption {text!r}")

    def _compare(self, node, text):
        if len(node.ops) != 1:
            # z3 cannot chain directly; rewrite as a conjunction.
            parts = []
            left = self._visit(node.left, text)
            for op, comp in zip(node.ops, node.comparators):
                right = self._visit(comp, text)
                parts.append(self._compare_one(left, op, right, text))
                left = right
            return z3.And(*parts)
        left = self._visit(node.left, text)
        right = self._visit(node.comparators[0], text)
        return self._compare_one(left, node.ops[0], right, text)

    def _compare_one(self, a0, op, b0, text):
        if self._sort_is_bool(a0) or self._sort_is_bool(b0):
            if not isinstance(op, (ast.Eq, ast.NotEq)):
                raise TimingError(f"Boolean operands only support ==/!= in {text!r}")
            a = self._bool(a0, text)
            b = self._bool(b0, text)
            return (a == b) if isinstance(op, ast.Eq) else (a != b)
        a, b, kind = self._coerce_pair(a0, b0, text)
        if isinstance(op, ast.Eq):
            return a == b
        if isinstance(op, ast.NotEq):
            return a != b
        if kind == "bv":
            if isinstance(op, ast.Lt):
                return z3.ULT(a, b)
            if isinstance(op, ast.LtE):
                return z3.ULE(a, b)
            if isinstance(op, ast.Gt):
                return z3.UGT(a, b)
            if isinstance(op, ast.GtE):
                return z3.UGE(a, b)
        else:
            if isinstance(op, ast.Lt):
                return a < b
            if isinstance(op, ast.LtE):
                return a <= b
            if isinstance(op, ast.Gt):
                return a > b
            if isinstance(op, ast.GtE):
                return a >= b
        raise TimingError(f"unsupported comparison in assumption {text!r}")

    def _subscript(self, node, text):
        if not isinstance(node.value, ast.Name) or node.value.id not in self.arrays:
            raise TimingError(f"only dmem[...], csr[...] and imem[...] subscripts are supported in {text!r}")
        name = node.value.id
        arr, idx_width = self.arrays[name]
        idx0 = self._visit(node.slice, text)
        idx = self._as_bv(idx0, idx_width, text)
        return z3.Select(arr, idx)


# ---------------------------------------------------------------------------
# Analyzer
# ---------------------------------------------------------------------------

class HammingTimingAnalyzer:
    """Bounded-but-budget-complete symbolic timing analyzer for one image."""

    def __init__(self, image, config=None):
        require_z3()
        self.image = image
        self.config = config or TimingConfig()
        if not 0 <= self.config.core_id < isa.N_CORES:
            raise TimingError(f"core_id must be 0..{isa.N_CORES - 1}")
        if self.config.event_model not in ("volatile", "or", "internal"):
            raise TimingError("event_model must be volatile, or, or internal")
        self.profile = self.config.timing_profile()
        self.fresh_counter = 0
        self.inputs = OrderedDict()
        self.base_constraints = []
        self.merge_count = 0
        self._init_imem_array()

    # ---------------- construction helpers ----------------

    def _init_imem_array(self):
        # SRAM0 code is fixed by the image.  K() gives zero for holes; explicit
        # stores cover nonzero instruction/table slots.  LD_IMEM uses slot index
        # (16-bit halfword index), matching hamming_core.v lines 253-255 and
        # 366-375.
        dom = z3.BitVecSort(16)
        arr = z3.K(dom, z3.BitVecVal(0, 16))
        for wi, w32 in enumerate(self.image.words):
            for half in range(2):
                val = (w32 >> (16 * half)) & 0xFFFF
                if val:
                    arr = z3.Store(arr, z3.BitVecVal(2 * wi + half, 16),
                                   z3.BitVecVal(val, 16))
        self.imem_array = arr

    def _fresh(self, prefix, kind="int"):
        self.fresh_counter += 1
        name = f"{prefix}_{self.fresh_counter}"
        if kind == "int":
            return z3.Int(name)
        if kind == "bool":
            return z3.Bool(name)
        if kind.startswith("bv"):
            return z3.BitVec(name, int(kind[2:]))
        raise AssertionError(kind)

    def _int(self, value):
        return value if z3.is_expr(value) else z3.IntVal(int(value))

    def _bool1(self, b):
        return z3.If(b, z3.BitVecVal(1, 1), z3.BitVecVal(0, 1))

    def _bv20_from_int(self, value):
        return z3.Int2BV(self._int(value), isa.TIMER_W)

    def _and_path(self, path, cond):
        sc = z3.simplify(cond)
        if z3.is_true(sc):
            return path
        if z3.is_false(sc):
            return z3.BoolVal(False)
        return z3.And(path, sc)

    def _is_definitely(self, expr, value):
        s = z3.simplify(expr)
        return z3.is_true(s) if value else z3.is_false(s)

    def _latency(self, what, lo, hi, description):
        """Return (latency-expr, validity-constraint, input-use tuple)."""
        if lo == hi:
            return z3.IntVal(lo), z3.BoolVal(True), ()
        v = self._fresh(what, "int")
        return v, z3.And(lo <= v, v <= hi), (InputUse(
            str(v), v, f"{description} service latency in [{lo},{hi}] cycles"),)

    def _fresh_input_bv(self, prefix, width, description):
        v = self._fresh(prefix, f"bv{width}")
        return v, (InputUse(str(v), v, description),)

    def _make_cache(self, prewarm):
        cache = [None] * isa.ICACHE_LINES
        for lo_tok, hi_tok in prewarm or ():
            lo = resolve_address(self.image, lo_tok, "prewarm start")
            hi = resolve_address(self.image, hi_tok, "prewarm end")
            if hi < lo:
                raise TimingError(f"prewarm range {lo_tok!r}..{hi_tok!r} is reversed")
            for pc in range(lo, hi + 1):
                waddr = (pc & isa.PC_MASK) >> 1
                cache[waddr & (isa.ICACHE_LINES - 1)] = waddr
        return tuple(cache)

    def _initial_state(self, start, prewarm):
        regs = tuple(z3.BitVec(f"r{i}_init", 16) for i in range(4))
        for i, r in enumerate(regs):
            self.inputs[f"r{i}"] = r
        timer0 = z3.BitVec("timer0", isa.TIMER_W)
        anchor_delta0 = z3.Int("anchor_delta0")
        ev0 = z3.BitVec("ev0", 8)
        core_out0 = z3.BitVec("core_out0", 4)
        core_oe0 = z3.BitVec("core_oe0", 4)
        z0 = z3.Bool("z0")
        c0 = z3.Bool("c0")
        eq0 = z3.Bool("eq0")
        dm0 = z3.Bool("deadline_missed0")
        self.inputs.update(OrderedDict((
            ("timer0", timer0), ("anchor_delta0", anchor_delta0), ("ev0", ev0),
            ("core_out0", core_out0), ("core_oe0", core_oe0), ("z0", z0),
            ("c0", c0), ("eq0", eq0), ("deadline_missed0", dm0),
        )))
        self.timer0 = timer0
        self.anchor_delta0 = anchor_delta0
        self.base_constraints = [0 <= anchor_delta0, anchor_delta0 < TIMER_MODULUS]

        csr0 = z3.Array("csr0", z3.BitVecSort(8), z3.BitVecSort(16))
        dmem0 = z3.Array("dmem0", z3.BitVecSort(16), z3.BitVecSort(8))
        self.csr0 = csr0
        self.dmem0 = dmem0

        self.scalar_symbols = {
            "r0": regs[0], "r1": regs[1], "r2": regs[2], "r3": regs[3],
            "timer0": timer0, "anchor_delta0": anchor_delta0,
            "ev0": ev0, "core_out0": core_out0, "core_oe0": core_oe0,
            "z0": z0, "c0": c0, "eq0": eq0, "deadline_missed0": dm0,
        }
        self.array_symbols = {
            "dmem": (dmem0, 16), "csr": (csr0, 8), "imem": (self.imem_array, 16),
        }

        return ExecState(
            pc=start,
            regs=regs,
            z=z0, c=c0, eq=eq0,
            anchor_off=anchor_delta0,
            deadline_missed=dm0,
            core_out=core_out0,
            core_oe=core_oe0,
            ev_state=ev0,
            csr=csr0,
            dmem=dmem0,
            cache=self._make_cache(prewarm),
            call_depth=0,
            call0=0,
            call1=0,
            cost=z3.IntVal(0),
            steps=0,
            path=z3.BoolVal(True),
            trace=(),
        )

    def compile_assumptions(self, assumptions):
        compiler = ConstraintCompiler(self.scalar_symbols, self.array_symbols,
                                      self.image.labels)
        return [compiler.compile(a) for a in assumptions or ()]

    # ---------------- z3/merge utilities ----------------

    def _ite(self, sel, a, b):
        if a is b:
            return a
        return z3.If(sel, a, b)

    def _merge_two(self, a, b):
        sel = self._fresh("merge_choice", "bool")
        path = z3.Or(z3.And(sel, a.path), z3.And(z3.Not(sel), b.path))
        merged_trace = (MergeRecord(sel, a.trace, b.trace),)
        regs = tuple(self._ite(sel, x, y) for x, y in zip(a.regs, b.regs))
        out = replace(
            a,
            regs=regs,
            z=self._ite(sel, a.z, b.z),
            c=self._ite(sel, a.c, b.c),
            eq=self._ite(sel, a.eq, b.eq),
            anchor_off=self._ite(sel, a.anchor_off, b.anchor_off),
            deadline_missed=self._ite(sel, a.deadline_missed, b.deadline_missed),
            core_out=self._ite(sel, a.core_out, b.core_out),
            core_oe=self._ite(sel, a.core_oe, b.core_oe),
            ev_state=self._ite(sel, a.ev_state, b.ev_state),
            csr=self._ite(sel, a.csr, b.csr),
            dmem=self._ite(sel, a.dmem, b.dmem),
            cost=self._ite(sel, a.cost, b.cost),
            path=path,
            trace=merged_trace,
        )
        self.merge_count += 1
        return out

    def _merge_active(self, states):
        if not self.config.merge_states:
            return states
        groups = defaultdict(list)
        for st in states:
            if self._is_definitely(st.path, False):
                continue
            groups[st.key()].append(st)
        merged = []
        for key in sorted(groups, key=repr):
            group = groups[key]
            cur = group[0]
            for other in group[1:]:
                cur = self._merge_two(cur, other)
            merged.append(cur)
        return merged

    # ---------------- timer and input helpers ----------------

    def _timer_bv_at(self, time_int):
        # The global timer increments once per clock (hamming_csr.v line 298).
        # cost/time_int is an unbounded integer elapsed-cycle count; conversion
        # to 20 bits is the architectural modular value.
        return self.timer0 + self._bv20_from_int(time_int)

    def _event_value(self, st, description):
        if self.config.event_model == "internal":
            return st.ev_state, ()
        ext, rec = self._fresh_input_bv("ev_sample", 8, description)
        if self.config.event_model == "or":
            return (st.ev_state | ext), rec
        return ext, rec

    def _observe_csr(self, st, addr, exec_time):
        """CSR combinational read model. Returns (value,inputuses,observations)."""
        inputs = ()
        observations = ()
        if addr == isa.CSR["EV_FLAGS"]:
            ev, inputs = self._event_value(st, "EV_FLAGS external/component state")
            return z3.ZeroExt(8, ev), inputs, observations
        if addr == isa.CSR["TIMER_LO"]:
            timer = self._timer_bv_at(exec_time)
            return z3.Extract(15, 0, timer), inputs, observations
        if addr == isa.CSR["TIMER_HI"]:
            timer = self._timer_bv_at(exec_time)
            return z3.ZeroExt(12, z3.Extract(19, 16, timer)), inputs, observations

        if isa.CSR_CORE_BANK <= addr < 0x80:
            off = addr & 0xF
            if off == isa.CORE_CSR["CORE_STATUS"]:
                # src/hamming_core.v line 281 packs status as
                # {11'h000, halted, deadline_missed, eq, c, z}.
                return z3.Concat(
                    z3.BitVecVal(0, 11), self._bool1(st.halted),
                    self._bool1(st.deadline_missed), self._bool1(st.eq),
                    self._bool1(st.c), self._bool1(st.z)), inputs, observations
            if off == isa.CORE_CSR["CORE_PC"]:
                return z3.BitVecVal(st.pc, 16), inputs, observations
            if off == isa.CORE_CSR["CORE_OUT"]:
                return z3.ZeroExt(12, st.core_out), inputs, observations
            if off == isa.CORE_CSR["CORE_OE"]:
                return z3.ZeroExt(12, st.core_oe), inputs, observations
            if off == isa.CORE_CSR["CORE_ID"]:
                # hamming_csr.v line 172 returns the reading core index, not
                # the addressed bank number; this is the firmware dispatch trick.
                return z3.BitVecVal(self.config.core_id, 16), inputs, observations
            if off in (isa.CORE_CSR["BCU_CRC_STATE_LO"], isa.CORE_CSR["BCU_CRC_STATE_HI"],
                       isa.CORE_CSR["BCU_RX_DATA"], isa.CORE_CSR["BCU_STAT_SNAP"]):
                v, rec = self._fresh_input_bv("csr_ext", 16,
                                              f"external/read-only CSR 0x{addr:02x} value")
                inputs += rec
                return v, inputs, observations

        key = z3.BitVecVal(addr, 8)
        value = z3.Select(st.csr, key)
        observations += (Observation("csr", key, value, f"csr_rd 0x{addr:02x}"),)
        return value, inputs, observations

    # ---------------- memory helpers ----------------

    def _mem_byte_addr(self, ea):
        low11 = z3.Extract(10, 0, ea)
        return z3.ZeroExt(5, low11)

    def _mem_word_byte_base(self, ea):
        # Word ops use ea[8:0] as the SRAM word number.  The equivalent byte
        # base is (ea[8:0] << 2), always within the 2 KiB bank.
        return z3.ZeroExt(5, z3.Concat(z3.Extract(8, 0, ea), z3.BitVecVal(0, 2)))

    def _inc_bv16(self, addr, n):
        return addr + z3.BitVecVal(n, 16)

    # ---------------- one symbolic instruction step ----------------

    def _step(self, st):
        pc = st.pc
        mnem, ops, raw_word = self.image.decode_slot(pc)
        illegal = mnem is None
        if illegal:
            mnem = ".word"
            ops = {}

        waddr = pc >> 1
        cache_index = waddr & (isa.ICACHE_LINES - 1)
        cache_hit = st.cache[cache_index] == waddr
        cache_after = st.cache
        inputs = ()
        validity = []
        if cache_hit:
            fetch_cost = z3.IntVal(1)
            fetch_desc = "hit"
        else:
            fetch_cost, vc, iu = self._latency(
                "imem_miss", self.profile.imem_miss_min, self.profile.imem_miss_max,
                "SRAM0 instruction-fetch miss")
            validity.append(vc)
            inputs += iu
            lst = list(st.cache)
            lst[cache_index] = waddr
            cache_after = tuple(lst)
            fetch_desc = "miss"

        # Execute-time is the final fetch-valid cycle.  A hit executes in the
        # first cycle; an M-cycle miss executes with the timer M-1 cycles after
        # the request cycle.  This is the fetch/execute overlap visible in
        # hamming_core.v lines 118-125 and 234-236.
        exec_time = st.cost + fetch_cost - 1

        regs = list(st.regs)
        zf, cf, eqf = st.z, st.c, st.eq
        anchor = st.anchor_off
        deadline_missed = st.deadline_missed
        core_out, core_oe = st.core_out, st.core_oe
        ev_state = st.ev_state
        csr, dmem = st.csr, st.dmem
        call_depth, call0, call1 = st.call_depth, st.call0, st.call1
        extra_cost = z3.IntVal(0)
        pc_next = (pc + 1) & isa.PC_MASK
        halted = False
        unsupported = ""
        semantic_detail = ""
        branch = None
        observations = ()

        def set_reg(idx, value):
            regs[idx] = value & 0xFFFF if isinstance(value, int) else value

        def set_z_from_bv(value):
            nonlocal zf
            zf = value == z3.BitVecVal(0, 16)

        if not illegal and mnem in isa.ALU_OPS:
            rd, rs = ops["rd"], ops["rs"]
            a = regs[rd]
            b = regs[rs]
            if mnem == "mov":
                res = b
                regs[rd] = res
                set_z_from_bv(res)
            elif mnem == "add":
                res17 = z3.ZeroExt(1, a) + z3.ZeroExt(1, b)
                cf = z3.Extract(16, 16, res17) == z3.BitVecVal(1, 1)
                res = z3.Extract(15, 0, res17)
                regs[rd] = res
                set_z_from_bv(res)
            elif mnem == "sub":
                res17 = z3.ZeroExt(1, a) - z3.ZeroExt(1, b)
                cf = z3.ULT(a, b)
                res = z3.Extract(15, 0, res17)
                regs[rd] = res
                set_z_from_bv(res)
            elif mnem == "and":
                res = a & b
                regs[rd] = res
                set_z_from_bv(res)
            elif mnem == "or":
                res = a | b
                regs[rd] = res
                set_z_from_bv(res)
            elif mnem == "xor":
                res = a ^ b
                regs[rd] = res
                set_z_from_bv(res)
            elif mnem == "shl":
                res = a << z3.BitVecVal(1, 16)
                cf = z3.Extract(15, 15, a) == z3.BitVecVal(1, 1)
                regs[rd] = res
                set_z_from_bv(res)
            elif mnem == "shr":
                res = z3.LShR(a, z3.BitVecVal(1, 16))
                cf = z3.Extract(0, 0, a) == z3.BitVecVal(1, 1)
                regs[rd] = res
                set_z_from_bv(res)
            elif mnem == "cmp":
                res = a - b
                cf = z3.ULT(a, b)
                eqf = a == b
                set_z_from_bv(res)
            elif mnem == "not":
                res = ~b
                regs[rd] = res
                set_z_from_bv(res)
            elif mnem == "inc":
                res = a + z3.BitVecVal(1, 16)
                regs[rd] = res
                set_z_from_bv(res)
            elif mnem == "dec":
                res = a - z3.BitVecVal(1, 16)
                regs[rd] = res
                set_z_from_bv(res)

        elif not illegal and mnem == "ldi8":
            rd = ops["rd"]
            imm = z3.BitVecVal(ops["imm8"], 8)
            old = regs[rd]
            if ops["half"]:
                regs[rd] = z3.Concat(imm, z3.Extract(7, 0, old))
            else:
                regs[rd] = z3.Concat(z3.Extract(15, 8, old), imm)

        elif not illegal and mnem in ("pin_out", "pin_oe"):
            mask = z3.BitVecVal(ops["mask4"], 4)
            val = z3.BitVecVal(ops["val4"], 4)
            if mnem == "pin_out":
                core_out = (core_out & ~mask) | (val & mask)
            else:
                core_oe = (core_oe & ~mask) | (val & mask)

        elif not illegal and mnem in ("pin_in", "lut_read"):
            width_name = "pin" if mnem == "pin_in" else "lut"
            v, rec = self._fresh_input_bv(
                width_name + "_sample", 1,
                f"{mnem} source {ops[width_name]} sampled after synchronizer/LUT")
            inputs += rec
            regs[ops["rd"]] = z3.ZeroExt(15, v)

        elif not illegal and mnem == "wait_phase":
            if "rs" in ops:
                # The RTL zero-extends the 16-bit register and masks to the
                # signed half-range.  Since rN is only 16 bits, the mask is
                # already implied; keeping it documents hamming_core.v:274-276.
                delta = z3.BV2Int(regs[ops["rs"]])
                delta_desc = f"r{ops['rs']}"
            else:
                delta = ops["imm8"]
                delta_desc = str(delta)
            deadline = anchor + delta
            # Exact affine form of signed20(timer - deadline).  timer and
            # anchor share the same initial-timer base, so the initial value
            # cancels.  This is the modular signed comparison in
            # hamming_core.v lines 274-278 and 335-339.
            raw = (exec_time - deadline) % TIMER_MODULUS
            signed = z3.If(raw >= TIMER_HALF, raw - TIMER_MODULUS, raw)
            is_future = signed < 0
            is_past = signed > 0
            extra_cost = z3.If(is_future, -signed, z3.IntVal(0))
            deadline_missed = z3.Or(deadline_missed, is_past)
            semantic_detail = f"wait delta={delta_desc}"

        elif not illegal and mnem == "sync_event":
            timeout = z3.BV2Int(regs[ops["rt"]])
            wake = self._fresh("sync_wait", "int")
            validity.append(z3.And(0 <= wake, wake <= timeout))
            inputs += (InputUse(str(wake), wake,
                                "SYNC_EVENT wake cycle, 0..timeout (external event envelope)"),)
            extra_cost = wake
            # Lines 340-345 of hamming_core.v re-anchor with the pre-edge timer
            # on the release cycle.  In affine time that is execute-time+wake.
            anchor = exec_time + wake
            semantic_detail = f"sync mask=0x{ops['mask4']:x} timeout=r{ops['rt']}"

        elif not illegal and mnem == "set_anchor":
            anchor = exec_time + ops["imm8"]
            semantic_detail = f"anchor=time+{ops['imm8']}"

        elif not illegal and mnem == "read_timer":
            timer = self._timer_bv_at(exec_time)
            if ops["sel"] == isa.READ_TIMER_HI:
                regs[ops["rd"]] = z3.Concat(
                    self._bool1(deadline_missed), z3.BitVecVal(0, 11),
                    z3.Extract(19, 16, timer))
            else:
                regs[ops["rd"]] = z3.Extract(15, 0, timer)

        elif not illegal and mnem in ("bcu_tx", "bcu_rx", "bcu_ctrl", "bcu_stat", "bcu_crc_rd"):
            if mnem == "bcu_rx":
                v, rec = self._fresh_input_bv("bcu_rx", 16, "BCU RX shadow value")
                inputs += rec
                regs[ops["rd"]] = v
            elif mnem == "bcu_stat":
                v, rec = self._fresh_input_bv("bcu_stat", 16, "BCU status value")
                inputs += rec
                regs[ops["rd"]] = v
            elif mnem == "bcu_crc_rd":
                v, rec = self._fresh_input_bv("bcu_crc", 32, "BCU live CRC state")
                inputs += rec
                if ops["sel"] == isa.BCU_CRC_HI:
                    regs[ops["rd"]] = z3.Extract(31, 16, v)
                else:
                    regs[ops["rd"]] = z3.Extract(15, 0, v)
            # bcu_tx/ctrl affect external hardware but have no core stall.

        elif not illegal and mnem in isa.MEM_OPS:
            ea = regs[ops["ra"]] + z3.BitVecVal(ops["imm2"], 16)
            rt = ops["rt"]
            if mnem in ("ld_dmem_w", "ld_dmem_b"):
                extra_cost, vc, iu = self._latency(
                    "dmem_read", self.profile.dmem_read_min, self.profile.dmem_read_max,
                    "SRAM1 read")
                validity.append(vc)
                inputs += iu
            elif mnem in ("st_dmem_w", "st_dmem_b"):
                extra_cost, vc, iu = self._latency(
                    "dmem_write", self.profile.dmem_write_min, self.profile.dmem_write_max,
                    "SRAM1 write grant/retire")
                validity.append(vc)
                inputs += iu
            else:  # ld_imem
                extra_cost, vc, iu = self._latency(
                    "ld_imem", self.profile.ld_imem_min, self.profile.ld_imem_max,
                    "SRAM0 table read")
                validity.append(vc)
                inputs += iu

            if mnem == "ld_dmem_w":
                base = self._mem_word_byte_base(ea)
                lo = z3.Select(dmem, base)
                hi = z3.Select(dmem, self._inc_bv16(base, 1))
                regs[rt] = z3.Concat(hi, lo)
                observations += (Observation("dmem", base, regs[rt], "ld_dmem_w low16"),)
            elif mnem == "st_dmem_w":
                base = self._mem_word_byte_base(ea)
                data = regs[rt]
                dmem = z3.Store(dmem, base, z3.Extract(7, 0, data))
                dmem = z3.Store(dmem, self._inc_bv16(base, 1), z3.Extract(15, 8, data))
                # Core word writes drive upper halves zero with a full mask
                # (hamming_core.v lines 247-262).
                dmem = z3.Store(dmem, self._inc_bv16(base, 2), z3.BitVecVal(0, 8))
                dmem = z3.Store(dmem, self._inc_bv16(base, 3), z3.BitVecVal(0, 8))
            elif mnem == "ld_dmem_b":
                addr = self._mem_byte_addr(ea)
                val = z3.Select(dmem, addr)
                regs[rt] = z3.ZeroExt(8, val)
                observations += (Observation("dmem", addr, val, "ld_dmem_b"),)
            elif mnem == "st_dmem_b":
                addr = self._mem_byte_addr(ea)
                dmem = z3.Store(dmem, addr, z3.Extract(7, 0, regs[rt]))
            elif mnem == "ld_imem":
                idx = z3.ZeroExt(6, z3.Extract(9, 0, ea))
                regs[rt] = z3.Select(self.imem_array, idx)
            semantic_detail = f"{mnem} extra-memory-service"

        elif not illegal and mnem == "csr_rd":
            value, iu, obs = self._observe_csr(st, ops["addr"], exec_time)
            inputs += iu
            observations += obs
            regs[ops["rd"]] = value
            semantic_detail = f"csr_rd 0x{ops['addr']:02x}"

        elif not illegal and mnem == "csr_wr":
            addr = ops["addr"]
            value = regs[ops["rs"]]
            if addr == isa.CSR["EV_SET"]:
                ev_state = ev_state | z3.Extract(7, 0, value)
            elif addr == isa.CSR["EV_CLR"]:
                ev_state = ev_state & ~z3.Extract(7, 0, value)
            else:
                base = isa.CSR_CORE_BANK + 16 * self.config.core_id
                off = addr - base
                # hamming_csr.v write_private lines 213-237 implements only
                # offsets 4,5,6,9,A,B,C.  In particular the live CRC state
                # offsets 7/8 and CORE_ID offset D are not core-writable.
                writable_offsets = {0x4, 0x5, 0x6, 0x9, 0xA, 0xB, 0xC}
                if 0 <= off < 16 and off in writable_offsets:
                    narrowed = value
                    if off == isa.CORE_CSR["BCU_BIT_LEN"]:
                        narrowed = value & z3.BitVecVal(0x1F, 16)
                    elif off == isa.CORE_CSR["BCU_PAT_LEN"]:
                        narrowed = value & z3.BitVecVal(0xF, 16)
                    elif off == isa.CORE_CSR["BCU_DPLL_CFG"]:
                        narrowed = value & z3.BitVecVal(0xFFF3, 16)
                    # ORCHESTRATOR-CHECK: RTL write_private has no cases for
                    # CRC_STATE_LO/HI offsets 7/8, unlike the current ISS.  The
                    # analyzer follows the RTL and drops those writes.
                    csr = z3.Store(csr, z3.BitVecVal(addr, 8), narrowed)
            semantic_detail = f"csr_wr 0x{addr:02x}"

        elif not illegal and mnem == "jmp":
            pc_next = ops["pc10"] & isa.PC_MASK

        elif not illegal and mnem in ("jz", "jnz"):
            cond = zf if mnem == "jz" else z3.Not(zf)
            branch = (cond, ops["pc10"] & isa.PC_MASK, mnem)

        elif not illegal and mnem == "call":
            ret = (pc + 1) & isa.PC_MASK
            if call_depth == 0:
                call0, call_depth = ret, 1
            elif call_depth == 1:
                call1, call_depth = ret, 2
            else:
                # Two-deep stack overflow drops the oldest entry
                # (hamming_core.v lines 531-541).
                call0, call1 = call1, ret
            pc_next = ops["pc10"] & isa.PC_MASK

        elif not illegal and mnem == "ret":
            if call_depth == 0:
                pc_next = 0
            elif call_depth == 1:
                pc_next, call_depth = call0, 0
            else:
                pc_next, call_depth = call1, 1

        elif not illegal and mnem == "jev":
            ev, rec = self._event_value(st, "JEV event observation")
            inputs += rec
            cond = (ev & z3.BitVecVal(ops["mask4"], 8)) != z3.BitVecVal(0, 8)
            target_expr = z3.simplify(regs[ops["rs"]])
            target = None
            if z3.is_bv_value(target_expr):
                target = target_expr.as_unsigned_long() & isa.PC_MASK
            if target is None:
                # Keep the exact fallthrough path executable and report the
                # taken path as unsupported rather than guessing a target.
                unsupported = (f"symbolic JEV target r{ops['rs']} at pc 0x{pc:03x}")
                branch = (cond, None, "jev")
            else:
                branch = (cond, target, "jev")

        elif not illegal and mnem == "halt":
            halted = True

        elif not illegal and mnem == "nop":
            pass

        elif illegal:
            # hamming_core.v only executes the case when instr_legal is true;
            # otherwise the shared ``pc <= pc+1`` still occurs (lines 376-379),
            # so reserved/data words are architectural NOPs.
            semantic_detail = "illegal/reserved word acts as NOP"

        # Compose one or two successor states.  The instruction's total cost is
        # fetch cost plus post-execution stall/service cost.  The affine timer
        # base is exactly this cumulative cost because hamming_csr.v increments
        # the timer once per clock.
        total_cost = st.cost + fetch_cost + extra_cost
        detail = (f"fetch {fetch_desc}; {semantic_detail}".rstrip(" ;"))
        step = StepRecord(pc, mnem, total_cost, detail, tuple(inputs), tuple(observations))

        def make_state(pc_out, path_cond, detail_suffix="", force_unsupported=""):
            nonlocal_step = replace(step, detail=(step.detail + detail_suffix).strip())
            path = st.path
            for vc in validity:
                path = self._and_path(path, vc)
            path = self._and_path(path, path_cond)
            return ExecState(
                pc=pc_out,
                regs=tuple(regs),
                z=zf, c=cf, eq=eqf,
                anchor_off=anchor,
                deadline_missed=deadline_missed,
                core_out=core_out,
                core_oe=core_oe,
                ev_state=ev_state,
                csr=csr,
                dmem=dmem,
                cache=cache_after,
                call_depth=call_depth,
                call0=call0,
                call1=call1,
                cost=total_cost,
                steps=st.steps + 1,
                path=path,
                trace=st.trace + (nonlocal_step,),
                halted=halted,
                unsupported=force_unsupported,
            )

        if branch is None or halted:
            return [make_state(pc_next, z3.BoolVal(True))]

        cond, target, bname = branch
        out = []
        scond = z3.simplify(cond)
        fall = (pc + 1) & isa.PC_MASK
        if target is None:
            # Symbolic JEV: fallthrough is known; the taken side is unsupported.
            if not z3.is_true(scond):
                out.append(make_state(fall, z3.Not(cond), f"; {bname} not-taken"))
            taken_state = make_state(pc_next, cond, f"; {bname} taken-target-unknown",
                                     force_unsupported=unsupported)
            if not z3.is_false(scond):
                out.append(taken_state)
            return out

        if not z3.is_false(scond):
            out.append(make_state(target, cond, f"; {bname} taken"))
        if not z3.is_true(scond):
            out.append(make_state(fall, z3.Not(cond), f"; {bname} not-taken"))
        return out

    # ---------------- exploration ----------------

    def _explore(self, start, end, core_budget, max_steps):
        initial = self._active_initial
        completed = []
        exceeded = []
        halted = []
        unsupported = []
        incomplete = []
        resource_exhausted = False

        if start == end:
            if core_budget >= 0:
                completed.append(initial)
            else:
                exceeded.append(initial)
            return completed, exceeded, halted, unsupported, incomplete, resource_exhausted

        active = [initial]
        steps_run = 0
        for _depth in range(max_steps):
            if not active:
                break
            steps_run += 1
            next_active = []
            for st in active:
                for ns in self._step(st):
                    over = ns.cost > core_budget
                    # Every retired instruction costs at least one cycle.  Once
                    # more than floor(budget) instructions have executed, the
                    # budget is necessarily crossed even if the exact latency
                    # expression is still symbolic.
                    definitely_over = self._is_definitely(over, True) or ns.steps > core_budget
                    definitely_not_over = (not definitely_over) and self._is_definitely(over, False)

                    def classify_compliant(cs):
                        if cs.unsupported:
                            unsupported.append(cs)
                        elif cs.halted:
                            halted.append(cs)
                        elif cs.pc == end:
                            completed.append(cs)
                        else:
                            next_active.append(cs)

                    if definitely_over:
                        exceeded.append(ns)
                    elif definitely_not_over:
                        classify_compliant(ns)
                    else:
                        over_state = replace(ns, path=self._and_path(ns.path, over))
                        exceeded.append(over_state)
                        cont_state = replace(ns, path=self._and_path(ns.path, z3.Not(over)))
                        classify_compliant(cont_state)

            merged = self._merge_active(next_active)
            if len(merged) > self.config.max_states:
                resource_exhausted = True
                incomplete.extend(merged)
                active = []
                break
            active = merged

        if active:
            incomplete.extend(active)
        return completed, exceeded, halted, unsupported, incomplete, resource_exhausted

    # ---------------- model/witness formatting ----------------

    def _flatten_trace(self, trace, model, limit):
        out = []

        def rec_walk(entries):
            for ent in entries:
                if len(out) >= limit:
                    return
                if isinstance(ent, MergeRecord):
                    chose_first = z3.is_true(model.eval(ent.selector, model_completion=True))
                    rec_walk(ent.first if chose_first else ent.second)
                else:
                    out.append(ent)

        rec_walk(trace)
        truncated = len(out) >= limit
        return out, truncated

    def _model_value_text(self, value):
        if z3.is_bv_value(value):
            return f"0x{value.as_unsigned_long():0{max(1, (value.size() + 3) // 4)}x}"
        if z3.is_int_value(value):
            return str(value.as_long())
        if z3.is_true(value):
            return "true"
        if z3.is_false(value):
            return "false"
        return str(value)

    def _eval_model(self, model, expr):
        try:
            return self._model_value_text(model.eval(expr, model_completion=True))
        except Exception:
            return "<unevaluated>"

    def _make_witness(self, st, model, kind, note=""):
        inputs = OrderedDict()
        for name, expr in self.inputs.items():
            inputs[name] = self._eval_model(model, expr)
        records, truncated = self._flatten_trace(st.trace, model, self.config.trace_limit)
        trace_lines = []
        dynamic_index = 0
        for rec in records:
            cost = self._eval_model(model, rec.cumulative_cost)
            line = f"pc=0x{rec.pc:03x} {rec.mnemonic:<12} cumulative_cost={cost} ; {rec.detail}"
            trace_lines.append(line)
            for iu in rec.inputs:
                if iu.name.startswith("merge_choice"):
                    continue
                key = f"input[{dynamic_index}] {iu.name} ({iu.description})"
                inputs[key] = self._eval_model(model, iu.expr)
                dynamic_index += 1
            for ob in rec.observations:
                addr = self._eval_model(model, ob.address)
                val = self._eval_model(model, ob.value)
                inputs[f"observation {ob.description} [{addr}]"] = val
        if truncated:
            trace_lines.append(f"... trace truncated at {self.config.trace_limit} records ...")
        if not trace_lines:
            trace_lines.append("<empty path: START equals END>")
        return Witness(kind, self._eval_model(model, st.cost), inputs, trace_lines, note)

    # ---------------- solver/query ----------------

    def _solver(self):
        s = z3.Solver()
        if self.config.solver_timeout_ms:
            s.set("timeout", int(self.config.solver_timeout_ms))
        s.add(*self.base_constraints)
        return s

    def _write_smt2(self, path, solver, status):
        try:
            try:
                text = solver.to_smt2()
            except AttributeError:
                text = solver.sexpr()
            Path(path).write_text(f"; hamming_z3_timing query, result {status}\n" + text)
            return str(path)
        except Exception as e:
            raise TimingError(f"could not write SMT2 file {path}: {e}")

    def check(self, start, end, budget, assumptions=(), prewarm=(), smt2_path=None):
        """Check START->END completion within budget.

        ``budget`` includes optional configured endpoint latencies.  Returned
        ``max_cost`` is the greatest core-only completion cost found when the
        proof is complete; ``max_total_cost`` adds endpoint constants.
        """
        require_z3()
        if budget < 0:
            raise TimingError("budget must be non-negative")
        start_pc = resolve_address(self.image, start, "start")
        end_pc = resolve_address(self.image, end, "end")
        endpoint_adjust = int(self.config.input_latency) + int(self.config.output_latency)
        core_budget = int(budget) - endpoint_adjust

        # Initial state construction also creates the symbols used by the
        # assumption compiler, so it must happen before compiling assumptions.
        self.fresh_counter = 0
        self.inputs = OrderedDict()
        self.merge_count = 0
        self._active_initial = self._initial_state(start_pc, prewarm)
        assume_exprs = self.compile_assumptions(assumptions)

        required_steps = int(math.floor(core_budget)) + 1 if core_budget >= 0 else 0
        max_steps = self.config.max_steps or max(1, required_steps)
        explore_truncated = bool(self.config.max_steps and self.config.max_steps < required_steps)
        if explore_truncated:
            max_steps = self.config.max_steps

        completed, exceeded, halted, unsupported, incomplete, exhausted = self._explore(
            start_pc, end_pc, core_budget, max_steps)

        stats = {
            "completed_states": len(completed),
            "budget_exceeded_states": len(exceeded),
            "halted_before_end_states": len(halted),
            "unsupported_states": len(unsupported),
            "incomplete_states": len(incomplete),
            "merge_count": self.merge_count,
            "max_steps": max_steps,
            "required_steps_for_budget": required_steps,
            "profile": self.profile.name,
        }

        assumptions_note = [
            "no host SRAM0 write/cache invalidation during the checked fragment",
            "SRAM service choices are independent bounded round-robin envelopes",
            f"event model: {self.config.event_model}",
        ]
        if endpoint_adjust:
            assumptions_note.append(
                f"endpoint constants included: input={self.config.input_latency}, output={self.config.output_latency}")

        # Vacuous assumptions are not a proof.  Check them before any query.
        base_solver = self._solver()
        base_solver.add(*assume_exprs)
        base_sat = base_solver.check()
        if base_sat == z3.unsat:
            return CheckResult("UNKNOWN", start_pc, end_pc, budget, core_budget,
                               message="assumptions are unsatisfiable; refusing vacuous proof",
                               stats=stats, assumptions=assumptions_note)
        if base_sat == z3.unknown:
            return CheckResult("UNKNOWN", start_pc, end_pc, budget, core_budget,
                               message="solver returned unknown while checking assumptions",
                               stats=stats, assumptions=assumptions_note)

        def with_assumptions():
            s = self._solver()
            s.add(*assume_exprs)
            return s

        violation_states = list(exceeded) + list(halted)
        violation_cond = z3.Or(*[st.path for st in violation_states]) if violation_states else z3.BoolVal(False)
        viol_solver = with_assumptions()
        viol_solver.add(violation_cond)
        viol_res = viol_solver.check() if violation_states else z3.unsat
        smt_written = ""
        if viol_res == z3.sat:
            model = viol_solver.model()
            chosen = violation_states[0]
            for st in violation_states:
                if z3.is_true(model.eval(st.path, model_completion=True)):
                    chosen = st
                    break
            kind = "halt-before-end" if chosen in halted else "budget-exceeded"
            wit = self._make_witness(chosen, model, kind)
            if smt2_path:
                smt_written = self._write_smt2(smt2_path, viol_solver, "sat")
            return CheckResult(
                "VIOLATION", start_pc, end_pc, budget, core_budget,
                witness=wit,
                message=("path crosses the cycle budget before END" if kind == "budget-exceeded"
                         else "path reaches HALT before END"),
                stats=stats, assumptions=assumptions_note, smt2_path=smt_written)
        if viol_res == z3.unknown:
            if smt2_path:
                smt_written = self._write_smt2(smt2_path, viol_solver, "unknown")
            return CheckResult("UNKNOWN", start_pc, end_pc, budget, core_budget,
                               message="solver returned unknown on the violation query",
                               stats=stats, assumptions=assumptions_note,
                               smt2_path=smt_written)

        # No compliant proof if a feasible unsupported/incomplete/resource-truncated
        # behavior remains.  Such states cannot silently pass per KTD10.
        unknown_states = list(unsupported) + list(incomplete)
        if explore_truncated and not incomplete:
            # If exploration ended naturally before the cap there are no active
            # states; otherwise the active list above already captures the risk.
            pass
        unknown_cond = z3.Or(*[st.path for st in unknown_states]) if unknown_states else z3.BoolVal(False)
        unknown_solver = with_assumptions()
        unknown_solver.add(unknown_cond)
        unknown_res = unknown_solver.check() if unknown_states else z3.unsat
        if unknown_res == z3.sat or explore_truncated or exhausted:
            reason = "unsupported or resource-truncated path remains"
            if unsupported:
                reason = unsupported[0].unsupported or reason
            elif explore_truncated:
                reason = (f"max_steps={max_steps} is below the budget-completeness "
                          f"requirement {required_steps}")
            elif exhausted:
                reason = f"active state count exceeded max_states={self.config.max_states}"
            wit = None
            if unknown_res == z3.sat and unknown_states:
                model = unknown_solver.model()
                chosen = unknown_states[0]
                for st in unknown_states:
                    if z3.is_true(model.eval(st.path, model_completion=True)):
                        chosen = st
                        break
                wit = self._make_witness(chosen, model, "unknown", reason)
            return CheckResult("UNKNOWN", start_pc, end_pc, budget, core_budget,
                               witness=wit, message=reason, stats=stats,
                               assumptions=assumptions_note)
        if unknown_res == z3.unknown:
            return CheckResult("UNKNOWN", start_pc, end_pc, budget, core_budget,
                               message="solver returned unknown on the completeness query",
                               stats=stats, assumptions=assumptions_note)

        # Coverage check: every otherwise-valid input must be represented by a
        # completed within-budget state.  This catches bookkeeping mistakes and
        # unmodeled control-flow loss instead of passing them.
        completion_cond = z3.Or(*[st.path for st in completed]) if completed else z3.BoolVal(False)
        cover_solver = with_assumptions()
        cover_solver.add(z3.Not(completion_cond))
        cover_res = cover_solver.check()
        if cover_res == z3.sat:
            return CheckResult("UNKNOWN", start_pc, end_pc, budget, core_budget,
                               message="an input has no accounted completed path; refusing proof",
                               stats=stats, assumptions=assumptions_note)
        if cover_res == z3.unknown:
            return CheckResult("UNKNOWN", start_pc, end_pc, budget, core_budget,
                               message="solver returned unknown on the coverage query",
                               stats=stats, assumptions=assumptions_note)

        max_cost = None
        max_total = None
        if self.config.compute_max and completed:
            opt = z3.Optimize()
            if self.config.optimize_timeout_ms:
                opt.set("timeout", int(self.config.optimize_timeout_ms))
            opt.add(*self.base_constraints)
            opt.add(*assume_exprs)
            obj = z3.Int("wcet_objective")
            opt.add(z3.Or(*[z3.And(st.path, obj == st.cost) for st in completed]))
            opt.maximize(obj)
            opt_res = opt.check()
            if opt_res == z3.sat:
                val = opt.model().eval(obj, model_completion=True)
                if z3.is_int_value(val):
                    max_cost = val.as_long()
                    max_total = max_cost + endpoint_adjust
                else:
                    max_cost = str(val)
                    max_total = str(val)

        return CheckResult(
            "PROVED", start_pc, end_pc, budget, core_budget,
            max_cost=max_cost, max_total_cost=max_total,
            message="all explored inputs reach END within budget",
            stats=stats, assumptions=assumptions_note)


# ---------------------------------------------------------------------------
# CLI and batch mode
# ---------------------------------------------------------------------------

def _result_line(result):
    mx = "-" if result.max_total_cost is None else str(result.max_total_cost)
    return (f"{result.status} start=0x{result.start:03x} end=0x{result.end:03x} "
            f"budget={result.budget} max_total_cost={mx} :: {result.message}")


def print_result(result, as_json=False):
    if as_json:
        print(json.dumps(result.to_dict(), indent=2, sort_keys=True))
        return
    print(_result_line(result))
    if result.assumptions:
        print("explicit analysis assumptions:")
        for a in result.assumptions:
            print(f"  - {a}")
    if result.witness is not None:
        print(f"witness ({result.witness.kind}), cost={result.witness.cost}")
        if result.witness.note:
            print(f"  note: {result.witness.note}")
        print("input conditions / observations:")
        for k, v in result.witness.inputs.items():
            print(f"  {k} = {v}")
        print("path trace:")
        for line in result.witness.trace_lines:
            print(f"  {line}")


def _config_from_args(args, defaults=None):
    d = dict(defaults or {})
    return TimingConfig(
        profile=d.get("profile", args.profile),
        core_id=int(d.get("core", d.get("core_id", args.core))),
        event_model=d.get("event_model", args.event_model),
        merge_states=bool(d.get("merge_states", args.merge_states)),
        max_steps=int(d.get("max_steps", args.max_steps)),
        max_states=int(d.get("max_states", args.max_states)),
        solver_timeout_ms=int(d.get("solver_timeout_ms", args.solver_timeout_ms)),
        optimize_timeout_ms=int(d.get("optimize_timeout_ms", args.optimize_timeout_ms)),
        compute_max=bool(d.get("compute_max", args.compute_max)),
        input_latency=int(d.get("input_latency", args.input_latency)),
        output_latency=int(d.get("output_latency", args.output_latency)),
        trace_limit=int(d.get("trace_limit", args.trace_limit)),
    )


def _normalize_prewarm(raw):
    if not raw:
        return []
    out = []
    for item in raw:
        if isinstance(item, (list, tuple)) and len(item) == 2:
            out.append((item[0], item[1]))
        else:
            raise TimingError(f"prewarm entries must be [start,end] pairs, got {item!r}")
    return out


def run_batch(spec_pathlike, args):
    spec_path = Path(spec_pathlike)
    try:
        spec = json.loads(spec_path.read_text())
    except Exception as e:
        raise TimingError(f"cannot read batch spec {spec_path}: {e}")
    checks = spec.get("checks")
    if not isinstance(checks, list) or not checks:
        raise TimingError("batch spec must contain a non-empty 'checks' list")
    defaults = dict(spec.get("defaults", {}))
    statuses = []
    any_violation = False
    any_nonproof = False
    for idx, chk in enumerate(checks):
        try:
            merged_defaults = dict(defaults)
            # Command-line arguments are the fallback; per-check keys win.
            cfg = _config_from_args(args, {**merged_defaults, **chk})
            fw = chk.get("firmware") or args.firmware
            if not fw:
                raise TimingError(f"batch check {idx}: missing firmware")
            fw_path = Path(fw)
            if not fw_path.is_absolute():
                fw_path = spec_path.parent / fw_path
            image = load_firmware(fw_path)
            analyzer = HammingTimingAnalyzer(image, cfg)
            assumptions = list(chk.get("assume", chk.get("assumptions", [])))
            prewarm = _normalize_prewarm(chk.get("prewarm", []))
            smt2 = chk.get("smt2") or args.smt2
            if smt2 and not Path(smt2).is_absolute():
                smt2 = spec_path.parent / smt2
            res = analyzer.check(chk["start"], chk["end"], int(chk["budget"]),
                                 assumptions=assumptions, prewarm=prewarm,
                                 smt2_path=smt2)
        except Exception as e:
            res = CheckResult("ERROR", 0, 0, 0, 0, message=str(e))
        statuses.append(res.status)
        any_violation |= res.status == "VIOLATION"
        any_nonproof |= res.status != "PROVED"
        print(f"[{idx}] ", end="")
        print_result(res, args.json)
    if any_violation:
        return 2
    return 1 if any_nonproof else 0


def build_arg_parser():
    epilog = """
examples:
  python tools/hamming_z3_timing.py firmware/can20a.hasm --check pb_loop pb_tm 60 \
      --core 3 --assume "r2 >= 1" --assume "r2 <= 32"
  python tools/hamming_z3_timing.py --batch timing_spec.json

Pin-to-pin checks can add the fixed fabric endpoints explicitly, for example
--input-latency 2 --output-latency 1.  Label-to-label firmware checks normally
leave both at zero.  JSON batch fields are documented in the module docstring.
"""
    ap = argparse.ArgumentParser(
        prog="hamming_z3_timing.py",
        description="Z3 static timing verifier for Hamming firmware",
        epilog=epilog,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("firmware", nargs="?", help=".hasm/.hex/.bin firmware image")
    ap.add_argument("--check", nargs=3, metavar=("START", "END", "BUDGET"),
                    help="prove START->END completion within BUDGET cycles")
    ap.add_argument("--batch", help="JSON batch spec file")
    ap.add_argument("--assume", action="append", default=[],
                    help="assumption expression (repeatable), e.g. '1 <= r0 <= 3'")
    ap.add_argument("--prewarm", nargs=2, action="append", default=[],
                    metavar=("START", "END"),
                    help="inclusive instruction-slot range initially resident in loop cache")
    ap.add_argument("--core", type=int, default=0, help="core id for CORE_ID/private CSR semantics")
    ap.add_argument("--profile", choices=sorted(TIMING_PROFILES), default="exact-rtl",
                    help="timing latency profile")
    ap.add_argument("--event-model", choices=("volatile", "or", "internal"), default="volatile",
                    help="external EV_FLAGS observation model")
    ap.add_argument("--max-steps", type=int, default=0,
                    help="symbolic execution step cap; 0 = budget-complete depth")
    ap.add_argument("--max-states", type=int, default=512,
                    help="active-state resource cap before reporting UNKNOWN")
    ap.add_argument("--solver-timeout-ms", type=int, default=10000)
    ap.add_argument("--optimize-timeout-ms", type=int, default=10000)
    ap.add_argument("--no-compute-max", dest="compute_max", action="store_false",
                    help="skip exact WCET optimization after a proof")
    ap.set_defaults(compute_max=True)
    ap.add_argument("--no-merge", dest="merge_states", action="store_false",
                    help="disable sound ITE state merging")
    ap.set_defaults(merge_states=True)
    ap.add_argument("--input-latency", type=int, default=0,
                    help="constant input endpoint cycles included in the supplied budget")
    ap.add_argument("--output-latency", type=int, default=0,
                    help="constant output endpoint cycles included in the supplied budget")
    ap.add_argument("--trace-limit", type=int, default=300)
    ap.add_argument("--smt2", help="write the SAT/UNKNOWN violation query to this .smt2 file")
    ap.add_argument("--print-cfg", action="store_true",
                    help="print the reachable basic-block CFG from --check START (or address 0)")
    ap.add_argument("--json", action="store_true", help="machine-readable result output")
    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    if z3 is None:
        print("error: z3-solver is required; install z3-solver and rerun", file=sys.stderr)
        return 1
    try:
        if args.batch:
            return run_batch(args.batch, args)
        if not args.firmware:
            raise TimingError("firmware image is required unless --batch is used")
        image = load_firmware(args.firmware)
        if args.print_cfg:
            entry = args.check[0] if args.check else 0
            print(format_cfg(image, entry))
            if not args.check:
                return 0
        if not args.check:
            raise TimingError("no check specified; use --check START END BUDGET or --batch")
        cfg = _config_from_args(args)
        analyzer = HammingTimingAnalyzer(image, cfg)
        prewarm = _normalize_prewarm(args.prewarm)
        result = analyzer.check(args.check[0], args.check[1], int(args.check[2], 0),
                                assumptions=args.assume, prewarm=prewarm,
                                smt2_path=args.smt2)
        print_result(result, args.json)
        if result.status == "PROVED":
            return 0
        if result.status == "VIOLATION":
            return 2
        return 1
    except TimingError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except hamming_asm.AsmError as e:
        print(f"assembler error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
