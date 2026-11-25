# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for the Hamming ISA spec, assembler, disassembler and Golden ISS.

Covers: bit-identical round-trip over every opcode/register/immediate
boundary, .lut boolean compilation against exhaustive truth-table eval,
exact WAIT_PHASE/SYNC_EVENT stall cycle counts incl. 20-bit wraparound and
deadline overshoot, encoding-collision freedom across all 16 major opcodes,
assembler rejections (out-of-range branch, >4-input .lut, >2048 B image),
and generated-file freshness for src/hamming_defs.vh + docs/ISA.md.
"""

from pathlib import Path

import pytest

from tools import hamming_asm as asm
from tools import hamming_isa as isa
from tools.hamming_iss import HammingISS

ROOT = Path(__file__).resolve().parents[2]


def slot(src, index=0):
    """Assemble `src` and return the 16-bit slot `index`."""
    words = asm.assemble(src)
    return (words[index >> 1] >> (16 * (index & 1))) & 0xFFFF


# ---------------------------------------------------------------------------
# Encoding round-trip: text -> word -> decode -> canonical text -> word
# ---------------------------------------------------------------------------

def _rt_cases():
    """(case-id, source-line, expected-word) tuples for round-trip tests."""
    cases = []
    # ALU binary ops x reg corners (full cross-product for mov/add/cmp,
    # diagonal coverage for the rest).
    for m in ("mov", "add", "sub", "cmp"):
        for rd in range(4):
            for rs in range(4):
                cases.append((f"{m}-r{rd}-r{rs}", f"{m} r{rd}, r{rs}",
                              isa.enc_alu(isa.ALU_OPS[m], rd, rs)))
    for m in ("and", "or", "xor", "not"):
        for rd, rs in ((0, 0), (1, 3), (2, 1), (3, 2)):
            cases.append((f"{m}-r{rd}-r{rs}", f"{m} r{rd}, r{rs}",
                          isa.enc_alu(isa.ALU_OPS[m], rd, rs)))
    for m in ("shl", "shr", "inc", "dec"):
        for rd in range(4):
            cases.append((f"{m}-r{rd}", f"{m} r{rd}",
                          isa.enc_alu(isa.ALU_OPS[m], rd, 0)))
    # LDI8 halves and immediate boundaries.
    for rd in range(4):
        for half, sfx in ((0, "l"), (1, "h")):
            for imm in (0x00, 0x01, 0x5A, 0xFF):
                cases.append((f"ldi8{sfx}-r{rd}-{imm:02x}",
                              f"ldi8.{sfx} r{rd}, 0x{imm:02X}",
                              isa.enc_ldi8(rd, half, imm)))
    # PIN ops.
    for m, fn in (("pin_out", isa.enc_pin_out), ("pin_oe", isa.enc_pin_oe)):
        for mask, val in ((0, 0), (0xF, 0xF), (0x5, 0xA), (0b0101, 0b1111)):
            cases.append((f"{m}-{mask:x}-{val:x}",
                          f"{m} 0x{mask:X}, 0x{val:X}", fn(mask, val)))
    for m, fn, field in (("pin_in", isa.enc_pin_in, "pin"),
                         ("lut_read", isa.enc_lut_read, "lut")):
        for rd in range(4):
            for src in (0, 7, 15):
                cases.append((f"{m}-r{rd}-{src}",
                              f"{m} r{rd}, {src}",
                              fn(rd, src)))
    # TIMING.
    for rs in range(4):
        cases.append((f"wait-r{rs}", f"wait_phase r{rs}", isa.enc_wait_rs(rs)))
    for imm in (0, 1, 100, 255):
        cases.append((f"wait-{imm:02x}", f"wait_phase {imm}",
                      isa.enc_wait_imm(imm)))
        cases.append((f"anchor-{imm:02x}", f"set_anchor {imm}",
                      isa.enc_set_anchor(imm)))
    for mask in (0, 1, 0xF):
        for rt in range(4):
            cases.append((f"sync-{mask:x}-r{rt}", f"sync_event 0x{mask:X}, r{rt}",
                          isa.enc_sync_event(mask, rt)))
    for rd in range(4):
        for sel, name in ((0, "lo"), (1, "hi")):
            cases.append((f"rdtim-r{rd}-{name}", f"read_timer r{rd}, {name}",
                          isa.enc_read_timer(rd, sel)))
    # BCU.
    for r in range(4):
        cases += [
            (f"bcutx-r{r}", f"bcu_tx r{r}", isa.enc_bcu_tx(r)),
            (f"bcurx-r{r}", f"bcu_rx r{r}", isa.enc_bcu_rx(r)),
            (f"bcustat-r{r}", f"bcu_stat r{r}", isa.enc_bcu_stat(r)),
        ]
        for sel, name in ((0, "lo"), (1, "hi")):
            cases.append((f"bcucrc-r{r}-{name}", f"bcu_crc_rd r{r}, {name}",
                          isa.enc_bcu_crc_rd(r, sel)))
    for imm in (0, 0x55, 0xFF):
        cases.append((f"bcuctrl-{imm:02x}", f"bcu_ctrl 0x{imm:02X}",
                      isa.enc_bcu_ctrl(imm)))
    # MEM.
    for m, sub in isa.MEM_OPS.items():
        for ra in (0, 3):
            for imm2 in (0, 1, 3):
                for rt in (0, 3):
                    cases.append((f"{m}-r{rt}-r{ra}-{imm2}",
                                  f"{m} r{rt}, [r{ra}+{imm2}]",
                                  isa.enc_mem(sub, ra, imm2, rt)))
    # CSR.
    for r in range(4):
        for addr in (0x00, 0x08, 0x42, 0xFF):
            cases.append((f"csrrd-r{r}-{addr:02x}", f"csr_rd r{r}, 0x{addr:02X}",
                          isa.enc_csr_rd(r, addr)))
            cases.append((f"csrwr-r{r}-{addr:02x}", f"csr_wr r{r}, 0x{addr:02X}",
                          isa.enc_csr_wr(r, addr)))
    # Flow.
    for target in (0, 1, 0x2AA, 1023):
        cases.append((f"jmp-{target:03x}", f"jmp {target}", isa.enc_jmp(target)))
        for m in ("jz", "jnz", "call"):
            cases.append((f"{m}-{target:03x}", f"{m} {target}",
                          isa.enc_br(isa.BR_OPS[m], target)))
    for mask in (0, 1, 0xF):
        for rs in range(4):
            cases.append((f"jev-{mask:x}-r{rs}", f"jev 0x{mask:X}, r{rs}",
                          isa.enc_jev(mask, rs)))
    for m in ("ret", "halt", "nop"):
        cases.append((m, m, isa.enc_misc(isa.MISC_OPS[m])))
    return cases


_RT = _rt_cases()


@pytest.mark.parametrize("case_id,text,expected", _RT,
                         ids=[c[0] for c in _RT])
def test_roundtrip_text(case_id, text, expected):
    word = slot(text)
    assert word == expected
    # decode -> canonical text -> re-encode is bit-identical.
    line = asm.disasm_slot(word)
    assert slot(line) == word
    dec = isa.decode(word)
    assert dec is not None
    assert isa.encode(dec[0], **dec[1]) == word


def test_roundtrip_whole_image():
    """asm -> disasm -> asm over a mixed image is bit-identical."""
    src = """
.protocol demo @clk 60MHz
.bcu mode=nrzi, crc=crc5_ref
.constraint 2 ui <= turnaround <= 6.5 ui
start:
    ldi8.l r0, 0x1f
    ldi8.h r0, 0xab
    add r0, r1
    pin_out 0b0101, 0b1111
    pin_oe 0x3, 0x3
    wait_phase @us(2)
    sync_event 0x3, r2
loop:
    ld_dmem_w r1, [r0+2]
    st_dmem_b r1, [r0]
    csr_wr r1, EV_SET
    jnz loop
    jev 0x1, r3
    call start
    ret
    halt
    nop
.word 0xBEEF
.lut se0 = (~dp) & (~dm)
.lut edge = (a ^ b) | c
"""
    words = asm.assemble(src)
    words2 = asm.assemble(asm.disassemble_to_source(words))
    assert words2 == words


def test_roundtrip_arbitrary_word():
    """disasm(.word fallback) round-trips every 16-bit pattern exactly."""
    for value in (0x0000, 0xFFFF, 0xB000, 0xF123, 0xDEAD, 0x55AA):
        assert slot(asm.disasm_slot(value)) == value


# ---------------------------------------------------------------------------
# Encoding collisions: every legal encoding is unique and decodes back
# ---------------------------------------------------------------------------

def test_no_encoding_collisions():
    seen = set()
    count = 0
    for word in isa.encode_space():
        assert word not in seen, f"encoding collision at 0x{word:04X}"
        seen.add(word)
        count += 1
        dec = isa.decode(word)
        assert dec is not None, f"legal word 0x{word:04X} does not decode"
        assert isa.encode(dec[0], **dec[1]) == word, \
            f"re-encode mismatch at 0x{word:04X}"
    assert count > 10_000


def test_all_16_majors_no_collisions():
    """Each major 0x0-0xA has legal encodings; 0xB-0xF are fully reserved."""
    legal_by_major = {m: set() for m in range(16)}
    for word in isa.encode_space():
        legal_by_major[word >> 12].add(word)
    for major in range(0xB):
        assert legal_by_major[major], f"major 0x{major:X} has no encodings"
    for major in range(0xB, 0x10):
        assert not legal_by_major[major]
        for sub in range(0x1000):
            assert isa.decode((major << 12) | sub) is None, \
                f"reserved major 0x{major:X} decodes 0x{sub:03X}"


# ---------------------------------------------------------------------------
# .lut boolean-expression compiler
# ---------------------------------------------------------------------------

_LUT_CASES = [
    ("se0", "(~dp) & (~dm)", ["dp", "dm"],
     lambda i: 1 if not (i & 1) and not (i & 2) else 0, 0x1111),
    ("sda_fall_det", "scl & sda_fall", ["scl", "sda_fall"],
     lambda i: 1 if (i & 1) and (i & 2) else 0, 0x8888),
    ("mix", "(a ^ b) | c", ["a", "b", "c"],
     lambda i: 1 if ((i ^ (i >> 1)) & 1) or (i & 4) else 0, 0xF6F6),
    ("all4", "(a & b) | (c & ~d)", ["a", "b", "c", "d"],
     lambda i: 1 if (i & 3) == 3 or ((i & 4) and not (i & 8)) else 0, 0x88F8),
]


@pytest.mark.parametrize("name,expr,names,fn,tt", _LUT_CASES,
                         ids=[c[0] for c in _LUT_CASES])
def test_lut_compilation(name, expr, names, fn, tt):
    got_tt, got_names = asm.compile_lut(expr)
    assert got_names == names
    assert got_tt == tt
    for idx in range(16):  # exhaustive 16-row verification
        assert ((got_tt >> idx) & 1) == fn(idx), f"row {idx}"


def test_lut_directive_emits_table_and_symbol():
    src = "nop\n.lut se0 = (~dp) & (~dm)\n.lut det = scl & sda_fall\n"
    prog = asm.assemble_program(src)
    assert prog.luts == {"se0": 1, "det": 2}      # tables follow code
    assert prog.lut_tt["se0"] == 0x1111
    assert prog.lut_tt["det"] == 0x8888
    # word0 = nop | se0_tt<<16 ; word1 low half = det_tt
    assert prog.words[0] == (isa.enc_misc(isa.MISC_OPS["nop"]) | (0x1111 << 16))
    assert prog.words[1] & 0xFFFF == 0x8888
    # symbol usable as an operand expression (t = slot 2, after 2 code slots)
    assert slot("nop\n.lut t = a\nldi8.l r1, t", 1) == isa.enc_ldi8(1, 0, 2)


def test_lut_rejects_five_inputs():
    with pytest.raises(asm.AsmError):
        asm.assemble(".lut bad = (a & b) | (c & d) | e\n")


@pytest.mark.parametrize("expr", ["a & & b", "a.b", "a(1)", "x | 2"],
                         ids=["badsyntax", "dot", "call", "const2"])
def test_lut_rejects_bad_expr(expr):
    with pytest.raises(asm.AsmError):
        asm.compile_lut(expr)


# ---------------------------------------------------------------------------
# Assembler: labels, .org/.word/.bcu/.constraint, @baud/@us, rejections
# ---------------------------------------------------------------------------

def test_labels_org_word_annotations():
    src = """
.protocol p @clk 60MHz
.bcu mode=nrzi, dpll=hard_snap(5, 2)
.constraint 2 ui <= t <= 6.5 ui
    nop
    .org 8
tgt:mov r1, r2
    jmp tgt
    .word 0x1234, tgt
"""
    prog = asm.assemble_program(src)
    assert prog.labels["tgt"] == 8
    assert slot(src, 8) == isa.enc_alu(isa.ALU_OPS["mov"], 1, 2)
    assert slot(src, 9) == isa.enc_jmp(8)
    assert slot(src, 10) == 0x1234
    assert slot(src, 11) == 8
    kinds = [a[0] for a in prog.annotations]
    assert kinds == ["protocol", "bcu", "constraint"]
    assert prog.annotations[1][1] == {"mode": "nrzi", "dpll": "hard_snap(5, 2)"}
    assert prog.annotations[2][1] == "2 ui <= t <= 6.5 ui"


def test_numeric_functions_us_baud():
    # @us(n) = 60*n cycles @60 MHz; @baud(n) = round(60e6 / n).
    assert slot("wait_phase @us(2)") == isa.enc_wait_imm(120)
    assert slot("wait_phase @us(4.25)") == isa.enc_wait_imm(255)
    assert slot("set_anchor @baud(600000)") == isa.enc_set_anchor(100)
    # .clk overrides the base.
    prog_src = ".clk 48MHz\nwait_phase @us(2)"
    assert slot(prog_src) == isa.enc_wait_imm(96)


def test_csr_symbolic_names():
    assert slot("csr_wr r1, EV_SET") == isa.enc_csr_wr(1, isa.CSR["EV_SET"])
    assert slot("csr_rd r2, TIMER_LO") == isa.enc_csr_rd(2, isa.CSR["TIMER_LO"])


@pytest.mark.parametrize("src", [
    "jmp 1024",
    "call 2048",
    "jz 4096",
], ids=["jmp1024", "call2048", "jz4096"])
def test_rejects_out_of_range_branch(src):
    with pytest.raises(asm.AsmError):
        asm.assemble(src)


def test_rejects_far_label_branch():
    src = "\n".join(["nop"] * 1024 + ["far:", "jmp far"])
    with pytest.raises(asm.AsmError):
        asm.assemble(src)


def test_accepts_largest_image():
    # 1024 slots -> 2048 bytes: exactly at the SRAM 0 budget.
    prog = asm.assemble_program("\n".join(["nop"] * 1024))
    assert prog.used_slots == 1024
    assert prog.used_bytes == 2048


def test_rejects_one_slot_overflow():
    with pytest.raises(asm.AsmError):
        asm.assemble("\n".join(["nop"] * 1025))


def test_rejects_overflow_via_lut_and_org():
    with pytest.raises(asm.AsmError):
        asm.assemble("\n".join(["nop"] * 1024 + [".lut t = a"]))
    with pytest.raises(asm.AsmError):
        asm.assemble(".org 1024\nnop")


@pytest.mark.parametrize("src", [
    "foo r1",
    "add r1, r4",
    "ldi8.l r0, 256",
    "pin_out 0x10, 0x1",
    "wait_phase 300",
    ".org 4\nnop\n.org 4\nhalt",
    "csr_rd r1, 0x100",
], ids=["badmnem", "badreg", "imm8big", "mask4big", "waitbig", "overlap",
        "csrbig"])
def test_rejects_bad_source(src):
    with pytest.raises(asm.AsmError):
        asm.assemble(src)


def test_reserved_majors_have_no_mnemonic():
    # Every mnemonic encodes into majors 0x0-0xA; nothing reaches 0xB-0xF.
    for word in isa.encode_space():
        assert word >> 12 <= 0xA


# ---------------------------------------------------------------------------
# ISS: fetch/cache timing, ALU, memory, CSR, pins, BCU
# ---------------------------------------------------------------------------

def _iss(src, **kw):
    return HammingISS(asm.assemble(src), **kw)


def test_iss_cache_hit_miss_timing():
    # 4 nops in 2 cache lines + halt: miss(4) + hit(1) + miss(4) + hit(1) + miss(4).
    iss = _iss("nop\nnop\nnop\nnop\nhalt")
    for _ in range(14):
        iss.step()
    assert iss.halted and iss.cycle == 14
    # Cached loop: 'jmp 0' self-loop, first iteration misses, rest all hit.
    iss = _iss("jmp 0")
    for _ in range(4):
        iss.step()
    assert iss.retired == 1     # first jmp retires on miss-completion cycle 4
    r0 = iss.retired
    for _ in range(5):
        iss.step()
    assert iss.retired - r0 == 5  # 1.0 IPC once cached


@pytest.mark.parametrize("miss,total", [(3, 11), (4, 14), (7, 23)],
                         ids=["miss3", "miss4", "miss7"])
def test_iss_configurable_miss_latency(miss, total):
    iss = _iss("nop\nnop\nnop\nnop\nhalt", imem_miss_cycles=miss)
    for _ in range(total):
        iss.step()
    assert iss.halted and iss.cycle == total


def test_iss_alu_flags():
    src = """
    ldi8.l r0, 0x10
    ldi8.h r0, 0x20
    ldi8.l r1, 0x10
    add r0, r1     ; r0=0x2010, z=0, c=0
    cmp r0, r1     ; z=0,c=0,eq=0
    sub r1, r0     ; r1=0x10-0x2010 -> borrow: c=1
    not r1, r1
    mov r2, r1
    shl r2         ; c = old bit15
    shr r2
    inc r2
    dec r2
    halt
"""
    iss = _iss(src)
    for _ in range(80):
        iss.step()
        if iss.halted:
            break
    assert iss.halted
    assert iss.regs[0] == 0x2020  # 0x2010 + 0x10
    assert iss.regs[1] == 0x200F  # ~(0x10 - 0x2020) = ~0xDFF0
    assert iss.regs[2] == 0x200F  # shl/shr/inc/dec round-trip
    assert iss.eq == 0
    assert iss.c == 0             # last c write: shr of 0x401E (bit0 = 0)
    # flags right after cmp (slot 4) executes
    iss = _iss(src)
    while iss.last_instr is None or iss.last_instr[0] != 4:
        iss.step()
    assert iss.z == 0 and iss.c == 0 and iss.eq == 0


def test_iss_cmp_sets_eq():
    iss = _iss("ldi8.l r0, 7\nldi8.l r1, 7\ncmp r0, r1\nhalt")
    while not iss.halted:
        iss.step()
    assert iss.eq == 1 and iss.z == 1 and iss.c == 0


def test_iss_pin_and_lut():
    src = "pin_out 0b0101, 0b1111\npin_oe 0xF, 0x3\npin_in r1, 5\nlut_read r2, 0\nhalt"
    iss = _iss(src)
    iss.lut_tt[0] = 0xFFFF
    while not iss.halted:
        iss.step(0x0020)  # pin 5 high
    assert iss.core_out == 0b0101
    assert iss.core_oe == 0x3
    assert iss.regs[1] == 1
    assert iss.regs[2] == 1


def test_iss_dmem_word_and_byte():
    src = """
    ldi8.l r0, 4          ; base word addr 4
    ldi8.h r1, 0xAB
    ldi8.l r1, 0xCD       ; r1 = 0xABCD
    st_dmem_w r1, [r0+0]  ; sram1[4] = 0xABCD
    ld_dmem_w r2, [r0+0]  ; r2 = 0xABCD
    ld_dmem_b r3, [r0+1]  ; byte addr 5 -> word1 lane1
    halt
"""
    iss = _iss(src)
    iss.sram1[1] = 0xAABBCCDD
    while not iss.halted:
        iss.step()
    assert iss.sram1[4] == 0x0000ABCD
    assert iss.regs[2] == 0xABCD
    assert iss.regs[3] == 0xCC


def test_iss_dmem_byte_store_lane():
    src = ("ldi8.l r0, 6\n"          # byte addr 6 -> word1 lane2
           "ldi8.l r1, 0x5A\n"
           "st_dmem_b r1, [r0+0]\n"
           "halt\n")
    iss = _iss(src)
    iss.sram1[1] = 0x11223344
    while not iss.halted:
        iss.step()
    assert iss.sram1[1] == 0x115A3344


@pytest.mark.parametrize("cyc,total", [(3, 8), (5, 10), (8, 13)],
                         ids=["rd3", "rd5", "rd8"])
def test_iss_dmem_read_latency(cyc, total):
    # ld: fetch miss (4) + read stall (cyc); halt: cached hit (1).
    iss = _iss("ld_dmem_w r1, [r0+0]\nhalt", dmem_read_cycles=cyc)
    n = 0
    while not iss.halted and n < 40:
        iss.step()
        n += 1
    assert iss.cycle == 4 + cyc + 1 == total


def test_iss_ld_imem_tables():
    # .lut lands in tables region; read it back via ld_imem.
    src = ("ldi8.l r0, tbl\n"
           "ld_imem r1, [r0+0]\n"
           "halt\n"
           ".lut tbl = a & b\n")
    iss = _iss(src)
    while not iss.halted:
        iss.step()
    assert iss.regs[1] == 0x8888


def test_iss_csr_access():
    src = ("ldi8.l r1, 0x05\n"
           "csr_wr r1, EV_SET\n"
           "csr_rd r2, EV_FLAGS\n"
           "ldi8.l r1, 0x01\n"
           "csr_wr r1, EV_CLR\n"
           "csr_rd r3, EV_FLAGS\n"
           "halt\n")
    iss = _iss(src)
    while not iss.halted:
        iss.step()
    assert iss.ev_flags == 0x04
    assert iss.regs[2] == 0x05
    assert iss.regs[3] == 0x04


def test_iss_csr_private_bank_ownership():
    # Core 0 owns 0x40-0x4F; writes to core-1 bank (0x50) are dropped.
    src = ("ldi8.l r1, 0x77\n"
           "csr_wr r1, 0x50\n"
           "csr_wr r1, 0x45\n"
           "csr_rd r2, 0x45\n"
           "csr_rd r3, 0x50\n"
           "halt\n")
    iss = _iss(src, core_id=0)
    while not iss.halted:
        iss.step()
    assert iss.regs[2] == 0x77          # own bank write took effect
    assert iss.csr[0x45] == 0x77
    assert iss.regs[3] == 0             # foreign bank write dropped
    assert iss.csr[0x50] == 0


def test_iss_bcu_shadow():
    src = ("ldi8.l r0, 0x42\n"
           "bcu_tx r0\n"
           "bcu_ctrl 0xA5\n"
           "bcu_rx r1\n"
           "bcu_stat r2\n"
           "bcu_crc_rd r3, hi\n"
           "halt\n")
    iss = _iss(src)
    iss.bcu.rx_queue.append(0x1234)
    iss.bcu.stat = 0x0029
    iss.bcu.crc_state = 0xCAFEBABE
    while not iss.halted:
        iss.step()
    assert iss.bcu.tx_shadow == 0x42 and iss.bcu.tx_pushed == 1
    assert iss.bcu.ctrl == 0xA5
    assert iss.regs[1] == 0x1234
    assert iss.regs[2] == 0x0029
    assert iss.regs[3] == 0xCAFE


def test_iss_call_ret():
    src = "call sub\nnop\nhalt\n.org 8\nsub: ret\n"
    iss = _iss(src)
    steps = 0
    while not iss.halted and steps < 40:
        iss.step()
        steps += 1
    assert iss.halted
    # call(4c miss) + ret(4c miss, different line) + nop(hit@slot1) + halt(4c miss word1)
    assert iss.retired == 4


def test_iss_jev():
    src = "ldi8.l r3, 4\njev 0x2, r3\nnop\nhalt\n.org 4\nhalt\n"
    iss = _iss(src)
    # Event not set: falls through to nop->halt at slot 3.
    while not iss.halted:
        iss.step()
    assert iss.last_instr[0] == 3
    # With EV_FLAGS[1] set the branch jumps to r3 (=4).
    iss = _iss(src)
    iss.ev_flags = 0x2
    while not iss.halted:
        iss.step()
    assert iss.last_instr[0] == 4


def test_iss_icache_inv_all_and_host_write():
    src = "jmp 2\n.org 2\nnop\nnop\njmp 2\n"
    iss = _iss(src)
    for _ in range(12):
        iss.step()
    assert iss.icache.hit(1)
    # Host rewrites the loop body; caches invalidate and the new instr retires.
    iss.sram0_write(1, (isa.enc_misc(isa.MISC_OPS["halt"]) << 16)
                    | isa.enc_misc(isa.MISC_OPS["halt"]))
    assert not iss.icache.hit(1)
    n = 0
    while not iss.halted and n < 20:
        iss.step()
        n += 1
    assert iss.halted


# ---------------------------------------------------------------------------
# ISS: WAIT_PHASE exact stall counts, wraparound, overshoot
# ---------------------------------------------------------------------------

def test_wait_phase_imm_exact_stall():
    # set_anchor 0 at slot0 (miss=4 -> executes cycle 4 with timer=3);
    # wait_phase 7 at slot1 (hit -> executes cycle 5 with timer=4).
    iss = _iss("set_anchor 0\nwait_phase 7\nnop\nhalt")
    iss.step()
    iss.step()
    iss.step()
    iss.step()                       # cycle4: set_anchor execs, anchor=3
    assert iss.anchor == 3
    iss.step()                       # cycle5: wait execs timer=4, deadline=10
    assert iss.pc == 1               # stalled on the wait instruction
    # timer must reach 10: stall on cycles 5..9, release at timer 10.
    for i in range(5):
        iss.step()
        assert iss.pc == 1, f"premature release at cycle {iss.cycle}"
    iss.step()                       # timer hits 10 -> release
    assert iss.pc == 2
    assert iss.timer - 1 == 10       # released the cycle timer reached deadline
    assert iss.deadline_missed == 0


def test_wait_phase_wraparound_exact():
    # anchor lands at 0xFFFFE; wait_phase 7 -> deadline 0x00005, stall 6.
    iss = _iss("set_anchor 0\nwait_phase 7\nnop\nhalt",
               timer_init=0xFFFFB)
    iss.step()
    iss.step()
    iss.step()
    iss.step()                       # exec at timer=0xFFFFE -> anchor=0xFFFFE
    assert iss.anchor == 0xFFFFE
    iss.step()                       # exec at timer=0xFFFFF: deadline=0x00005
    assert iss.pc == 1
    assert iss.deadline_missed == 0
    for _ in range(5):
        iss.step()
        assert iss.pc == 1           # still stalled through timer 0..4
    iss.step()                       # timer == 0x00005 -> release
    assert iss.pc == 2
    assert iss.timer - 1 == 0x00005  # released when timer reached deadline
    assert iss.deadline_missed == 0


def test_wait_phase_overshoot_sets_deadline_missed():
    # anchor at 0xFFFFE, delta 0 -> deadline already 1 cycle past at exec.
    iss = _iss("set_anchor 0\nwait_phase 0\nnop\nhalt", timer_init=0xFFFFB)
    for _ in range(5):
        iss.step()
    assert iss.pc == 2               # no stall at all
    assert iss.deadline_missed == 1


def test_wait_phase_exactly_on_time():
    # delta=1 -> deadline 0xFFFFF == exec-cycle timer: on time, no stall/miss.
    iss = _iss("set_anchor 0\nwait_phase 1\nhalt", timer_init=0xFFFFB)
    for _ in range(6):
        iss.step()
    assert iss.pc == 2               # completed with no stall
    assert iss.deadline_missed == 0
    n = 0
    while not iss.halted and n < 20:
        iss.step()
        n += 1
    assert iss.halted


def test_wait_phase_overshoot_reg_delta():
    # Reg-form wait whose anchor is already past at exec: miss, no stall.
    src = "set_anchor 0\nnop\nwait_phase r1\nhalt\n"  # r1 = 0 -> deadline=anchor
    iss = _iss(src)
    n = 0
    while not iss.halted and n < 40:
        iss.step()
        n += 1
    assert iss.halted
    assert iss.deadline_missed == 1


# ---------------------------------------------------------------------------
# ISS: SYNC_EVENT wait, timeout, re-anchor, immediate event
# ---------------------------------------------------------------------------

def test_sync_event_wakes_on_event():
    iss = _iss("ldi8.l r0, 8\nsync_event 0x2, r0\nnop\nhalt")
    for _ in range(5):
        iss.step()                   # cycle5: sync execs at timer=4
    assert iss.pc == 1               # stalled (ev_flags=0)
    iss.step()                       # elapsed 1
    iss.step()                       # elapsed 2
    iss.ev_flags |= 0x2
    iss.step()                       # elapsed 3, event seen -> wake
    assert iss.pc == 2
    assert iss.anchor == 7           # re-anchored to the wakeup cycle


def test_sync_event_timeout():
    iss = _iss("ldi8.l r0, 3\nsync_event 0x2, r0\nnop\nhalt")
    for _ in range(5):
        iss.step()                   # exec timer=4, timeout=3
    assert iss.pc == 1
    for _ in range(3):
        iss.step()                   # elapsed 1,2,3 -> wake on timeout
    assert iss.pc == 2
    assert iss.anchor == 7           # re-anchored to the timeout cycle


def test_sync_event_immediate_and_zero_timeout():
    # Event already pending -> no stall.
    iss = _iss("ldi8.l r0, 9\nsync_event 0x1, r0\nnop\nhalt")
    iss.ev_flags = 0x1
    for _ in range(5):
        iss.step()
    assert iss.pc == 2
    assert iss.anchor == 4
    # Zero timeout -> pure poll, re-anchor, no stall.
    iss = _iss("ldi8.l r0, 0\nsync_event 0x2, r0\nnop\nhalt")
    for _ in range(5):
        iss.step()
    assert iss.pc == 2
    assert iss.anchor == 4


# ---------------------------------------------------------------------------
# ISS: timer readback, halt behavior
# ---------------------------------------------------------------------------

def test_read_timer_lo_hi():
    src = ("set_anchor 0\n"
           "wait_phase 0\n"          # overshoot? no: delta 0, exec 1c later...
           "read_timer r1, lo\n"
           "read_timer r2, hi\n"
           "halt\n")
    iss = _iss(src, timer_init=0xFFFFB)
    n = 0
    while not iss.halted and n < 40:
        iss.step()
        n += 1
    # wait_phase 0: deadline = anchor = 0xFFFFE, exec at timer 0xFFFFF ->
    # missed; read_timer.hi bit15 = deadline_missed.
    assert iss.deadline_missed == 1
    assert iss.regs[2] == 0x8000     # {missed, 11'b0, timer[19:16]=0}
    assert iss.regs[1] == 3          # timer[15:0] sampled at its exec cycle


def test_halt_stops_retiring_but_timer_free_runs():
    iss = _iss("halt\nnop\nnop")
    for _ in range(10):
        iss.step()
    c = iss.cycle
    for _ in range(5):
        iss.step()
    assert iss.halted
    assert iss.cycle == c + 5          # free-running timer keeps going
    assert iss.retired == 1            # only halt retired


# ---------------------------------------------------------------------------
# CLI + generated-file freshness
# ---------------------------------------------------------------------------

def test_cli_assemble_disassemble(tmp_path):
    src = tmp_path / "prog.hasm"
    src.write_text("nop\nldi8.h r0, 0xAB\nhalt\n.lut t = a | b\n")
    rc = asm.main([str(src)])
    assert rc == 0
    bin_path, hex_path = src.with_suffix(".bin"), src.with_suffix(".hex")
    assert bin_path.exists() and hex_path.exists()
    assert len(bin_path.read_bytes()) == 512 * 4
    assert asm.from_bin_bytes(bin_path.read_bytes()) == \
        asm.from_hex_text(hex_path.read_text())
    out = tmp_path / "back.hasm"
    rc = asm.main([str(bin_path), "--disassemble", "-o", str(out)])
    assert rc == 0
    assert asm.assemble(out.read_text()) == \
        asm.assemble(src.read_text())


def test_generated_files_fresh():
    assert (ROOT / "src" / "hamming_defs.vh").read_text() == isa.emit_verilog()
    assert (ROOT / "docs" / "ISA.md").read_text() == isa.emit_docs()
