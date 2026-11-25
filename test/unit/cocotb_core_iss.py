# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0

import random
import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools import hamming_isa as isa  # noqa: E402
from tools.hamming_iss import HammingISS  # noqa: E402

_CLOCK_STARTED = False


def u(sig):
    value = sig.value
    if hasattr(value, "to_unsigned"):
        return value.to_unsigned()
    return int(value)


def set_slice(word, index, width, value):
    mask = ((1 << width) - 1) << (index * width)
    return (word & ~mask) | ((value & ((1 << width) - 1)) << (index * width))


def get_slice(word, index, width):
    return (word >> (index * width)) & ((1 << width) - 1)


def pack_instructions(instrs, words=512):
    out = [0] * words
    for pc, instr in enumerate(instrs):
        waddr = (pc >> 1) & 0x1FF
        out[waddr] |= (instr & 0xFFFF) << (16 * (pc & 1))
    return out


async def sample_after_rise(clk):
    await RisingEdge(clk)
    await Timer(1, unit="ns")


async def init_inputs(dut):
    for name in (
        "run", "pin_inputs", "lut_outputs", "icache_inv_all",
        "host_csr_we", "host_csr_addr", "host_csr_wdata",
        "bcu_rx_data", "bcu_stat", "bcu_crc_state",
        "imem_bwe", "imem_baddr", "imem_bdata",
        "dmem_bwe", "dmem_baddr", "dmem_bdata",
    ):
        getattr(dut, name).value = 0


async def reset_harness(dut):
    await init_inputs(dut)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 6)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 3)


async def backdoor_write(dut, bank, addr, data):
    await FallingEdge(dut.clk)
    getattr(dut, f"{bank}_baddr").value = addr
    getattr(dut, f"{bank}_bdata").value = data
    getattr(dut, f"{bank}_bwe").value = 1
    await FallingEdge(dut.clk)
    getattr(dut, f"{bank}_bwe").value = 0
    getattr(dut, f"{bank}_baddr").value = 0
    getattr(dut, f"{bank}_bdata").value = 0


async def load_imem(dut, words):
    for addr, word in enumerate(words):
        if word:
            await backdoor_write(dut, "imem", addr, word)


async def load_dmem(dut, words):
    for addr, word in enumerate(words):
        if word:
            await backdoor_write(dut, "dmem", addr, word)


async def start_core(dut):
    await FallingEdge(dut.clk)
    dut.run.value = 1
    return u(dut.timer)


async def load_imem_span(dut, words, span_words):
    # SRAM contents are X-initialized.  Write every word in the program span,
    # including zero-valued instruction words, so a legal 0x0000 encoding can
    # never accidentally consume Xs.
    for addr in range(span_words):
        await backdoor_write(dut, "imem", addr, words[addr])


def unpack_regs(flat):
    return [get_slice(flat, i, 16) for i in range(4)]


def make_iss(imem_words, dmem_words=None, *, timer_init=0, pin_inputs=0,
             lutvec=0, bcu_rx=0, bcu_stat=0, bcu_crc=0):
    iss = HammingISS(
        imem_words,
        dmem_words or [],
        imem_miss_cycles=isa.IMEM_MISS_DFLT,
        dmem_read_cycles=isa.DMEM_RD_DFLT,
        dmem_write_cycles=isa.DMEM_WR_MIN,
        timer_init=timer_init,
    )
    iss.pin_inputs = pin_inputs & 0xFFFF
    iss.lut_eval = lambda inputs: lutvec & 0xFFFF
    iss.bcu.rx_shadow = bcu_rx & 0xFFFF
    iss.bcu.stat = bcu_stat & 0xFFFF
    iss.bcu.crc_state = bcu_crc & 0xFFFFFFFF
    return iss


def assert_iss_rtl_equal(dut, iss, *, check_timer=True):
    rtl_regs = unpack_regs(u(dut.dbg_regs_flat))
    assert rtl_regs == iss.regs, f"regs rtl={rtl_regs} iss={iss.regs}"
    assert u(dut.core_pc) == iss.pc, f"pc rtl={u(dut.core_pc):#x} iss={iss.pc:#x}"
    status = u(dut.core_status)
    exp_status = (iss.z | (iss.c << 1) | (iss.eq << 2) |
                  (iss.deadline_missed << 3) | (int(iss.halted) << 4))
    assert (status & 0x1F) == exp_status, f"status rtl={status & 0x1f:#x} iss={exp_status:#x}"
    assert u(dut.core_out) == iss.core_out
    assert u(dut.core_oe) == iss.core_oe
    assert u(dut.ev_flags) == iss.ev_flags
    assert u(dut.bcu_ctrl) == iss.bcu.ctrl
    if check_timer:
        assert u(dut.timer) == iss.timer, f"timer rtl={u(dut.timer):#x} iss={iss.timer:#x}"


async def ensure_clock(dut):
    global _CLOCK_STARTED
    if not _CLOCK_STARTED:
        cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
        _CLOCK_STARTED = True


async def run_diff(dut, program, *, cycles=160, pin_inputs=0, lutvec=0,
                   dmem_words=None, bcu_rx=0, bcu_stat=0, bcu_crc=0,
                   check_timer=True, timer_preset=None):
    await ensure_clock(dut)
    await reset_harness(dut)
    words = pack_instructions(program)
    # The behavioral SRAMs are X-initialized while the ISS memories are
    # zero-filled.  Initialize both full arrays so randomized table/data reads
    # test architectural behavior rather than simulator X artifacts.
    for addr in range(512):
        await backdoor_write(dut, "imem", addr, 0)
        await backdoor_write(dut, "dmem", addr, 0)
    span = ((len(program) + 1) // 2) + 2
    await load_imem_span(dut, words, span)
    if dmem_words:
        await load_dmem(dut, dmem_words)

    dut.pin_inputs.value = pin_inputs
    dut.lut_outputs.value = lutvec
    dut.bcu_rx_data.value = bcu_rx
    dut.bcu_stat.value = bcu_stat
    dut.bcu_crc_state.value = bcu_crc

    if timer_preset is not None:
        dut.i_csr.timer_r.value = timer_preset & isa.TIMER_MASK
    timer0 = await start_core(dut)
    iss = make_iss(words, dmem_words or [], timer_init=timer0,
                   pin_inputs=pin_inputs, lutvec=lutvec, bcu_rx=bcu_rx,
                   bcu_stat=bcu_stat, bcu_crc=bcu_crc)
    await ClockCycles(dut.clk, cycles)
    await Timer(1, unit="ns")  # let the final edge's nonblocking updates settle
    iss.run(cycles, pin_inputs)
    assert_iss_rtl_equal(dut, iss, check_timer=check_timer)
    return iss


@cocotb.test()
async def test_core_basic_alu_fetch_and_halt(dut):
    """A cold cache fetch executes LDI8/ADD once per hit-cycle and HALT stops
    the core with the expected register and architectural status state."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_harness(dut)

    program = [
        isa.enc_ldi8(0, 0, 3),
        isa.enc_ldi8(1, 0, 4),
        isa.enc_alu(isa.ALU_OPS["add"], 0, 1),
        isa.enc_misc(isa.MISC_OPS["halt"]),
    ]
    await load_imem(dut, pack_instructions(program))
    await start_core(dut)
    await ClockCycles(dut.clk, 30)

    regs = u(dut.dbg_regs_flat)
    assert get_slice(regs, 0, 16) == 7, f"r0={get_slice(regs, 0, 16):#x}"
    assert get_slice(regs, 1, 16) == 4
    assert (u(dut.core_status) >> 4) & 1, f"not halted: {u(dut.core_status):#x}"


@cocotb.test()
async def test_core_alu_ldi_differential(dut):
    """All ALU forms, including unary must-be-zero forms and flag updates,
    retire with the same register/flag state as the golden ISS."""
    A = isa.ALU_OPS
    program = [
        isa.enc_ldi8(0, 0, 0xFF), isa.enc_ldi8(0, 1, 0x12),
        isa.enc_ldi8(1, 0, 0x34), isa.enc_ldi8(1, 1, 0x00),
        isa.enc_alu(A["mov"], 2, 0), isa.enc_alu(A["add"], 2, 1),
        isa.enc_alu(A["sub"], 2, 1), isa.enc_alu(A["and"], 2, 1),
        isa.enc_alu(A["or"], 2, 1),  isa.enc_alu(A["xor"], 2, 1),
        isa.enc_alu(A["shl"], 2, 0), isa.enc_alu(A["shr"], 2, 0),
        isa.enc_alu(A["inc"], 2, 0), isa.enc_alu(A["dec"], 2, 0),
        isa.enc_alu(A["not"], 3, 2), isa.enc_alu(A["cmp"], 3, 2),
        isa.enc_misc(isa.MISC_OPS["halt"]),
    ]
    await run_diff(dut, program, cycles=180)


@cocotb.test()
async def test_core_pin_lut_bcu_differential(dut):
    """Pin input/output/OE, LUT readback, and all BCU operand paths match the
    ISS against fixed external BCU/pin stimulus."""
    program = [
        isa.enc_pin_out(0xF, 0xA), isa.enc_pin_oe(0xF, 0x5),
        isa.enc_pin_in(0, 3), isa.enc_lut_read(1, 2),
        isa.enc_bcu_rx(2), isa.enc_bcu_stat(3),
        isa.enc_bcu_crc_rd(0, isa.BCU_CRC_LO),
        isa.enc_bcu_crc_rd(1, isa.BCU_CRC_HI),
        isa.enc_bcu_ctrl(0x5A), isa.enc_bcu_tx(2),
        isa.enc_misc(isa.MISC_OPS["halt"]),
    ]
    await run_diff(dut, program, cycles=180, pin_inputs=0x0008, lutvec=0x0004,
                   bcu_rx=0xBEEF, bcu_stat=0x00A5, bcu_crc=0x1234ABCD)


@cocotb.test()
async def test_core_memory_word_byte_and_tables_differential(dut):
    """SRAM 1 word/byte stores and loads use the documented address, lane, and
    mask rules; ld_imem reads the corresponding SRAM 0 halfword."""
    M = isa.MEM_OPS
    program = [
        isa.enc_ldi8(0, 0, 5),                 # base word/byte address
        isa.enc_ldi8(1, 0, 0x34), isa.enc_ldi8(1, 1, 0x12),
        isa.enc_mem(M["st_dmem_w"], 0, 0, 1),
        isa.enc_mem(M["ld_dmem_w"], 0, 0, 2),
        isa.enc_ldi8(1, 0, 0xAB), isa.enc_ldi8(1, 1, 0x00),
        isa.enc_mem(M["st_dmem_b"], 0, 1, 1),  # byte address 6 = word1 lane2
        isa.enc_mem(M["ld_dmem_b"], 0, 1, 3),
        isa.enc_ldi8(0, 0, 1),
        isa.enc_mem(M["ld_imem"], 0, 0, 1),
        isa.enc_misc(isa.MISC_OPS["halt"]),
    ]
    await run_diff(dut, program, cycles=240)


@cocotb.test()
async def test_core_control_flow_call_return_differential(dut):
    """Nested calls use the two-deep architectural return stack; branches and
    unconditional jumps select the same PCs as the ISS."""
    program = [
        isa.enc_ldi8(0, 0, 1),          # 0
        isa.enc_alu(isa.ALU_OPS["mov"], 1, 0),
        isa.enc_br(isa.BR_OPS["jnz"], 5),  # 2
        isa.enc_ldi8(2, 0, 0xEE),       # 3 skipped
        isa.enc_misc(isa.MISC_OPS["halt"]),  # 4 skipped
        isa.enc_br(isa.BR_OPS["call"], 8),   # 5
        isa.enc_ldi8(3, 0, 0x33),       # 6 after outer return
        isa.enc_misc(isa.MISC_OPS["halt"]),  # 7
        isa.enc_br(isa.BR_OPS["call"], 10),  # 8
        isa.enc_misc(isa.MISC_OPS["ret"]),   # 9
        isa.enc_ldi8(2, 0, 0x55),       # 10
        isa.enc_misc(isa.MISC_OPS["ret"]),   # 11
    ]
    await run_diff(dut, program, cycles=200)


@cocotb.test()
async def test_core_csr_event_and_ownership_differential(dut):
    """Atomic core EV_SET/EV_CLR and private-bank writes match the production
    CSR bank and the ISS ownership model; writes outside the owned bank drop."""
    evset = isa.CSR["EV_SET"]
    evclr = isa.CSR["EV_CLR"]
    txdata = isa.CSR_CORE_BANK + isa.CORE_CSR["BCU_TX_DATA"]
    lut0 = isa.CSR_LUT_TT_BASE
    program = [
        isa.enc_ldi8(0, 0, 0x5A),
        isa.enc_csr_wr(0, txdata),
        isa.enc_csr_rd(1, txdata),
        isa.enc_ldi8(0, 0, 0x99),
        isa.enc_csr_wr(0, lut0),       # outside core private bank: dropped
        isa.enc_csr_rd(2, lut0),
        isa.enc_ldi8(0, 0, 0x03),
        isa.enc_csr_wr(0, evset),
        isa.enc_ldi8(0, 0, 0x01),
        isa.enc_csr_wr(0, evclr),      # leaves event bit1 set
        isa.enc_csr_rd(3, isa.CSR["EV_FLAGS"]),
        isa.enc_misc(isa.MISC_OPS["halt"]),
    ]
    await run_diff(dut, program, cycles=220)


@cocotb.test()
async def test_core_timing_wait_sync_timer_differential(dut):
    """Anchored modular waits, timeout-based sync re-anchoring, timer reads,
    and the sticky missed-deadline bit match cycle-for-cycle."""
    program = [
        isa.enc_set_anchor(0),
        isa.enc_wait_imm(5),
        isa.enc_read_timer(0, isa.READ_TIMER_LO),
        isa.enc_wait_imm(1),            # just-expired deadline: no missed flag
        isa.enc_set_anchor(0),
        isa.enc_wait_imm(0),            # exactly on time: no stall
        isa.enc_read_timer(1, isa.READ_TIMER_HI),
        isa.enc_ldi8(2, 0, 3),
        isa.enc_sync_event(0, 2),       # zero timeout is an immediate poll
        isa.enc_misc(isa.MISC_OPS["halt"]),
    ]
    await run_diff(dut, program, cycles=180)


@cocotb.test()
async def test_core_cache_conflict_loop_differential(dut):
    """A loop alternating SRAM 0 word addresses 0 and 8 conflicts in the
    direct-mapped cache and still retires the correct number of iterations."""
    program = [isa.enc_misc(isa.MISC_OPS["nop"])] * 19
    program[0] = isa.enc_ldi8(0, 0, 3)
    program[1] = isa.enc_jmp(16)
    program[16] = isa.enc_alu(isa.ALU_OPS["dec"], 0, 0)
    program[17] = isa.enc_br(isa.BR_OPS["jnz"], 1)
    program[18] = isa.enc_misc(isa.MISC_OPS["halt"])
    await run_diff(dut, program, cycles=320)


@cocotb.test()
async def test_core_jev_and_sync_timeout_differential(dut):
    """Event-conditioned indirect branches use rs[9:0], while a zero-mask
    sync_event with a nonzero timeout stalls for the architectural timeout
    and re-anchors before the next instruction."""
    evset = isa.CSR["EV_SET"]
    program = [
        isa.enc_ldi8(0, 0, 6),          # indirect JEV target
        isa.enc_ldi8(1, 0, 1),
        isa.enc_csr_wr(1, evset),
        isa.enc_jev(1, 0),
        isa.enc_ldi8(2, 0, 0xEE),       # skipped
        isa.enc_misc(isa.MISC_OPS["halt"]),
        isa.enc_ldi8(2, 0, 4),          # 6: nonzero timeout, zero event mask
        isa.enc_sync_event(0, 2),
        isa.enc_read_timer(3, isa.READ_TIMER_LO),
        isa.enc_misc(isa.MISC_OPS["halt"]),
    ]
    await run_diff(dut, program, cycles=220)


@cocotb.test()
async def test_core_host_write_invalidates_coincident_loop_cache(dut):
    """A host SRAM 0 write plus invalidation drops an already-cached looping
    line; the replacement HALT retires instead of the stale NOP/JUMP loop."""
    await ensure_clock(dut)
    await reset_harness(dut)
    old_program = [
        isa.enc_jmp(2),
        isa.enc_misc(isa.MISC_OPS["nop"]),
        isa.enc_misc(isa.MISC_OPS["nop"]),
        isa.enc_jmp(2),
    ]
    new_program = list(old_program)
    new_program[2] = isa.enc_misc(isa.MISC_OPS["halt"])
    old_words = pack_instructions(old_program)
    new_words = pack_instructions(new_program)
    await load_imem_span(dut, old_words, 4)

    timer0 = await start_core(dut)
    iss = make_iss(old_words, timer_init=timer0)

    # Let both copies fill and execute the tight cached loop.
    await ClockCycles(dut.clk, 24)
    await Timer(1, unit="ns")
    iss.run(24)

    # Commit the replacement word and assert invalidation for the same edge.
    await FallingEdge(dut.clk)
    dut.imem_baddr.value = 1
    dut.imem_bdata.value = new_words[1]
    dut.imem_bwe.value = 1
    dut.icache_inv_all.value = 1
    iss.sram0_write(1, new_words[1])
    await RisingEdge(dut.clk)
    iss.step()
    await FallingEdge(dut.clk)
    dut.imem_bwe.value = 0
    dut.icache_inv_all.value = 0
    dut.imem_baddr.value = 0
    dut.imem_bdata.value = 0

    await ClockCycles(dut.clk, 100)
    await Timer(1, unit="ns")
    iss.run(100)
    assert_iss_rtl_equal(dut, iss, check_timer=True)
    assert (u(dut.core_status) >> 4) & 1, "stale cached loop did not retire replacement HALT"


def random_straight_instr(rng):
    """Generate one pseudo-random instruction that cannot terminate or alter
    control flow.  Illegal/reserved encodings are included as architectural
    NOPs.  Timing waits are small and immediate-only so the closed loop keeps
    making forward progress."""
    A = isa.ALU_OPS
    kind = rng.randrange(100)
    if kind < 32:
        op = rng.choice(list(A.values()))
        rd = rng.randrange(4)
        rs = 0 if op in (A["shl"], A["shr"], A["inc"], A["dec"]) else rng.randrange(4)
        return isa.enc_alu(op, rd, rs)
    if kind < 52:
        return isa.enc_ldi8(rng.randrange(4), rng.randrange(2), rng.randrange(256))
    if kind < 61:
        sub = rng.randrange(4)
        if sub == 0:
            return isa.enc_pin_out(rng.randrange(16), rng.randrange(16))
        if sub == 1:
            return isa.enc_pin_oe(rng.randrange(16), rng.randrange(16))
        if sub == 2:
            return isa.enc_pin_in(rng.randrange(4), rng.randrange(16))
        return isa.enc_lut_read(rng.randrange(4), rng.randrange(16))
    if kind < 71:
        return isa.enc_mem(rng.randrange(5), rng.randrange(4), rng.randrange(4), rng.randrange(4))
    if kind < 80:
        private = [isa.CSR_CORE_BANK + off for off in (4, 5, 6, 9, 10, 11, 12, 13)]
        write_addrs = [isa.CSR["EV_SET"], isa.CSR["EV_CLR"], isa.CSR_LUT_TT_BASE,
                       isa.CSR["PIN_ROUTER0"]] + private
        read_addrs = [isa.CSR["EV_FLAGS"], isa.CSR["TIMER_LO"], isa.CSR["TIMER_HI"],
                      isa.CSR_CORE_BANK + isa.CORE_CSR["CORE_STATUS"],
                      isa.CSR_CORE_BANK + isa.CORE_CSR["CORE_PC"],
                      isa.CSR_CORE_BANK + isa.CORE_CSR["CORE_OUT"],
                      isa.CSR_CORE_BANK + isa.CORE_CSR["CORE_OE"]] + write_addrs
        if rng.randrange(2):
            return isa.enc_csr_wr(rng.randrange(4), rng.choice(write_addrs))
        return isa.enc_csr_rd(rng.randrange(4), rng.choice(read_addrs))
    if kind < 86:
        sub = rng.randrange(5)
        if sub == 0:
            return isa.enc_bcu_tx(rng.randrange(4))
        if sub == 1:
            return isa.enc_bcu_rx(rng.randrange(4))
        if sub == 2:
            return isa.enc_bcu_ctrl(rng.randrange(256))
        if sub == 3:
            return isa.enc_bcu_stat(rng.randrange(4))
        return isa.enc_bcu_crc_rd(rng.randrange(4), rng.randrange(2))
    if kind < 93:
        sub = rng.randrange(3)
        if sub == 0:
            return isa.enc_set_anchor(rng.randrange(16))
        if sub == 1:
            return isa.enc_read_timer(rng.randrange(4), rng.randrange(2))
        return isa.enc_wait_imm(rng.randrange(8))
    if kind < 97:
        return (rng.randrange(0xB, 0x10) << 12) | rng.randrange(0x1000)
    # Illegal unary ALU encoding: nonzero must-be-zero rs field.
    return ((isa.ALU_OPS["shl"] << 8) | (rng.randrange(4) << 6) |
            ((1 + rng.randrange(3)) << 4))


@cocotb.test()
async def test_core_random_loop_100k_cycles_differential(dut):
    """Closed-loop randomized differential fuzz.  The loop is deliberately
    wider than the eight-line cache, so ordinary instruction execution,
    conflict misses, SRAM 0/1 stalls, CSR state, pins, BCU reads, timer reads,
    waits, and illegal-word NOPs all interleave for 100,000 cycles."""
    rng = random.Random(0x1CEC0DE)
    program = [random_straight_instr(rng) for _ in range(96)]
    program.append(isa.enc_jmp(0))
    await run_diff(dut, program, cycles=100000, pin_inputs=0xA5C3,
                   lutvec=0x3C5A, bcu_rx=0xC0DE, bcu_stat=0x0055,
                   bcu_crc=0x89ABCDEF, check_timer=True)


@cocotb.test()
async def test_core_host_write_cancels_inflight_and_coincident_fills(dut):
    """Invalidate during the SRAM 0 fill window and again across the exact
    response edge.  In both races the stale pre-write fill must be dropped and
    the core must refetch the replacement HALT."""
    nop = isa.enc_misc(isa.MISC_OPS["nop"])
    halt = isa.enc_misc(isa.MISC_OPS["halt"])
    old_w0 = set_slice(0, 0, 16, nop) | set_slice(0, 1, 16, nop)
    new_w0 = set_slice(0, 0, 16, halt) | set_slice(0, 1, 16, nop)

    for delay_cycles in (1, 2):
        await ensure_clock(dut)
        await reset_harness(dut)
        for addr in range(4):
            await backdoor_write(dut, "imem", addr, 0)
        await backdoor_write(dut, "imem", 0, old_w0)

        await start_core(dut)
        await ClockCycles(dut.clk, delay_cycles)
        await FallingEdge(dut.clk)
        dut.imem_baddr.value = 0
        dut.imem_bdata.value = new_w0
        dut.imem_bwe.value = 1
        dut.icache_inv_all.value = 1
        await RisingEdge(dut.clk)
        await FallingEdge(dut.clk)
        dut.imem_bwe.value = 0
        dut.icache_inv_all.value = 0
        dut.imem_baddr.value = 0
        dut.imem_bdata.value = 0

        await ClockCycles(dut.clk, 40)
        await Timer(1, unit="ns")
        assert (u(dut.core_status) >> 4) & 1, (
            f"stale fill retired after invalidation with delay {delay_cycles}"
        )


@cocotb.test()
async def test_core_reset_start_stop_freezes_architectural_state(dut):
    """Deasserting run stops architectural retirement without losing PC or
    register state; reasserting run resumes and reaches HALT."""
    await ensure_clock(dut)
    await reset_harness(dut)
    program = [
        isa.enc_ldi8(0, 0, 3),
        isa.enc_ldi8(1, 0, 4),
        isa.enc_alu(isa.ALU_OPS["add"], 0, 1),
        isa.enc_misc(isa.MISC_OPS["halt"]),
    ]
    await load_imem_span(dut, pack_instructions(program), 4)
    await start_core(dut)
    await ClockCycles(dut.clk, 3)
    await Timer(1, unit="ns")

    await FallingEdge(dut.clk)
    dut.run.value = 0
    await ClockCycles(dut.clk, 2)
    await Timer(1, unit="ns")
    frozen_pc = u(dut.core_pc)
    frozen_regs = u(dut.dbg_regs_flat)
    await ClockCycles(dut.clk, 12)
    await Timer(1, unit="ns")
    assert u(dut.core_pc) == frozen_pc
    assert u(dut.dbg_regs_flat) == frozen_regs

    await FallingEdge(dut.clk)
    dut.run.value = 1
    await ClockCycles(dut.clk, 40)
    await Timer(1, unit="ns")
    assert get_slice(u(dut.dbg_regs_flat), 0, 16) == 7
    assert (u(dut.core_status) >> 4) & 1


@cocotb.test()
async def test_core_timer_wraparound_wait_differential(dut):
    """The signed modular 20-bit timer comparison wraps through zero without
    releasing early or missing a future deadline."""
    program = [
        isa.enc_set_anchor(0),
        isa.enc_wait_imm(7),
        isa.enc_read_timer(0, isa.READ_TIMER_LO),
        isa.enc_read_timer(1, isa.READ_TIMER_HI),
        isa.enc_misc(isa.MISC_OPS["halt"]),
    ]
    await run_diff(dut, program, cycles=180,
                   timer_preset=isa.TIMER_MASK - 3, check_timer=True)
