# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0

import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge

GATES = os.environ.get("GATES", "no").lower() in ("yes", "1", "true")

CMD_IDLE = 0b000
CMD_ADDR = 0b001
CMD_WRITE = 0b010
CMD_WRBOTH = 0b011
CMD_READ = 0b100


async def reset_dut(dut, cycles=10):
    await FallingEdge(dut.clk)
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, cycles)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)


async def cmd(dut, op, data=0, bank=0, lane=0, ahi=0):
    """Drive one pin command for exactly one capture edge.

    Inputs are driven on the falling edge so they are stable well before the
    rising edge that samples them (no drive-at-posedge race).
    """
    await FallingEdge(dut.clk)
    dut.ui_in.value = (op << 5) | (bank << 4) | (lane << 2) | (ahi & 3)
    dut.uio_in.value = data & 0xFF
    await ClockCycles(dut.clk, 1)
    await FallingEdge(dut.clk)
    dut.ui_in.value = 0
    dut.uio_in.value = 0


async def set_addr(dut, addr):
    await cmd(dut, CMD_ADDR, data=addr & 0xFF, ahi=(addr >> 8) & 1)


async def read_back(dut, bank, lane):
    """Select {bank, lane} of the last read word and sample uo_out."""
    await cmd(dut, CMD_IDLE, bank=bank, lane=lane)
    assert dut.uo_out.value.is_resolvable, "uo_out has X/Z after read"
    return dut.uo_out.value.to_unsigned()


async def read_word(dut, bank):
    """Return the last-read 32-bit word of `bank` by sampling all 4 lanes."""
    word = 0
    for lane in (0, 1, 2, 3):
        word |= (await read_back(dut, bank, lane)) << (lane * 8)
    return word


def assert_no_x(sig, name):
    v = sig.value
    assert v.is_resolvable, f"{name} is not resolvable: {v.binstr}"


@cocotb.test()
async def test_reset_safety(dut):
    """X-initialised SRAM + reset + random pins must keep outputs strictly 0.

    The read registers must stay 0 (they only capture on a real read), the
    SRAM enables must stay low, and nothing X may reach uo_out / uio_out /
    uio_oe, with ena both low and high.
    """
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())

    rng = random.Random(0xCAFE)
    dut.ena.value = 0
    dut.rst_n.value = 0
    for _ in range(200):
        await FallingEdge(dut.clk)
        dut.ui_in.value = rng.randrange(256)
        dut.uio_in.value = rng.randrange(256)
        await ClockCycles(dut.clk, 1)
        assert_no_x(dut.uo_out, "uo_out")
        assert_no_x(dut.uio_out, "uio_out")
        assert_no_x(dut.uio_oe, "uio_oe")
        assert dut.uo_out.value.to_unsigned() == 0, "uo_out not 0 in reset"
        assert dut.uio_out.value.to_unsigned() == 0
        assert dut.uio_oe.value.to_unsigned() == 0

    dut.ena.value = 1
    for _ in range(60):
        await FallingEdge(dut.clk)
        dut.ui_in.value = rng.randrange(256)
        await ClockCycles(dut.clk, 1)
        assert dut.uo_out.value.to_unsigned() == 0, "uo_out not 0 in reset"

    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    assert_no_x(dut.uo_out, "uo_out post-reset")
    assert dut.uo_out.value.to_unsigned() == 0


@cocotb.test()
async def test_read_latency(dut):
    """A READ result must land in the output registers exactly two edges
    after the command edge (macro data out one edge after its inputs)."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    await reset_dut(dut)

    # Word at addr 0, bank 0: bytes 11 22 33 44 (lane3..lane0 written last).
    await set_addr(dut, 0x000)
    for lane, byte in enumerate((0x11, 0x22, 0x33, 0x44)):
        await cmd(dut, CMD_WRITE, data=byte, bank=0, lane=lane)

    # First read: establishes rd_data. Then a second read is timed.
    await cmd(dut, CMD_READ)
    await ClockCycles(dut.clk, 3)

    await set_addr(dut, 0x001)
    for lane, byte in enumerate((0xAA, 0xBB, 0xCC, 0xDD)):
        await cmd(dut, CMD_WRITE, data=byte, bank=0, lane=lane)

    # Issue READ at addr 1 and observe uo_out on consecutive posedges. The
    # command registers one edge after sampling, the macro outputs the word
    # one edge after its inputs, and rd_data captures on the third edge.
    await cmd(dut, CMD_READ)
    observed = []
    for _ in range(4):
        await ClockCycles(dut.clk, 1)
        observed.append(dut.uo_out.value.to_unsigned())
    assert observed[:2] == [0x11, 0x11] and observed[2] == 0xAA, (
        f"read latency: expected [0x11, 0x11, 0xAA, 0xAA], got {observed}"
    )
    assert observed[3] == 0xAA


@cocotb.test()
async def test_both_banks_independent(dut):
    """Write and read SRAM 0 and SRAM 1 independently at the first and last
    logical addresses; bank selects must not cross."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    await reset_dut(dut)

    for addr in (0x000, 0x001, 0x100, 0x1FF):
        await set_addr(dut, addr)
        b0_word = (0x10101010 + addr) & 0xFFFFFFFF
        b1_word = (0xEFEFEF00 + addr) & 0xFFFFFFFF
        for lane in range(4):
            await cmd(dut, CMD_WRITE, data=(b0_word >> (lane * 8)) & 0xFF, bank=0, lane=lane)
            await cmd(dut, CMD_WRITE, data=(b1_word >> (lane * 8)) & 0xFF, bank=1, lane=lane)
        await cmd(dut, CMD_READ)
        await ClockCycles(dut.clk, 3)
        assert (await read_word(dut, bank=0)) == b0_word, f"bank0 @ {addr:#x}"
        assert (await read_word(dut, bank=1)) == b1_word, f"bank1 @ {addr:#x}"


@cocotb.test()
async def test_byte_mask_preserves_other_lanes(dut):
    """A_BM must write only the selected byte lane; neighbouring lanes keep
    their previous contents."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    await reset_dut(dut)

    await set_addr(dut, 0x010)
    # Initialise all four lanes of bank 0 with 0xAA.
    for lane in range(4):
        await cmd(dut, CMD_WRITE, data=0xAA, bank=0, lane=lane)
    # Overwrite only lane 2.
    await cmd(dut, CMD_WRITE, data=0x55, bank=0, lane=2)
    await cmd(dut, CMD_READ)
    await ClockCycles(dut.clk, 3)
    word = await read_word(dut, bank=0)
    assert word == 0xAA55AAAA, f"mask leaked: {word:#010x}"


@cocotb.test()
async def test_same_cycle_both_banks(dut):
    """WRBOTH drives the same lane into both banks in a single macro access;
    READ then returns each bank's word from one dual-bank access."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    await reset_dut(dut)

    await set_addr(dut, 0x020)
    # Prime all lanes of both banks with 0.
    for lane in range(4):
        await cmd(dut, CMD_WRBOTH, data=0x00, lane=lane)
    await cmd(dut, CMD_WRBOTH, data=0x7E, lane=1)
    await cmd(dut, CMD_READ)
    await ClockCycles(dut.clk, 3)
    assert (await read_word(dut, bank=0)) == 0x00007E00
    assert (await read_word(dut, bank=1)) == 0x00007E00
    # Now overwrite bank 1 only; bank 0 must keep 0x7E in lane 1.
    await cmd(dut, CMD_WRITE, data=0x99, bank=1, lane=1)
    await cmd(dut, CMD_READ)
    await ClockCycles(dut.clk, 3)
    assert (await read_word(dut, bank=0)) == 0x00007E00
    assert (await read_word(dut, bank=1)) == 0x00009900


@cocotb.test()
async def test_consecutive_accesses(dut):
    """Back-to-back READs on consecutive addresses stream results in order."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    await reset_dut(dut)

    pattern = {a: (0xA5000000 | (a << 4) | 0x5) & 0xFFFFFFFF for a in (0x030, 0x031, 0x032)}
    for addr, word in pattern.items():
        await set_addr(dut, addr)
        for lane in range(4):
            await cmd(dut, CMD_WRITE, data=(word >> (lane * 8)) & 0xFF, bank=0, lane=lane)
    for addr, word in pattern.items():
        await set_addr(dut, addr)
        await cmd(dut, CMD_READ)
        await ClockCycles(dut.clk, 3)
        got = await read_word(dut, bank=0)
        assert got == word, f"addr {addr:#x}: got {got:#010x} want {word:#010x}"
