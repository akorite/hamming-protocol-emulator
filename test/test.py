# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Pin-only full-chip tests for tt_um_akorite_hamming (U6 slice 4).

Every interaction is through the chip pins: host SPI bit-banging on
ui_in[7:5] / uo_out[7] and protocol stimulus on uio_in / ui_in[4:0].
No hierarchical access, so the same tests run on RTL and gate level.

Cycle budget: the gl_test harness allows <= 30,000 cycles total.  At
half=5 (10 system clocks per SPI bit) a CSR write costs ~430 cycles, an
SRAM word write ~590 and a word read ~670; the whole file stays well
under 10k cycles.
"""

import os
import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import hamming_isa as isa  # noqa: E402

GATES = os.environ.get("GATES", "no").lower() in ("yes", "1", "true")

CMD_WR_SRAM0 = 0x01
CMD_WR_SRAM1 = 0x02
CMD_WR_CSR = 0x03
CMD_RD_SRAM0 = 0x81
CMD_RD_SRAM1 = 0x82
CMD_RD_CSR = 0x83

CSR_CTRL_RUN = 0x00
CSR_EV_SET = 0x02
CSR_RLE_BASE = 0x06
CSR_RLE_MASK = 0x07
CSR_RLE_CTRL = 0x0C
CSR_RLE_PROBE_MASK = 0x0D
CSR_PIN_ROUTER0 = 0x20
CSR_PIN_ROUTER4 = 0x24

HALF = 5  # SPI half-period in system clocks (5 MHz at 60 MHz sysclk)


def u(sig):
    v = sig.value
    return v.to_unsigned() if hasattr(v, "to_unsigned") else int(v)


async def cycles(dut, n):
    await ClockCycles(dut.clk, n)


# ----------------------------------------------------------------------
# Host SPI master BFM (Mode 0, MSB first) driving the ui_in pin bits.
# ----------------------------------------------------------------------
_shadow = [0x80]


def _drive(dut, cs_n=None, sclk=None, mosi=None):
    cur = _shadow[0]
    if cs_n is not None:
        cur = (cur & ~0x80) | (cs_n << 7)
    if sclk is not None:
        cur = (cur & ~0x40) | (sclk << 6)
    if mosi is not None:
        cur = (cur & ~0x20) | (mosi << 5)
    _shadow[0] = cur
    dut.ui_in.value = cur


async def spi_bit(dut, bit):
    _drive(dut, mosi=bit & 1)
    await cycles(dut, HALF)
    miso = (u(dut.uo_out) >> 7) & 1
    _drive(dut, sclk=1)
    await cycles(dut, HALF)
    _drive(dut, sclk=0)
    return miso


async def spi_byte(dut, val):
    r = 0
    for i in range(7, -1, -1):
        r = (r << 1) | await spi_bit(dut, (val >> i) & 1)
    return r


async def spi_begin(dut):
    _drive(dut, sclk=0, mosi=0)
    await cycles(dut, 2)
    _drive(dut, cs_n=0)
    await cycles(dut, 6)


async def spi_end(dut):
    _drive(dut, sclk=0)
    await cycles(dut, 4)
    _drive(dut, cs_n=1, mosi=0)
    await cycles(dut, 10)


async def spi_write(dut, cmd, addr, data, nbytes):
    await spi_begin(dut)
    await spi_byte(dut, cmd)
    await spi_byte(dut, (addr >> 8) & 0xFF)
    await spi_byte(dut, addr & 0xFF)
    for i in range(nbytes - 1, -1, -1):
        await spi_byte(dut, (data >> (8 * i)) & 0xFF)
    await spi_end(dut)


async def spi_read(dut, cmd, addr, nbytes):
    await spi_begin(dut)
    await spi_byte(dut, cmd)
    await spi_byte(dut, (addr >> 8) & 0xFF)
    await spi_byte(dut, addr & 0xFF)
    await spi_byte(dut, 0x00)  # mandatory dummy byte
    r = 0
    for _ in range(nbytes):
        r = (r << 8) | await spi_byte(dut, 0x00)
    await spi_end(dut)
    return r


async def write_sram0(dut, word_addr, word):
    await spi_write(dut, CMD_WR_SRAM0, word_addr * 4, word, 4)


async def write_sram1(dut, word_addr, word):
    await spi_write(dut, CMD_WR_SRAM1, word_addr * 4, word, 4)


async def read_sram0(dut, word_addr):
    return await spi_read(dut, CMD_RD_SRAM0, word_addr * 4, 4)


async def read_sram1(dut, word_addr):
    return await spi_read(dut, CMD_RD_SRAM1, word_addr * 4, 4)


async def write_csr(dut, addr, value):
    await spi_write(dut, CMD_WR_CSR, addr, value, 2)


async def read_csr(dut, addr):
    return await spi_read(dut, CMD_RD_CSR, addr, 2)


async def reset_dut(dut):
    # one clock coroutine per test: cocotb kills test-scoped coroutines at
    # test end, so a persistent guard would strand later tests clockless.
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    _shadow[0] = 0x80
    dut.ui_in.value = 0x80  # CS_N high, all else low
    dut.uio_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await cycles(dut, 8)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await cycles(dut, 4)


# ----------------------------------------------------------------------
@cocotb.test()
async def test_ae1_reset_xsafe(dut):
    """AE1: after reset every output is defined (no X/Z) and all
    bidirectional pins are inputs."""
    await reset_dut(dut)
    await cycles(dut, 4)
    assert dut.uo_out.value.is_resolvable, "uo_out has X/Z after reset"
    assert dut.uio_out.value.is_resolvable, "uio_out has X/Z after reset"
    assert dut.uio_oe.value.is_resolvable, "uio_oe has X/Z after reset"
    assert u(dut.uio_oe) == 0, f"uio_oe {u(dut.uio_oe):#x} not released"
    assert u(dut.uio_out) == 0, f"uio_out {u(dut.uio_out):#x} not zero"
    # HOST_MISO idles low out of reset; router channels are zero-configured
    assert u(dut.uo_out) == 0, f"uo_out {u(dut.uo_out):#x} not zero"


@cocotb.test()
async def test_host_spi_sram_and_csr(dut):
    """Host SPI writes and reads back both SRAM banks and the CSR space,
    entirely through the pin interface."""
    await reset_dut(dut)

    await write_sram0(dut, 0, 0xDEADBEEF)
    await write_sram0(dut, 1, 0x12345678)
    await write_sram1(dut, 0, 0xCAFEF00D)
    assert await read_sram0(dut, 0) == 0xDEADBEEF, "sram0[0] readback"
    assert await read_sram0(dut, 1) == 0x12345678, "sram0[1] readback"
    assert await read_sram1(dut, 0) == 0xCAFEF00D, "sram1[0] readback"

    # auto-incrementing burst: two words in one transaction
    await spi_begin(dut)
    await spi_byte(dut, CMD_WR_SRAM0)
    await spi_byte(dut, 0x00)
    await spi_byte(dut, 4 * 4)  # byte address of word 4
    for byte in (0xAA, 0x11, 0xBB, 0x22, 0xCC, 0x33, 0xDD, 0x44):
        await spi_byte(dut, byte)
    await spi_end(dut)
    assert await read_sram0(dut, 4) == 0xAA11BB22, "burst word 0"
    assert await read_sram0(dut, 5) == 0xCC33DD44, "burst word 1"

    # CSR write/readback round trip
    await write_csr(dut, CSR_PIN_ROUTER0, 0x0005)
    assert await read_csr(dut, CSR_PIN_ROUTER0) == 0x0005, "csr readback"
    await write_csr(dut, CSR_PIN_ROUTER0, 0x0000)


@cocotb.test()
async def test_multicore_ctrl_run(dut):
    """Firmware loaded through host SPI executes on the cores after one
    CTRL_RUN write; a core output reaches the pin through the router."""
    await reset_dut(dut)

    # route core0 out bit2 onto uo_out[0] (channel 16, PIN_ROUTER4[3:0])
    await write_csr(dut, CSR_PIN_ROUTER4, 0x000B)  # source 11 = paired core

    # firmware: pin_out(mask=4, val=4) sets core_out bit2; then spin.
    fw = [isa.enc_pin_out(4, 4), isa.enc_jmp(1)]
    words = []
    for i in range(0, len(fw), 2):
        lo = fw[i]
        hi = fw[i + 1] if i + 1 < len(fw) else isa.enc_misc(2)
        words.append((hi << 16) | lo)
    for wa, word in enumerate(words):
        await write_sram0(dut, wa, word)

    assert (u(dut.uo_out) & 1) == 0, "uo_out[0] high before CTRL_RUN"
    await write_csr(dut, CSR_CTRL_RUN, 0x000F)

    # all four cores run the same image; core 0 owns the routed pin.
    seen = False
    for _ in range(400):
        await RisingEdge(dut.clk)
        if (u(dut.uo_out) & 1) == 1:
            seen = True
            break
    assert seen, "core0 pin_out bit2 never reached uo_out[0]"

    # stop the cores and watch nothing change further on the pin
    await write_csr(dut, CSR_CTRL_RUN, 0x0000)
    await cycles(dut, 10)
    assert (u(dut.uo_out) & 1) == 1, "output dropped when cores stopped"
    await write_csr(dut, CSR_PIN_ROUTER4, 0x0000)


@cocotb.test()
async def test_rle_capture_through_pins(dut):
    """RLE capture records pin transitions into SRAM 1 with cycle deltas,
    readable back over host SPI."""
    await reset_dut(dut)

    # ring: base 0, mask 0xF (16 words); probe bit 0 = uio_in[0]
    await write_csr(dut, CSR_RLE_BASE, 0x0000)
    await write_csr(dut, CSR_RLE_MASK, 0x000F)
    await write_csr(dut, CSR_RLE_PROBE_MASK, 0x0001)
    await write_csr(dut, CSR_RLE_CTRL, isa.pack_rle_ctrl(mode="capture"))

    # drive a known edge sequence on uio_in[0]: three rising edges with
    # distinct gaps.  Input synchronizer latency is constant, so deltas
    # between records are exact regardless of the absolute delay.
    dut.uio_in.value = 0x00
    await cycles(dut, 40)
    dut.uio_in.value = 0x01
    await cycles(dut, 30)
    dut.uio_in.value = 0x00
    await cycles(dut, 50)
    dut.uio_in.value = 0x01
    await cycles(dut, 20)

    await write_csr(dut, CSR_RLE_CTRL, isa.pack_rle_ctrl(mode="off"))

    # read the first three records.  Edges: rise/fall/rise.  Each record
    # is {delta[15:0], probe[15:0]}; the first record carries the delta
    # from capture start, later records the inter-edge deltas.
    r0 = await read_sram1(dut, 0)
    r1 = await read_sram1(dut, 1)
    r2 = await read_sram1(dut, 2)

    probe0 = r0 & 0xFFFF
    probe1 = r1 & 0xFFFF
    probe2 = r2 & 0xFFFF
    assert probe0 == 0x0001, f"rec0 probe {probe0:#x}: first edge is a rise"
    assert probe1 == 0x0000, f"rec1 probe {probe1:#x}: second edge is fall"
    assert probe2 == 0x0001, f"rec2 probe {probe2:#x}: third edge is rise"

    # inter-edge deltas: falling edge arrived 30 cycles after the rise,
    # next rise 50 cycles after the fall (within synchronizer tolerance).
    d1 = (r1 >> 16) & 0xFFFF
    d2 = (r2 >> 16) & 0xFFFF
    assert 28 <= d1 <= 34, f"delta1 {d1} not near 30"
    assert 48 <= d2 <= 54, f"delta2 {d2} not near 50"
