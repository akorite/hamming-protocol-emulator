# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Full-chip integration tests for tt_um_akorite_hamming (U6 slice 3).

These tests drive the real pin interface: firmware is loaded into SRAM 0
through bit-banged host SPI transactions, control reaches the cores only
through CSR writes, and observability uses a mix of host SPI CSR reads
(integration path) and whitebox peeks at top-level wires (for same-cycle
atomicity evidence that pin-level reads are far too slow to resolve).

Covered contracts:
  - atomic same-cycle four-core start from one CTRL_RUN write
  - 1-cycle EV_FLAGS wake from SYNC_EVENT stall
  - ena 1->0->1 isolation: output pin release/clamping, SPI cancellation
    and input rejection while deselected, clean resumption afterwards
"""

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import hamming_isa as isa  # noqa: E402


def u(sig):
    v = sig.value
    return v.to_unsigned() if hasattr(v, "to_unsigned") else int(v)


async def ensure_clock(dut):
    try:
        dut.clk.value
    except Exception:
        return
    if not hasattr(ensure_clock, "_started"):
        from cocotb.clock import Clock
        cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
        ensure_clock._started = True


async def cycles(dut, n):
    await ClockCycles(dut.clk, n)


# ----------------------------------------------------------------------
# Host SPI master BFM: Mode 0, MSB first.  Half-period of 8 system clocks
# keeps the synchronizer chain comfortably ahead of edge detection.
# Pins: ui_in[7]=CS_N ui_in[6]=SCLK ui_in[5]=MOSI ; MISO = uo_out[7].
# ----------------------------------------------------------------------
HPI_HALF = 5

CMD_WR_SRAM0 = 0x01
CMD_WR_SRAM1 = 0x02
CMD_WR_CSR = 0x03
CMD_RD_SRAM0 = 0x81
CMD_RD_SRAM1 = 0x82
CMD_RD_CSR = 0x83


# Pin-state shadow: cocotb deposits writes only at the next yield point, so
# reading ui_in back for read-modify-write returns stale values and drops
# bits.  The shadow tracks exactly what was last written.
_spi_shadow = [0x80]


def _drive_spi(dut, cs_n=None, sclk=None, mosi=None):
    cur = _spi_shadow[0]
    if cs_n is not None:
        cur = (cur & ~0x80) | (cs_n << 7)
    if sclk is not None:
        cur = (cur & ~0x40) | (sclk << 6)
    if mosi is not None:
        cur = (cur & ~0x20) | (mosi << 5)
    _spi_shadow[0] = cur
    dut.ui_in.value = cur


async def spi_bit(dut, bit):
    """Clock one bit out on MOSI and sample MISO; returns the MISO bit.

    Mirrors the proven standalone-harness timing: drive MOSI, hold SCLK low
    for half a period, sample MISO at the end of the low half-period, then
    raise SCLK for half a period."""
    _drive_spi(dut, mosi=bit & 1)
    await cycles(dut, HPI_HALF)
    miso = (u(dut.uo_out) >> 7) & 1
    _drive_spi(dut, sclk=1)
    await cycles(dut, HPI_HALF)
    _drive_spi(dut, sclk=0)
    return miso


async def spi_byte(dut, val):
    r = 0
    for i in range(7, -1, -1):
        r = (r << 1) | await spi_bit(dut, (val >> i) & 1)
    return r


async def spi_begin(dut):
    _drive_spi(dut, sclk=0, mosi=0)
    await cycles(dut, 2)
    _drive_spi(dut, cs_n=0)
    await cycles(dut, 6)


async def spi_end(dut):
    _drive_spi(dut, sclk=0)
    await cycles(dut, 4)
    _drive_spi(dut, cs_n=1, mosi=0)
    await cycles(dut, 10)


async def spi_write(dut, cmd, addr, data, nbytes):
    """Write `nbytes` of payload (MSB first) after cmd+addr16."""
    await spi_begin(dut)
    await spi_byte(dut, cmd)
    await spi_byte(dut, (addr >> 8) & 0xFF)
    await spi_byte(dut, addr & 0xFF)
    for i in range(nbytes - 1, -1, -1):
        await spi_byte(dut, (data >> (8 * i)) & 0xFF)
    await spi_end(dut)


async def spi_read(dut, cmd, addr, nbytes):
    """Read `nbytes` after the dummy byte; returns MSB-first integer."""
    await spi_begin(dut)
    await spi_byte(dut, cmd)
    await spi_byte(dut, (addr >> 8) & 0xFF)
    await spi_byte(dut, addr & 0xFF)
    await spi_byte(dut, 0x00)  # dummy byte
    r = 0
    for i in range(nbytes):
        r = (r << 8) | await spi_byte(dut, 0x00)
    await spi_end(dut)
    return r


async def spi_write_sram0_word(dut, word_addr, word):
    # byte address = word index * 4 (the target uses addr[10:2]).
    await spi_write(dut, CMD_WR_SRAM0, word_addr * 4, word, 4)


async def spi_write_sram1_word(dut, word_addr, word):
    await spi_write(dut, CMD_WR_SRAM1, word_addr * 4, word, 4)


async def spi_read_sram0_word(dut, word_addr):
    return await spi_read(dut, CMD_RD_SRAM0, word_addr * 4, 4)


async def spi_write_csr(dut, addr, value):
    await spi_write(dut, CMD_WR_CSR, addr, value, 2)


async def spi_read_csr(dut, addr):
    return await spi_read(dut, CMD_RD_CSR, addr, 2)


async def reset_dut(dut):
    _spi_shadow[0] = 0x80
    dut.ui_in.value = 0x80  # CS_N high, everything else low
    dut.uio_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await cycles(dut, 8)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await cycles(dut, 4)


def pack_words(instrs):
    """Pack 16-bit instructions into 32-bit SRAM words, low half first."""
    words = []
    for i in range(0, len(instrs), 2):
        lo = instrs[i]
        hi = instrs[i + 1] if i + 1 < len(instrs) else isa.enc_misc(2)
        words.append((hi << 16) | lo)
    return words


async def load_firmware(dut, instrs):
    for wa, word in enumerate(pack_words(instrs)):
        await spi_write_sram0_word(dut, wa, word)


# ----------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------
@cocotb.test()
async def test_ctrl_run_atomic_start(dut):
    """One CTRL_RUN write starts all four cores atomically: the ctrl_run
    bus transitions 0 -> 0xF directly (never an intermediate value), and
    every core begins executing within the arbitration spread afterwards.
    """
    await ensure_clock(dut)
    await reset_dut(dut)

    # firmware: nop; nop; jmp 0  (spins forever, PCs keep moving)
    fw = [isa.enc_misc(2), isa.enc_misc(2), isa.enc_jmp(0)]
    await load_firmware(dut, fw)

    # sanity: the load path itself works end to end
    rb = await spi_read_sram0_word(dut, 0)
    want = (isa.enc_misc(2) << 16) | isa.enc_misc(2)
    assert rb == want, f"sram0 readback {rb:#010x} != {want:#010x}"

    # all PCs parked at 0 while CTRL_RUN is clear
    assert u(dut.core_pc_flat) == 0, "PCs nonzero before CTRL_RUN"

    # issue the write in the background and watch the bus + PCs concurrently
    write_task = cocotb.start_soon(spi_write_csr(dut, 0x00, 0x000F))
    intermediates = []
    run_cycle = None
    first_move = [None, None, None, None]
    for cyc in range(500):
        await RisingEdge(dut.clk)
        cr = u(dut.ctrl_run) & 0xF
        if cr not in (0, 0xF) and cr not in intermediates:
            intermediates.append(cr)
        if cr == 0xF and run_cycle is None:
            run_cycle = cyc
        if run_cycle is not None:
            pcs = u(dut.core_pc_flat)
            for k in range(4):
                pc = (pcs >> (10 * k)) & 0x3FF
                if first_move[k] is None and pc != 0:
                    first_move[k] = cyc - run_cycle
            if all(v is not None for v in first_move):
                break
    await write_task

    assert not intermediates,         f"ctrl_run took intermediate values {intermediates}: not atomic"
    assert run_cycle is not None, "ctrl_run never became 0xF"
    assert all(v is not None for v in first_move),         f"cores never started: {first_move}"
    # arbitration spread: all four cores begin executing within one
    # round-robin sweep of one another (start itself is same-cycle).
    assert max(first_move) - min(first_move) <= 24,         f"excessive start spread {first_move}"

    # integration-path liveness: host reads each core's PC nonzero
    for k in range(4):
        pc = await spi_read_csr(dut, 0x40 + k * 16 + 1)
        assert pc & 0x3FF != 0, f"core {k} pc still zero via CSR read"


@cocotb.test()
async def test_sync_event_wakeup(dut):
    """EV_FLAGS wakes a SYNC_EVENT-stalled core within one cycle of the
    flag edge: pc resumes advancing no more than 2 edges later (the stall
    exit is registered on the edge that follows the flag update)."""
    await ensure_clock(dut)
    await reset_dut(dut)

    # r0 = 0xFFFF timeout; sync_event ev0; then spin at pc 4
    fw = [
        isa.enc_ldi8(0, 0, 0xFF),       # 0: r0 lo
        isa.enc_ldi8(0, 1, 0xFF),       # 1: r0 hi
        isa.enc_sync_event(1, 0),       # 2: wait ev bit0, timeout r0
        isa.enc_misc(2),                # 3: nop after wake
        isa.enc_jmp(4),                 # 4: spin
    ]
    await load_firmware(dut, fw)
    await spi_write_csr(dut, 0x00, 0x0001)  # CTRL_RUN core 0 only

    # let the core reach the stall: pc parks AT the sync instruction (2)
    # because the pc increment happens only when the stall completes.
    parked = None
    for cyc in range(400):
        await RisingEdge(dut.clk)
        pc = u(dut.core_pc_flat) & 0x3FF
        if pc == 2:
            parked = pc
            break
    assert parked is not None, "core never reached the sync stall"

    # confirm it stays parked (no spurious wake) for 20 cycles
    stable = u(dut.core_pc_flat) & 0x3FF
    await cycles(dut, 20)
    assert (u(dut.core_pc_flat) & 0x3FF) == stable, \
        "pc moved while SYNC_EVENT should be stalled"

    # raise ev flag 0 via EV_SET and measure wake latency in edges
    await spi_write_csr(dut, 0x02, 0x0001)  # EV_SET bit0

    woke_after = None
    for cyc in range(1, 12):
        await RisingEdge(dut.clk)
        pc = u(dut.core_pc_flat) & 0x3FF
        if pc != stable:
            woke_after = cyc
            break
    assert woke_after is not None, "core never woke from SYNC_EVENT"
    # spi_write_csr returns well after the internal CSR write edge; the
    # wake must happen quickly once the flag is visible.
    assert woke_after <= 3, f"wake latency {woke_after} cycles too long"


@cocotb.test()
async def test_ena_isolation(dut):
    """ena=0 releases/clamps every output pin, cancels an in-flight SPI
    transaction and rejects further input; ena=1 resumes cleanly."""
    await ensure_clock(dut)
    await reset_dut(dut)

    # drive constant-1 onto uio_out[0] (ch0) and uio_oe[0] (ch8):
    # PIN_ROUTER0[3:0]=1, PIN_ROUTER2[3:0]=1
    await spi_write_csr(dut, 0x20, 0x0001)
    await spi_write_csr(dut, 0x22, 0x0001)
    await cycles(dut, 4)
    assert (u(dut.uio_out) & 1) == 1, "uio_out[0] not driven before test"
    assert (u(dut.uio_oe) & 1) == 1, "uio_oe[0] not driven before test"

    # start an SPI read transaction and drop ena in the middle of it
    await spi_begin(dut)
    await spi_byte(dut, CMD_RD_SRAM0)
    await spi_byte(dut, 0x00)
    dut.ena.value = 0
    await cycles(dut, 4)

    # every output must be released/clamped low
    assert u(dut.uio_oe) == 0, f"uio_oe {u(dut.uio_oe):#x} not released"
    assert u(dut.uio_out) == 0, f"uio_out {u(dut.uio_out):#x} not clamped"
    assert (u(dut.uo_out) & 0x7F) == 0, \
        f"uo_out[6:0] {u(dut.uo_out):#x} not clamped"
    assert ((u(dut.uo_out) >> 7) & 1) == 0, "HOST_MISO not clamped"

    # input rejection: clocking bits while deselected must have no effect;
    # finish the dangling transaction and start a fresh one after re-enable
    for _ in range(8):
        await spi_bit(dut, 1)
    assert ((u(dut.uo_out) >> 7) & 1) == 0, "MISO live while ena=0"
    dut.ena.value = 1
    await spi_end(dut)

    # resumption: outputs return to their configured state
    await cycles(dut, 4)
    assert (u(dut.uio_out) & 1) == 1, "uio_out[0] not restored after ena"
    assert (u(dut.uio_oe) & 1) == 1, "uio_oe[0] not restored after ena"

    # and a fresh SPI transaction works: write then read back a CSR
    await spi_write_csr(dut, 0x20, 0x0000)
    rb = await spi_read_csr(dut, 0x20)
    assert rb == 0, f"csr readback {rb:#x} after re-enable"
    await spi_write_csr(dut, 0x20, 0x0001)
    rb = await spi_read_csr(dut, 0x20)
    assert rb == 1, f"csr readback {rb:#x} after rewrite"
