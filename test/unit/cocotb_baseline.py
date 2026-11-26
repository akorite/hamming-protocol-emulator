# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Pin-level full-chip harness for the U7 baseline-protocol matrix.

The harness deliberately interacts with the chip the same way the RP2350 host
will: firmware and packet mailboxes are loaded through bit-banged Host SPI,
fabric configuration is written through CSRs, and protocol traffic is driven
only on ui/uio pins.  It adds burst SRAM writes so multi-instruction firmware
images do not cost one Host SPI transaction per word.
"""

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "test") not in sys.path:
    sys.path.insert(0, str(ROOT / "test"))

from tools import hamming_asm as asm  # noqa: E402
from tools import hamming_isa as isa  # noqa: E402

HPI_HALF = 5
CMD_WR_SRAM0 = 0x01
CMD_WR_SRAM1 = 0x02
CMD_WR_CSR = 0x03
CMD_RD_SRAM0 = 0x81
CMD_RD_SRAM1 = 0x82
CMD_RD_CSR = 0x83

CSR_CTRL_RUN = isa.CSR["CTRL_RUN"]
CSR_EV_SET = isa.CSR["EV_SET"]
CSR_EV_CLR = isa.CSR["EV_CLR"]
CSR_PIN_FILT_EN = isa.CSR["PIN_FILT_EN"]
CSR_OPEN_DRAIN_EN = isa.CSR["OPEN_DRAIN_EN"]
CSR_RLE_BASE = isa.CSR["RLE_BASE"]
CSR_RLE_MASK = isa.CSR["RLE_MASK"]
CSR_RLE_CTRL = isa.CSR["RLE_CTRL"]
CSR_RLE_PROBE_MASK = isa.CSR["RLE_PROBE_MASK"]
CSR_RLE_STATUS = isa.CSR["RLE_STATUS"]
CSR_PIN_ROUTER0 = isa.CSR["PIN_ROUTER0"]

SRC_ZERO = isa.ROUTER_SRC["zero"]
SRC_ONE = isa.ROUTER_SRC["one"]
SRC_CORE = isa.ROUTER_SRC["core_out"]
SRC_BCU_P = isa.ROUTER_SRC["bcu_tx_p"]
SRC_BCU_N = isa.ROUTER_SRC["bcu_tx_n"]
SRC_BCU_OE = isa.ROUTER_SRC["bcu_tx_oe"]

MEM_STATUS = 1
MEM_CFG = 2
MEM_COUNT = 3
MEM_CONTROL = 4
MEM_TX = 8
MEM_RX = 0x40


def u(sig):
    v = sig.value
    return v.to_unsigned() if hasattr(v, "to_unsigned") else int(v)


async def cycles(dut, n):
    await ClockCycles(dut.clk, int(n))


# ----------------------------------------------------------------------
# Pin-input shadows.  Host SPI and the protocol BFMs share ui_in/uio_in, so
# every writer goes through these bit-preserving helpers (never read-modify-
# write the cocotb input signal itself).
# ----------------------------------------------------------------------
_ui_shadow = [0x80]
_uio_shadow = [0x00]
_router_shadow = [{}]


def write_ui_shadow(dut, value):
    _ui_shadow[0] = value & 0xFF
    dut.ui_in.value = _ui_shadow[0]


def write_uio_shadow(dut, value):
    _uio_shadow[0] = value & 0xFF
    dut.uio_in.value = _uio_shadow[0]


def drive_ui_bit(dut, bit, level):
    value = (_ui_shadow[0] & ~(1 << bit)) | ((level & 1) << bit)
    write_ui_shadow(dut, value)


def drive_uio_bit(dut, bit, level):
    value = (_uio_shadow[0] & ~(1 << bit)) | ((level & 1) << bit)
    write_uio_shadow(dut, value)


def ui_bit(dut, bit):
    return (u(dut.ui_in) >> bit) & 1


def uio_out_bit(dut, bit):
    return (u(dut.uio_out) >> bit) & 1


def uio_oe_bit(dut, bit):
    return (u(dut.uio_oe) >> bit) & 1


def uo_bit(dut, bit):
    return (u(dut.uo_out) >> bit) & 1


def uart_tx_line_bit(dut, bit):
    """Sample a UART TX pad with its hi-Z interval treated as idle high.

    RS-232-style UART has a positive idle level.  The BCU correctly releases
    its output enable between independent firmware-pushed frames; while the
    pad is released there is no driven low start/data level, so the wire model
    must not expose the router's dormant data bit as a dominant low.
    """
    return uio_out_bit(dut, bit) if uio_oe_bit(dut, bit) else 1


# ----------------------------------------------------------------------
# Host SPI master BFM (Mode 0), compatible with the proven U6 harness.
# ----------------------------------------------------------------------
def _drive_spi(dut, cs_n=None, sclk=None, mosi=None):
    cur = _ui_shadow[0]
    if cs_n is not None:
        cur = (cur & ~0x80) | ((cs_n & 1) << 7)
    if sclk is not None:
        cur = (cur & ~0x40) | ((sclk & 1) << 6)
    if mosi is not None:
        cur = (cur & ~0x20) | ((mosi & 1) << 5)
    write_ui_shadow(dut, cur)


async def spi_bit(dut, bit):
    _drive_spi(dut, mosi=bit)
    await cycles(dut, HPI_HALF)
    miso = uo_bit(dut, 7)
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
    await spi_begin(dut)
    await spi_byte(dut, cmd)
    await spi_byte(dut, (addr >> 8) & 0xFF)
    await spi_byte(dut, addr & 0xFF)
    for i in range(nbytes - 1, -1, -1):
        await spi_byte(dut, (data >> (8 * i)) & 0xFF)
    await spi_end(dut)


async def spi_write_payload(dut, cmd, addr, payload):
    """One auto-incrementing write transaction for an MSB-first byte list."""
    await spi_begin(dut)
    await spi_byte(dut, cmd)
    await spi_byte(dut, (addr >> 8) & 0xFF)
    await spi_byte(dut, addr & 0xFF)
    for byte in payload:
        await spi_byte(dut, byte & 0xFF)
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


async def write_sram0_words(dut, start_word, words):
    payload = []
    for word in words:
        payload.extend((word >> shift) & 0xFF for shift in (24, 16, 8, 0))
    await spi_write_payload(dut, CMD_WR_SRAM0, start_word * 4, payload)


async def write_sram1_word(dut, word_addr, word):
    await spi_write(dut, CMD_WR_SRAM1, word_addr * 4, word, 4)


async def write_sram1_words(dut, start_word, words):
    payload = []
    for word in words:
        payload.extend((word >> shift) & 0xFF for shift in (24, 16, 8, 0))
    await spi_write_payload(dut, CMD_WR_SRAM1, start_word * 4, payload)


async def read_sram1_word(dut, word_addr):
    return await spi_read(dut, CMD_RD_SRAM1, word_addr * 4, 4)


async def write_csr(dut, addr, value):
    await spi_write(dut, CMD_WR_CSR, addr, value, 2)


async def read_csr(dut, addr):
    return await spi_read(dut, CMD_RD_CSR, addr, 2)


async def reset_dut(dut, start_clock=True):
    # Every cocotb test starts its own clock because test-scoped coroutines
    # are killed when the test ends.  Multi-scenario tests may reset the DUT
    # again with start_clock=False while that same clock coroutine continues.
    if start_clock:
        cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    write_ui_shadow(dut, 0x80)
    write_uio_shadow(dut, 0x00)
    _router_shadow[0] = {}
    dut.ena.value = 1
    dut.rst_n.value = 0
    await cycles(dut, 8)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await cycles(dut, 4)


# ----------------------------------------------------------------------
# Firmware image construction/loading
# ----------------------------------------------------------------------
def firmware_text(name):
    return (ROOT / "firmware" / name).read_text()


def program_words_for_source(source):
    prog = asm.assemble_program(source)
    nwords = (prog.used_slots + 1) // 2
    return prog.words[:nwords], prog


async def load_program_source(dut, source):
    words, prog = program_words_for_source(source)
    await write_sram0_words(dut, 0, words)
    return prog


async def load_firmware(dut, name):
    return await load_program_source(dut, firmware_text(name))


def strip_annotation_only_lines(text):
    """Keep a firmware file intact; annotations are harmless when concatenated."""
    return text.strip() + "\n"


def controller_concurrent_source():
    """Build the U7 concurrent image using private-bank core-ID probing.

    CORE_ID is not exposed architecturally, but a core can write only its own
    private CSR bank.  A probe of the banked BCU_PAT_MATCH scratch register
    therefore identifies the executing core without modifying RTL.  Section
    origins are sized for the real firmware bodies; the whole image remains
    below the architectural 1024-instruction limit.
    """
    dispatch = """
; concurrent-image core selector: write token to own bank, scan bank0..bank3
    ldi8.l r0, 0x5a
    csr_wr r0, 0x4a
    csr_rd r1, 0x4a
    cmp r0, r1
    jz uart_entry
    csr_wr r0, 0x5a
    csr_rd r1, 0x5a
    cmp r0, r1
    jz spi_ctrl_entry
    csr_wr r0, 0x6a
    csr_rd r1, 0x6a
    cmp r0, r1
    jz i2c_ctrl_entry
    csr_wr r0, 0x7a
    csr_rd r1, 0x7a
    cmp r0, r1
    jz wave_entry
    halt

; Deterministic waveform on core3 pin bit2: four 20-cycle pulses separated
; by 20-cycle low gaps.  It is intentionally inside the dispatch section.
wave_entry:
    pin_out 0x4, 0x0
    ldi8.l r3, 20
    pin_out 0x4, 0x4
    set_anchor 0
    wait_phase r3
    pin_out 0x4, 0x0
    set_anchor 0
    wait_phase r3
    pin_out 0x4, 0x4
    set_anchor 0
    wait_phase r3
    pin_out 0x4, 0x0
    set_anchor 0
    wait_phase r3
    pin_out 0x4, 0x4
    set_anchor 0
    wait_phase r3
    pin_out 0x4, 0x0
    set_anchor 0
    wait_phase r3
    pin_out 0x4, 0x4
    set_anchor 0
    wait_phase r3
    pin_out 0x4, 0x0
    halt

.org 0x020
"""
    parts = [dispatch, strip_annotation_only_lines(firmware_text("uart.hasm"))]
    parts.append("\n.org 0x1a0\n")
    parts.append(strip_annotation_only_lines(firmware_text("spi_ctrl.hasm")))
    parts.append("\n.org 0x2c0\n")
    parts.append(strip_annotation_only_lines(firmware_text("i2c_ctrl.hasm")))
    return "\n".join(parts)


# ----------------------------------------------------------------------
# Fabric configuration helpers
# ----------------------------------------------------------------------
def lut_tt_identity_in0():
    # Truth table output = input0: set for every index whose low bit is one.
    return 0xAAAA


def lut_tt_expr2(fn):
    """Compile a two-input Python fn(in0,in1) to a 16-bit truth table."""
    tt = 0
    for idx in range(16):
        bit = 1 if fn(idx & 1, (idx >> 1) & 1) else 0
        tt |= bit << idx
    return tt


def pack_lut_sel(sel0, sel1=15, sel2=15, sel3=15):
    return ((sel0 & 0xF) | ((sel1 & 0xF) << 4) |
            ((sel2 & 0xF) << 8) | ((sel3 & 0xF) << 12))


async def write_lut(dut, index, truth_table, selectors):
    await write_csr(dut, isa.CSR_LUT_TT_BASE + index, truth_table)
    await write_csr(dut, isa.CSR_LUT_SEL_BASE + index,
                    pack_lut_sel(*selectors))


async def write_pin_router(dut, sources):
    """Merge and write the 23-channel router from a dict {channel: source}.

    Protocol setup helpers are composable in the concurrent test, so this is a
    host-side shadow of the reset-only CSR array rather than a destructive
    sparse write.
    """
    _router_shadow[0].update(sources)
    merged = _router_shadow[0]
    packed = []
    for group in range(6):
        value = 0
        for nib in range(4):
            ch = group * 4 + nib
            if ch < 23:
                value |= (merged.get(ch, SRC_ZERO) & 0xF) << (4 * nib)
        packed.append(value)
    for index, value in enumerate(packed):
        await write_csr(dut, CSR_PIN_ROUTER0 + index, value)


async def write_bcu_config(dut, core, mode, bit_len, dpll_cfg, pat_match=0, pat_len=0):
    base = isa.CSR_CORE_BANK + core * 16
    await write_csr(dut, base + isa.CORE_CSR["BCU_MODE"], mode)
    await write_csr(dut, base + isa.CORE_CSR["BCU_BIT_LEN"], bit_len)
    await write_csr(dut, base + isa.CORE_CSR["BCU_DPLL_CFG"], dpll_cfg)
    await write_csr(dut, base + isa.CORE_CSR["BCU_PAT_MATCH"], pat_match)
    await write_csr(dut, base + isa.CORE_CSR["BCU_PAT_LEN"], pat_len)


async def configure_uart_bcu_pins(dut, rx_pin=8, tx_uio=0):
    # LUT0 is BCU0 rx_p.  uio0 data/OE come from BCU0 tx_p/tx_oe.
    await write_lut(dut, 0, lut_tt_identity_in0(), (rx_pin, 15, 15, 15))
    await write_pin_router(dut, {
        tx_uio: SRC_BCU_P,
        8 + tx_uio: SRC_BCU_OE,
    })


async def configure_uart_bitbang_pins(dut, tx_uio=0):
    await write_pin_router(dut, {
        tx_uio: SRC_CORE,
        8 + tx_uio: SRC_CORE,
    })


async def configure_spi_controller_pins(dut):
    # core1: SCLK=uio1 (core bit0), MOSI=uio5 (BCU1), CS_N=uo_out1 (core bit2).
    # LUT2 passes MISO on ui_in[1] (pin index 9) into BCU1 rx_p.
    await write_lut(dut, 2, lut_tt_identity_in0(), (9, 15, 15, 15))
    await write_pin_router(dut, {
        1: SRC_CORE, 9: SRC_CORE,
        5: SRC_BCU_P, 13: SRC_BCU_OE,
        17: SRC_CORE,
    })


async def configure_spi_target_pins(dut, mode):
    # core3 target pin map: MISO=uio3, SCLK=uio4, CS_N=uio5, MOSI=uio6.
    # LUT6 passes MOSI to BCU3 rx_p.  LUT7 generates the gated/polarity-
    # corrected external clock: E = ((invert ? ~SCLK : SCLK) | CS_N).
    invert = 1 if ((mode & 1) != ((mode >> 1) & 1)) else 0

    def extclk_fn(sclk, cs_n):
        active_clock = (1 - sclk) if invert else sclk
        return 1 if (active_clock or cs_n) else 0

    await write_lut(dut, 6, lut_tt_identity_in0(), (6, 15, 15, 15))
    await write_lut(dut, 7, lut_tt_expr2(extclk_fn), (4, 5, 15, 15))
    await write_pin_router(dut, {
        3: SRC_BCU_P,
        11: SRC_CORE,      # core3 gates the actual pad OE with CS
    })


async def configure_i2c_pins(dut, pin_filter=True):
    # core2: SDA=uio2 (core bit0), SCL=uio6 (core bit1).  The post-router
    # open-drain clamp is enabled as a second layer of AE5 safety; firmware
    # nevertheless uses OE-only open-drain control.
    await write_pin_router(dut, {
        2: SRC_CORE, 10: SRC_CORE,
        6: SRC_CORE, 14: SRC_CORE,
    })
    mask = (1 << 2) | (1 << 6)
    await write_csr(dut, CSR_OPEN_DRAIN_EN, mask)
    await write_csr(dut, CSR_PIN_FILT_EN, mask if pin_filter else 0)


async def configure_waveform_pin(dut):
    await write_pin_router(dut, {
        3: SRC_CORE,
        11: SRC_CORE,
    })


# ----------------------------------------------------------------------
# SRAM 1 mailbox helpers (byte lanes are little-endian in each 32-bit word)
# ----------------------------------------------------------------------
def pack_le_bytes(data, width_multiple=4):
    data = list(data)
    while len(data) % width_multiple:
        data.append(0)
    words = []
    for pos in range(0, len(data), 4):
        words.append(sum((data[pos + lane] & 0xFF) << (8 * lane)
                         for lane in range(4)))
    return words


async def write_sram1_bytes(dut, byte_addr, data):
    assert byte_addr % 4 == 0, "helper requires word-aligned byte address"
    await write_sram1_words(dut, byte_addr // 4, pack_le_bytes(data))


async def read_sram1_bytes(dut, byte_addr, count):
    assert byte_addr % 4 == 0, "helper requires word-aligned byte address"
    out = []
    for pos in range(0, count, 4):
        word = await read_sram1_word(dut, byte_addr // 4 + pos // 4)
        take = min(4, count - pos)
        out.extend((word >> (8 * lane)) & 0xFF for lane in range(take))
    return bytes(out)


async def clear_mailbox(dut):
    await write_sram1_words(dut, 0, [0] * 32)


async def core_status(dut, core):
    return await read_csr(dut, isa.CSR_CORE_BANK + core * 16 +
                          isa.CORE_CSR["CORE_STATUS"])


async def wait_mailbox_status(dut, expected_mask=1, timeout_writes=20):
    """Poll status byte through SRAM1 word reads (host-speed, non-realtime)."""
    for _ in range(timeout_writes):
        word = await read_sram1_word(dut, 0)
        status = (word >> 8) & 0xFF
        if status & expected_mask:
            return status
    raise AssertionError("mailbox status never became ready")


# ----------------------------------------------------------------------
# Small pad helpers
# ----------------------------------------------------------------------
async def loopback_uio_bits(dut, bits):
    """Model pad input loopback for output-only waveform bits."""
    while True:
        await RisingEdge(dut.clk)
        for bit in bits:
            if uio_oe_bit(dut, bit):
                drive_uio_bit(dut, bit, uio_out_bit(dut, bit))


# ----------------------------------------------------------------------
# U7 protocol-matrix tests
# ----------------------------------------------------------------------
from bfm.uart_bfm import (  # noqa: E402
    UARTReceiver, UARTTransmitter, encode_frame,
)
from bfm.spi_bfm import SPIControllerBfm, SPITargetBfm  # noqa: E402
from bfm.i2c_bfm import (  # noqa: E402
    I2CControllerBfm, I2CTargetBfm, OpenDrainI2CBus, sda_arbitration_pulse,
)


async def _uart_high_case(dut, drift, first_clock):
    """One 3 Mbps BCU UART full-duplex case with signed external drift."""
    await reset_dut(dut, start_clock=first_clock)
    drive_ui_bit(dut, 0, 1)  # asynchronous RX idle high
    await load_firmware(dut, "uart.hasm")
    await configure_uart_bcu_pins(dut, rx_pin=8, tx_uio=0)
    await write_bcu_config(
        dut, 0,
        isa.pack_bcu_mode(codec="nrz", stuff="off"),
        (11 - 1),
        isa.pack_dpll_cfg(mode="hard_snap", win=0, period=20),
    )
    await clear_mailbox(dut)
    dut_tx_bytes = bytes([0x55, 0xA5, 0x3C, 0xC3])
    bfm_tx_bytes = bytes([0x96, 0x17, 0xE1, 0x42])
    await write_sram1_bytes(dut, MEM_TX, dut_tx_bytes)
    # cfg byte 1 selects the BCU high-speed UART engine in uart.hasm.
    await write_sram1_bytes(dut, 0, bytes([0, 0, 1, 4]))

    tx = UARTTransmitter(
        dut.clk, lambda bit: drive_ui_bit(dut, 0, bit),
        bit_cycles=20, parity="even", drift=drift,
        start_delay_cycles=100)
    rx = UARTReceiver(
        dut.clk, lambda: uart_tx_line_bit(dut, 0),
        bit_cycles=20, parity="even", drift=0)
    # CTRL_RUN becomes visible near the end of its Host SPI transaction and
    # the BCU emits the first start bit only a few cycles later.  The wire
    # receiver must therefore already be listening while the host write is in
    # flight; waiting for write_csr to return misses the first frame.
    rx_task = cocotb.start_soon(rx.receive_bytes(len(dut_tx_bytes), 30000))
    run_task = cocotb.start_soon(write_csr(dut, CSR_CTRL_RUN, 0x0001))
    await run_task
    tx_task = cocotb.start_soon(tx.send_bytes(bfm_tx_bytes))
    frames = await rx_task
    await tx_task

    assert [f.data for f in frames] == list(dut_tx_bytes), \
        f"DUT UART TX data mismatch: {[f.data for f in frames]}"
    assert not any(f.framing_error for f in frames), "DUT UART TX framing error"
    assert not any(f.parity_error for f in frames), "DUT UART TX parity error"

    status = await wait_mailbox_status(dut, 1, 8)
    assert status == 1, f"UART mailbox error/status 0x{status:02X}"
    got = await read_sram1_bytes(dut, MEM_RX, len(bfm_tx_bytes))
    assert got == bfm_tx_bytes, \
        f"DUT UART RX mailbox {got.hex()} != {bfm_tx_bytes.hex()}"
    st0 = await core_status(dut, 0)
    assert (st0 >> 3) & 1 == 0, f"UART core deadline_missed status {st0:#x}"


@cocotb.test()
async def test_uart_full_duplex_matrix(dut):
    """UART 3 Mbps full-duplex with +/-2% external baud mismatch."""
    await _uart_high_case(dut, +0.02, first_clock=True)
    await _uart_high_case(dut, -0.02, first_clock=False)


@cocotb.test()
async def debug_uart_probe(dut):
    """Temporary whitebox probe used only while bring-up is red."""
    await reset_dut(dut, start_clock=True)
    drive_ui_bit(dut, 0, 1)
    prog = await load_firmware(dut, "uart.hasm")
    rb0 = await spi_read(dut, CMD_RD_SRAM0, 0, 4)
    print(f"DBG uart slots={prog.used_slots} sram0w0={rb0:08x} want={prog.words[0]:08x}")
    await configure_uart_bcu_pins(dut, rx_pin=8, tx_uio=0)
    await write_bcu_config(
        dut, 0,
        isa.pack_bcu_mode(codec="nrz", stuff="off"),
        (11 - 1),
        isa.pack_dpll_cfg(mode="hard_snap", win=0, period=20),
    )
    await clear_mailbox(dut)
    await write_sram1_bytes(dut, MEM_TX, bytes([0x55, 0xA5, 0x3C, 0xC3]))
    await write_sram1_bytes(dut, 0, bytes([0, 0, 1, 4]))
    rb = await read_sram1_word(dut, 0)
    rb2 = await read_sram1_word(dut, 2)
    print(f"DBG mailbox0={rb:08x} txword={rb2:08x}")
    print(f"DBG pre router={u(dut.pin_router_flat):x} lutsel={u(dut.lut_sel_flat):x} luttt={u(dut.lut_tt_flat):x}")
    async def mon():
        last = None
        for i in range(1200):
            await RisingEdge(dut.clk)
            line = uart_tx_line_bit(dut, 0)
            raw = (uio_out_bit(dut, 0), uio_oe_bit(dut, 0),
                   u(dut.gen_core[0].u_bcu.tx_p), u(dut.gen_core[0].u_bcu.tx_oe))
            if line != last:
                print(f"DBG mon cyc{i} line={line} raw={raw} pc={u(dut.core_pc_flat)&0x3ff:03x}")
                last = line
    cocotb.start_soon(mon())
    async def rxmon():
        last = None
        lastfull = 0
        for i in range(2200):
            await RisingEdge(dut.clk)
            rxp = u(dut.gen_core[0].u_bcu.rx_p)
            sample = u(dut.gen_core[0].u_bcu.dpll_sample)
            full = u(dut.gen_core[0].u_bcu.rx_full)
            if rxp != last:
                print(f"DBG rxpin cyc{i} rxp={rxp} pc={u(dut.core_pc_flat)&0x3ff:03x}")
                last = rxp
            if sample:
                print(f"DBG sample cyc{i} lvl={rxp} cnt={u(dut.gen_core[0].u_bcu.rx_cnt)} sh={u(dut.gen_core[0].u_bcu.rx_shift):04x}")
            if full and not lastfull:
                print(f"DBG rxfull cyc{i} data={u(dut.gen_core[0].u_bcu.rx_data):04x}")
            lastfull = full
    cocotb.start_soon(rxmon())
    rx = UARTReceiver(dut.clk, lambda: uart_tx_line_bit(dut, 0), 20, parity="even")
    rx_task = cocotb.start_soon(rx.receive_bytes(1, 5000))
    run_task = cocotb.start_soon(write_csr(dut, CSR_CTRL_RUN, 0x0001))
    await run_task
    tx = UARTTransmitter(dut.clk, lambda bit: drive_ui_bit(dut, 0, bit), 20, parity="even", drift=0.02)
    tx_task = cocotb.start_soon(tx.send_bytes(bytes([0x96, 0x17, 0xE1, 0x42])))
    try:
        frames = await rx_task
        print("DBG local rx frames", frames)
    except Exception as e:
        print("DBG local rx err", repr(e))
    await tx_task
    for n in (20, 100, 500, 2000):
        await cycles(dut, n)
        pcs = u(dut.core_pc_flat)
        print(
            "DBG t",
            n,
            f"pc={pcs & 0x3ff:03x}",
            f"ctrl={u(dut.ctrl_run):x}",
            f"uo={u(dut.uo_out):02x} uio_out={u(dut.uio_out):02x} uio_oe={u(dut.uio_oe):02x}",
            f"bcu_ctrl={u(dut.gen_core[0].core_bcu_ctrl):02x}",
            f"txp={u(dut.gen_core[0].u_bcu.tx_p)} txoe={u(dut.gen_core[0].u_bcu.tx_oe)}",
            f"txs={u(dut.gen_core[0].u_bcu.txs)} txshv={u(dut.gen_core[0].u_bcu.tx_shadow_v)}",
            f"txcnt={u(dut.gen_core[0].u_bcu.tx_sym_cnt)} stat={u(dut.gen_core[0].u_bcu.stat):04x}",
            f"rxfull={u(dut.gen_core[0].u_bcu.rx_full)} rxp={u(dut.gen_core[0].u_bcu.rx_p)}",
        )


async def _uart_low_case(dut, first_clock):
    """115200 bps full duplex using the firmware's core0/core1 bitbang roles."""
    await reset_dut(dut, start_clock=first_clock)
    drive_ui_bit(dut, 0, 1)  # external RX idle high
    await load_firmware(dut, "uart.hasm")
    await configure_uart_bitbang_pins(dut, tx_uio=0)
    await clear_mailbox(dut)
    dut_tx_bytes = bytes([0x55, 0xA5, 0x3C, 0xC3])
    bfm_tx_bytes = bytes([0x96, 0x17, 0xE1, 0x42])
    await write_sram1_bytes(dut, MEM_TX, dut_tx_bytes)
    # cfg byte 2 clear selects the low-rate split-role bitbang path.
    await write_sram1_bytes(dut, 0, bytes([0, 0, 0, 4]))

    nominal = 521  # round(60 MHz / 115200 baud)
    tx = UARTTransmitter(
        dut.clk, lambda bit: drive_ui_bit(dut, 0, bit),
        bit_cycles=nominal, parity="even", drift=0,
        start_delay_cycles=100)
    rx = UARTReceiver(
        dut.clk, lambda: uart_tx_line_bit(dut, 0),
        bit_cycles=nominal, parity="even", drift=0)
    rx_task = cocotb.start_soon(rx.receive_bytes(len(dut_tx_bytes), 40000))
    run_task = cocotb.start_soon(write_csr(dut, CSR_CTRL_RUN, 0x0003))
    await run_task
    tx_task = cocotb.start_soon(tx.send_bytes(bfm_tx_bytes))
    frames = await rx_task
    await tx_task

    assert [f.data for f in frames] == list(dut_tx_bytes), \
        f"115200 DUT TX data mismatch: {[f.data for f in frames]}"
    assert not any(f.framing_error or f.parity_error for f in frames)
    status = await wait_mailbox_status(dut, 1, 10)
    assert status == 1, f"115200 UART mailbox error/status 0x{status:02X}"
    got = await read_sram1_bytes(dut, MEM_RX, len(bfm_tx_bytes))
    assert got == bfm_tx_bytes, \
        f"115200 DUT RX mailbox {got.hex()} != {bfm_tx_bytes.hex()}"
    for core in (0, 1):
        st = await core_status(dut, core)
        assert (st >> 3) & 1 == 0, f"115200 UART core{core} deadline_missed"


async def _uart_injected_error_case(dut, kind, first_clock):
    """Inject one malformed high-speed frame and require firmware error status."""
    await reset_dut(dut, start_clock=first_clock)
    drive_ui_bit(dut, 0, 1)
    await load_firmware(dut, "uart.hasm")
    await configure_uart_bcu_pins(dut, rx_pin=8, tx_uio=0)
    await write_bcu_config(
        dut, 0,
        isa.pack_bcu_mode(codec="nrz", stuff="off"),
        (11 - 1),
        isa.pack_dpll_cfg(mode="hard_snap", win=0, period=20),
    )
    await clear_mailbox(dut)
    await write_sram1_bytes(dut, MEM_TX, bytes([0x55, 0xA5, 0x3C, 0xC3]))
    await write_sram1_bytes(dut, 0, bytes([0, 0, 1, 4]))

    bits = list(encode_frame(0x96))
    if kind == "framing":
        bits[-1] = 0          # missing stop bit
        expected = 0x10
    elif kind == "parity":
        bits[9] ^= 1          # corrupt even parity
        expected = 0x20
    else:
        raise ValueError(kind)

    tx = UARTTransmitter(
        dut.clk, lambda bit: drive_ui_bit(dut, 0, bit),
        bit_cycles=20, parity="even", start_delay_cycles=100)
    run_task = cocotb.start_soon(write_csr(dut, CSR_CTRL_RUN, 0x0001))
    await run_task
    await tx.send_bits(bits)
    drive_ui_bit(dut, 0, 1)   # restore idle after an intentionally low stop
    await cycles(dut, 300)
    # Whitebox inspection during bring-up showed rx_full set and the expected
    # error bit in BCU status; the externally meaningful result is checked
    # through the mailbox below.
    word = await read_sram1_word(dut, 0)
    status = (word >> 8) & 0xFF
    assert status == expected, \
        f"UART injected {kind}: status 0x{status:02X}, expected 0x{expected:02X}"


@cocotb.test()
async def test_uart_low_speed_and_error_injection(dut):
    """115200 bps full duplex plus hardware framing/parity-error detection."""
    await _uart_low_case(dut, first_clock=True)
    await _uart_injected_error_case(dut, "framing", first_clock=False)
    await _uart_injected_error_case(dut, "parity", first_clock=False)


# ----------------------------------------------------------------------
# SPI matrices
# ----------------------------------------------------------------------
async def _spi_ctrl_case(dut, mode, fast, first_clock):
    await reset_dut(dut, start_clock=first_clock)
    await load_firmware(dut, "spi_ctrl.hasm")
    await configure_spi_controller_pins(dut)
    period = 6 if fast else 60
    await write_bcu_config(
        dut, 1, isa.pack_bcu_mode(codec="nrz", stuff="off"),
        (8 - 1) | 0x10, isa.pack_dpll_cfg(mode="none", win=0, period=period))
    await clear_mailbox(dut)
    mosi_expect = bytes([0xA5, 0x3C])
    miso_expect = bytes([0x18, 0xE7])
    await write_sram1_bytes(dut, MEM_TX, mosi_expect)
    cfg = mode | (0x10 if fast else 0)
    await write_sram1_bytes(dut, 0, bytes([0, 0, cfg, 4]))

    async def serve_independent_byte_cs():
        """Serve the firmware's four legal one-byte chip-select transactions."""
        captured = bytearray()
        for byte in miso_expect:
            target = SPITargetBfm(
                dut.clk,
                lambda: uo_bit(dut, 1),
                lambda: uio_out_bit(dut, 1),
                lambda: uio_out_bit(dut, 5),
                lambda bit: drive_ui_bit(dut, 1, bit),
                mode=mode, expected_clock_cycles=period,
                clock_tolerance=0.22)
            captured.extend(await target.transfer([byte], timeout_cycles=20000))
        return bytes(captured)

    serve = cocotb.start_soon(serve_independent_byte_cs())
    await write_csr(dut, CSR_CTRL_RUN, 0x0002)
    captured = await serve
    assert captured == mosi_expect, \
        f"SPI ctrl mode{mode} fast={fast}: MOSI {captured.hex()} != {mosi_expect.hex()}"
    status = await wait_mailbox_status(dut, 1, 8)
    assert status == 1, f"SPI ctrl status/error 0x{status:02X}"
    got = await read_sram1_bytes(dut, MEM_RX, len(miso_expect))
    assert got == miso_expect, \
        f"SPI ctrl mode{mode} fast={fast}: MISO mailbox {got.hex()} != {miso_expect.hex()}"
    st = await core_status(dut, 1)
    assert (st >> 3) & 1 == 0, "SPI controller deadline_missed"


@cocotb.test()
async def test_spi_controller_matrix(dut):
    """SPI controller modes 0-3, each at 1 MHz and 10 MHz."""
    first = True
    for mode in range(4):
        for fast in (False, True):
            await _spi_ctrl_case(dut, mode, fast, first)
            first = False


@cocotb.test()
async def debug_spi_ctrl_probe(dut):
    """Temporary whitebox SPI controller phase probe (mode0, 1 MHz)."""
    await reset_dut(dut, start_clock=True)
    await load_firmware(dut, "spi_ctrl.hasm")
    await configure_spi_controller_pins(dut)
    await write_bcu_config(
        dut, 1, isa.pack_bcu_mode(codec="nrz", stuff="off"),
        (8 - 1) | 0x10, isa.pack_dpll_cfg(mode="none", win=0, period=6))
    await clear_mailbox(dut)
    await write_sram1_bytes(dut, MEM_TX, bytes([0xA5, 0x3C, 0x96, 0x5A]))
    await write_sram1_bytes(dut, 0, bytes([0, 0, 0x10, 4]))
    drive_ui_bit(dut, 1, 0)

    async def mon():
        last = {}
        for i in range(700):
            await RisingEdge(dut.clk)
            vals = {
                "cs": uo_bit(dut, 1),
                "sclk": uio_out_bit(dut, 1),
                "mosi": uio_out_bit(dut, 5),
                "txp": u(dut.gen_core[1].u_bcu.tx_p),
                "txtick": u(dut.gen_core[1].u_bcu.tx_tick),
                "rxsamp": u(dut.gen_core[1].u_bcu.dpll_sample),
                "pc": (u(dut.core_pc_flat) >> 10) & 0x3FF,
                "txcnt": u(dut.gen_core[1].u_bcu.tx_sym_cnt),
                "dcnt": u(dut.gen_core[1].u_bcu.u_dpll.cnt),
                "rxsh": u(dut.gen_core[1].u_bcu.rx_shift),
            }
            for key in ("cs", "sclk", "mosi", "txp"):
                if key not in last or vals[key] != last[key]:
                    print(f"DBG spi cyc{i:04d} {key}={vals[key]} pc={vals['pc']:03x} txcnt={vals['txcnt']} dcnt={vals['dcnt']}")
            if vals["txtick"]:
                print(f"DBG spi cyc{i:04d} TXTICK txp={vals['txp']} pc={vals['pc']:03x} txcnt={vals['txcnt']}")
            if vals["rxsamp"]:
                print(f"DBG spi cyc{i:04d} RXSAMP pc={vals['pc']:03x} dcnt={vals['dcnt']} rxsh={vals['rxsh']:04x}")
            last = vals
    cocotb.start_soon(mon())
    await write_csr(dut, CSR_CTRL_RUN, 0x0002)
    await cycles(dut, 700)


async def _spi_target_case(dut, mode, first_clock):
    await reset_dut(dut, start_clock=first_clock)
    await load_firmware(dut, "spi_target.hasm")
    await configure_spi_target_pins(dut, mode)
    await write_csr(dut, CSR_PIN_FILT_EN, 0)  # KTD4 external-SCLK requirement
    await write_bcu_config(
        dut, 3, isa.pack_bcu_mode(codec="nrz_extclk", stuff="off"),
        (8 - 1) | 0x10, isa.pack_dpll_cfg(mode="none", win=0, period=0))
    await clear_mailbox(dut)
    mosi_expect = bytes([0xC3, 0x5A, 0x96, 0x24])
    miso_expect = bytes([0x66, 0x99, 0x42, 0xBD])
    await write_sram1_bytes(dut, MEM_TX, miso_expect)

    idle = (mode >> 1) & 1
    drive_uio_bit(dut, 4, idle)  # SCLK
    drive_uio_bit(dut, 5, 1)     # CS_N
    drive_uio_bit(dut, 6, 0)     # MOSI
    ctl = SPIControllerBfm(
        dut.clk,
        lambda bit: drive_uio_bit(dut, 5, bit),
        lambda bit: drive_uio_bit(dut, 4, bit),
        lambda bit: drive_uio_bit(dut, 6, bit),
        lambda: uio_out_bit(dut, 3),
        mode=mode, half_period_cycles=6)
    await write_csr(dut, CSR_CTRL_RUN, 0x0008)
    got_miso = await ctl.transfer(mosi_expect, cs_setup_cycles=10, cs_hold_cycles=10)
    assert got_miso == miso_expect, \
        f"SPI target mode{mode}: MISO {got_miso.hex()} != {miso_expect.hex()}"
    status = await wait_mailbox_status(dut, 1, 8)
    assert status == 1, f"SPI target status/error 0x{status:02X}"
    got_mosi = await read_sram1_bytes(dut, MEM_RX, 4)
    assert got_mosi == mosi_expect, \
        f"SPI target mode{mode}: MOSI mailbox {got_mosi.hex()} != {mosi_expect.hex()}"
    await cycles(dut, 12)
    assert uio_oe_bit(dut, 3) == 0, "SPI target MISO still driven after CS_N high"


@cocotb.test()
async def test_spi_target_matrix(dut):
    """SPI target modes 0-3 at 5 MHz, including CPHA=0 preload behavior."""
    first = True
    for mode in range(4):
        await _spi_target_case(dut, mode, first)
        first = False


# ----------------------------------------------------------------------
# I2C matrices
# ----------------------------------------------------------------------
def _make_i2c_bus(dut):
    bus = OpenDrainI2CBus(
        dut.clk,
        lambda pin, level: drive_uio_bit(dut, pin, level),
        lambda: uio_out_bit(dut, 2), lambda: uio_oe_bit(dut, 2),
        lambda: uio_out_bit(dut, 6), lambda: uio_oe_bit(dut, 6),
        2, 6)
    # The wired-AND model initializes both open-drain lines high; the resolver
    # continuously writes that pull-up state into the chip input shadow.
    bus.start_resolver()
    return bus


async def _i2c_ctrl_case(dut, bus, fast, read, stretch, first_clock):
    await reset_dut(dut, start_clock=first_clock)
    await load_firmware(dut, "i2c_ctrl.hasm")
    await configure_i2c_pins(dut, pin_filter=True)
    await clear_mailbox(dut)
    half = 75 if fast else 300
    write_data = bytes([0x5A, 0xC3])
    read_data = bytes([0x96, 0x3D])
    addr_rw = (0x42 << 1) | (1 if read else 0)
    cfg_speed = 1 if fast else 0
    direction = 1 if read else 0
    await write_sram1_bytes(dut, 0, bytes([0, 0, cfg_speed, 0, direction, 0, 0, 0]))
    await write_sram1_bytes(dut, MEM_TX, bytes([addr_rw]) + write_data)

    target = I2CTargetBfm(bus, address=0x42, read_data=read_data,
                          byte_count=2, timeout_cycles=max(20000, half * 300))
    stretch_cycles = 1200 if stretch else 0
    serve = cocotb.start_soon(target.serve(stretch_after_byte=1 if stretch else 0,
                                           stretch_cycles=stretch_cycles))
    await write_csr(dut, CSR_CTRL_RUN, 0x0004)
    result = await serve
    if read:
        got = await read_sram1_bytes(dut, MEM_RX, 2)
        assert got == read_data, f"I2C ctrl read {got.hex()} != {read_data.hex()}"
    else:
        assert result.write_data == write_data, \
            f"I2C target BFM got {result.write_data.hex()} != {write_data.hex()}"
    assert not bus.contention_errors, bus.contention_errors
    status = await wait_mailbox_status(dut, 1, 8)
    assert status == 1, f"I2C ctrl status/error 0x{status:02X}"
    st = await core_status(dut, 2)
    assert (st >> 3) & 1 == 0, "I2C controller deadline_missed"


@cocotb.test()
async def test_i2c_controller_matrix(dut):
    """I2C controller 100/400 kHz read/write, stretch, open-drain safety."""
    await reset_dut(dut, start_clock=True)
    bus = _make_i2c_bus(dut)
    await _i2c_ctrl_case(dut, bus, fast=False, read=False, stretch=False, first_clock=False)
    await _i2c_ctrl_case(dut, bus, fast=True, read=False, stretch=True, first_clock=False)
    await _i2c_ctrl_case(dut, bus, fast=True, read=True, stretch=False, first_clock=False)


async def _i2c_target_case(dut, bus, read, first_clock):
    await reset_dut(dut, start_clock=first_clock)
    await load_firmware(dut, "i2c_target.hasm")
    await configure_i2c_pins(dut, pin_filter=True)
    await clear_mailbox(dut)
    tx_data = bytes([0x69, 0xA7])
    write_data = bytes([0x55, 0xCC])
    await write_sram1_bytes(dut, MEM_TX, tx_data)
    # enable the firmware target's optional 20 us stretch after byte1 ACK
    await write_sram1_bytes(dut, 0, bytes([0, 0, 1, 0]))
    master = I2CControllerBfm(bus, half_period_cycles=75, timeout_cycles=30000)
    await write_csr(dut, CSR_CTRL_RUN, 0x0004)
    if read:
        got = await master.read_transaction(0x42, 2)
        assert got == tx_data, f"I2C target read {got.hex()} != {tx_data.hex()}"
    else:
        await master.write_transaction(0x42, write_data)
        got = await read_sram1_bytes(dut, MEM_RX, 2)
        assert got == write_data, \
            f"I2C target mailbox {got.hex()} != {write_data.hex()}"
    status = await wait_mailbox_status(dut, 1, 8)
    assert status == 1, f"I2C target status/error 0x{status:02X}"
    assert not bus.contention_errors, bus.contention_errors


@cocotb.test()
async def test_i2c_target_matrix(dut):
    """Hamming I2C target multi-byte write/read against an independent master."""
    await reset_dut(dut, start_clock=True)
    bus = _make_i2c_bus(dut)
    await _i2c_target_case(dut, bus, read=False, first_clock=False)
    await _i2c_target_case(dut, bus, read=True, first_clock=False)


@cocotb.test()
async def test_i2c_nack_arbitration_and_spike(dut):
    """AE5 negatives: NACK status, arbitration loss release, 50ns spike rejection."""
    await reset_dut(dut, start_clock=True)
    bus = _make_i2c_bus(dut)

    # NACK: no target acknowledges the address.  Firmware must report 0x40.
    await load_firmware(dut, "i2c_ctrl.hasm")
    await configure_i2c_pins(dut, pin_filter=True)
    await clear_mailbox(dut)
    await write_sram1_bytes(dut, MEM_TX, bytes([(0x42 << 1), 1, 2]))
    await write_csr(dut, CSR_CTRL_RUN, 0x0004)
    await cycles(dut, 12000)
    status_word = await read_sram1_word(dut, 0)
    assert ((status_word >> 8) & 0xFF) == 0x40, "NACK did not produce status 0x40"

    # Arbitration: reset and let a competing master win while the controller
    # releases SDA for a one bit.  Status 0x41 proves detection and release.
    await reset_dut(dut, start_clock=False)
    await load_firmware(dut, "i2c_ctrl.hasm")
    await configure_i2c_pins(dut, pin_filter=True)
    await clear_mailbox(dut)
    await write_sram1_bytes(dut, MEM_TX, bytes([(0x42 << 1), 0xFF, 0xFF]))
    pulse = cocotb.start_soon(sda_arbitration_pulse(bus, "competitor", 1300, 300))
    await write_csr(dut, CSR_CTRL_RUN, 0x0004)
    await pulse
    await cycles(dut, 2000)
    status_word = await read_sram1_word(dut, 0)
    assert ((status_word >> 8) & 0xFF) == 0x41, "arbitration loss not reported"
    assert (u(dut.uio_oe) & ((1 << 2) | (1 << 6))) == 0, \
        "I2C lines not released after arbitration loss"
    assert not bus.contention_errors, bus.contention_errors


@cocotb.test()
async def test_concurrent_four_core_protocols(dut):
    """U7 concurrency proof: UART+SPI+I2C+timed waveform on four cores.

    The same mailbox bytes intentionally serve all three protocol programs in
    this fixed demonstration.  External BFMs judge the wire traffic, avoiding
    any dependence on overlapping per-demo status bytes.
    """
    await reset_dut(dut, start_clock=True)
    bus = _make_i2c_bus(dut)
    await load_program_source(dut, controller_concurrent_source())

    # Compose all output routes; the host-side router shadow prevents each
    # setup helper from erasing the previous protocol's channels.
    await configure_uart_bcu_pins(dut, rx_pin=8, tx_uio=0)
    await configure_spi_controller_pins(dut)
    await configure_i2c_pins(dut, pin_filter=True)
    await write_pin_router(dut, {19: SRC_CORE})  # core3 waveform -> uo_out3

    # BCU0 UART 3 Mbps and BCU1 SPI mode1/1 MHz (the shared cfg byte is 1).
    await write_bcu_config(
        dut, 0, isa.pack_bcu_mode(codec="nrz", stuff="off"), 10,
        isa.pack_dpll_cfg(mode="hard_snap", win=0, period=20))
    await write_bcu_config(
        dut, 1, isa.pack_bcu_mode(codec="nrz", stuff="off"),
        7 | 0x10, isa.pack_dpll_cfg(mode="none", win=0, period=60))

    # Shared fixed payload: byte8 doubles as I2C address 0x42/write and as
    # UART/SPI byte0; bytes9/10 are the I2C write payload.
    shared_tx = bytes([0x84, 0x5A, 0xC3, 0x96])
    uart_in = bytes([0x42, 0x24, 0x81, 0x18])
    spi_miso = bytes([0x66, 0x99, 0x42, 0xBD])
    await clear_mailbox(dut)
    await write_sram1_bytes(dut, MEM_TX, shared_tx)
    await write_sram1_bytes(dut, 0, bytes([0, 0, 1, 4, 0, 0, 0, 0]))

    # RLE captures the waveform through an external pin loopback uo_out3 ->
    # ui_in[2] (pin_inputs bit10).  Use SRAM1 word 96, clear of the mailbox.
    await write_csr(dut, CSR_RLE_BASE, 96)
    await write_csr(dut, CSR_RLE_MASK, 0x001F)
    await write_csr(dut, CSR_RLE_PROBE_MASK, 1 << 10)
    await write_csr(dut, CSR_RLE_CTRL, isa.pack_rle_ctrl(mode="capture"))

    async def wave_loopback():
        while True:
            await RisingEdge(dut.clk)
            drive_ui_bit(dut, 2, uo_bit(dut, 3))

    cocotb.start_soon(wave_loopback())

    uart_tx = UARTTransmitter(
        dut.clk, lambda bit: drive_ui_bit(dut, 0, bit),
        bit_cycles=20, parity="even", drift=0)
    uart_rx = UARTReceiver(
        dut.clk, lambda: uio_out_bit(dut, 0),
        bit_cycles=20, parity="even", drift=0)
    spi_target = SPITargetBfm(
        dut.clk, lambda: uo_bit(dut, 1), lambda: uio_out_bit(dut, 1),
        lambda: uio_out_bit(dut, 5), lambda bit: drive_ui_bit(dut, 1, bit),
        mode=1, expected_clock_cycles=60, clock_tolerance=0.25)
    i2c_target = I2CTargetBfm(bus, address=0x42, byte_count=2,
                              timeout_cycles=30000)

    uart_rx_task = cocotb.start_soon(uart_rx.receive_bytes(4, 20000))
    uart_tx_task = cocotb.start_soon(uart_tx.send_bytes(uart_in))
    spi_task = cocotb.start_soon(spi_target.transfer(spi_miso, timeout_cycles=20000))
    i2c_task = cocotb.start_soon(i2c_target.serve())

    await write_csr(dut, CSR_CTRL_RUN, 0x000F)
    uart_frames = await uart_rx_task
    spi_mosi = await spi_task
    i2c_result = await i2c_task
    await uart_tx_task

    assert [f.data for f in uart_frames] == list(shared_tx)
    assert spi_mosi == shared_tx
    assert i2c_result.write_data == shared_tx[1:3]
    assert not bus.contention_errors, bus.contention_errors

    await write_csr(dut, CSR_RLE_CTRL, isa.pack_rle_ctrl(mode="off"))
    recs = [await read_sram1_word(dut, 96 + i) for i in range(4)]
    probes = [r & 0xFFFF for r in recs]
    assert probes[0] & (1 << 10), "RLE missed waveform rising edge"
    assert not (probes[1] & (1 << 10)), "RLE missed waveform falling edge"
    # Inter-edge deltas should be close to the programmed 20-cycle width.
    d1 = (recs[1] >> 16) & 0xFFFF
    assert 15 <= d1 <= 27, f"core3 waveform spacing delta {d1}, expected ~20"

    for core in range(4):
        st = await core_status(dut, core)
        assert (st >> 3) & 1 == 0, f"core{core} deadline_missed status {st:#x}"
