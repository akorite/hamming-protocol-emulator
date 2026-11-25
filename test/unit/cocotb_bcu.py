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

_CLOCK_STARTED = False


def u(sig):
    value = sig.value
    if hasattr(value, "to_unsigned"):
        return value.to_unsigned()
    return int(value)


async def ensure_clock(dut):
    global _CLOCK_STARTED
    if not _CLOCK_STARTED:
        cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
        _CLOCK_STARTED = True


async def reset_dut(dut):
    dut.cfg_width_m1.value = 0
    dut.cfg_msb.value = 0
    dut.cfg_poly.value = 0
    dut.load_en.value = 0
    dut.load_state.value = 0
    dut.step_en.value = 0
    dut.step_bit.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def crc_run(dut, bits, poly, width, msb_first, init1, seed=None):
    """Configure the engine, seed it, shift `bits` LSB/MSB-first per config,
    and return the raw post-shift state."""
    dut.cfg_width_m1.value = width - 1
    dut.cfg_msb.value = int(msb_first)
    dut.cfg_poly.value = poly
    mask = isa.crc_width_mask(width)
    await FallingEdge(dut.clk)
    dut.load_state.value = (seed if seed is not None
                            else (mask if init1 else 0)) & mask
    dut.load_en.value = 1
    await FallingEdge(dut.clk)
    dut.load_en.value = 0
    dut.load_state.value = 0
    for bit in bits:
        dut.step_bit.value = bit & 1
        dut.step_en.value = 1
        await FallingEdge(dut.clk)
    dut.step_en.value = 0
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    return u(dut.state)


def bits_of(data, msb_order=False):
    out = []
    for b in data:
        rng = range(7, -1, -1) if msb_order else range(8)
        for i in rng:
            out.append((b >> i) & 1)
    return out


@cocotb.test()
async def test_crc_catalogue_check_vectors(dut):
    """Catalogue check values for ASCII '123456789': CRC-5/USB 0x19,
    CRC-16/USB 0xB4C8, CRC-32 reflected zlib 0xCBF43926, CRC-15/CAN 0x059E."""
    await ensure_clock(dut)
    await reset_dut(dut)
    data = b"123456789"
    for name, spec, msb_order in [
        ("usb5", isa.CRC_USB5, False),
        ("usb16", isa.CRC_USB16, False),
        ("eth32", isa.CRC_ETH32, False),
        ("can15", isa.CRC_CAN15, True),
    ]:
        bits = bits_of(data, msb_order)
        raw = await crc_run(dut, bits, spec["poly"], spec["width"],
                            spec["msb_first"], spec["init1"])
        mask = isa.crc_width_mask(spec["width"])
        got = (raw ^ mask) & mask if spec["xor1"] else raw & mask
        want = isa.crc_compute(bits, spec["poly"], spec["width"],
                               spec["msb_first"], spec["init1"], spec["xor1"])
        assert got == want, f"{name}: rtl={got:08X} ref={want:08X}"


@cocotb.test()
async def test_crc_residue_and_empty_payload(dut):
    """Message+CRC validation yields the protocol magic residue; a 0-byte
    payload leaves the masked init state."""
    await ensure_clock(dut)
    await reset_dut(dut)
    for name, spec, payload, msb_order in [
        ("usb5", isa.CRC_USB5, bytes([0xA5, 0x3C]), False),
        ("usb16", isa.CRC_USB16, bytes([0x80, 0x06, 0x01, 0x00]), False),
        ("eth32", isa.CRC_ETH32, bytes([0xFF, 0x01, 0x55, 0xD5]), False),
        ("can15", isa.CRC_CAN15, bytes([0x22, 0x90, 0x11]), True),
    ]:
        bits = bits_of(payload, msb_order)
        crc = isa.crc_compute(bits, spec["poly"], spec["width"],
                              spec["msb_first"], spec["init1"], spec["xor1"])
        if spec["msb_first"]:
            crc_wire = [(crc >> i) & 1 for i in range(spec["width"] - 1, -1, -1)]
        else:
            crc_wire = [(crc >> i) & 1 for i in range(spec["width"])]
        raw = await crc_run(dut, bits + crc_wire, spec["poly"], spec["width"],
                            spec["msb_first"], spec["init1"])
        assert raw == spec["residue"], (
            f"{name}: residue rtl={raw:08X} want={spec['residue']:08X}")

        empty = await crc_run(dut, [], spec["poly"], spec["width"],
                              spec["msb_first"], spec["init1"])
        mask = isa.crc_width_mask(spec["width"])
        want_empty = mask if spec["init1"] else 0
        assert empty == want_empty, f"{name}: empty rtl={empty:08X}"


@cocotb.test()
async def test_crc_randomized_reference_fuzz(dut):
    """Random widths/polys/seeds/bitstreams match the Python reference,
    including exotic core-written seeds."""
    await ensure_clock(dut)
    await reset_dut(dut)
    rng = random.Random(0xC0C123)
    for trial in range(60):
        width = rng.randrange(1, 33)
        poly = rng.getrandbits(32)
        msb_first = rng.randrange(2)
        seed = rng.getrandbits(32)
        nbits = rng.randrange(0, 96)
        bits = [rng.randrange(2) for _ in range(nbits)]
        raw = await crc_run(dut, bits, poly, width, msb_first, False, seed=seed)
        mask = isa.crc_width_mask(width)
        ref = seed & mask
        for bit in bits:
            ref = isa.crc_step_ref(ref, bit, poly, width, msb_first)
        assert raw == ref, (
            f"trial {trial}: width={width} rtl={raw:08X} ref={ref:08X}")
