# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""pytest/cocotb runner for the U8 stretch protocol matrix."""

import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import pytest
from cocotb_tools.runner import get_runner

UNIT_DIR = Path(__file__).resolve().parent
ROOT = UNIT_DIR.parents[1]
SRC = ROOT / "src"
BUILD_ROOT = ROOT / "test" / "sim_build" / "unit_stretch"

CHIP_SOURCES = [
    "project.v",
    "hamming_sram_2kb.v",
    "hamming_host_spi.v",
    "hamming_csr.v",
    "hamming_sram_arb.v",
    "hamming_icache.v",
    "hamming_core.v",
    "hamming_bcu_crc.v",
    "hamming_lut_matrix.v",
    "hamming_bcu_dpll.v",
    "hamming_bcu.v",
    "hamming_rle_engine.v",
]


@lru_cache(maxsize=None)
def _build_chip():
    build_dir = BUILD_ROOT / "chip"
    build_dir.mkdir(parents=True, exist_ok=True)
    runner = get_runner("icarus")
    runner.build(
        sources=[str(SRC / f) for f in CHIP_SOURCES]
        + [str(ROOT / "test" / "sim_models"
              / "RM_IHPSG13_1P_512x32_c2_bm_bist.v")],
        hdl_toplevel="tt_um_akorite_hamming",
        defines={"SIM": 1, "FUNCTIONAL": 1},
        includes=[str(SRC)],
        build_dir=str(build_dir),
        always=True,
    )
    return runner, build_dir


def _run(testcase):
    runner, build_dir = _build_chip()
    results_xml = build_dir / f"results_{testcase}.xml"
    runner.test(
        hdl_toplevel="tt_um_akorite_hamming",
        test_module="cocotb_stretch",
        testcase=testcase,
        results_xml=str(results_xml),
        test_dir=str(UNIT_DIR),
        build_dir=str(build_dir),
    )
    bad = [
        tc.get("name")
        for tc in ET.parse(results_xml).getroot().iter("testcase")
        for child in tc
        if child.tag in ("failure", "error")
    ]
    assert not bad, f"{testcase}: failing cocotb tests: {bad}"


# ----------------------------------------------------------------------
# Pure-BFM non-vacuity checks: the independent CAN model must implement the
# rules it later uses to judge the silicon.
# ----------------------------------------------------------------------
def test_can_codec_rejects_mutated_frames():
    import sys
    if str(ROOT / "test") not in sys.path:
        sys.path.insert(0, str(ROOT / "test"))
    from bfm.can_bfm import (
        CANProtocolError, decode_frame_bits, encode_frame_bits,
    )
    bits, crc, _ = encode_frame_bits(0x123, bytes([0xA5, 0x3C]))
    dec = decode_frame_bits(bits)
    assert dec["identifier"] == 0x123
    assert dec["data"] == bytes([0xA5, 0x3C])
    bad = list(bits)
    bad[25] ^= 1
    with pytest.raises(CANProtocolError):
        decode_frame_bits(bad)


def test_can_codec_enforces_stuffing_rule():
    import sys
    if str(ROOT / "test") not in sys.path:
        sys.path.insert(0, str(ROOT / "test"))
    from bfm.can_bfm import (
        CANProtocolError, decode_frame_bits, encode_frame_bits,
    )
    bits, crc, _ = encode_frame_bits(0x001, bytes([0xFF, 0xFF]))
    dec = decode_frame_bits(bits)
    assert dec["data"] == bytes([0xFF, 0xFF])
    # find a 5-run and break its stuff bit
    run = 1
    broke = False
    for i in range(1, len(bits)):
        if bits[i] == bits[i - 1]:
            run += 1
            if run == 5 and i + 1 < len(bits):
                bad = list(bits)
                bad[i + 1] = bits[i]
                with pytest.raises(CANProtocolError):
                    decode_frame_bits(bad)
                broke = True
                break
        else:
            run = 1
    assert broke, "test vector lacks a stuffing run"


# ----------------------------------------------------------------------
# Chip-level CAN tests
# ----------------------------------------------------------------------
def test_can20a_tx_frame():
    _run("test_can20a_tx_frame")
