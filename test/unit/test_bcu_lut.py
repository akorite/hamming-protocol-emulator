# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""pytest/cocotb wire-level suite for the Bitstream Conditioning Units and the
2-tier 16-LUT pin logic matrix."""

import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

from cocotb_tools.runner import get_runner

UNIT_DIR = Path(__file__).resolve().parent
ROOT = UNIT_DIR.parents[1]
SRC = ROOT / "src"
BUILD_ROOT = ROOT / "test" / "sim_build" / "unit_bcu_lut"


@lru_cache(maxsize=None)
def _build_crc():
    build_dir = BUILD_ROOT / "crc"
    build_dir.mkdir(parents=True, exist_ok=True)
    runner = get_runner("icarus")
    runner.build(
        sources=[str(SRC / "hamming_bcu_crc.v")],
        hdl_toplevel="hamming_bcu_crc",
        defines={"SIM": 1, "FUNCTIONAL": 1},
        includes=[str(SRC)],
        build_dir=str(build_dir),
        always=True,
    )
    return runner, build_dir


def _run(testcase, build, top, module="cocotb_bcu"):
    runner, build_dir = build()
    results_xml = build_dir / f"results_{testcase}.xml"
    runner.test(
        hdl_toplevel=top,
        test_module=module,
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


@lru_cache(maxsize=None)
def _build_lut():
    build_dir = BUILD_ROOT / "lut"
    build_dir.mkdir(parents=True, exist_ok=True)
    runner = get_runner("icarus")
    runner.build(
        sources=[str(SRC / "hamming_lut_matrix.v")],
        hdl_toplevel="hamming_lut_matrix",
        defines={"SIM": 1, "FUNCTIONAL": 1},
        includes=[str(SRC)],
        build_dir=str(build_dir),
        always=True,
    )
    return runner, build_dir


def test_crc_catalogue_check_vectors():
    _run("test_crc_catalogue_check_vectors", _build_crc, "hamming_bcu_crc")


def test_crc_residue_and_empty_payload():
    _run("test_crc_residue_and_empty_payload", _build_crc, "hamming_bcu_crc")


def test_crc_randomized_reference_fuzz():
    _run("test_crc_randomized_reference_fuzz", _build_crc, "hamming_bcu_crc")


def test_lut_two_tier_truth_tables():
    _run("test_lut_two_tier_truth_tables", _build_lut, "hamming_lut_matrix",
         module="cocotb_lut")


def test_lut_glitch_filter_rejects_short_spikes():
    _run("test_lut_glitch_filter_rejects_short_spikes", _build_lut,
         "hamming_lut_matrix", module="cocotb_lut")


def test_router_single_owner_and_replay_gate():
    _run("test_router_single_owner_and_replay_gate", _build_lut,
         "hamming_lut_matrix", module="cocotb_lut")


def test_router_open_drain_clamp():
    _run("test_router_open_drain_clamp", _build_lut, "hamming_lut_matrix",
         module="cocotb_lut")


def test_lut_output_register_enable():
    _run("test_lut_output_register_enable", _build_lut, "hamming_lut_matrix",
         module="cocotb_lut")


@lru_cache(maxsize=None)
def _build_dpll():
    build_dir = BUILD_ROOT / "dpll"
    build_dir.mkdir(parents=True, exist_ok=True)
    runner = get_runner("icarus")
    runner.build(
        sources=[str(SRC / "hamming_bcu_dpll.v")],
        hdl_toplevel="hamming_bcu_dpll",
        defines={"SIM": 1, "FUNCTIONAL": 1},
        includes=[str(SRC)],
        build_dir=str(build_dir),
        always=True,
    )
    return runner, build_dir


def test_free_run_sample_period():
    _run("test_free_run_sample_period", _build_dpll, "hamming_bcu_dpll",
         module="cocotb_dpll")


def test_hard_snap_rephase():
    _run("test_hard_snap_rephase", _build_dpll, "hamming_bcu_dpll",
         module="cocotb_dpll")


def test_hold_off_rejects_midcell():
    _run("test_hold_off_rejects_midcell", _build_dpll, "hamming_bcu_dpll",
         module="cocotb_dpll")


def test_hold_off_accepts_boundary():
    _run("test_hold_off_accepts_boundary", _build_dpll, "hamming_bcu_dpll",
         module="cocotb_dpll")


def test_eof_count_nine():
    _run("test_eof_count_nine", _build_dpll, "hamming_bcu_dpll",
         module="cocotb_dpll")


def test_no_samples_when_stopped():
    _run("test_no_samples_when_stopped", _build_dpll, "hamming_bcu_dpll",
         module="cocotb_dpll")


@lru_cache(maxsize=None)
def _build_bcu():
    build_dir = BUILD_ROOT / "bcu"
    build_dir.mkdir(parents=True, exist_ok=True)
    runner = get_runner("icarus")
    runner.build(
        sources=[str(SRC / "hamming_bcu.v"), str(SRC / "hamming_bcu_crc.v"),
                 str(SRC / "hamming_bcu_dpll.v")],
        hdl_toplevel="hamming_bcu",
        defines={"SIM": 1, "FUNCTIONAL": 1},
        includes=[str(SRC)],
        build_dir=str(build_dir),
        always=True,
    )
    return runner, build_dir


def test_nrz_loopback_lsb():
    _run("test_nrz_loopback_lsb", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_nrz_loopback_msb():
    _run("test_nrz_loopback_msb", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_nrzi_loopback_diff():
    _run("test_nrzi_loopback_diff", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_manchester_rx_decode():
    _run("test_manchester_rx_decode", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_manchester_tx_waveform():
    _run("test_manchester_tx_waveform", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_extclk_rx_reassembly():
    _run("test_extclk_rx_reassembly", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_pattern_matcher_sfd():
    _run("test_pattern_matcher_sfd", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_crc_feed_tx():
    _run("test_crc_feed_tx", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_usb6_stuff_trailing_before_eop():
    _run("test_usb6_stuff_trailing_before_eop", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_usb6_rx_stuff_err_and_dribble():
    _run("test_usb6_rx_stuff_err_and_dribble", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_can5_unstuff_rx():
    _run("test_can5_unstuff_rx", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_bit_len_partial_transfer():
    _run("test_bit_len_partial_transfer", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")


def test_tx_empty_and_shadow_streaming():
    _run("test_tx_empty_and_shadow_streaming", _build_bcu, "hamming_bcu",
         module="cocotb_bcu_top")
