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
