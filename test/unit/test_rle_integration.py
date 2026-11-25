# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""pytest/cocotb wire-level suite for the RLE logic analyzer / waveform
replay engine (standalone unit; multi-core integration tests join in
slice 3)."""

import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

from cocotb_tools.runner import get_runner

UNIT_DIR = Path(__file__).resolve().parent
ROOT = UNIT_DIR.parents[1]
SRC = ROOT / "src"
BUILD_ROOT = ROOT / "test" / "sim_build" / "unit_rle"


@lru_cache(maxsize=None)
def _build_rle():
    build_dir = BUILD_ROOT / "rle"
    build_dir.mkdir(parents=True, exist_ok=True)
    runner = get_runner("icarus")
    runner.build(
        sources=[str(SRC / "hamming_rle_engine.v")],
        hdl_toplevel="hamming_rle_engine",
        defines={"SIM": 1, "FUNCTIONAL": 1},
        includes=[str(SRC)],
        build_dir=str(build_dir),
        always=True,
    )
    return runner, build_dir


def _run(testcase):
    runner, build_dir = _build_rle()
    results_xml = build_dir / f"results_{testcase}.xml"
    runner.test(
        hdl_toplevel="hamming_rle_engine",
        test_module="cocotb_rle",
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


def test_capture_edges_and_wr_ptr():
    _run("test_capture_edges_and_wr_ptr")


def test_capture_heartbeat_and_masked_edges():
    _run("test_capture_heartbeat_and_masked_edges")


def test_capture_burst_no_overflow_with_service():
    _run("test_capture_burst_no_overflow_with_service")


def test_capture_overflow_sticky():
    _run("test_capture_overflow_sticky")


def test_replay_reproduces_edge_timing():
    _run("test_replay_reproduces_edge_timing")


def test_ring_wrap_and_packet_buffer_untouched():
    _run("test_ring_wrap_and_packet_buffer_untouched")


def test_exact_ffff_collision():
    _run("test_exact_ffff_collision")
