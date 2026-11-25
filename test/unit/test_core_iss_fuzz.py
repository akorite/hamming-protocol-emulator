# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""pytest/cocotb differential suite for the Hamming protocol core, loop cache,
and Python Golden ISS."""

import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

from cocotb_tools.runner import get_runner

UNIT_DIR = Path(__file__).resolve().parent
ROOT = UNIT_DIR.parents[1]
SRC = ROOT / "src"
MODELS = ROOT / "test" / "sim_models"
BUILD_ROOT = ROOT / "test" / "sim_build" / "unit_core_iss"


def _sources():
    return [
        UNIT_DIR / "core_iss_harness.v",
        SRC / "hamming_core.v",
        SRC / "hamming_icache.v",
        SRC / "hamming_csr.v",
    ]


@lru_cache(maxsize=None)
def _build():
    build_dir = BUILD_ROOT / "harness"
    build_dir.mkdir(parents=True, exist_ok=True)
    runner = get_runner("icarus")
    runner.build(
        sources=[str(p) for p in _sources()],
        hdl_toplevel="core_iss_harness",
        defines={"SIM": 1, "FUNCTIONAL": 1},
        includes=[str(SRC)],
        build_dir=str(build_dir),
        always=True,
    )
    return runner, build_dir


def _run_case(testcase):
    runner, build_dir = _build()
    results_xml = build_dir / f"results_{testcase}.xml"
    runner.test(
        hdl_toplevel="core_iss_harness",
        test_module="cocotb_core_iss",
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


def test_core_basic_alu_fetch_and_halt():
    _run_case("test_core_basic_alu_fetch_and_halt")


def test_core_alu_ldi_differential():
    _run_case("test_core_alu_ldi_differential")


def test_core_pin_lut_bcu_differential():
    _run_case("test_core_pin_lut_bcu_differential")


def test_core_memory_word_byte_and_tables_differential():
    _run_case("test_core_memory_word_byte_and_tables_differential")


def test_core_control_flow_call_return_differential():
    _run_case("test_core_control_flow_call_return_differential")


def test_core_csr_event_and_ownership_differential():
    _run_case("test_core_csr_event_and_ownership_differential")


def test_core_timing_wait_sync_timer_differential():
    _run_case("test_core_timing_wait_sync_timer_differential")


def test_core_cache_conflict_loop_differential():
    _run_case("test_core_cache_conflict_loop_differential")


def test_core_jev_and_sync_timeout_differential():
    _run_case("test_core_jev_and_sync_timeout_differential")


def test_core_host_write_invalidates_coincident_loop_cache():
    _run_case("test_core_host_write_invalidates_coincident_loop_cache")


def test_core_random_loop_100k_cycles_differential():
    _run_case("test_core_random_loop_100k_cycles_differential")


def test_core_host_write_cancels_inflight_and_coincident_fills():
    _run_case("test_core_host_write_cancels_inflight_and_coincident_fills")


def test_core_reset_start_stop_freezes_architectural_state():
    _run_case("test_core_reset_start_stop_freezes_architectural_state")


def test_core_timer_wraparound_wait_differential():
    _run_case("test_core_timer_wraparound_wait_differential")
