# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""pytest/cocotb unit suite for the Host SPI target, CSR bank, and bounded
 dual-SRAM arbiters. The pytest layer only builds/runs Icarus simulations; the
 pin/transaction-level scenarios live in cocotb_host_spi_arb.py."""

import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

from cocotb_tools.runner import get_runner

UNIT_DIR = Path(__file__).resolve().parent
ROOT = UNIT_DIR.parents[1]
SRC = ROOT / "src"
MODELS = ROOT / "test" / "sim_models"
BUILD_ROOT = ROOT / "test" / "sim_build" / "unit_host_spi_arb"


def _base_sources():
    return [
        SRC / "hamming_host_spi.v",
        SRC / "hamming_csr.v",
        SRC / "hamming_sram_arb.v",
        SRC / "hamming_sram_2kb.v",
        MODELS / "RM_IHPSG13_1P_512x32_c2_bm_bist.v",
    ]


@lru_cache(maxsize=None)
def _build(kind):
    if kind == "arb6":
        top = "hamming_sram_arb"
        sources = _base_sources()
        parameters = {"NCLIENTS": 6}
    elif kind == "arb5":
        top = "hamming_sram_arb"
        sources = _base_sources()
        parameters = {"NCLIENTS": 5}
    elif kind == "harness":
        top = "host_spi_arb_harness"
        sources = [UNIT_DIR / "host_spi_arb_harness.v"] + _base_sources()
        parameters = {}
    else:
        raise ValueError(kind)

    build_dir = BUILD_ROOT / kind
    build_dir.mkdir(parents=True, exist_ok=True)
    runner = get_runner("icarus")
    runner.build(
        sources=[str(p) for p in sources],
        hdl_toplevel=top,
        defines={"SIM": 1, "FUNCTIONAL": 1},
        includes=[str(SRC)],
        parameters=parameters,
        build_dir=str(build_dir),
        always=True,
    )
    return runner, build_dir, top


def _run_case(kind, testcase):
    runner, build_dir, top = _build(kind)
    results_xml = build_dir / f"results_{testcase}.xml"
    runner.test(
        hdl_toplevel=top,
        test_module="cocotb_host_spi_arb",
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


def test_arb_single_read_write_latency():
    _run_case("arb6", "test_arb_single_read_write_latency")


def test_arb_byte_lanes_and_end_addresses():
    _run_case("arb6", "test_arb_byte_lanes_and_end_addresses")


def test_arb_round_robin_ii1_fairness():
    _run_case("arb6", "test_arb_round_robin_ii1_fairness")


def test_arb_random_exclusion_stress():
    _run_case("arb6", "test_arb_random_exclusion_stress")


def test_arb5_round_robin_ii1_fairness():
    _run_case("arb5", "test_arb_round_robin_ii1_fairness")


def test_arb5_random_exclusion_stress():
    _run_case("arb5", "test_arb_random_exclusion_stress")


def test_host_spi_csr_config_and_core_ownership():
    _run_case("harness", "test_host_spi_csr_config_and_core_ownership")


def test_host_spi_sram_bursts_and_abort():
    _run_case("harness", "test_host_spi_sram_bursts_and_abort")


def test_host_spi_write_commit_and_contention():
    _run_case("harness", "test_host_spi_write_commit_and_contention")


def test_host_spi_ena_isolation():
    _run_case("harness", "test_host_spi_ena_isolation")
