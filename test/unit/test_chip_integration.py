# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""pytest/cocotb full-chip integration suite for tt_um_akorite_hamming."""

import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

from cocotb_tools.runner import get_runner

UNIT_DIR = Path(__file__).resolve().parent
ROOT = UNIT_DIR.parents[1]
SRC = ROOT / "src"
BUILD_ROOT = ROOT / "test" / "sim_build" / "unit_chip"

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


def _run(testcase, module="cocotb_chip"):
    runner, build_dir = _build_chip()
    results_xml = build_dir / f"results_{testcase}.xml"
    runner.test(
        hdl_toplevel="tt_um_akorite_hamming",
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


def test_ctrl_run_atomic_start():
    _run("test_ctrl_run_atomic_start")


def test_sync_event_wakeup():
    _run("test_sync_event_wakeup")


def test_ena_isolation():
    _run("test_ena_isolation")


