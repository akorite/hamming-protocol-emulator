# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Native-Windows cocotb runner: simulates the pin-only test suite against the
default IHP-model build and the -DTARGET_FPGA inferred-RAM build, reading the
source list straight out of info.yaml so the two can never drift."""

import os
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from cocotb_tools.runner import get_runner

TEST_DIR = Path(__file__).resolve().parent
ROOT = TEST_DIR.parent
SRC_DIR = ROOT / "src"
MODELS_DIR = TEST_DIR / "sim_models"


def _project_sources():
    """Parse `source_files:` from info.yaml (simple list, no yaml dep)."""
    files = []
    in_list = False
    for line in (ROOT / "info.yaml").read_text().splitlines():
        stripped = line.strip()
        if stripped == "source_files:" or stripped.startswith("source_files:"):
            in_list = True
            continue
        if in_list:
            if stripped.startswith("- "):
                files.append(stripped[2:].strip().strip('"').strip("'"))
            elif stripped and not line.startswith(" "):
                break
    assert files, "info.yaml: no source_files parsed"
    return [SRC_DIR / f for f in files]


@pytest.mark.parametrize("fpga", [False, True], ids=["ihp-sram", "fpga-bram"])
def test_pin_smoke(fpga, tmp_path):
    defines = {"SIM": 1, "FUNCTIONAL": 1}
    sources = list(_project_sources())
    if fpga:
        defines["TARGET_FPGA"] = 1
    else:
        sources.append(MODELS_DIR / "RM_IHPSG13_1P_512x32_c2_bm_bist.v")
    sources.append(TEST_DIR / "tb.v")

    name = "fpga" if fpga else "asic"
    build_dir = TEST_DIR / "sim_build" / f"runner_{name}"
    build_dir.mkdir(parents=True, exist_ok=True)

    runner = get_runner("icarus")
    runner.build(
        sources=[str(s) for s in sources],
        hdl_toplevel="tb",
        defines=defines,
        includes=[str(SRC_DIR)],
        build_dir=str(build_dir),
        always=True,
    )
    results_xml = build_dir / "results.xml"
    runner.test(
        hdl_toplevel="tb",
        test_module="test",
        results_xml=str(results_xml),
        test_dir=str(TEST_DIR),
    )

    bad = [
        tc.get("name")
        for tc in ET.parse(results_xml).getroot().iter("testcase")
        for child in tc
        if child.tag in ("failure", "error")
    ]
    assert not bad, f"{name}: failing cocotb tests: {bad}"
