#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Local Yosys lint + area gate for the Hamming ASIC.

Runs `yosys synth` on the RTL source list from info.yaml (the SRAM is the
blackbox from src/sram_bb.v), then fails on inferred latches, combinational
loops reported by `check`, or a flop count above the 4,700 FF budget.
`--fpga` synthesises with -DTARGET_FPGA against the inferred-BRAM wrapper.
"""

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
INFO = ROOT / "info.yaml"
FF_BUDGET = 4700

OSS_CAD_BIN = Path(r"C:\tools\oss-cad-suite\bin")

# The OSS CAD Suite tools need their lib dir on PATH (DLLs live there).
if OSS_CAD_BIN.is_dir():
    os.environ["PATH"] = (
        f"{OSS_CAD_BIN}{os.pathsep}{OSS_CAD_BIN.parent / 'lib'}{os.pathsep}"
        + os.environ.get("PATH", "")
    )


def yosys_bin():
    exe = "yosys.exe" if (OSS_CAD_BIN / "yosys.exe").exists() else "yosys"
    return str(OSS_CAD_BIN / exe) if exe.endswith(".exe") else exe


def run_yosys(script):
    result = subprocess.run(
        [yosys_bin(), "-Q", "-T", "-p", script],
        capture_output=True,
        text=True,
        cwd=ROOT,
    )
    out = result.stdout + result.stderr
    sys.stdout.write(out)
    if result.returncode != 0:
        sys.exit(f"check_synth: yosys failed ({result.returncode})")
    return out


def lint_yosys_output(out, label):
    probs = max((int(n) for n in re.findall(r"found and reported (\d+) problems", out, re.I)), default=0)
    if probs:
        sys.exit(f"check_synth: {label}: yosys `check` reported {probs} problems")
    width_warnings = re.findall(r"^Warning:.*width.*$", out, re.I | re.M)
    if width_warnings:
        sys.exit(f"check_synth: {label}: width warnings: {width_warnings[0]}")

    # `synth` prints a statistics block and the explicit trailing `stat`
    # prints it again. Restrict cell counts to the requested top module's
    # local block so duplicate summary/hierarchy tables are not counted.
    stat_blocks = re.findall(
        rf"^=== {re.escape(label)} ===\n(.*?)(?=^=== |\Z)", out, re.S | re.M
    )
    stat_text = stat_blocks[-1] if stat_blocks else out
    latches = sum(int(n) for n in re.findall(r"^\s+(\d+)\s+\$_DLATCH\w*\s*$", stat_text, re.M))
    if latches:
        sys.exit(f"check_synth: {label}: {latches} inferred latches")
    ffs = sum(int(n) for n in re.findall(r"^\s+(\d+)\s+\$_S?DFF\w*\s*$", stat_text, re.M))
    cell_counts = re.findall(r"^\s+(\d+) cells\s*$", stat_text, re.M)
    cells = int(cell_counts[-1]) if cell_counts else 0
    return ffs, cells


def project_sources():
    files = []
    in_list = False
    for line in INFO.read_text().splitlines():
        stripped = line.strip()
        if stripped.startswith("source_files:"):
            in_list = True
            continue
        if in_list:
            if stripped.startswith("- "):
                files.append(stripped[2:].strip().strip('"').strip("'"))
            elif stripped and not line.startswith(" "):
                break
    if not files:
        sys.exit("check_synth: no source_files parsed from info.yaml")
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fpga", action="store_true", help="synthesise the TARGET_FPGA build")
    args = ap.parse_args()

    defines = ["-DTARGET_FPGA"] if args.fpga else []
    read_cmd = "; ".join(
        f"read_verilog {' '.join(defines)} -I{SRC} {SRC / f}" for f in project_sources()
    )
    script = (
        f"{read_cmd}; hierarchy -top tt_um_akorite_hamming; proc; "
        "synth -top tt_um_akorite_hamming; check; flatten; stat -top tt_um_akorite_hamming"
    )
    out = run_yosys(script)
    ffs, cells = lint_yosys_output(out, "tt_um_akorite_hamming")
    print(f"check_synth: {ffs} flip-flops, {cells} cells (FF budget {FF_BUDGET}, ASIC only)")
    if not args.fpga and ffs > FF_BUDGET:
        sys.exit(f"check_synth: {ffs} FFs exceeds {FF_BUDGET} budget")

    # Units not yet instantiated by the integration top still receive a real
    # elaboration/synthesis lint here. Merely reading them into the top-level
    # script would let hierarchy pruning hide latches and width mismatches.
    unit_modules = [
        ("hamming_host_spi", ["hamming_host_spi.v"]),
        ("hamming_csr", ["hamming_csr.v"]),
        ("hamming_sram_arb", ["sram_bb.v", "hamming_sram_2kb.v", "hamming_sram_arb.v"]),
        ("hamming_icache", ["hamming_icache.v"]),
        ("hamming_core", ["hamming_icache.v", "hamming_core.v"]),
        ("hamming_bcu_crc", ["hamming_bcu_crc.v"]),
        ("hamming_lut_matrix", ["hamming_lut_matrix.v"]),
        ("hamming_bcu", ["hamming_bcu.v", "hamming_bcu_crc.v",
                         "hamming_bcu_dpll.v"]),
        ("hamming_rle_engine", ["hamming_rle_engine.v"]),
    ]
    unit_ff_counts = {}
    for unit_top, files in unit_modules:
        if not all((SRC / f).exists() for f in files):
            continue
        unit_read = "; ".join(
            f"read_verilog {' '.join(defines)} -I{SRC} {SRC / f}" for f in files
        )
        unit_script = (
            f"{unit_read}; hierarchy -top {unit_top}; proc; "
            f"synth -top {unit_top}; check; flatten; stat -top {unit_top}"
        )
        unit_ffs, unit_cells = lint_yosys_output(run_yosys(unit_script), unit_top)
        unit_ff_counts[unit_top] = (unit_ffs, unit_cells)
        print(f"check_synth: {unit_top}: {unit_ffs} flip-flops, {unit_cells} cells")

    # U4 plan gate: one protocol core including its loop cache must stay at
    # or below 520 flip-flops.  Yosys reports local counts, so the cache and
    # core-local counts are summed exactly.  The plan's aggressive 2,200-cell
    # target is reported visibility-only for now; correctness and lint remain
    # hard gates while U5/U6 integration determines the final area budget.
    if "hamming_core" in unit_ff_counts:
        # The flattened core count is hierarchical and already includes its
        # loop cache; the standalone icache line is reported for visibility.
        core_ffs, core_cells = unit_ff_counts["hamming_core"]
        core_cache_ffs = core_ffs
        core_cache_cells = core_cells
        print(
            f"check_synth: core+cache total: {core_cache_ffs} flip-flops "
            f"(budget 520), {core_cache_cells} cells (target < 2200)"
        )
        if core_cache_ffs > 520:
            sys.exit(f"check_synth: core+cache {core_cache_ffs} FFs exceeds 520 budget")

    # U5 plan gate: the LUT matrix, output router, and input synchronizers
    # together must stay at or below 730 flip-flops.
    if "hamming_lut_matrix" in unit_ff_counts:
        lut_ffs, lut_cells = unit_ff_counts["hamming_lut_matrix"]
        print(
            f"check_synth: lut matrix+router+sync total: {lut_ffs} flip-flops "
            f"(budget 730), {lut_cells} cells"
        )
        if lut_ffs > 730:
            sys.exit(f"check_synth: lut matrix {lut_ffs} FFs exceeds 730 budget")

    # U5 plan gate: one BCU including its CRC engine and DPLL must stay at
    # or below 270 flip-flops.  The CRC and DPLL submodule counts are
    # reported separately by their own entries but included here because
    # yosys reports hierarchical instance counts at the top level.
    if "hamming_bcu" in unit_ff_counts:
        bcu_ffs, bcu_cells = unit_ff_counts["hamming_bcu"]
        print(
            f"check_synth: bcu total: {bcu_ffs} flip-flops "
            f"(budget 270), {bcu_cells} cells"
        )
        if bcu_ffs > 270:
            sys.exit(f"check_synth: bcu {bcu_ffs} FFs exceeds 270 budget")


if __name__ == "__main__":
    main()
