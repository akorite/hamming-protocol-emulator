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
        "synth -top tt_um_akorite_hamming; check; stat -top tt_um_akorite_hamming"
    )
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
    probs = max((int(n) for n in re.findall(r"found and reported (\d+) problems", out, re.I)), default=0)
    if probs:
        sys.exit(f"check_synth: yosys `check` reported {probs} problems")
    latches = sum(int(n) for n in re.findall(r"^\s+(\d+)\s+\$_DLATCH\w*\s*$", out, re.M))
    if latches:
        sys.exit(f"check_synth: {latches} inferred latches")
    ffs = sum(int(n) for n in re.findall(r"^\s+(\d+)\s+\$_S?DFF\w*\s*$", out, re.M))
    print(f"check_synth: {ffs} flip-flops (budget {FF_BUDGET}, ASIC only)")
    if not args.fpga and ffs > FF_BUDGET:
        sys.exit(f"check_synth: {ffs} FFs exceeds {FF_BUDGET} budget")


if __name__ == "__main__":
    main()
