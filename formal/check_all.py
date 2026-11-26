#!/usr/bin/env python3
"""Run and audit the five Hamming SymbiYosys proof harnesses.

The audit is deliberately stricter than merely checking the process return
code: every expected task work directory must contain a SymbiYosys status
file whose first token is PASS.  Thus a crashed or partially-written run can
never be reported as a green 15/15 suite.

Bitwuzla is the primary solver as required by the project plan.  If a task
ends UNKNOWN/ERROR or the solver executable fails before producing a status,
that task is retried once with z3 in an isolated ``z3_fallback`` directory.
A real assertion FAIL is never hidden by the fallback.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


ROOT = Path(__file__).resolve().parent
HARNESSES = [
    "sram_arb",
    "lut_matrix",
    "bcu_stuffer",
    "bcu_dpll",
    "deadline_sched",
]
TASKS = ["bmc", "prove", "cover"]
LOCAL_VERILOG = ["bcu_stuffer_top.v", "deadline_sched_top.v"]
LOGDIR = ROOT / "check_all_logs"
FALLBACK_ROOT = ROOT / "z3_fallback"


def _candidate_paths() -> List[Path]:
    candidates: List[Path] = []
    env_root = os.environ.get("OSS_CAD_SUITE")
    roots = []
    if env_root:
        roots.append(Path(env_root))
    roots.append(Path(r"C:\tools\oss-cad-suite"))
    for root in roots:
        for name in ("sby.exe", "sby.bat", "sby"):
            candidates.append(root / "bin" / name)
    return candidates


def locate_sby() -> str:
    """Find sby, preferring the documented native Windows OSS CAD Suite.

    Native Windows yosys.exe also needs the OSS CAD Suite ``lib`` directory on
    PATH for its DLL dependencies.  Add both directories before spawning sby;
    merely returning the absolute sby path is not sufficient on a clean shell.
    """
    env_root = os.environ.get("OSS_CAD_SUITE")
    roots = []
    if env_root:
        roots.append(Path(env_root))
    roots.append(Path(r"C:\tools\oss-cad-suite"))

    for root in roots:
        bin_dir = root / "bin"
        lib_dir = root / "lib"
        if bin_dir.is_dir():
            path_text = os.environ.get("PATH", "")
            path_entries = path_text.split(os.pathsep) if path_text else []
            prepend = []
            if str(bin_dir) not in path_entries:
                prepend.append(str(bin_dir))
            if lib_dir.is_dir() and str(lib_dir) not in path_entries:
                prepend.append(str(lib_dir))
            if prepend:
                os.environ["PATH"] = os.pathsep.join(prepend + [path_text])
        for name in ("sby.exe", "sby.bat", "sby"):
            candidate = bin_dir / name
            if candidate.is_file():
                return str(candidate)

    found = shutil.which("sby") or shutil.which("sby.exe") or shutil.which("sby.bat")
    if found:
        return found
    raise FileNotFoundError(
        "could not locate SymbiYosys (sby); expected "
        r"C:\tools\oss-cad-suite\bin\sby.exe or OSS_CAD_SUITE/bin on PATH"
    )


def status_in(workdir: Path, harness: str, task: str) -> Optional[str]:
    path = workdir / f"{harness}_{task}" / "status"
    if not path.is_file():
        return None
    try:
        first = path.read_text(errors="replace").split(None, 1)[0]
    except IndexError:
        return "EMPTY"
    return first.upper()


def run_sby(
    sby: str,
    sby_file: Path,
    cwd: Path,
    log_path: Path,
    task: Optional[str] = None,
) -> Tuple[int, str]:
    if sby_file.resolve().parent == cwd.resolve():
        target = sby_file.name
    else:
        target = str(sby_file)
    cmd = [sby, "-f", target]
    if task is not None:
        cmd.append(task)
    proc = subprocess.run(
        cmd,
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
        check=False,
    )
    log_path.parent.mkdir(parents=True, exist_ok=True)
    command_line = "> " + " ".join(cmd) + "\n\n"
    log_path.write_text(command_line + proc.stdout, encoding="utf-8")
    return proc.returncode, proc.stdout


def prepare_fallback(harness: str) -> Tuple[Path, Path]:
    """Create a same-layout z3 copy one directory below formal/."""
    outdir = FALLBACK_ROOT / harness
    outdir.mkdir(parents=True, exist_ok=True)
    src = ROOT / f"{harness}.sby"
    text = src.read_text(encoding="utf-8")
    text = text.replace("smtbmc bitwuzla", "smtbmc z3")
    # The generated file is one level deeper, so repository-source relative
    # paths need one more parent component.  Local harness Verilog is copied.
    text = text.replace("../src/", "../../src/")
    for local in LOCAL_VERILOG:
        local_src = ROOT / local
        if local_src.is_file():
            shutil.copy2(local_src, outdir / local)
    dst = outdir / f"{harness}.sby"
    dst.write_text(text, encoding="utf-8")
    return dst, outdir


def fallback_needed(status: Optional[str], output: str, returncode: int) -> bool:
    if status == "PASS":
        return False
    if status == "FAIL":
        return False
    if status in {"UNKNOWN", "ERROR", "TIMEOUT", "EMPTY", None}:
        return True
    upper = output.upper()
    if returncode != 0 and "DONE (FAIL" not in upper:
        return True
    return False


def run_harness(sby: str, harness: str) -> Tuple[bool, Dict[str, str], Dict[str, str]]:
    """Run one harness.  Return (all_pass, task_status, task_solver)."""
    statuses: Dict[str, str] = {}
    solvers: Dict[str, str] = {}
    primary_sby = ROOT / f"{harness}.sby"
    primary_log = LOGDIR / f"{harness}.log"

    rc, out = run_sby(sby, primary_sby, ROOT, primary_log)

    for task in TASKS:
        st = status_in(ROOT, harness, task)
        if st == "PASS":
            statuses[task] = "PASS"
            solvers[task] = "bitwuzla"
            continue

        if fallback_needed(st, out, rc):
            fb_sby, fb_dir = prepare_fallback(harness)
            fb_log = LOGDIR / f"{harness}_{task}_z3.log"
            fb_rc, fb_out = run_sby(sby, fb_sby, fb_dir, fb_log, task=task)
            fb_st = status_in(fb_dir, harness, task)
            if fb_st == "PASS":
                statuses[task] = "PASS"
                solvers[task] = "z3-fallback"
            else:
                statuses[task] = fb_st or f"NO-STATUS(rc={fb_rc})"
                solvers[task] = "z3-fallback"
        else:
            statuses[task] = st or f"NO-STATUS(rc={rc})"
            solvers[task] = "bitwuzla"

    return all(statuses.get(task) == "PASS" for task in TASKS), statuses, solvers


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args(argv)

    try:
        sby = locate_sby()
    except FileNotFoundError as exc:
        print(f"check_all: ERROR: {exc}", file=sys.stderr)
        return 2

    selected = HARNESSES
    total_expected = 15
    total_pass = 0
    all_green = True

    print(f"check_all: using sby: {sby}")
    for harness in selected:
        ok, statuses, solvers = run_harness(sby, harness)
        for task in TASKS:
            status = statuses.get(task, "MISSING")
            solver = solvers.get(task, "unknown")
            passed = status == "PASS"
            total_pass += int(passed)
            suffix = f" ({solver})" if passed and solver != "bitwuzla" else ""
            print(f"{harness:15s} {task:5s} {status:12s}{suffix}")
        if not ok:
            all_green = False

    print(f"check_all: {total_pass}/{total_expected} tasks PASS")
    if len(selected) == len(HARNESSES) and total_pass != 15:
        # This is redundant with all_green for subsets, but makes the contract
        # explicit in the final output for the full suite.
        print("check_all: FAIL (required full-suite result is 15/15)")
    elif not all_green:
        print("check_all: FAIL")
    elif total_expected == 15:
        print("check_all: PASS (15/15)")
    else:
        print(f"check_all: PASS (selected {total_expected}/{total_expected})")

    return 0 if all_green else 1


if __name__ == "__main__":
    sys.exit(main())
