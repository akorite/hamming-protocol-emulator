# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Windows-native pytest bootstrap: put the project venv first on PATH so the
OSS CAD Suite tools (iverilog/vvp/yosys/sby) resolve but Python stays ours."""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VENV_BIN = ROOT / ".venv" / ("Scripts" if os.name == "nt" else "bin")
OSS_CAD_BIN = Path(os.environ.get("OSS_CAD_BIN", r"C:\tools\oss-cad-suite\bin"))

extra = [str(VENV_BIN)]
if OSS_CAD_BIN.is_dir():
    extra += [str(OSS_CAD_BIN), str(OSS_CAD_BIN.parent / "lib")]
os.environ["PATH"] = os.pathsep.join(extra + [os.environ.get("PATH", "")])

sys.path.insert(0, str(ROOT))
