# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Self-tests for tools/hamming_z3_timing.py.

These tests are deliberately executable without the RTL simulator.  They use
small fixed-latency programs for exact numeric oracles, include negative
(mutation) checks, and finish with a real deadline-critical fragment of the
shipped 10BASE-T firmware.
"""

import json
from pathlib import Path

import pytest

z3 = pytest.importorskip("z3", reason="z3-solver is required for timing-verifier tests")

from tools import hamming_z3_timing as zt  # noqa: E402


def make_analyzer(src, *, profile="iss-default", **config_kw):
    image = zt.image_from_source(src)
    cfg = zt.TimingConfig(
        profile=profile,
        solver_timeout_ms=5000,
        optimize_timeout_ms=5000,
        **config_kw,
    )
    return zt.HammingTimingAnalyzer(image, cfg)


def test_straight_line_exact_cycle_bound_and_smt2(tmp_path):
    # Cold miss(4) for slot0 + same-word hit(1) for slot1.  END is reached
    # before the HALT instruction, so exactly five cycles are required.
    src = """
start:
    nop
    nop
end:
    halt
"""
    analyzer = make_analyzer(src)
    proved = analyzer.check("start", "end", 5)
    assert proved.status == "PROVED", proved.message
    assert proved.max_cost == 5
    assert proved.max_total_cost == 5

    smt2 = tmp_path / "straight_cex.smt2"
    mutation = analyzer.check("start", "end", 4, smt2_path=smt2)
    assert mutation.status == "VIOLATION"
    assert mutation.witness is not None
    assert "pc=0x000" in "\n".join(mutation.witness.trace_lines)
    assert smt2.exists()
    assert "check-sat" in smt2.read_text()


def test_data_dependent_loop_worst_case_and_one_less_mutation():
    # r0 is an input.  With 1 <= r0 <= 3, the first iteration costs
    # miss(4)+hit(1); later iterations are two hit cycles each because slots
    # 0/1 share one SRAM0 word.  Worst case r0=3 is therefore 5+2+2 = 9.
    src = """
loop:
    dec r0
    jnz loop
end:
    halt
"""
    analyzer = make_analyzer(src)
    assumptions = ["r0 != 0", "r0 <= 3"]

    proved = analyzer.check("loop", "end", 9, assumptions=assumptions)
    assert proved.status == "PROVED", proved.message
    assert proved.max_cost == 9

    mutation = analyzer.check("loop", "end", 8, assumptions=assumptions)
    assert mutation.status == "VIOLATION"
    assert mutation.witness is not None
    assert mutation.witness.kind == "budget-exceeded"
    # The concrete input conditions must identify a three-iteration case.
    assert mutation.witness.inputs.get("r0") == "0x0003"


def test_direct_mapped_icache_conflict_misses():
    # Slot0 uses SRAM0 word0 and slot16 uses word8.  Both map to cache index0,
    # so the target fetch conflicts with the line filled for slot0.
    src = """
start:
    jmp target
.org 16
target:
    nop
end:
    halt
"""
    analyzer = make_analyzer(src)
    result = analyzer.check("start", "end", 8)
    assert result.status == "PROVED", result.message
    assert result.max_cost == 8

    # Reconstructing the trace requires a violation; use budget7 and inspect
    # that both independent miss envelopes appear in the counterexample trace.
    mutation = analyzer.check("start", "end", 7)
    assert mutation.status == "VIOLATION"
    text = "\n".join(mutation.witness.trace_lines)
    assert text.count("fetch miss") == 2


def test_wait_phase_anchor_exactness_small_program():
    # set_anchor costs four cold-miss cycles in the fixed ISS profile.  The
    # following hit instruction executes one cycle later, but WAIT_PHASE then
    # absorbs that overhead: total completion is 4+20 = 24 cycles.
    src = """
start:
    set_anchor 0
    wait_phase 20
end:
    halt
"""
    analyzer = make_analyzer(src)
    proved = analyzer.check("start", "end", 24)
    assert proved.status == "PROVED", proved.message
    assert proved.max_cost == 24
    assert analyzer.check("start", "end", 23).status == "VIOLATION"


def test_real_eth_tx_finish_deadline_fragment():
    # firmware/eth_10baset.hasm:tx_finish starts the mandatory 576-cycle IFG
    # with set_anchor + wait_phase r0.  END is the instruction immediately
    # after wait_phase, so no mailbox/branch behavior is involved.  The cold
    # first fetch is four cycles in the fixed ISS profile; the anchored wait
    # absorbs intermediate fetch overhead exactly, giving 4+576 = 580 cycles.
    fw = Path(__file__).resolve().parents[1] / "firmware" / "eth_10baset.hasm"
    image = zt.load_firmware(fw)
    cfg = zt.TimingConfig(profile="iss-default", solver_timeout_ms=8000,
                          optimize_timeout_ms=8000)
    analyzer = zt.HammingTimingAnalyzer(image, cfg)
    start = image.labels["tx_finish"]
    end = start + 4  # before the first post-WAIT mailbox instruction

    proved = analyzer.check(start, end, 580)
    assert proved.status == "PROVED", proved.message
    assert proved.max_cost == 580

    mutation = analyzer.check(start, end, 579)
    assert mutation.status == "VIOLATION"


def test_batch_mode_json_spec(tmp_path, capsys):
    fw = tmp_path / "tiny.hasm"
    fw.write_text("start:\n nop\nend:\n halt\n")
    spec = tmp_path / "spec.json"
    spec.write_text(json.dumps({
        "defaults": {"profile": "iss-default"},
        "checks": [{
            "firmware": "tiny.hasm",
            "start": "start", "end": "end", "budget": 4
        }],
    }))
    rc = zt.main(["--batch", str(spec)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "PROVED" in out
