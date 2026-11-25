"""Cocotb tests for hamming_bcu_dpll: hard-snap/hold-off bit-center recovery."""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

PERIOD = 10  # cycles per bit cell, half = 5


async def setup(dut, mode=0, win=0, period=PERIOD):
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    dut.rst_n.value = 0
    dut.run.value = 0
    dut.edge_in.value = 0
    dut.cfg_mode.value = mode
    dut.cfg_win.value = win
    dut.cfg_period.value = period
    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    for _ in range(2):
        await RisingEdge(dut.clk)


async def cycles(dut, n):
    for _ in range(n):
        await RisingEdge(dut.clk)


async def pulse_edge(dut):
    dut.edge_in.value = 1
    await RisingEdge(dut.clk)
    dut.edge_in.value = 0


@cocotb.test()
async def test_free_run_sample_period(dut):
    """Mode none: sample strobes repeat exactly every `period` cycles."""
    await setup(dut, mode=0, period=PERIOD)
    dut.run.value = 1
    times = []
    for cyc in range(PERIOD * 6):
        await RisingEdge(dut.clk)
        if dut.sample.value == 1:
            times.append(cyc)
    assert len(times) >= 4, f"too few samples: {times}"
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert all(g == PERIOD for g in gaps), f"sample gaps {gaps} != {PERIOD}"


@cocotb.test()
async def test_hard_snap_rephase(dut):
    """Hard snap: an edge restarts the cell so the next sample lands at
    the bit centre, half a period after the edge."""
    await setup(dut, mode=1, period=PERIOD)
    dut.run.value = 1
    # Free-run a while, then inject an edge at an arbitrary phase.
    await cycles(dut, 7)
    await pulse_edge(dut)  # edge cycle count starts after this edge
    # sample should appear half-1..half+1 cycles after the edge
    found = -1
    for cyc in range(1, PERIOD + 2):
        await RisingEdge(dut.clk)
        if dut.sample.value == 1:
            found = cyc
            break
    half = PERIOD // 2
    assert found != -1, "no sample within one period of edge"
    assert abs(found - half) <= 1, f"sample at +{found}, expected ~+{half}"


@cocotb.test()
async def test_hold_off_rejects_midcell(dut):
    """Hold-off: mid-cell edges are ignored, near-boundary edges accepted."""
    await setup(dut, mode=2, win=1, period=PERIOD)
    dut.run.value = 1
    # Acquire lock with an initial edge (edge-recovered modes only emit
    # samples once locked), then establish the sample phase.
    await pulse_edge(dut)
    first = None
    for cyc in range(PERIOD * 2):
        await RisingEdge(dut.clk)
        if dut.sample.value == 1 and first is None:
            first = cyc
            break
    assert first is not None, "no sample after lock edge"
    # Now inject an edge one cycle after the sample (deep mid-cell).
    await RisingEdge(dut.clk)
    await pulse_edge(dut)
    # The next sample must still arrive on the original free-running
    # schedule: PERIOD cycles after the first one.
    second = None
    for cyc in range(1, PERIOD + 3):
        await RisingEdge(dut.clk)
        if dut.sample.value == 1:
            second = cyc
            break
    # Measurement starts two clocks after the first sample (one idle edge
    # plus the edge-injection clock), so an unrephased free-running DPLL
    # samples again at +PERIOD-2 in this loop's frame.
    assert second == PERIOD - 2, (
        f"mid-cell edge rephased the DPLL: second sample at +{second}, "
        f"expected +{PERIOD - 2}")


@cocotb.test()
async def test_hold_off_accepts_boundary(dut):
    """Hold-off: an edge close to the cell boundary is accepted."""
    await setup(dut, mode=2, win=2, period=PERIOD)
    dut.run.value = 1
    # Wait for a cell_end, then drive an edge within the window.
    for cyc in range(PERIOD * 3):
        await RisingEdge(dut.clk)
        if dut.cell_end.value == 1:
            break
    # One cycle after cell_end is within win=2 (win+1=3 cycles) of boundary.
    await RisingEdge(dut.clk)
    await pulse_edge(dut)
    # Sample should rephase to half a period after this edge.
    found = -1
    for cyc in range(1, PERIOD + 2):
        await RisingEdge(dut.clk)
        if dut.sample.value == 1:
            found = cyc
            break
    half = PERIOD // 2
    assert found != -1 and abs(found - half) <= 1, (
        f"boundary edge not accepted: sample at +{found}, expected ~+{half}")


@cocotb.test()
async def test_eof_count_nine(dut):
    """EOF: with no edges, eof strobes when miss count reaches 9 half-cells."""
    await setup(dut, mode=1, period=PERIOD)
    dut.run.value = 1
    half = PERIOD // 2
    saw_eof = False
    eof_cycle = -1
    for cyc in range(half * 12):
        await RisingEdge(dut.clk)
        if dut.eof.value == 1 and not saw_eof:
            saw_eof = True
            eof_cycle = cyc
    assert saw_eof, "eof never asserted"
    # 9 half-cells, with the count starting one half-cell in: tolerance band
    # around 9*half cycles.
    assert abs(eof_cycle - 9 * half) <= half, (
        f"eof at cycle {eof_cycle}, expected ~{9 * half}")
    # An edge before EOF prevents it.
    await setup(dut, mode=1, period=PERIOD)
    dut.run.value = 1
    for _ in range(3):
        await cycles(dut, half * 4)
        await pulse_edge(dut)
    for cyc in range(half * 3):
        await RisingEdge(dut.clk)
        assert dut.eof.value == 0, "eof asserted while wire active"


@cocotb.test()
async def test_no_samples_when_stopped(dut):
    """With run low, the DPLL produces no sample or eof strobes."""
    await setup(dut, mode=1, period=PERIOD)
    for cyc in range(PERIOD * 12):
        await RisingEdge(dut.clk)
        assert dut.sample.value == 0
        assert dut.eof.value == 0
        assert dut.miss_cnt.value == 0
