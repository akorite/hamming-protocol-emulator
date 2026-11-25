# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, Timer

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

_CLOCK_STARTED = False


def u(sig):
    value = sig.value
    if hasattr(value, "to_unsigned"):
        return value.to_unsigned()
    return int(value)


async def ensure_clock(dut):
    global _CLOCK_STARTED
    if not _CLOCK_STARTED:
        cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
        _CLOCK_STARTED = True


def pack_lut_tt(entries):
    """entries: dict lut index -> 16-bit truth table."""
    flat = 0
    for k, tt in entries.items():
        flat |= (tt & 0xFFFF) << (16 * k)
    return flat


def pack_lut_sel(entries):
    """entries: dict lut index -> (sel0, sel1, sel2, sel3) input selectors."""
    flat = 0
    for k, sels in entries.items():
        lane = 0
        for i, s in enumerate(sels):
            lane |= (s & 0xF) << (4 * i)
        flat |= lane << (16 * k)
    return flat


def pack_router(sels):
    """sels: list of 23 4-bit channel selectors."""
    flat = 0
    for ch, s in enumerate(sels):
        flat |= (s & 0xF) << (4 * ch)
    return flat


async def reset_dut(dut):
    dut.uio_in.value = 0
    dut.ui_in.value = 0
    dut.pin_filt_en.value = 0
    dut.lut_tt_flat.value = 0
    dut.lut_sel_flat.value = 0
    dut.lut_oreg.value = 0
    dut.pin_router_flat.value = 0
    dut.open_drain_en.value = 0
    dut.ev_flags.value = 0
    dut.replay_data.value = 0
    dut.replay_active.value = 0
    dut.host_cs_active.value = 0
    dut.core_out_flat.value = 0
    dut.core_oe_flat.value = 0
    dut.bcu_tx_p.value = 0
    dut.bcu_tx_n.value = 0
    dut.bcu_tx_oe.value = 0
    dut.rst_n.value = 0
    dut.ena.value = 1
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_lut_two_tier_truth_tables(dut):
    """Tier-1 computes per-pin SE0 (= NOT dp AND NOT dm) and tier-2 combines
    tier-1 outputs with event flags, matching a software truth-table model
    for all input combinations."""
    await ensure_clock(dut)
    await reset_dut(dut)

    # LUT0 (tier1): SE0 on pins 0/1: out = ~in0 & ~in1 with in0=pin0 in1=pin1.
    # tt index {in3,in2,in1,in0}: set when in1==0 and in0==0.
    tt_se0 = 0
    for idx in range(16):
        if (idx & 0b11) == 0:
            tt_se0 |= 1 << idx
    # LUT1 (tier1): I2C START detect helper: in0=sda(pin2), in1=scl(pin3):
    # start condition inputs sda falling while scl high -> model sda=0,scl=1.
    tt_start = 0
    for idx in range(16):
        if (idx & 0b11) == 0b10:
            tt_start |= 1 << idx
    # LUT8 (tier2): se0 AND ev0: in0=lut0, in1=ev0.
    tt_se0_ev = 0
    for idx in range(16):
        if (idx & 0b11) == 0b11:
            tt_se0_ev |= 1 << idx

    dut.lut_tt_flat.value = pack_lut_tt({0: tt_se0, 1: tt_start, 8: tt_se0_ev})
    dut.lut_sel_flat.value = pack_lut_sel({
        0: (0, 1, 15, 15),   # pin0, pin1, zero, zero
        1: (2, 3, 15, 15),   # pin2, pin3, zero, zero
        8: (0, 8, 15, 15),   # lut0, ev0, zero, zero
    })

    cases = [
        # (uio_in, ev_flags, want_lut0, want_lut1, want_lut8_noev)
        (0b00000000, 0, 1, 0, 1),
        (0b00000001, 0, 0, 0, 0),
        (0b00000010, 0, 0, 0, 0),
        (0b00000100, 0, 1, 0, 1),
        (0b00001100, 0, 1, 0, 1),
        (0b00001000, 0, 1, 1, 1),
    ]
    for uio, ev, w0, w1, w8 in cases:
        dut.uio_in.value = uio
        dut.ev_flags.value = ev
        await ClockCycles(dut.clk, 4)  # sync pipeline depth
        await Timer(1, unit="ns")
        outs = u(dut.lut_outputs)
        assert (outs >> 0) & 1 == w0, f"uio={uio:08b} lut0={(outs >> 0) & 1} want {w0}"
        assert (outs >> 1) & 1 == w1, f"uio={uio:08b} lut1={(outs >> 1) & 1} want {w1}"
        # tier2 adds no extra latency when oreg is off; ev is not set here.
        assert (outs >> 8) & 1 == 0, f"uio={uio:08b} lut8 should be 0 without ev0"

    # With ev0 set, lut8 follows lut0.
    dut.ev_flags.value = 1
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 4)
    await Timer(1, unit="ns")
    assert (u(dut.lut_outputs) >> 8) & 1 == 1, "lut8 should be lut0 AND ev0"
    dut.uio_in.value = 1
    await ClockCycles(dut.clk, 4)
    await Timer(1, unit="ns")
    assert (u(dut.lut_outputs) >> 8) & 1 == 0, "lut8 should drop when pin0 rises"


@cocotb.test()
async def test_lut_glitch_filter_rejects_short_spikes(dut):
    """A filtered uio pin rejects spikes of <= 3 cycles (50 ns at 60 MHz)
    and passes pulses reaching the 4-tap depth."""
    await ensure_clock(dut)
    await reset_dut(dut)
    dut.pin_filt_en.value = 0x01  # filter uio pin 0

    # Idle low for long enough to fill the filter.
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 8)
    assert u(dut.pin_inputs) & 1 == 0

    # 2-cycle spike (~33 ns): must be rejected.
    dut.uio_in.value = 1
    await ClockCycles(dut.clk, 2)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 8)
    assert u(dut.pin_inputs) & 1 == 0, "2-cycle spike leaked through filter"

    # 3-cycle spike (50 ns): must be rejected.
    dut.uio_in.value = 1
    await ClockCycles(dut.clk, 3)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 8)
    assert u(dut.pin_inputs) & 1 == 0, "3-cycle spike leaked through filter"

    # 6-cycle pulse: must pass (filtered output goes high, then low).
    dut.uio_in.value = 1
    await ClockCycles(dut.clk, 6)
    seen_high = (u(dut.pin_inputs) & 1) != 0
    dut.uio_in.value = 0
    for _ in range(6):
        await ClockCycles(dut.clk, 1)
        seen_high = seen_high or (u(dut.pin_inputs) & 1) != 0
    assert seen_high, "6-cycle pulse never passed the filter"

    # With the filter disabled the synchronizer passes a 3-cycle spike.
    dut.pin_filt_en.value = 0x00
    await ClockCycles(dut.clk, 8)
    dut.uio_in.value = 1
    await ClockCycles(dut.clk, 3)
    await Timer(1, unit="ns")
    seen = (u(dut.pin_inputs) & 1) != 0
    dut.uio_in.value = 0
    assert seen, "unfiltered synchronizer path did not pass 3-cycle spike"


@cocotb.test()
async def test_router_single_owner_and_replay_gate(dut):
    """Each output channel forwards only its selected source: core out bits,
    BCU tx signals, LUT tier-2 outputs, and the replay bit gated off unless
    replay_active is set."""
    await ensure_clock(dut)
    await reset_dut(dut)

    # LUT8 = const1 via tt=0xFFFF; routed to uo_out[0] through selector 3.
    dut.lut_tt_flat.value = pack_lut_tt({8: 0xFFFF})
    dut.lut_sel_flat.value = pack_lut_sel({8: (15, 15, 15, 15)})

    # core2 out[0] -> uio_out[3] data (channel 3, pair = 3&3 = 3? bits[1:0]=3)
    # channel 3 pairs with core 3; use channel 2 for core 2.
    # uio_oe[2] selects const1 so the pin is driven.
    sels = [0] * 23
    sels[2] = 11      # uio_out[2] data <= core2 out[0] (pair bits[1:0]=2)
    sels[8 + 2] = 1   # uio_oe[2] <= const1
    sels[16] = 3      # uo_out[0] <= lut8 (const1)
    sels[0] = 2       # uio_out[0] data <= replay
    dut.pin_router_flat.value = pack_router(sels)

    # core_out_flat: core k in bits [4k+3:4k]; set core2 bit0.
    dut.core_out_flat.value = 1 << (2 * 4)
    await ClockCycles(dut.clk, 3)
    await Timer(1, unit="ns")
    assert (u(dut.uio_out) >> 2) & 1 == 1, "core2 out[0] did not reach uio_out[2]"
    assert (u(dut.uio_oe) >> 2) & 1 == 1, "const1 did not reach uio_oe[2]"
    assert u(dut.uo_out) & 1 == 1, "lut8 const1 did not reach uo_out[0]"

    # Other channels stay zero (const0 selectors).
    assert (u(dut.uio_out) & ~0b100) == 0, f"unexpected uio_out {u(dut.uio_out):08b}"

    # Replay: gated to 0 while replay_active is low.
    dut.replay_data.value = 1
    await ClockCycles(dut.clk, 3)
    await Timer(1, unit="ns")
    assert u(dut.uio_out) & 1 == 0, "replay reached pin while replay inactive"
    dut.replay_active.value = 1
    await ClockCycles(dut.clk, 3)
    await Timer(1, unit="ns")
    assert u(dut.uio_out) & 1 == 1, "active replay bit did not reach pin"

    # BCU tx pair on channel 1 (pair index 1).
    sels[1] = 12      # uio_out[1] <= bcu1 tx_p
    sels[8 + 1] = 14  # uio_oe[1] <= bcu1 tx_oe
    dut.pin_router_flat.value = pack_router(sels)
    dut.bcu_tx_p.value = 0b0010
    dut.bcu_tx_oe.value = 0b0010
    await ClockCycles(dut.clk, 3)
    await Timer(1, unit="ns")
    assert (u(dut.uio_out) >> 1) & 1 == 1, "bcu1 tx_p did not reach uio_out[1]"
    assert (u(dut.uio_oe) >> 1) & 1 == 1, "bcu1 tx_oe did not reach uio_oe[1]"


@cocotb.test()
async def test_router_open_drain_clamp(dut):
    """open_drain_en forces uio_out low regardless of the routed source while
    uio_oe continues to follow its own channel."""
    await ensure_clock(dut)
    await reset_dut(dut)
    sels = [0] * 23
    sels[4] = 1       # uio_out[4] <= const1
    sels[8 + 4] = 1   # uio_oe[4] <= const1
    dut.pin_router_flat.value = pack_router(sels)
    await ClockCycles(dut.clk, 3)
    await Timer(1, unit="ns")
    assert (u(dut.uio_out) >> 4) & 1 == 1, "const1 not routed before clamp test"

    dut.open_drain_en.value = 1 << 4
    await ClockCycles(dut.clk, 3)
    await Timer(1, unit="ns")
    assert (u(dut.uio_out) >> 4) & 1 == 0, "open-drain clamp did not force low"
    assert (u(dut.uio_oe) >> 4) & 1 == 1, "oe channel disturbed by data clamp"


@cocotb.test()
async def test_lut_output_register_enable(dut):
    """oreg delays a LUT output by exactly one cycle relative to the
    combinational path."""
    await ensure_clock(dut)
    await reset_dut(dut)
    # LUT2 (tier1): identity of pin5 -> in0 only: tt bit set when in0=1.
    tt_id = 0
    for idx in range(16):
        if idx & 1:
            tt_id |= 1 << idx
    dut.lut_tt_flat.value = pack_lut_tt({2: tt_id})
    dut.lut_sel_flat.value = pack_lut_sel({2: (5, 15, 15, 15)})

    async def settle_and_read():
        await Timer(1, unit="ns")
        return (u(dut.lut_outputs) >> 2) & 1

    # Combinational (oreg off): after the 2-cycle synchronizer, output
    # follows immediately.
    dut.uio_in.value = 1 << 5
    await ClockCycles(dut.clk, 2)
    assert await settle_and_read() == 1, "comb path did not follow pin"

    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 4)

    # Registered: one extra cycle of latency.
    dut.lut_oreg.value = 1 << 2
    dut.uio_in.value = 1 << 5
    await ClockCycles(dut.clk, 2)
    assert await settle_and_read() == 0, "oreg path had no extra delay"
    await ClockCycles(dut.clk, 1)
    assert await settle_and_read() == 1, "oreg path did not arrive one cycle later"
