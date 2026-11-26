# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Pin-level full-chip harness for the two signature competition demos.

Demo 1 - UART man-in-the-middle fault injection (mitm_uart.hasm):
  upstream UART frames arrive on ui_in[0], are store-and-forwarded through
  core0/BCU0 onto uio0 with a host-programmed fault policy (clean, surgical
  bit flip, byte drop, or silent parity-compensated corruption), while the
  RLE engine captures both the upstream wire (probe bit 8) and the looped-
  back downstream wire (probe bit 0) into the SRAM1 ring.  The test decodes
  the capture and proves the injected corruption appears on the wire at
  exactly the commanded bit position.

Demo 2 - RLE capture/replay (rle_replay.hasm):
  the RLE engine captures a UART stream off ui_in[0]; the host decodes the
  ring in Python, verifies the capture against the driven bitstream at the
  RLE engine's clock granularity, repacks the bits into BCU push words, and
  the firmware streams them back out through the BCU NRZ serializer on uio0.
  An independent UART receiver decodes the replayed stream, and a second
  RLE capture of the replayed wire closes the loop bit-exactly.

Everything interactions with the chip the way the RP2350 host will: firmware
and mailboxes are loaded through bit-banged Host SPI, fabric configuration is
written through CSRs, and protocol traffic is driven/observed only on pins.
"""

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "test") not in sys.path:
    sys.path.insert(0, str(ROOT / "test"))

from tools import hamming_isa as isa  # noqa: E402

# Reuse the entire proven host/fabric interaction layer from U7.
from cocotb_baseline import (  # noqa: E402
    CSR_CTRL_RUN,
    CSR_RLE_BASE,
    CSR_RLE_CTRL,
    CSR_RLE_MASK,
    CSR_RLE_PROBE_MASK,
    SRC_BCU_OE,
    SRC_BCU_P,
    clear_mailbox,
    configure_uart_bcu_pins,
    core_status,
    cycles,
    drive_ui_bit,
    drive_uio_bit,
    load_firmware,
    read_csr,
    read_sram1_bytes,
    read_sram1_word,
    reset_dut,
    uart_tx_line_bit,
    uio_oe_bit,
    uio_out_bit,
    wait_mailbox_status,
    write_bcu_config,
    write_csr,
    write_pin_router,
    write_sram1_bytes,
)

from bfm.uart_bfm import (  # noqa: E402
    UARTReceiver, UARTTransmitter, encode_frame,
)

BIT_CYCLES = 20          # 3 Mbps at 60 MHz, the proven BCU UART operating point
UPSTREAM_PROBE_BIT = 8   # pin_inputs bit 8 = ui_in[0]
DOWNSTREAM_PROBE_BIT = 0  # pin_inputs bit 0 = uio_in[0] (pad loopback)
RLE_RING_BASE = 128      # SRAM1 words 128..255: the 512-byte ring region
RLE_RING_MASK = 0x7F


# ----------------------------------------------------------------------
# Wire loopback: uio0 is the downstream UART wire.  The chip drives it only
# while tx_oe is asserted; while released the wire idles high (positive UART
# idle).  The RLE probe can only see input pins, so the testbench models the
# pad input path with this loopback, matching uart_tx_line_bit exactly.
# ----------------------------------------------------------------------
def _safe_bit(sig, bit):
    """X-tolerant bit read: treat X/Z/uninitialised as 0 (pre-reset startup)."""
    try:
        return (int(sig.value) >> bit) & 1
    except ValueError:
        return 0


async def uart_wire_loopback(dut, bit):
    while True:
        await RisingEdge(dut.clk)
        if _safe_bit(dut.uio_oe, bit):
            drive_uio_bit(dut, bit, _safe_bit(dut.uio_out, bit))
        else:
            drive_uio_bit(dut, bit, 1)


# ----------------------------------------------------------------------
# RLE ring helpers
# ----------------------------------------------------------------------
async def arm_rle_capture(dut, probe_mask):
    await write_csr(dut, CSR_RLE_BASE, RLE_RING_BASE)
    await write_csr(dut, CSR_RLE_MASK, RLE_RING_MASK)
    await write_csr(dut, CSR_RLE_PROBE_MASK, probe_mask)
    await write_csr(dut, CSR_RLE_CTRL, isa.pack_rle_ctrl(mode="capture"))


async def stop_rle_capture(dut):
    await write_csr(dut, CSR_RLE_CTRL, isa.pack_rle_ctrl(mode="off"))


async def read_rle_records(dut):
    """Read exactly the records written since the capture arm.

    ring_wptr resets to rle_base on every capture start, so while the record
    count is below the ring length the CSR write pointer is an exact count.
    """
    wr_ptr = await read_csr(dut, isa.CSR["RLE_WR_PTR"])
    count = wr_ptr - RLE_RING_BASE
    assert 0 < count <= RLE_RING_MASK + 1, \
        f"RLE write pointer {wr_ptr:#x} outside ring base {RLE_RING_BASE:#x}"
    words = [await read_sram1_word(dut, RLE_RING_BASE + i) for i in range(count)]
    # Record format: {delta[15:0], probe[15:0]} in one 32-bit ring word.
    return [((w >> 16) & 0xFFFF, w & 0xFFFF) for w in words]


# ----------------------------------------------------------------------
# Capture decode helpers.  Everything works in the RLE engine's capture-clock
# domain; only relative edge positions are physically meaningful, so all
# comparisons are aligned on the first edge.
# ----------------------------------------------------------------------
def bit_events(records, bit, initial=1):
    """(time, level) change events for one probe bit out of (delta, probe).

    Records are triggered by edges on ANY masked probe bit but carry the
    whole probe word, so a record that was actually caused by the other wire
    must not fabricate an event on this one.  Both demo wires are UART and
    therefore idle high before traffic, so the correct reconstruction is to
    track changes against the idle-high initial level: the first real edge
    of every stream is a falling start edge, phantom-free either way.
    """
    events = []
    t = 0
    last = initial
    for delta, probe in records:
        t += delta
        lvl = (probe >> bit) & 1
        if lvl != last:
            events.append((t, lvl))
            last = lvl
    return events


def level_at(events, t, initial=1):
    lvl = initial
    for te, le in events:
        if te <= t:
            lvl = le
        else:
            break
    return lvl


def decode_uart_stream(events, bit_cycles, nframes):
    """Decode nframes 11-bit UART frames from (time, level) wire events.

    Each frame re-locks on its own falling start edge, exactly like the
    independent UART receiver, so arbitrary inter-frame idle gaps (including
    the dropped-frame gap in MITM drop mode) are tolerated.  Bit centres are
    sampled half a bit cell after the start edge; at 20 cycles/bit the
    capture synchroniser jitter of a few cycles is far inside the +-10-cycle
    centre tolerance.
    """
    frames = []
    pos = 0  # ignore edges before this capture time
    for _ in range(nframes):
        start = None
        prev = 1
        for te, le in events:
            if te < pos:
                prev = le
                continue
            if prev == 1 and le == 0:
                start = te
                break
            prev = le
        if start is None:
            raise AssertionError("capture holds fewer UART frames than expected")
        bits = [level_at(events, start + bit_cycles // 2 + k * bit_cycles)
                for k in range(11)]
        frames.append(bits)
        pos = start + 11 * bit_cycles - bit_cycles // 4
    return frames


def assert_edge_timing(events, stream_bits, bit_cycles, tol, label):
    """Capture-fidelity check at the RLE engine's clock granularity.

    The wire-bit sequence is expanded into ideal transition times on the
    integer bit grid (the BFM drives exact 20-cycle cells with zero drift),
    the capture is aligned on its first edge, and every edge must match in
    count, level, and position within tol cycles.  tol covers the two-flop
    input synchroniser plus capture sampling quantisation; it does not cover
    a bit-time error, so a wrong bit position can never pass vacuously.
    """
    exp = []
    prev = 1  # UART idle before the first frame
    for k, b in enumerate(stream_bits):
        if b != prev:
            exp.append((k * bit_cycles, b))
            prev = b
    assert events, f"{label}: no capture events recorded"
    t0 = events[0][0]
    got = [(t - t0, lvl) for t, lvl in events]
    assert len(got) == len(exp), \
        f"{label}: captured edge count {len(got)} != ideal {len(exp)}"
    for i, ((gt, gl), (et, el)) in enumerate(zip(got, exp)):
        assert gl == el, f"{label}: edge {i} level {gl} != ideal {el}"
        assert abs(gt - et) <= tol, \
            f"{label}: edge {i} at +{gt} cycles vs ideal +{et} (tol {tol})"


def expected_downstream_frames(upstream, mode, target, bitpos):
    """Ideal downstream frame-bit sequences after the MITM fault policy."""
    out = []
    for i, byte in enumerate(upstream):
        if mode == 2 and i == target:
            continue  # dropped frame never reaches the downstream wire
        bits = list(encode_frame(byte))
        if i == target and mode in (1, 3):
            bits[bitpos] ^= 1
            if mode == 3:
                bits[9] ^= 1  # parity compensation keeps the fault silent
        out.append(bits)
    return out


# ----------------------------------------------------------------------
# Demo 1: UART man-in-the-middle fault injection with RLE wire capture
# ----------------------------------------------------------------------
async def _mitm_case(dut, mode, target, bitpos, first_clock):
    await reset_dut(dut, start_clock=first_clock)
    drive_ui_bit(dut, 0, 1)  # upstream UART idle high
    await load_firmware(dut, "mitm_uart.hasm")
    # Same fabric shape as the proven baseline UART BCU path: upstream RX on
    # ui_in[0] (pin input 8) through tier-1 LUT0 into BCU0 rx_p; downstream
    # TX on uio0 data/OE from BCU0 tx_p/tx_oe.
    await configure_uart_bcu_pins(dut, rx_pin=8, tx_uio=0)
    await write_bcu_config(
        dut, 0,
        isa.pack_bcu_mode(codec="nrz", stuff="off"),
        (11 - 1),
        isa.pack_dpll_cfg(mode="hard_snap", win=0, period=BIT_CYCLES),
    )
    await clear_mailbox(dut)
    # Fault-policy mailbox: byte2 mode, byte3 target index, byte4 bit position.
    await write_sram1_bytes(dut, 0, bytes([0, 0, mode, target, bitpos, 0, 0, 0]))

    upstream = bytes([0x96, 0x17, 0xE1, 0x42])

    tx = UARTTransmitter(
        dut.clk, lambda bit: drive_ui_bit(dut, 0, bit),
        bit_cycles=BIT_CYCLES, parity="even", drift=0,
        start_delay_cycles=100)
    rx = UARTReceiver(
        dut.clk, lambda: uart_tx_line_bit(dut, 0),
        bit_cycles=BIT_CYCLES, parity="even", drift=0)

    # Arm the RLE capture of both wires just before releasing the core: probe
    # bit 8 = upstream ui_in[0], probe bit 0 = looped-back downstream uio0.
    await arm_rle_capture(dut, (1 << UPSTREAM_PROBE_BIT) | (1 << DOWNSTREAM_PROBE_BIT))

    n_expected = 3 if mode == 2 else 4
    rx_task = cocotb.start_soon(rx.receive_bytes(n_expected, 30000))
    run_task = cocotb.start_soon(write_csr(dut, CSR_CTRL_RUN, 0x0001))
    await run_task
    # The upstream transmitter starts only after CTRL_RUN is released, the
    # same lead-in contract as the proven baseline full-duplex test.  Frames
    # are contiguous (zero idle gap), which the firmware header documents as
    # the hard-snap RX alignment contract.
    tx_task = cocotb.start_soon(tx.send_bytes(upstream))
    frames = await rx_task
    await tx_task

    # Let the last downstream frame finish, the forwarding status byte land,
    # and the capture FIFO drain into the ring before disarming.
    await cycles(dut, 300)
    await stop_rle_capture(dut)

    # ---- downstream frame-level verdict (independent UART receiver) ----
    exp_frames = expected_downstream_frames(upstream, mode, target, bitpos)
    exp_bytes = []
    for bits in exp_frames:
        exp_bytes.append(sum((bits[1 + i] & 1) << i for i in range(8)))
    evidence_dbg = await read_sram1_bytes(dut, 0x40, 4)
    assert [f.data for f in frames] == exp_bytes, \
        f"mode{mode}: downstream data {[hex(f.data) for f in frames]} != " \
        f"{[hex(b) for b in exp_bytes]}; evidence={evidence_dbg.hex()}"
    assert not any(f.framing_error for f in frames), \
        f"mode{mode}: downstream framing error in injected stream"
    if mode == 1:
        # Data bit flipped but parity bit untouched: the downstream parity
        # check must fire on exactly the target frame - the visible fault.
        assert frames[target].parity_error, \
            "bit-flip fault did not produce the expected downstream parity error"
        others = [f for i, f in enumerate(frames) if i != target]
        assert not any(f.parity_error for f in others), \
            "bit-flip fault leaked into non-target frames"
    elif mode == 3:
        # Parity-compensated corruption: no downstream validator fires even
        # though the payload is wrong.  Only the wire capture exposes it.
        assert not any(f.parity_error or f.framing_error for f in frames), \
            "silent corruption was unexpectedly visible to downstream checks"
    else:
        assert not any(f.parity_error for f in frames), \
            f"mode{mode}: unexpected downstream parity error"

    # ---- firmware-level verdict: status, evidence mailbox, deadlines ----
    status = await wait_mailbox_status(dut, 1, 10)
    assert status == 1, f"mode{mode}: MITM mailbox status 0x{status:02X}"
    evidence = await read_sram1_bytes(dut, 0x40, 4)
    for i in range(4):
        if mode == 2 and i == target:
            continue  # dropped frames leave no evidence byte
        exp_ev = upstream[i]
        if i == target and mode in (1, 3) and 1 <= bitpos <= 8:
            exp_ev ^= 1 << (bitpos - 1)
        assert evidence[i] == exp_ev, \
            f"mode{mode}: evidence byte {i} 0x{evidence[i]:02X} != 0x{exp_ev:02X}"
    st0 = await core_status(dut, 0)
    assert (st0 >> 3) & 1 == 0, f"mode{mode}: MITM core0 deadline_missed {st0:#x}"

    # ---- the money shot: RLE capture decode of both wires ----
    records = await read_rle_records(dut)
    up_events = bit_events(records, UPSTREAM_PROBE_BIT)
    down_events = bit_events(records, DOWNSTREAM_PROBE_BIT)

    # Upstream capture decodes to exactly the bytes the BFM drove.
    up_decoded = decode_uart_stream(up_events, BIT_CYCLES, 4)
    up_ideal = [list(encode_frame(b)) for b in upstream]
    assert up_decoded == up_ideal, \
        f"mode{mode}: upstream capture decode mismatch " \
        f"{up_decoded} vs {up_ideal}"
    up_stream = [b for frame in up_ideal for b in frame]
    assert_edge_timing(up_events, up_stream, BIT_CYCLES, 4,
                       f"mode{mode} upstream capture")

    # Downstream capture decodes to exactly the corrupted stream the fault
    # policy commands - the injected bit position appears in the capture.
    down_decoded = decode_uart_stream(down_events, BIT_CYCLES, n_expected)
    assert down_decoded == exp_frames, \
        f"mode{mode}: downstream capture decode mismatch " \
        f"{down_decoded} vs {exp_frames}"
    if mode != 2:
        # Drop mode inserts a pipeline idle gap, so a single fixed bit grid
        # does not apply; contiguous modes are additionally edge-verified.
        down_stream = [b for frame in exp_frames for b in frame]
        assert_edge_timing(down_events, down_stream, BIT_CYCLES, 4,
                           f"mode{mode} downstream capture")
    dut._log.info(
        "MITM mode%d target%d bitpos%d: %d RLE records, capture decode exact",
        mode, target, bitpos, len(records))


@cocotb.test()
async def test_mitm_uart_fault_injection(dut):
    """UART MITM: clean pass-through, surgical bit flip, byte drop, and
    silent parity-compensated corruption - each verdict read back out of the
    RLE wire capture, not just the downstream receiver."""
    cocotb.start_soon(uart_wire_loopback(dut, 0))
    await _mitm_case(dut, mode=0, target=2, bitpos=0, first_clock=True)
    await _mitm_case(dut, mode=1, target=2, bitpos=5, first_clock=False)
    await _mitm_case(dut, mode=2, target=1, bitpos=0, first_clock=False)
    await _mitm_case(dut, mode=3, target=1, bitpos=2, first_clock=False)


async def _diag_mitm_monitor(dut):
    """Cycle-level MITM probe: RX completes/pops, TX pushes/emissions, DPLL."""
    from cocotb.triggers import FallingEdge
    bcu = dut.gen_core[0].u_bcu
    core = dut.gen_core[0].u_core
    dpll = bcu.u_dpll
    prev_full = 0
    prev_wire = 1
    prev_txp = 1
    while True:
        await FallingEdge(dut.clk)
        t = int(dut.timer.value)
        wire = _safe_bit(bcu.rx_p, 0)
        txp = _safe_bit(bcu.tx_p, 0) if _safe_bit(bcu.tx_oe, 0) else 1
        if wire != prev_wire:
            print(f"DIAG t={t} RXWIRE {prev_wire}->{wire} dpll_cnt={int(dpll.cnt.value)}")
            prev_wire = wire
        if txp != prev_txp:
            print(f"DIAG t={t} TXWIRE {prev_txp}->{txp} oe={_safe_bit(bcu.tx_oe,0)} sym={int(bcu.tx_sym_cnt.value)} left={int(bcu.tx_bits_left.value)}")
            prev_txp = txp
        if _safe_bit(bcu.rx_tick, 0):
            print(f"DIAG t={t} RXTICK lvl={wire} rx_cnt={int(bcu.rx_cnt.value)} shift={int(bcu.rx_shift.value):04x} dcnt={int(dpll.cnt.value)}")
        if _safe_bit(bcu.wire_edge, 0):
            print(f"DIAG t={t} EDGE accept={int(dpll.accept_edge.value)} dcnt={int(dpll.cnt.value)} sample={int(dpll.sample.value)}")
        if prev_full == 0 and _safe_bit(bcu.rx_full, 0):
            print(f"DIAG t={t} RXFULL shadow={int(bcu.rx_shadow.value):04x} shift={int(bcu.rx_shift.value):04x}")
        if _safe_bit(bcu.rx_pop, 0):
            print(f"DIAG t={t} RXPOP rxdata={int(bcu.rx_data.value):04x} r0={int(core.rf0.value):04x} pc={int(core.pc.value):03x}")
        if _safe_bit(bcu.tx_push, 0):
            print(f"DIAG t={t} TXPUSH data={int(bcu.tx_data.value):04x} txs={int(bcu.txs.value)} shadow_v={int(bcu.tx_shadow_v.value)} active={int(bcu.tx_active.value)} sym={int(bcu.tx_sym_cnt.value)} left={int(bcu.tx_bits_left.value)} r0={int(core.rf0.value):04x} pc={int(core.pc.value):03x}")
        prev_full = _safe_bit(bcu.rx_full, 0)


@cocotb.test()
async def diag_mitm_probe(dut):
    """Temporary MITM internal probe for byte-4 corruption bring-up."""
    cocotb.start_soon(_diag_mitm_monitor(dut))
    cocotb.start_soon(uart_wire_loopback(dut, 0))
    try:
        await _mitm_case(dut, mode=0, target=2, bitpos=0, first_clock=True)
    finally:
        print("DIAG evidence", (await read_sram1_bytes(dut, 0x40, 4)).hex())
        print("DIAG mailbox0", hex(await read_sram1_word(dut, 0)))


# ----------------------------------------------------------------------
# Demo 2: RLE capture of a UART stream, host decode, BCU/firmware replay
# ----------------------------------------------------------------------
@cocotb.test()
async def test_rle_capture_replay(dut):
    """Capture a UART wire into the SRAM1 ring, reconstruct it in Python,
    verify capture fidelity at the RLE clock granularity, then replay it
    through firmware/BCU and decode the regenerated stream - closing the
    loop with a second capture of the replayed wire."""
    await reset_dut(dut, start_clock=True)
    drive_ui_bit(dut, 0, 1)  # source wire idle high before the stream
    await load_firmware(dut, "rle_replay.hasm")
    # Replay path fabric: BCU0 TX on uio0.  No LUT is needed: capture probes
    # the synchronized pin inputs directly and the BCU RX engine is unused.
    await write_pin_router(dut, {
        0: SRC_BCU_P,      # uio0 data <- BCU0 tx_p
        8: SRC_BCU_OE,     # uio0 oe   <- BCU0 tx_oe
    })
    # len16 LSB-first NRZ at the 20-cycle symbol clock: one push = 16 wire bits.
    await write_bcu_config(
        dut, 0,
        isa.pack_bcu_mode(codec="nrz", stuff="off"),
        (16 - 1),
        isa.pack_dpll_cfg(mode="none", win=0, period=BIT_CYCLES),
    )
    await clear_mailbox(dut)

    original = bytes([0x96, 0x17, 0xE1, 0x42])
    ideal_frames = [list(encode_frame(b)) for b in original]
    ideal_stream = [b for frame in ideal_frames for b in frame]

    # ---- phase 1: capture the externally driven stream (hardware only) ----
    await arm_rle_capture(dut, 1 << UPSTREAM_PROBE_BIT)
    tx = UARTTransmitter(
        dut.clk, lambda bit: drive_ui_bit(dut, 0, bit),
        bit_cycles=BIT_CYCLES, parity="even", drift=0,
        start_delay_cycles=100)
    await tx.send_bytes(original)
    # Margin for the final stop bit plus capture FIFO drain into the ring.
    await cycles(dut, 200)
    await stop_rle_capture(dut)

    # ---- phase 2: host decode and capture-fidelity verification ----
    records = await read_rle_records(dut)
    up_events = bit_events(records, UPSTREAM_PROBE_BIT)
    cap_frames = decode_uart_stream(up_events, BIT_CYCLES, 4)
    assert cap_frames == ideal_frames, \
        f"capture decode mismatch {cap_frames} vs {ideal_frames}"
    assert_edge_timing(up_events, ideal_stream, BIT_CYCLES, 4,
                       "original capture")
    dut._log.info("RLE capture: %d records decode bit-exactly", len(records))

    # ---- phase 3: repack the bitstream into BCU push words ----
    stream = list(cap_frames[0])
    for frame in cap_frames[1:]:
        stream.extend(frame)
    while len(stream) % 16:
        stream.append(1)  # pad with UART idle level
    npushes = len(stream) // 16
    pair_bytes = bytearray()
    for p in range(npushes):
        lo = sum(stream[16 * p + j] << j for j in range(8))
        hi = sum(stream[16 * p + 8 + j] << j for j in range(8))
        pair_bytes.extend([lo, hi])
    await write_sram1_bytes(dut, 0, bytes([0, 0, npushes, 0]))
    pad = (-len(pair_bytes)) % 4
    await write_sram1_bytes(dut, 0x40, bytes(pair_bytes) + bytes(pad))

    # ---- phase 4: firmware streams the buffer through the BCU ----
    cocotb.start_soon(uart_wire_loopback(dut, 0))
    rx = UARTReceiver(
        dut.clk, lambda: uart_tx_line_bit(dut, 0),
        bit_cycles=BIT_CYCLES, parity="even", drift=0)
    # Re-arm the capture on the replayed wire (probe bit 0).  The ring is
    # reused; the original records were already extracted above, and arming
    # resets the write pointer to the ring base.
    await write_csr(dut, CSR_RLE_PROBE_MASK, 1 << DOWNSTREAM_PROBE_BIT)
    await write_csr(dut, CSR_RLE_CTRL, isa.pack_rle_ctrl(mode="capture"))
    rx_task = cocotb.start_soon(rx.receive_bytes(4, 20000))
    await write_csr(dut, CSR_CTRL_RUN, 0x0001)
    frames = await rx_task

    # ---- replay verdict 1: independent UART receiver decodes the stream ----
    assert [f.data for f in frames] == list(original), \
        f"replayed data {[hex(f.data) for f in frames]} != " \
        f"{[hex(b) for b in original]}"
    assert not any(f.framing_error or f.parity_error for f in frames), \
        "replayed stream failed framing/parity validation"

    status = await wait_mailbox_status(dut, 1, 10)
    assert status == 1, f"replay mailbox status 0x{status:02X}"
    st0 = await core_status(dut, 0)
    assert (st0 >> 3) & 1 == 0, f"replay core0 deadline_missed {st0:#x}"

    # ---- replay verdict 2: capture the replayed wire and close the loop ----
    await cycles(dut, 200)
    await stop_rle_capture(dut)
    records2 = await read_rle_records(dut)
    down_events = bit_events(records2, DOWNSTREAM_PROBE_BIT)
    re_frames = decode_uart_stream(down_events, BIT_CYCLES, 4)
    assert re_frames == ideal_frames, \
        f"replayed-wire capture decode mismatch {re_frames} vs {ideal_frames}"
    assert_edge_timing(down_events, ideal_stream, BIT_CYCLES, 4,
                       "replayed wire capture")
    dut._log.info(
        "RLE replay: %d push words streamed; recapture (%d records) bit-exact",
        npushes, len(records2))
