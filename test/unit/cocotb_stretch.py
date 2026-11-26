# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Pin-level full-chip harness for the U8 stretch-protocol matrix.

Reuses the proven U7 host interaction layer (bit-banged Host SPI loading,
CSR fabric configuration, mailbox polling) and adds the independent
stretch-protocol bus models.  Protocol traffic is driven and observed only
on ui/uio pins, exactly as the RP2350 host would.
"""

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "test") not in sys.path:
    sys.path.insert(0, str(ROOT / "test"))

from tools import hamming_isa as isa  # noqa: E402

# Reuse the entire proven host/fabric interaction layer from U7.
from cocotb_baseline import (  # noqa: E402
    CSR_CTRL_RUN,
    SRC_CORE,
    SRC_BCU_P,
    SRC_BCU_N,
    SRC_BCU_OE,
    clear_mailbox,
    configure_waveform_pin,  # noqa: F401  (re-exported for MITM use)
    core_status,
    cycles,
    drive_ui_bit,
    drive_uio_bit,
    load_firmware,
    lut_tt_identity_in0,
    read_csr,
    read_sram1_bytes,
    reset_dut,
    ui_bit,
    uio_oe_bit,
    uio_out_bit,
    uo_bit,
    wait_mailbox_status,
    write_csr,
    write_lut,
    write_pin_router,
    write_sram1_bytes,
    write_ui_shadow,
    write_uio_shadow,
    u,
)

from bfm.can_bfm import (  # noqa: E402
    CANBus, CANNodeBfm, CANProtocolError, encode_frame_bits,
)
from bfm.eth_bfm import (  # noqa: E402
    Eth10BaseTNode, EthWire, EthernetProtocolError, mutate_fcs_bits,
)
from bfm.usb_bfm import (  # noqa: E402
    PID_ACK, PID_DATA0, PID_IN, USBBus, USBDeviceReceiver, USBHostBfm,
    token_packet_bytes, wire_levels_for_packet,
)


# ----------------------------------------------------------------------
# 10BASE-T fabric configuration.  Core0/BCU0 transmits on the dedicated
# output pins.  Both TX pins must be paired with BCU0: router channels 16
# (uo0) and 20 (uo4) both have channel-index low bits 0.  Core1/BCU1 receives
# the external comparator pair through LUT2/LUT3.
# ----------------------------------------------------------------------
ETH_TXP_PIN = 0     # uo_out[0], router channel 16
ETH_TXN_PIN = 4     # uo_out[4], router channel 20
ETH_RXP_PIN = 0     # ui_in[0], tier-1 pin input 8
ETH_RXN_PIN = 1     # ui_in[1], tier-1 pin input 9


async def configure_eth_pins(dut, invert_rx=False):
    await write_pin_router(dut, {
        16: SRC_BCU_P,      # uo0 <- BCU0 tx_p
        20: SRC_BCU_N,      # uo4 <- BCU0 tx_n
    })
    tt = 0x5555 if invert_rx else lut_tt_identity_in0()
    # Swapping the pair is the inversion that corrects the sampled
    # half-cell pairing, while crucially preserving idle as (0,0).
    await write_lut(dut, 2, tt, (9, 15, 15, 15))
    await write_lut(dut, 3, tt, (8, 15, 15, 15))


def make_eth_wire(dut):
    wire = EthWire(
        dut.clk,
        lambda level: drive_ui_bit(dut, ETH_RXP_PIN, level),
        lambda level: drive_ui_bit(dut, ETH_RXN_PIN, level),
        lambda: uo_bit(dut, ETH_TXP_PIN),
        lambda: uo_bit(dut, ETH_TXN_PIN),
    )
    wire.start_monitor()
    return wire


def eth_frame60(tag=0):
    """Deterministic minimum-size frame: broadcast dst, locally administered
    source, length field 46, then 46 pad bytes.  The length field is a valid
    IEEE 802.3 length rather than an EtherType."""
    frame = bytearray(bytes.fromhex("ffffffffffff"))
    frame.extend((0x02, 0x00, 0x5e, 0x10, tag & 0xFF, 0x01))
    frame.extend((46).to_bytes(2, "big"))
    frame.extend(((tag + i) & 0xFF) for i in range(46))
    assert len(frame) == 60
    return bytes(frame)


def eth_mailbox_header(frame_len, repeat=0):
    return bytes([1, 0, frame_len & 0xFF, (frame_len >> 8) & 0xFF,
                  0, 0, 0, repeat & 0xFF])


async def queue_eth_tx(dut, frame, repeat=0):
    """Write the TX mailbox/frame through Host SPI and raise EV1."""
    await write_sram1_bytes(dut, 0, eth_mailbox_header(len(frame), repeat))
    # The frame base 0x10 is word aligned; write the whole 60-byte minimum or
    # rounded payload in as few burst transactions as the aligned helper allows.
    data = bytes(frame)
    pad = (-len(data)) % 4
    await write_sram1_bytes(dut, 0x10, data + bytes(pad))
    await write_csr(dut, isa.CSR["EV_SET"], 0x0002)


async def eth_wait_tx_status(dut, timeout_writes=30):
    for _ in range(timeout_writes):
        status = (await read_sram1_bytes(dut, 0, 4))[1]
        if status & 1:
            return status
    raise AssertionError("Ethernet TX done status never appeared")


async def assert_eth_core_clean(dut):
    for core in (0, 1):
        st = await core_status(dut, core)
        assert (st >> 3) & 1 == 0, \
            f"Ethernet core{core} deadline_missed status {st:#x}"


async def _eth_tx_case(dut, frame, first_clock=True):
    """DUT transmits one frame; the independent BFM validates every layer."""
    await reset_dut(dut, start_clock=first_clock)
    prog = await load_firmware(dut, "eth_10baset.hasm")
    await configure_eth_pins(dut)
    await clear_mailbox(dut)
    wire = make_eth_wire(dut)
    node = Eth10BaseTNode(wire)
    capture_task = cocotb.start_soon(node.capture_frame(timeout_cycles=180000))
    await queue_eth_tx(dut, frame)
    await write_csr(dut, CSR_CTRL_RUN, 0x0003)
    result = await capture_task
    assert not wire.errors, f"Ethernet electrical violations: {wire.errors}"
    assert not result.halfcell_violations, \
        f"Manchester half-cell violations: {result.halfcell_violations[:3]}"
    assert result.frame == bytes(frame), \
        f"Ethernet TX payload mismatch: {result.frame.hex()} vs {bytes(frame).hex()}"
    assert result.residue == 0xDEBB20E3
    assert 15 <= result.tp_idl_cycles <= 18, \
        f"TP_IDL pulse {result.tp_idl_cycles} cycles outside 15..18"
    status = await eth_wait_tx_status(dut)
    assert status & 1, f"Ethernet TX status 0x{status:02x}"
    await assert_eth_core_clean(dut)
    return result, prog


@cocotb.test()
async def test_eth10baset_tx_frame(dut):
    """10BASE-T TX: minimum-size frame, Manchester/SFD/FCS/TP_IDL checked."""
    await _eth_tx_case(dut, eth_frame60(0x21), first_clock=True)


async def eth_wait_status_mask(dut, mask, timeout_writes=240):
    for _ in range(timeout_writes):
        status = (await read_sram1_bytes(dut, 0, 4))[1]
        if status & mask:
            return status
    # Failure diagnostics are deliberately host-observable architectural
    # state, not waveform-only guessing: core PC, BCU mode/DPLL/status/live
    # CRC, and the leading capture bytes.
    pc = await read_csr(dut, isa.CSR_CORE_BANK + 0x10 + 1)
    mode = await read_csr(dut, isa.CSR_CORE_BANK + 0x10 + 4)
    dpll = await read_csr(dut, isa.CSR_CORE_BANK + 0x10 + 12)
    stat = await read_csr(dut, isa.CSR_CORE_BANK + 0x10 + 15)
    crclo = await read_csr(dut, isa.CSR_CORE_BANK + 0x10 + 7)
    crchi = await read_csr(dut, isa.CSR_CORE_BANK + 0x10 + 8)
    cap = await read_sram1_bytes(dut, 0x08, 80)
    dut._log.info(
        "ETH RX diag pc=%03x mode=%04x dpll=%04x stat=%04x crc=%04x%04x cap=%s",
        pc, mode, dpll, stat, crchi, crclo, cap.hex())
    raise AssertionError(f"Ethernet status mask 0x{mask:02x} never appeared")


async def _eth_rx_start(dut):
    await reset_dut(dut, start_clock=True)
    await load_firmware(dut, "eth_10baset.hasm")
    await configure_eth_pins(dut)
    await clear_mailbox(dut)
    wire = make_eth_wire(dut)
    await write_csr(dut, CSR_CTRL_RUN, 0x0003)
    return wire, Eth10BaseTNode(wire)


async def _eth_rx_case(dut, frame, jitter_pattern=None):
    wire, node = await _eth_rx_start(dut)
    await node.send_frame(frame, jitter_pattern=jitter_pattern)
    status = await eth_wait_status_mask(dut, 0x02)
    assert status & 0x02, f"Ethernet RX valid missing, status 0x{status:02x}"
    assert not (status & 0x70), f"Ethernet RX error status 0x{status:02x}"
    hdr = await read_sram1_bytes(dut, 0, 8)
    rx_len = hdr[4] | (hdr[5] << 8)
    assert rx_len == len(frame), f"RX length {rx_len}, expected {len(frame)}"
    pad = (-len(frame)) % 4
    got = (await read_sram1_bytes(dut, 0x10, len(frame) + pad))[:len(frame)]
    assert got == bytes(frame), \
        f"RX frame mismatch: {got.hex()} vs {bytes(frame).hex()}"
    await assert_eth_core_clean(dut)
    return status


@cocotb.test()
async def test_eth10baset_tx_repeat_ifg(dut):
    """Back-to-back TX repeat: both frames valid and inter-frame gap >= 9.6 us."""
    frame = eth_frame60(0x42)
    await reset_dut(dut, start_clock=True)
    await load_firmware(dut, "eth_10baset.hasm")
    await configure_eth_pins(dut)
    await clear_mailbox(dut)
    wire = make_eth_wire(dut)
    node = Eth10BaseTNode(wire)
    first_task = cocotb.start_soon(node.capture_frame(timeout_cycles=180000))
    await queue_eth_tx(dut, frame, repeat=1)
    await write_csr(dut, CSR_CTRL_RUN, 0x0003)
    first = await first_task
    second = await node.capture_frame(timeout_cycles=20000)
    assert first.frame == frame and second.frame == frame
    ifg = second.start_cycle - first.idle_start_cycle
    assert ifg >= 576, f"IFG {ifg} cycles below 576-cycle minimum"
    assert 15 <= first.tp_idl_cycles <= 18
    assert 15 <= second.tp_idl_cycles <= 18
    status = await eth_wait_tx_status(dut)
    assert status & 1
    await assert_eth_core_clean(dut)
    dut._log.info("ETH TX measured IFG cycles=%d", ifg)


@cocotb.test()
async def test_eth10baset_rx_frame(dut):
    """10BASE-T RX: clean independent peer frame, exact payload/length."""
    await _eth_rx_case(dut, eth_frame60(0x34))


@cocotb.test()
async def test_eth10baset_rx_jitter(dut):
    """10BASE-T RX with +/-1 cycle bit-center jitter inside the DPLL window."""
    await _eth_rx_case(dut, eth_frame60(0x35), jitter_pattern=[-1, 1, 0, 1, -1, 0])


@cocotb.test()
async def test_eth10baset_rx_bad_fcs(dut):
    """A one-bit FCS mutation must never set RX-valid (non-vacuity)."""
    frame = eth_frame60(0x36)
    wire, node = await _eth_rx_start(dut)
    await node.send_frame(frame, fcs_mutate=0)
    status = await eth_wait_status_mask(dut, 0x72)
    assert status & 0x20, f"corrupted FCS was not rejected: status 0x{status:02x}"
    assert not (status & 0x02), f"corrupted FCS falsely set RX valid: 0x{status:02x}"
    await assert_eth_core_clean(dut)


# ----------------------------------------------------------------------
# CAN fabric configuration: core3 bit-bangs uio3 (data+oe, open-drain).
# ----------------------------------------------------------------------
async def configure_can_pins(dut):
    await write_pin_router(dut, {
        3: SRC_CORE,        # uio3 data <- core3 out bit0
        8 + 3: SRC_CORE,    # uio3 oe   <- core3 oe  bit0
    })
    # BCU3 rx_p is LUT6 (2*core).  Feed it the synchronized uio3 wire level.
    await write_lut(dut, 6, lut_tt_identity_in0(), (3, 15, 15, 15))


def make_can_bus(dut):
    """Wire the CAN wired-AND bus model onto uio3."""
    bus = CANBus(
        dut.clk,
        lambda level: drive_uio_bit(dut, 3, level),
        lambda: uio_out_bit(dut, 3),
        lambda: uio_oe_bit(dut, 3),
    )
    bus.start_resolver()
    return bus


async def _can_tx_case(dut, identifier, data, first_clock):
    """DUT transmits one CAN 2.0A frame; independent node validates+ACKs."""
    await reset_dut(dut, start_clock=first_clock)
    await load_firmware(dut, "can20a.hasm")
    await configure_can_pins(dut)
    await clear_mailbox(dut)

    idlo = identifier & 0xFF
    idhi = (identifier >> 8) & 0x7
    header = bytes([0, 0, 1, len(data), idlo, idhi, 0, 0])
    await write_sram1_bytes(dut, 0, header)
    payload = bytes(data) + bytes(4 - len(data) if len(data) < 4 else 0)
    if len(data) >= 4:
        await write_sram1_bytes(dut, 8, bytes(data[:4]))
        if len(data) > 4:
            rest = bytes(data[4:]) + bytes(8 - len(data))
            await write_sram1_bytes(dut, 12, rest[:4])
    else:
        await write_sram1_bytes(dut, 8, bytes(data) + bytes(4 - len(data)))

    bus = make_can_bus(dut)
    node = CANNodeBfm(bus, owner="bfm", bit_cycles=60, sample_point=0.7)
    rx_task = cocotb.start_soon(node.receive_frame(timeout_cycles=40000))
    run_task = cocotb.start_soon(write_csr(dut, CSR_CTRL_RUN, 0x0008))
    await run_task
    frame = await rx_task

    assert frame.identifier == identifier, \
        f"CAN id mismatch: wire 0x{frame.identifier:03x} vs 0x{identifier:03x}"
    assert frame.data == bytes(data), \
        f"CAN data mismatch: wire {frame.data.hex()} vs {bytes(data).hex()}"
    assert frame.acked, "independent node did not validate/ACK the frame"
    assert not bus.contention_errors, \
        f"CAN electrical violations: {bus.contention_errors}"

    status = await wait_mailbox_status(dut, 1, 10)
    assert status == 1, f"CAN TX mailbox status 0x{status:02X} (not ACKed?)"
    st3 = await core_status(dut, 3)
    assert (st3 >> 3) & 1 == 0, f"CAN core3 deadline_missed status {st3:#x}"


@cocotb.test()
async def test_can20a_tx_frame(dut):
    """CAN 2.0A TX: id 0x123, payload A5 3C, independently validated+ACKed."""
    await _can_tx_case(dut, 0x123, bytes([0xA5, 0x3C]), first_clock=True)


async def _can_rx_case(dut, identifier, data, first_clock):
    """Independent node transmits; DUT receives, validates, ACKs, unpacks."""
    await reset_dut(dut, start_clock=first_clock)
    await load_firmware(dut, "can20a.hasm")
    await configure_can_pins(dut)
    await clear_mailbox(dut)
    # role byte 2 = receive one frame
    await write_sram1_bytes(dut, 0, bytes([0, 0, 2, 0]))

    bus = make_can_bus(dut)
    node = CANNodeBfm(bus, owner="bfm", bit_cycles=60, sample_point=0.7)
    run_task = cocotb.start_soon(write_csr(dut, CSR_CTRL_RUN, 0x000C))
    await run_task
    # The CTRL_RUN commit reaches the cores near the end of its Host SPI
    # transaction.  Give the short BCU-configuration prologue a small,
    # deterministic margin before the peer starts SOF.
    await cycles(dut, 120)
    acked, lost, crc = await node.send_frame(identifier, bytes(data))

    assert acked, "DUT did not produce an in-slot dominant ACK"
    assert not lost, "independent node unexpectedly lost arbitration to a pure receiver"
    status = await wait_mailbox_status(dut, 1, 20)
    assert status == 1, f"CAN RX mailbox status 0x{status:02X}"

    ident_bytes = await read_sram1_bytes(dut, 0x30, 4)
    got_id = ident_bytes[0] | (ident_bytes[1] << 8)
    got_dlc = ident_bytes[2]
    assert got_id == identifier, \
        f"CAN RX id mismatch: mailbox 0x{got_id:03x} vs 0x{identifier:03x}"
    assert got_dlc == len(data), \
        f"CAN RX DLC mismatch: mailbox {got_dlc} vs {len(data)}"
    payload = await read_sram1_bytes(dut, 0x40, max(4, len(data)))
    assert payload[:len(data)] == bytes(data), \
        f"CAN RX payload mismatch: {payload[:len(data)].hex()} vs {bytes(data).hex()}"
    assert not bus.contention_errors, \
        f"CAN electrical violations: {bus.contention_errors}"
    st3 = await core_status(dut, 3)
    assert (st3 >> 3) & 1 == 0, f"CAN RX core3 deadline_missed status {st3:#x}"


@cocotb.test()
async def test_can20a_rx_frame(dut):
    """CAN 2.0A RX: id 0x155, payload 5A C3 0F, CRC-validated and ACKed."""
    await _can_rx_case(dut, 0x155, bytes([0x5A, 0xC3, 0x0F]), first_clock=True)




def mutate_crc_wire(identifier, data, crc_bit_index=0):
    """Flip one logical CRC bit after stuffing, preserving trailer length."""
    bits, crc, stuffs = encode_frame_bits(identifier, bytes(data))
    stuffset = set(stuffs)
    total_logical = 19 + 8 * len(data) + 15
    target = (total_logical - 14) + crc_bit_index
    logical = 0
    out = list(bits)
    for pos, bit in enumerate(bits):
        if pos in stuffset:
            continue
        logical += 1
        if logical == target:
            out[pos] ^= 1
            return out
    raise AssertionError("CRC mutation target not found")


def mutate_first_stuff_wire(identifier, data):
    """Break the first inserted stuff bit so six equal bits reach the wire."""
    bits, crc, stuffs = encode_frame_bits(identifier, bytes(data))
    assert stuffs, "test vector has no stuff bit to mutate"
    out = list(bits)
    pos = stuffs[0]
    out[pos] = out[pos - 1]
    return out


async def _can_rx_negative_case(dut, identifier, data, wire_bits,
                                expected_status, first_clock):
    """A mutated independent frame must not be ACKed and must be classified."""
    await reset_dut(dut, start_clock=first_clock)
    await load_firmware(dut, "can20a.hasm")
    await configure_can_pins(dut)
    await clear_mailbox(dut)
    await write_sram1_bytes(dut, 0, bytes([0, 0, 2, 0]))
    bus = make_can_bus(dut)
    node = CANNodeBfm(bus, owner="bfm", bit_cycles=60, sample_point=0.7)
    await write_csr(dut, CSR_CTRL_RUN, 0x000C)
    await cycles(dut, 120)
    acked, lost, _ = await node.send_frame(
        identifier, bytes(data), wire_bits_override=wire_bits)
    assert not acked, "DUT ACKed a mutated CAN frame"
    assert not lost
    status = await wait_mailbox_status(dut, 0x70, 20)
    assert status == expected_status,         f"negative RX status 0x{status:02X}, expected 0x{expected_status:02X}"
    assert not bus.contention_errors, bus.contention_errors
    st3 = await core_status(dut, 3)
    assert (st3 >> 3) & 1 == 0, f"negative RX deadline miss {st3:#x}"


@cocotb.test()
async def test_can20a_rx_rejects_bad_crc(dut):
    """CRC mutation is not ACKed; mailbox status is 0x20."""
    identifier, data = 0x155, bytes([0x5A, 0xC3, 0x0F])
    bad = mutate_crc_wire(identifier, data, 0)
    await _can_rx_negative_case(dut, identifier, data, bad, 0x20, True)


@cocotb.test()
async def test_can20a_rx_detects_stuff_error(dut):
    """A sixth consecutive equal bit is not ACKed; status is 0x10."""
    identifier, data = 0x155, bytes([0x5A, 0xC3, 0x0F])
    bad = mutate_first_stuff_wire(identifier, data)
    await _can_rx_negative_case(dut, identifier, data, bad, 0x10, True)


@cocotb.test()
async def diag_can_rx_probe(dut):
    """Diagnostic RX capture dump: pair bytes, PC, BCU status/config."""
    await reset_dut(dut, start_clock=True)
    await load_firmware(dut, "can20a.hasm")
    await configure_can_pins(dut)
    await clear_mailbox(dut)
    await write_sram1_bytes(dut, 0, bytes([0, 0, 2, 0]))
    bus = make_can_bus(dut)
    node = CANNodeBfm(bus, owner="bfm", bit_cycles=60, sample_point=0.7)
    await write_csr(dut, CSR_CTRL_RUN, 0x000C)
    await cycles(dut, 120)
    acked, lost, crc = await node.send_frame(0x155, bytes([0x5A, 0xC3, 0x0F]))
    await cycles(dut, 5000)
    print(f"DIAGRX acked={acked} lost={lost} crc={crc:#x} contention={bus.contention_errors}")
    print("DIAGRX bytes00-3f=", (await read_sram1_bytes(dut, 0, 64)).hex())
    print("DIAGRX payload40-4f=", (await read_sram1_bytes(dut, 0x40, 16)).hex())
    print("DIAGRX top handles=", sorted(getattr(dut, "_sub_handles", {}).keys()))
    for hn in sorted(getattr(dut, "_sub_handles", {}).keys()):
        if "gen_core" in hn:
            h = dut._sub_handles[hn]
            print("DIAGRX core handle", hn, sorted(getattr(h, "_sub_handles", {}).keys()))
    st3 = await core_status(dut, 3)
    pc3 = await read_csr(dut, isa.CSR_CORE_BANK + 3 * 16 + isa.CORE_CSR["CORE_PC"])
    bstat = await read_csr(dut, isa.CSR_CORE_BANK + 3 * 16 + isa.CORE_CSR["BCU_STAT_SNAP"])
    mode = await read_csr(dut, isa.CSR_CORE_BANK + 3 * 16 + isa.CORE_CSR["BCU_MODE"])
    bitlen = await read_csr(dut, isa.CSR_CORE_BANK + 3 * 16 + isa.CORE_CSR["BCU_BIT_LEN"])
    dpll = await read_csr(dut, isa.CSR_CORE_BANK + 3 * 16 + isa.CORE_CSR["BCU_DPLL_CFG"])
    print(f"DIAGRX st3={st3:#x} pc3={pc3} bstat={bstat:#x} mode={mode:#x} bitlen={bitlen:#x} dpll={dpll:#x}")


@cocotb.test()
async def diag_can_tx_probe(dut):
    """Diagnostic: does the firmware complete and wiggle uio3 oe at all?"""
    await reset_dut(dut, start_clock=True)
    await load_firmware(dut, "can20a.hasm")
    await configure_can_pins(dut)
    await clear_mailbox(dut)
    header = bytes([0, 0, 1, 2, 0x23, 0x01, 0, 0])
    await write_sram1_bytes(dut, 0, header)
    await write_sram1_bytes(dut, 8, bytes([0xA5, 0x3C, 0, 0]))

    # count oe3 rising edges for 20000 cycles after CTRL_RUN
    edges = [0]
    hi_cycles = [0]

    async def watch():
        prev = 0
        for _ in range(20000):
            await cycles(dut, 1)
            cur = uio_oe_bit(dut, 3)
            if cur and not prev:
                edges[0] += 1
            if cur:
                hi_cycles[0] += 1
            prev = cur

    watch_task = cocotb.start_soon(watch())
    await write_csr(dut, CSR_CTRL_RUN, 0x0008)
    await watch_task
    print(f"DIAG: oe3 rising edges={edges[0]} hi_cycles={hi_cycles[0]}")
    word0 = await read_sram1_bytes(dut, 0, 4)
    print(f"DIAG: mailbox bytes 0..3 = {word0.hex()}")
    buf = await read_sram1_bytes(dut, 0x80, 16)
    print(f"DIAG: buffer 0x80..0x8f = {buf.hex()}")
    st3 = await core_status(dut, 3)
    print(f"DIAG: core3 status = {st3:#x} (bit4 halted={((st3>>4)&1)})")
    pc3 = await read_csr(dut, isa.CSR_CORE_BANK + 3 * 16 + isa.CORE_CSR["CORE_PC"])
    print(f"DIAG: core3 pc = {pc3:#x}")


# ----------------------------------------------------------------------
# USB 1.1 LS/FS fabric and mailbox helpers.  uio0 is D+, uio4 is D-; both
# channels are congruent modulo four and therefore pair directly with BCU0.
# LUT0 supplies BCU rx_p from D- (required by the RTL's reset rx_prev=0),
# while LUT1 supplies rx_n from D+.
# ----------------------------------------------------------------------
USB_DP_PIN = 0
USB_DM_PIN = 4
USB_ROLE_TX_TOKEN = 1
USB_ROLE_RX_TOKEN = 2
USB_ROLE_RESPOND_IN = 3
USB_ROLE_DATA_ACK = 4
USB_ROLE_TX_DATA = 5


async def configure_usb_pins(dut):
    await write_pin_router(dut, {
        USB_DP_PIN: SRC_BCU_P,
        USB_DM_PIN: SRC_BCU_N,
        8 + USB_DP_PIN: SRC_BCU_OE,
        8 + USB_DM_PIN: SRC_BCU_OE,
    })
    await write_lut(dut, 0, lut_tt_identity_in0(), (USB_DM_PIN, 15, 15, 15))
    await write_lut(dut, 1, lut_tt_identity_in0(), (USB_DP_PIN, 15, 15, 15))


def make_usb_bus(dut):
    bus = USBBus(
        dut.clk,
        lambda level: drive_uio_bit(dut, USB_DP_PIN, level),
        lambda level: drive_uio_bit(dut, USB_DM_PIN, level),
        lambda: uio_out_bit(dut, USB_DP_PIN),
        lambda: uio_out_bit(dut, USB_DM_PIN),
        lambda: uio_oe_bit(dut, USB_DP_PIN),
        lambda: uio_oe_bit(dut, USB_DM_PIN),
    )
    bus.start_resolver()
    return bus


async def usb_setup(dut, role, speed=0, pid=PID_IN, addr=5, endp=0,
                    payload=b"", handshake_pid=PID_ACK):
    await reset_dut(dut, start_clock=True)
    await load_firmware(dut, "usb_fs.hasm")
    await configure_usb_pins(dut)
    await clear_mailbox(dut)
    # role,speed,pid,addr,endp,len at bytes2..7; payload input at byte8.
    hdr = bytes([role, speed, pid if role != USB_ROLE_DATA_ACK else handshake_pid,
                 addr, endp, len(payload)])
    await write_sram1_bytes(dut, 2, hdr)
    if payload:
        await write_sram1_bytes(dut, 8, bytes(payload))
    bus = make_usb_bus(dut)
    bit_cycles = 5 if speed == 0 else 40
    host = USBHostBfm(bus, bit_cycles=bit_cycles)
    receiver = USBDeviceReceiver(bus, bit_cycles=bit_cycles)
    await write_csr(dut, CSR_CTRL_RUN, 0x0003)  # USB core0 wire + core1 assistant
    await cycles(dut, 160)  # relaxed builder plus one-bit NRZI training pulse
    return bus, host, receiver


async def usb_assert_no_device_packet(dut, bus, cycles_count=120):
    for _ in range(cycles_count):
        await cycles(dut, 1)
        assert bus.line != "K", "device started a packet despite invalid host traffic"
    assert not bus.contention_errors, bus.contention_errors


@cocotb.test()
async def test_usb_tx_token(dut):
    """DUT transmits a complete NRZI/stuffed/token-CRC5 packet."""
    bus, _host, receiver = await usb_setup(
        dut, USB_ROLE_TX_TOKEN, pid=PID_IN, addr=5, endp=0)
    got = await receiver.capture_packet()
    assert got.kind == "token"
    assert got.pid == PID_IN
    assert got.addr == 5 and got.endp == 0
    status = await wait_mailbox_status(dut, 0x01, 20)
    assert status & 0x01
    assert not bus.contention_errors, bus.contention_errors


@cocotb.test()
async def test_usb_rx_token(dut):
    """DUT captures and firmware-validates OUT/IN/SETUP-style token bytes."""
    bus, host, _receiver = await usb_setup(dut, USB_ROLE_RX_TOKEN)
    await host.send_token(pid=PID_IN, addr=5, endp=0)
    status = await wait_mailbox_status(dut, 0x02, 30)
    assert status & 0x02
    got = await read_sram1_bytes(dut, 0x30, 6)
    assert got[0] == 4, f"USB token capture count {got[0]}"
    assert got[1] == PID_IN
    assert got[2] == 5 and got[3] == 0
    assert not bus.contention_errors, bus.contention_errors


@cocotb.test()
async def test_usb_response_window(dut):
    """IN token elicits DATA0 in the USB 2..6.5 UI turnaround window."""
    payload = bytes([0xFF, 0xFF])  # forces real USB bit stuffing in the response
    bus, host, receiver = await usb_setup(
        dut, USB_ROLE_RESPOND_IN, pid=PID_DATA0, payload=payload)
    sent = await host.send_token(pid=PID_IN, addr=5, endp=0)
    got = await receiver.capture_packet(reference_j_cycle=sent.j_cycle)
    assert got.kind == "data" and got.pid == PID_DATA0
    assert got.payload == payload
    assert got.stuff_positions, "response did not exercise bit stuffing"
    ui_cycles = got.latency_cycles / 5.0
    assert 2.0 <= ui_cycles <= 6.5,         f"USB response latency {got.latency_cycles} cycles ({ui_cycles:.2f} UI)"
    status = await wait_mailbox_status(dut, 0x04, 30)
    assert status & 0x04
    assert not bus.contention_errors, bus.contention_errors


@cocotb.test()
async def test_usb_crc5_err(dut):
    """Corrupted token CRC5 is rejected and produces no device response."""
    bus, host, _receiver = await usb_setup(
        dut, USB_ROLE_RESPOND_IN, pid=PID_DATA0, payload=b"A")
    await host.send_token(pid=PID_IN, addr=5, endp=0, crc_mutate=0)
    status = await wait_mailbox_status(dut, 0x20, 30)
    assert status & 0x20
    await usb_assert_no_device_packet(dut, bus, 100)


@cocotb.test()
async def test_usb_crc16_err(dut):
    """Corrupted DATA0 CRC16 is rejected; no ACK handshake is emitted."""
    payload = bytes([1, 2, 3])
    bus, host, _receiver = await usb_setup(
        dut, USB_ROLE_DATA_ACK, payload=payload, handshake_pid=PID_ACK)
    await host.send_data(payload, pid=PID_DATA0, crc_mutate=0)
    status = await wait_mailbox_status(dut, 0x20, 30)
    assert status & 0x20
    await usb_assert_no_device_packet(dut, bus, 100)


@cocotb.test()
async def test_usb_stuff_err(dut):
    """A seventh consecutive one raises the BCU sticky stuff-error status."""
    payload = bytes([0xFF, 0xFF, 0xFF])
    packet = bytes([0x80, PID_DATA0]) + payload + b"\x00\x00"
    _levels, positions = wire_levels_for_packet(packet)
    assert positions, "non-vacuous stuff-error mutation requires a stuff bit"
    bus, host, _receiver = await usb_setup(
        dut, USB_ROLE_DATA_ACK, payload=payload, handshake_pid=PID_ACK)
    await host.send_data(payload, pid=PID_DATA0, stuff_error=True)
    status = await wait_mailbox_status(dut, 0x40, 30)
    assert status & 0x40
    await usb_assert_no_device_packet(dut, bus, 100)


@cocotb.test()
async def test_usb_ls_smoke(dut):
    """Low-speed smoke: one DUT TX token and one DUT RX token at 40 cycles/UI."""
    bus, _host, receiver = await usb_setup(
        dut, USB_ROLE_TX_TOKEN, speed=1, pid=PID_IN, addr=3, endp=0)
    got = await receiver.capture_packet(timeout_cycles=40000)
    assert got.kind == "token" and got.addr == 3
    assert not bus.contention_errors, bus.contention_errors

    bus, host, _receiver = await usb_setup(
        dut, USB_ROLE_RX_TOKEN, speed=1)
    await host.send_token(pid=PID_IN, addr=3, endp=0)
    status = await wait_mailbox_status(dut, 0x02, 30)
    assert status & 0x02
    assert not bus.contention_errors, bus.contention_errors
