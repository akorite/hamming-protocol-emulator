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
