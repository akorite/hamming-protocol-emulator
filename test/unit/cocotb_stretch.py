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
    read_sram1_bytes,
    reset_dut,
    ui_bit,
    uio_oe_bit,
    uio_out_bit,
    uo_bit,
    wait_mailbox_status,
    write_csr,
    write_pin_router,
    write_sram1_bytes,
    write_ui_shadow,
    write_uio_shadow,
    u,
)

from bfm.can_bfm import CANBus, CANNodeBfm, CANProtocolError  # noqa: E402


# ----------------------------------------------------------------------
# CAN fabric configuration: core3 bit-bangs uio3 (data+oe, open-drain).
# ----------------------------------------------------------------------
async def configure_can_pins(dut):
    await write_pin_router(dut, {
        3: SRC_CORE,        # uio3 data <- core3 out bit0
        8 + 3: SRC_CORE,    # uio3 oe   <- core3 oe  bit0
    })


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
