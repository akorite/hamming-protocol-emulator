"""Debug harness: replicate the MITM clean pass-through scenario and dump
wire-level edge events for both UART wires out of the RLE capture ring."""
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from tools import hamming_isa as isa  # noqa: E402
from cocotb_baseline import *  # noqa: E402,F401,F403
from cocotb_demos import (  # noqa: E402
    bit_events, arm_rle_capture, stop_rle_capture, read_rle_records,
    BIT_CYCLES, UPSTREAM_PROBE_BIT, DOWNSTREAM_PROBE_BIT,
    configure_uart_bcu_pins, write_bcu_config, clear_mailbox,
    write_sram1_bytes, uart_wire_loopback,
)
from bfm.uart_bfm import UARTTransmitter, UARTReceiver  # noqa: E402


async def wire_monitor(dut, bit, log, start_after):
    """System-clock-domain wire witness: record (cycle, level) at every change."""
    last = None
    cyc = 0
    while True:
        await RisingEdge(dut.clk)
        cyc += 1
        if cyc < start_after:
            continue
        lvl = uart_tx_line_bit(dut, bit)
        if lvl != last:
            log.append((cyc, lvl))
            last = lvl


@cocotb.test()
async def debug_wire_dump(dut):
    await reset_dut(dut, start_clock=True)
    drive_ui_bit(dut, 0, 1)
    await load_firmware(dut, "mitm_uart.hasm")
    await configure_uart_bcu_pins(dut, rx_pin=8, tx_uio=0)
    await write_bcu_config(
        dut, 0,
        isa.pack_bcu_mode(codec="nrz", stuff="off"),
        (11 - 1),
        isa.pack_dpll_cfg(mode="hard_snap", win=0, period=BIT_CYCLES),
    )
    await clear_mailbox(dut)
    await write_sram1_bytes(dut, 0, bytes([0, 0, 0, 2, 0, 0, 0, 0]))
    cocotb.start_soon(uart_wire_loopback(dut, 0))

    upstream = bytes([0x96, 0x17, 0xE1, 0x42])
    tx = UARTTransmitter(
        dut.clk, lambda bit: drive_ui_bit(dut, 0, bit),
        bit_cycles=BIT_CYCLES, parity="even", drift=0,
        start_delay_cycles=100)
    rx = UARTReceiver(
        dut.clk, lambda: uart_tx_line_bit(dut, 0),
        bit_cycles=BIT_CYCLES, parity="even", drift=0)

    wlog = []
    cocotb.start_soon(wire_monitor(dut, 0, wlog, 0))
    await arm_rle_capture(dut, (1 << UPSTREAM_PROBE_BIT) | (1 << DOWNSTREAM_PROBE_BIT))
    rx_task = cocotb.start_soon(rx.receive_bytes(4, 30000))
    await write_csr(dut, CSR_CTRL_RUN, 0x0001)
    tx_task = cocotb.start_soon(tx.send_bytes(upstream))
    frames = await rx_task
    await tx_task
    await cycles(dut, 300)
    await stop_rle_capture(dut)

    print("DOWNSTREAM frames:", [hex(f.data) for f in frames])
    # system-domain wire transitions, compressed to (cycle, level); report the
    # gaps so narrow glitch pulses are visible at cycle resolution
    if wlog:
        print("wire transitions (sys cycles):")
        for i in range(1, len(wlog)):
            dt = wlog[i][0] - wlog[i-1][0]
            print(f"  cyc {wlog[i][0]} -> {wlog[i][1]} (dwell {dt})")
    records = await read_rle_records(dut)
    print(f"capture records: {len(records)}")
    from cocotb_demos import decode_uart_stream
    for bit, name in ((UPSTREAM_PROBE_BIT, "upstream"), (DOWNSTREAM_PROBE_BIT, "downstream")):
        ev = bit_events(records, bit)
        print(f"{name}: {len(ev)} edges")
        try:
            fr = decode_uart_stream(ev, BIT_CYCLES, 4)
            for i, fbits in enumerate(fr):
                data = sum(b << k for k, b in enumerate(fbits[1:9]))
                print(f"{name} frame{i}: bits={fbits} data=0x{data:02x} parity_bit={fbits[9]} stop={fbits[10]}")
        except Exception as e:
            print(f"{name} decode error: {e}")
        if ev:
            t0 = ev[0][0]
            rel = [(round((t - t0), 1), lvl) for t, lvl in ev]
            print(f"{name} raw edges (cycles from first): {rel[:60]}")
