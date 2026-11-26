# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Independent UART bus-functional model.

The model knows nothing about the Hamming firmware.  It implements asynchronous
framing directly: start bit, eight data bits (LSB first), optional even parity,
and one stop bit.  Transmit drift is applied to the ideal bit-time accumulator
rather than by changing every wait by a fixed rounded integer, so a +/-2% test
keeps the correct long-term wire rate.
"""

from dataclasses import dataclass

import cocotb
from cocotb.triggers import ClockCycles


class UARTProtocolError(AssertionError):
    """Raised when a wire violates UART framing/parity/protocol rules."""


@dataclass
class UARTFrameResult:
    data: int
    framing_error: bool = False
    parity_error: bool = False
    bits: tuple = ()


def _parity8(value):
    return bin(value & 0xFF).count("1") & 1


def encode_frame(byte, parity="even"):
    """Return wire bits (first bit first) for one 8-bit frame."""
    if not 0 <= byte <= 0xFF:
        raise ValueError("UART byte out of range")
    bits = [0]
    bits.extend((byte >> i) & 1 for i in range(8))
    if parity == "even":
        bits.append(_parity8(byte))
    elif parity in (None, "none"):
        pass
    else:
        raise ValueError(f"unsupported parity {parity!r}")
    bits.append(1)
    return bits


def decode_frame_bits(bits, parity="even"):
    """Decode and independently validate one frame-bit sequence.

    Raises UARTProtocolError for either error class.  The returned object is
    useful in tests that intentionally inject an error and want to inspect it.
    """
    need = 11 if parity == "even" else 10
    if len(bits) < need:
        raise UARTProtocolError(f"short UART frame: {len(bits)} bits")
    bits = list(bits[:need])
    framing = bits[0] != 0 or bits[-1] != 1
    data = sum((bits[1 + i] & 1) << i for i in range(8))
    par_err = False
    if parity == "even":
        par_err = bits[9] != _parity8(data)
    if framing:
        raise UARTProtocolError(f"UART framing error in bits {bits}")
    if par_err:
        raise UARTProtocolError(
            f"UART parity error: data=0x{data:02X} bits={bits}")
    return UARTFrameResult(data=data, bits=tuple(bits))


class _FractionalClock:
    """Integer-clock scheduler tracking a fractional ideal phase."""

    def __init__(self):
        self.ideal = 0.0
        self.elapsed = 0

    async def advance(self, clk, amount):
        self.ideal += amount
        target = int(self.ideal + 0.5)
        n = target - self.elapsed
        if n < 1:
            n = 1
        await ClockCycles(clk, n)
        self.elapsed += n


class UARTTransmitter:
    """Bit-bang UART frames onto one input pin through a caller callback.

    `drive(bit)` is synchronous and must update the externally-driven pin
    shadow.  `clk` is the system clock used only as the timing reference.
    """

    def __init__(self, clk, drive, bit_cycles, parity="even", drift=0.0,
                 start_delay_cycles=0):
        if bit_cycles <= 0:
            raise ValueError("bit_cycles must be positive")
        self.clk = clk
        self.drive = drive
        self.bit_cycles = float(bit_cycles) * (1.0 + float(drift))
        self.parity = parity
        self.start_delay_cycles = int(start_delay_cycles)

    async def send_frame(self, byte):
        await self.send_bytes([byte])

    async def send_bits(self, wire):
        """Transmit an explicit wire-bit sequence (first bit first).

        This is intentionally separate from `send_bytes`: valid frames are the
        normal path, while malformed-bit injection is explicit at the call
        site and cannot accidentally weaken the independent encoder.
        """
        wire = list(wire)
        if not wire:
            return
        if self.start_delay_cycles:
            await ClockCycles(self.clk, self.start_delay_cycles)
        sched = _FractionalClock()
        self.drive(wire[0])
        for bit in wire[1:]:
            await sched.advance(self.clk, self.bit_cycles)
            self.drive(bit & 1)
        # Hold the final level for one nominal bit time so the end of the
        # injected frame is well defined before the caller restores idle.
        await sched.advance(self.clk, self.bit_cycles)

    async def send_bytes(self, data):
        wire = []
        for byte in data:
            wire.extend(encode_frame(byte, self.parity))
        await self.send_bits(wire)


class UARTReceiver:
    """Recover UART frames from a digital input bit independently."""

    def __init__(self, clk, sample, bit_cycles, parity="even", drift=0.0):
        self.clk = clk
        self.sample = sample
        self.bit_cycles = float(bit_cycles) * (1.0 + float(drift))
        self.parity = parity

    async def receive_frame(self, timeout_cycles=100000):
        """Wait for a falling start edge and decode one frame.

        The receiver validates the start and stop levels and even parity.  It
        returns UARTFrameResult instead of raising so tests can deliberately
        exercise error frames; valid-path tests should assert both flags false.
        """
        prev = self.sample() & 1
        for _ in range(timeout_cycles):
            await ClockCycles(self.clk, 1)
            cur = self.sample() & 1
            if prev == 1 and cur == 0:
                break
            prev = cur
        else:
            raise UARTProtocolError("UART receive timeout waiting for start")

        sched = _FractionalClock()
        # Move to the nominal centre of the start bit and prove it is still low.
        await sched.advance(self.clk, self.bit_cycles / 2.0)
        if (self.sample() & 1) != 0:
            raise UARTProtocolError("UART false start bit")
        bits = [0]
        nbits = 11 if self.parity == "even" else 10
        for _ in range(1, nbits):
            await sched.advance(self.clk, self.bit_cycles)
            bits.append(self.sample() & 1)

        data = sum((bits[1 + i] & 1) << i for i in range(8))
        framing_error = bits[-1] != 1
        parity_error = False
        if self.parity == "even":
            parity_error = bits[9] != _parity8(data)
        return UARTFrameResult(data=data, framing_error=framing_error,
                               parity_error=parity_error, bits=tuple(bits))

    async def receive_bytes(self, count, timeout_cycles=100000):
        out = []
        for _ in range(count):
            out.append(await self.receive_frame(timeout_cycles))
        return out


async def run_uart_full_duplex(tx_bfm, rx_bfm, tx_bytes, rx_count):
    """Concurrently run one transmitter and receiver; returns received frames."""
    rx_task = cocotb.start_soon(rx_bfm.receive_bytes(rx_count))
    tx_task = cocotb.start_soon(tx_bfm.send_bytes(tx_bytes))
    frames = await rx_task
    await tx_task
    return frames
