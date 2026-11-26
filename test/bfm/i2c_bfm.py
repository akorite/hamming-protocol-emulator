# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Independent I2C open-drain bus, target, controller, and fault BFMs.

The bus model resolves the pad-level wired-AND behavior that the split
`uio_in`/`uio_out` cocotb interface cannot represent by itself.  Protocol
agents only ever pull a line low or release it; a pull-up makes a released line
high.  The controller/target implementations are intentionally independent of
Hamming's firmware state machines.
"""

from dataclasses import dataclass, field

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge


class I2CProtocolError(AssertionError):
    """Raised when I2C wired-AND, ACK, or clock rules are violated."""


@dataclass
class I2CTransactionResult:
    address: int
    read: bool
    write_data: bytes = b""
    read_data: bytes = b""
    acked: bool = True
    events: list = field(default_factory=list)


class OpenDrainI2CBus:
    """Wired-AND pad model for two uio pins.

    `write_pin(pin, level)` must update the external uio_in shadow.  The DUT
    output/output-enable accessors are synchronous zero-argument functions.
    External agents are named drivers; a line is low iff any agent or the DUT
    pulls it low.  Otherwise the modeled pull-up makes it high.
    """

    def __init__(self, clk, write_pin, dut_sda_out, dut_sda_oe,
                 dut_scl_out, dut_scl_oe, sda_pin, scl_pin):
        self.clk = clk
        self.write_pin = write_pin
        self.dut_sda_out = dut_sda_out
        self.dut_sda_oe = dut_sda_oe
        self.dut_scl_out = dut_scl_out
        self.dut_scl_oe = dut_scl_oe
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.ext_sda_low = set()
        self.ext_scl_low = set()
        self.line_sda = 1
        self.line_scl = 1
        self.contention_errors = []
        self._resolver = None

    def start_resolver(self):
        if self._resolver is None:
            self._resolver = cocotb.start_soon(self._resolve_loop())

    def set_drive(self, owner, line, low):
        drivers = self.ext_sda_low if line == "sda" else self.ext_scl_low
        if low:
            drivers.add(owner)
        else:
            drivers.discard(owner)

    def _dut_low(self, line):
        if line == "sda":
            out, oe = self.dut_sda_out() & 1, self.dut_sda_oe() & 1
        else:
            out, oe = self.dut_scl_out() & 1, self.dut_scl_oe() & 1
        if oe and out:
            msg = f"DUT drives {line.upper()} high while OE is asserted"
            if msg not in self.contention_errors:
                self.contention_errors.append(msg)
        return oe and not out

    def sample(self, line):
        if line == "sda":
            return 0 if (self.ext_sda_low or self._dut_low("sda")) else 1
        return 0 if (self.ext_scl_low or self._dut_low("scl")) else 1

    async def _resolve_loop(self):
        while True:
            await RisingEdge(self.clk)
            sda = self.sample("sda")
            scl = self.sample("scl")
            self.line_sda = sda
            self.line_scl = scl
            self.write_pin(self.sda_pin, sda)
            self.write_pin(self.scl_pin, scl)

    async def wait_line(self, line, level, timeout_cycles):
        for _ in range(timeout_cycles):
            await ClockCycles(self.clk, 1)
            if self.sample(line) == (level & 1):
                return
        raise I2CProtocolError(
            f"I2C timeout waiting for {line.upper()}={level & 1}")


class I2CTargetBfm:
    """Independent clocked I2C target used when Hamming is the controller."""

    def __init__(self, bus, address=0x42, read_data=b"", byte_count=2,
                 owner="target", timeout_cycles=20000):
        self.bus = bus
        self.clk = bus.clk
        self.address = int(address) & 0x7F
        self.read_data = bytes(read_data)
        self.byte_count = int(byte_count)
        self.owner = owner
        self.timeout_cycles = int(timeout_cycles)

    async def wait_start(self):
        prev = self.bus.sample("sda")
        for _ in range(self.timeout_cycles):
            await ClockCycles(self.clk, 1)
            cur = self.bus.sample("sda")
            if prev == 1 and cur == 0 and self.bus.sample("scl") == 1:
                # A START is not complete for byte framing until the master
                # has pulled SCL low.  Otherwise the still-high START clock
                # phase would be mistaken for the first bit clock.
                await self._wait_scl(0)
                return
            prev = cur
        raise I2CProtocolError("I2C target timeout waiting for START")

    async def _wait_scl(self, level):
        # A real I2C input stage rejects narrow spikes.  Require four
        # consecutive samples so the intentional 3-cycle fault pulse does not
        # clock the independent target; pass/fail then isolates the DUT filter.
        want = level & 1
        stable = 0
        for _ in range(self.timeout_cycles):
            await ClockCycles(self.clk, 1)
            if self.bus.sample("scl") == want:
                stable += 1
                if stable >= 4:
                    return
            else:
                stable = 0
        raise I2CProtocolError(f"I2C target timeout waiting for filtered SCL={want}")

    async def receive_byte(self):
        value = 0
        for _ in range(8):
            await self._wait_scl(1)
            value = (value << 1) | self.bus.sample("sda")
            await self._wait_scl(0)
        return value

    async def send_ack(self, stretch_cycles=0):
        # Entered while SCL is low after the eighth bit.  A target stretches by
        # additionally holding SCL low, then releases it and waits for the
        # controller's acknowledge clock pulse like any normal target.
        self.bus.set_drive(self.owner, "sda", True)
        if stretch_cycles:
            self.bus.set_drive(self.owner, "scl", True)
            await ClockCycles(self.clk, stretch_cycles)
            self.bus.set_drive(self.owner, "scl", False)
        await self._wait_scl(1)
        await self._wait_scl(0)
        self.bus.set_drive(self.owner, "sda", False)

    async def read_ack(self):
        self.bus.set_drive(self.owner, "sda", False)
        await self._wait_scl(1)
        ack_low = self.bus.sample("sda") == 0
        await self._wait_scl(0)
        return ack_low

    async def send_byte(self, value):
        for bit in range(7, -1, -1):
            # The byte starts in the SCL-low phase.  Data changes only there.
            self.bus.set_drive(self.owner, "sda", ((value >> bit) & 1) == 0)
            await self._wait_scl(1)
            await self._wait_scl(0)
        self.bus.set_drive(self.owner, "sda", False)

    async def serve(self, stretch_after_byte=0, stretch_cycles=0):
        """Serve one transaction and return an I2CTransactionResult."""
        await self.wait_start()
        first = await self.receive_byte()
        addr = (first >> 1) & 0x7F
        read = bool(first & 1)
        events = []
        if addr != self.address:
            # NACK by simply releasing SDA.  Do not consume later bytes: the
            # controller is required to abandon or terminate the transaction.
            return I2CTransactionResult(addr, read, acked=False,
                                        events=["wrong-address-nack"])
        await self.send_ack()
        events.append("address-ack")

        if not read:
            data = []
            for index in range(self.byte_count):
                byte = await self.receive_byte()
                data.append(byte)
                stretch = stretch_cycles if (index + 1) == stretch_after_byte else 0
                await self.send_ack(stretch)
                events.append(f"write-byte-{index}-ack")
            return I2CTransactionResult(addr, read, write_data=bytes(data),
                                        events=events)

        tx = list(self.read_data[:self.byte_count])
        tx.extend([0] * (self.byte_count - len(tx)))
        for index, byte in enumerate(tx):
            await self.send_byte(byte)
            acked = await self.read_ack()
            events.append(f"read-byte-{index}-{'ack' if acked else 'nack'}")
            if not acked:
                break
        return I2CTransactionResult(addr, read, read_data=bytes(tx),
                                    events=events)


class I2CControllerBfm:
    """Independent open-drain I2C controller used when Hamming is the target."""

    def __init__(self, bus, half_period_cycles=75, owner="master",
                 timeout_cycles=20000):
        self.bus = bus
        self.clk = bus.clk
        self.half = int(half_period_cycles)
        self.owner = owner
        self.timeout_cycles = int(timeout_cycles)
        self.arbitration_lost = False

    async def _wait_scl_high(self):
        await self.bus.wait_line("scl", 1, self.timeout_cycles)

    async def start(self):
        self.bus.set_drive(self.owner, "sda", False)
        self.bus.set_drive(self.owner, "scl", False)
        await ClockCycles(self.clk, self.half)
        self.bus.set_drive(self.owner, "sda", True)
        await ClockCycles(self.clk, self.half)
        self.bus.set_drive(self.owner, "scl", True)
        await ClockCycles(self.clk, self.half)

    async def stop(self):
        self.bus.set_drive(self.owner, "sda", True)
        await ClockCycles(self.clk, self.half)
        self.bus.set_drive(self.owner, "scl", False)
        await self._wait_scl_high()
        await ClockCycles(self.clk, self.half)
        self.bus.set_drive(self.owner, "sda", False)
        await ClockCycles(self.clk, self.half)

    async def write_bit(self, bit):
        # Entered in the SCL-low phase.  Release means logic one on the bus.
        self.bus.set_drive(self.owner, "sda", not (bit & 1))
        await ClockCycles(self.clk, max(2, self.half // 2))
        self.bus.set_drive(self.owner, "scl", False)
        await self._wait_scl_high()
        if (bit & 1) and self.bus.sample("sda") == 0:
            self.arbitration_lost = True
            self.bus.set_drive(self.owner, "sda", False)
            self.bus.set_drive(self.owner, "scl", False)
            raise I2CProtocolError("I2C controller BFM lost arbitration")
        await ClockCycles(self.clk, self.half)
        self.bus.set_drive(self.owner, "scl", True)
        await ClockCycles(self.clk, max(2, self.half // 2))

    async def write_byte(self, value):
        for bit in range(7, -1, -1):
            await self.write_bit((value >> bit) & 1)
        # Ninth clock: release SDA and sample the target ACK/NACK.
        self.bus.set_drive(self.owner, "sda", False)
        await ClockCycles(self.clk, max(2, self.half // 2))
        self.bus.set_drive(self.owner, "scl", False)
        await self._wait_scl_high()
        ack = self.bus.sample("sda") == 0
        await ClockCycles(self.clk, self.half)
        self.bus.set_drive(self.owner, "scl", True)
        await ClockCycles(self.clk, max(2, self.half // 2))
        return ack

    async def read_byte(self, send_ack):
        self.bus.set_drive(self.owner, "sda", False)
        value = 0
        for _ in range(8):
            self.bus.set_drive(self.owner, "scl", False)
            # Give the target a realistic data-setup interval while SCL is
            # low before requesting the next sample clock.
            await ClockCycles(self.clk, max(2, self.half // 2))
            self.bus.set_drive(self.owner, "scl", False)
            await self._wait_scl_high()
            value = (value << 1) | self.bus.sample("sda")
            await ClockCycles(self.clk, self.half)
            self.bus.set_drive(self.owner, "scl", True)
            await ClockCycles(self.clk, max(2, self.half // 2))
        await self.write_bit(0 if send_ack else 1)
        return value

    async def write_transaction(self, address, data):
        await self.start()
        ack = await self.write_byte(((address & 0x7F) << 1) | 0)
        if not ack:
            await self.stop()
            raise I2CProtocolError(f"I2C target NACKed write address 0x{address:02X}")
        for byte in data:
            ack = await self.write_byte(byte)
            if not ack:
                await self.stop()
                raise I2CProtocolError("I2C target NACKed write data")
        await self.stop()

    async def read_transaction(self, address, count):
        await self.start()
        ack = await self.write_byte(((address & 0x7F) << 1) | 1)
        if not ack:
            await self.stop()
            raise I2CProtocolError(f"I2C target NACKed read address 0x{address:02X}")
        data = []
        for index in range(count):
            data.append(await self.read_byte(send_ack=(index + 1 < count)))
        await self.stop()
        return bytes(data)


async def sda_arbitration_pulse(bus, owner, delay_cycles, low_cycles):
    """Competing-master fault: pull SDA low for a bounded interval."""
    await ClockCycles(bus.clk, delay_cycles)
    bus.set_drive(owner, "sda", True)
    await ClockCycles(bus.clk, low_cycles)
    bus.set_drive(owner, "sda", False)
