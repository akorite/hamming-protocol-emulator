# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Independent SPI controller and target BFMs for modes 0-3.

The BFMs implement SPI clock/data phase directly from the mode number.  They
sample MOSI only on the mode's sampling edge and change MISO only after the
opposite (shift) edge.  No code is shared with the Hamming firmware.
"""

import cocotb
from cocotb.triggers import ClockCycles


class SPIProtocolError(AssertionError):
    """Raised for SPI framing, clock, or chip-select violations."""


def spi_cpol(mode):
    return (int(mode) >> 1) & 1


def spi_cpha(mode):
    return int(mode) & 1


def validate_clock_periods(periods, nominal_cycles, tolerance=0.15):
    """Independently validate a list of full-clock edge periods in cycles."""
    if not periods:
        raise SPIProtocolError("SPI clock had no complete periods")
    lo = nominal_cycles * (1.0 - tolerance)
    hi = nominal_cycles * (1.0 + tolerance)
    bad = [p for p in periods if not lo <= p <= hi]
    if bad:
        raise SPIProtocolError(
            f"SPI clock periods {bad} outside {lo:.2f}..{hi:.2f} cycles")


class SPITargetBfm:
    """Independent SPI target used when the DUT is the controller.

    Callbacks are synchronous zero-argument functions except `drive_miso(bit)`:
      sample_cs_n(), sample_sclk(), sample_mosi(), drive_miso(bit).
    """

    def __init__(self, clk, sample_cs_n, sample_sclk, sample_mosi, drive_miso,
                 mode=0, expected_clock_cycles=None, clock_tolerance=0.15):
        if not 0 <= mode <= 3:
            raise ValueError("SPI mode must be 0..3")
        self.clk = clk
        self.sample_cs_n = sample_cs_n
        self.sample_sclk = sample_sclk
        self.sample_mosi = sample_mosi
        self.drive_miso = drive_miso
        self.mode = int(mode)
        self.cpol = spi_cpol(mode)
        self.cpha = spi_cpha(mode)
        self.expected_clock_cycles = expected_clock_cycles
        self.clock_tolerance = clock_tolerance
        self.edge_periods = []

    async def _wait_sclk_change(self, old_level, timeout_cycles):
        for elapsed in range(1, timeout_cycles + 1):
            await ClockCycles(self.clk, 1)
            if (self.sample_cs_n() & 1) == 1:
                raise SPIProtocolError("SPI CS_N deasserted mid-transfer")
            level = self.sample_sclk() & 1
            if level != (old_level & 1):
                return level, elapsed
        raise SPIProtocolError("SPI timeout waiting for clock edge")

    async def transfer(self, tx_bytes, timeout_cycles=100000):
        """Serve one chip-select transaction and return captured MOSI bytes."""
        tx_bytes = list(tx_bytes)
        tx_wire = []
        for byte in tx_bytes:
            if not 0 <= byte <= 0xFF:
                raise ValueError("SPI byte out of range")
            tx_wire.extend((byte >> i) & 1 for i in range(7, -1, -1))
        total_bits = len(tx_wire)

        # Require an observed idle-high interval before activation.  The chip
        # reset/release state is zero on the router leaf, so treating the
        # initial low as a real chip-select would make every bring-up look
        # like a mid-transfer deassertion.
        while (self.sample_cs_n() & 1) == 0:
            await ClockCycles(self.clk, 1)
        # Now wait for the genuine high-to-low activation.  The clock must
        # already be in the mode's idle polarity, an independent protocol check.
        while (self.sample_cs_n() & 1) == 1:
            await ClockCycles(self.clk, 1)
        if (self.sample_sclk() & 1) != self.cpol:
            raise SPIProtocolError(
                f"SPI mode {self.mode}: SCLK idle is not CPOL={self.cpol}")

        # CPHA=0 requires the first MISO bit to be valid on CS_N assertion.
        tx_pos = 0
        if self.cpha == 0 and tx_wire:
            self.drive_miso(tx_wire[0])
            tx_pos = 1
        else:
            self.drive_miso(0)

        level = self.sample_sclk() & 1
        rx_bits = []
        edge_index = 0
        half_periods = []
        while len(rx_bits) < total_bits:
            new_level, elapsed = await self._wait_sclk_change(level, timeout_cycles)
            edge_index += 1
            half_periods.append(elapsed)
            first_edge_of_pair = (edge_index & 1) == 1
            if not first_edge_of_pair:
                # A bit clock period is the pair of half-periods within one
                # bit.  The first bit of each byte is excluded because its
                # leading half-period is the legal CS setup/inter-byte clock
                # idle interval rather than part of the burst rate.
                bit_index = edge_index // 2 - 1
                if (bit_index % 8) != 0:
                    self.edge_periods.append(half_periods[-1] + half_periods[-2])
            sample_edge = first_edge_of_pair if self.cpha == 0 else not first_edge_of_pair
            if sample_edge:
                rx_bits.append(self.sample_mosi() & 1)
            else:
                if tx_pos < total_bits:
                    self.drive_miso(tx_wire[tx_pos])
                    tx_pos += 1
            level = new_level

        if self.expected_clock_cycles is not None:
            validate_clock_periods(self.edge_periods, self.expected_clock_cycles,
                                   self.clock_tolerance)

        out = []
        for pos in range(0, len(rx_bits), 8):
            byte = 0
            for bit in rx_bits[pos:pos + 8]:
                byte = (byte << 1) | bit
            out.append(byte)
        return bytes(out)


class SPIControllerBfm:
    """Independent SPI controller used when the DUT is the target."""

    def __init__(self, clk, drive_cs_n, drive_sclk, drive_mosi, sample_miso,
                 mode=0, half_period_cycles=6):
        if not 0 <= mode <= 3:
            raise ValueError("SPI mode must be 0..3")
        if half_period_cycles < 2:
            raise ValueError("half_period_cycles must allow edge visibility")
        self.clk = clk
        self.drive_cs_n = drive_cs_n
        self.drive_sclk = drive_sclk
        self.drive_mosi = drive_mosi
        self.sample_miso = sample_miso
        self.mode = int(mode)
        self.cpol = spi_cpol(mode)
        self.cpha = spi_cpha(mode)
        self.half = int(half_period_cycles)

    async def _wait_half(self):
        await ClockCycles(self.clk, self.half)

    async def transfer(self, tx_bytes, cs_setup_cycles=8, cs_hold_cycles=8):
        """Clock `tx_bytes` MSB-first and return the simultaneous MISO bytes."""
        tx_bytes = list(tx_bytes)
        tx_wire = []
        for byte in tx_bytes:
            tx_wire.extend((byte >> i) & 1 for i in range(7, -1, -1))

        idle = self.cpol
        active = 1 - self.cpol
        self.drive_sclk(idle)
        self.drive_cs_n(1)
        self.drive_mosi(tx_wire[0] if (tx_wire and self.cpha == 0) else 0)
        await ClockCycles(self.clk, cs_setup_cycles)
        self.drive_cs_n(0)
        await ClockCycles(self.clk, max(2, self.half // 2))

        rx_bits = []
        for bit_index, mosi_bit in enumerate(tx_wire):
            # First edge of the clock pair.
            self.drive_sclk(active)
            if self.cpha == 1:
                # CPHA=1: the first edge is the shift edge; new controller data
                # becomes valid immediately after it.
                self.drive_mosi(mosi_bit)
            await ClockCycles(self.clk, 1)
            if self.cpha == 0:
                rx_bits.append(self.sample_miso() & 1)
            await ClockCycles(self.clk, self.half - 1)

            # Second edge returns to idle.
            self.drive_sclk(idle)
            await ClockCycles(self.clk, 1)
            if self.cpha == 1:
                rx_bits.append(self.sample_miso() & 1)
            await ClockCycles(self.clk, self.half - 1)

            if self.cpha == 0:
                # CPHA=0: the idle-returning edge is the shift edge.  Preload
                # the next bit while the clock is idle.
                if bit_index + 1 < len(tx_wire):
                    self.drive_mosi(tx_wire[bit_index + 1])
                else:
                    self.drive_mosi(0)

        await ClockCycles(self.clk, cs_hold_cycles)
        self.drive_cs_n(1)
        self.drive_mosi(0)
        self.drive_sclk(idle)

        out = []
        for pos in range(0, len(rx_bits), 8):
            byte = 0
            for bit in rx_bits[pos:pos + 8]:
                byte = (byte << 1) | bit
            out.append(byte)
        return bytes(out)


async def run_spi_controller_transaction(controller, tx_bytes):
    task = cocotb.start_soon(controller.transfer(tx_bytes))
    return await task
