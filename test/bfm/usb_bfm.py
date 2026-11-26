# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Independent USB 1.1 low/full-speed host-side wire model and BFM.

This module is deliberately independent of the Hamming firmware.  It uses
USB's logical-bit ordering (bytes LSB-first), six-consecutive-ones stuffing
starting with SYNC (including the final one bit of SYNC, per USB 2.0
section 7.1.9), NRZI with idle-J initial level, and a two-SE0 plus one-J
full/low-speed EOP.  The two-pin digital bus model also checks host/device
output-enable contention.
"""

from dataclasses import dataclass, field

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge


class USBProtocolError(AssertionError):
    """Raised for USB electrical, NRZI, stuffing, framing, or CRC errors."""


J = "J"
K = "K"
SE0 = "SE0"
HIZ = "HIZ"

PID_OUT = 0xE1
PID_IN = 0x69
PID_SETUP = 0x2D
PID_DATA0 = 0xC3
PID_DATA1 = 0x4B
PID_ACK = 0xD2
PID_NAK = 0x5A
PID_STALL = 0x1E

TOKEN_PIDS = {PID_OUT, PID_IN, PID_SETUP}
DATA_PIDS = {PID_DATA0, PID_DATA1}
HANDSHAKE_PIDS = {PID_ACK, PID_NAK, PID_STALL}


# ----------------------------------------------------------------------
# Pure independent codec helpers.
# ----------------------------------------------------------------------
def bits_lsb_first(data):
    return [(byte >> bit) & 1 for byte in data for bit in range(8)]


def bytes_lsb_first(bits):
    if len(bits) % 8:
        raise USBProtocolError(f"USB logical bit count {len(bits)} not byte aligned")
    out = bytearray()
    for pos in range(0, len(bits), 8):
        value = 0
        for bitpos, bit in enumerate(bits[pos:pos + 8]):
            value |= (bit & 1) << bitpos
        out.append(value)
    return bytes(out)


def crc5_step(state, bit):
    fb = (state ^ bit) & 1
    state >>= 1
    if fb:
        state ^= 0x14
    return state & 0x1F


def crc16_step(state, bit):
    fb = (state ^ bit) & 1
    state >>= 1
    if fb:
        state ^= 0xA001
    return state & 0xFFFF


def crc_raw(bits, width):
    state = (1 << width) - 1
    step = crc5_step if width == 5 else crc16_step
    for bit in bits:
        state = step(state, bit)
    return state


def crc5_token(addr, endp):
    """Transmitted (complemented) CRC5 for an 11-bit USB token field."""
    if not 0 <= addr < 128 or not 0 <= endp < 16:
        raise ValueError("USB token ADDR/ENDP out of range")
    token_bits = [(addr >> i) & 1 for i in range(7)]
    token_bits += [(endp >> i) & 1 for i in range(4)]
    return crc_raw(token_bits, 5) ^ 0x1F


def crc16_data(payload):
    """Transmitted (complemented) USB CRC16 integer for payload bytes."""
    return crc_raw(bits_lsb_first(bytes(payload)), 16) ^ 0xFFFF


def token_packet_bytes(pid, addr, endp, crc_mutate=None):
    if pid not in TOKEN_PIDS:
        raise ValueError(f"not a token PID: 0x{pid:02x}")
    crc = crc5_token(addr, endp)
    if crc_mutate is not None:
        crc ^= 1 << (crc_mutate % 5)
    tb0 = (addr & 0x7F) | ((endp & 1) << 7)
    tb1 = ((endp >> 1) & 0x7) | ((crc & 0x1F) << 3)
    return bytes([0x80, pid, tb0, tb1])


def data_packet_bytes(pid, payload, crc_mutate=None):
    if pid not in DATA_PIDS:
        raise ValueError(f"not a data PID: 0x{pid:02x}")
    payload = bytes(payload)
    crc = crc16_data(payload)
    if crc_mutate is not None:
        crc ^= 1 << (crc_mutate % 16)
    return bytes([0x80, pid]) + payload + crc.to_bytes(2, "little")


def handshake_packet_bytes(pid):
    if pid not in HANDSHAKE_PIDS:
        raise ValueError(f"not a handshake PID: 0x{pid:02x}")
    return bytes([0x80, pid])


def stuff_bits(bits):
    """Insert USB stuff zeros after six consecutive logical one bits.

    The counter starts empty at the first SYNC bit.  Because SYNC ends in a
    single one, that one is naturally counted as the first one of a run,
    matching the USB specification.
    """
    out = []
    stuff_positions = []
    run = 0
    for bit in bits:
        out.append(bit & 1)
        if bit & 1:
            run += 1
        else:
            run = 0
        if run == 6:
            out.append(0)
            stuff_positions.append(len(out) - 1)
            run = 0
    return out, stuff_positions


def destuff_bits(bits):
    """Remove and validate USB stuff bits; return (logical_bits, positions)."""
    out = []
    positions = []
    run = 0
    expect_stuff = False
    for pos, bit in enumerate(bits):
        bit &= 1
        if expect_stuff:
            if bit != 0:
                raise USBProtocolError(
                    f"USB bit-stuff error at wire bit {pos}: seventh consecutive one")
            positions.append(pos)
            expect_stuff = False
            run = 0
            continue
        out.append(bit)
        if bit:
            run += 1
        else:
            run = 0
        if run == 6:
            expect_stuff = True
    if expect_stuff:
        raise USBProtocolError("truncated USB packet immediately after six ones")
    return out, positions


def nrzi_encode(bits, initial=J):
    """Encode logical bits as D+ wire states, 0=toggle and 1=hold."""
    level = initial
    out = []
    for bit in bits:
        if not (bit & 1):
            level = K if level == J else J
        out.append(level)
    return out


def nrzi_decode(levels, initial=J):
    """Decode D+ J/K levels to logical bits, 0=toggle and 1=hold."""
    prev = initial
    out = []
    for level in levels:
        if level not in (J, K):
            raise USBProtocolError(f"non-data state {level} inside NRZI body")
        out.append(1 if level == prev else 0)
        prev = level
    return out


def logical_packet_bits(packet_bytes):
    return bits_lsb_first(packet_bytes)


def wire_levels_for_packet(packet_bytes):
    logical = logical_packet_bits(packet_bytes)
    stuffed, positions = stuff_bits(logical)
    return nrzi_encode(stuffed), positions


def parse_packet_bytes(packet):
    """Independently validate and parse one destuffed logical USB packet."""
    packet = bytes(packet)
    if len(packet) < 2:
        raise USBProtocolError("USB packet shorter than SYNC+PID")
    if packet[0] != 0x80:
        raise USBProtocolError(f"bad USB SYNC byte 0x{packet[0]:02x}")
    pid = packet[1]
    if ((pid ^ (pid >> 4)) & 0xF) != 0xF:
        raise USBProtocolError(f"bad USB PID complement in 0x{pid:02x}")
    result = {"pid": pid, "type": "unknown", "raw": packet}
    if pid in TOKEN_PIDS:
        if len(packet) != 4:
            raise USBProtocolError(f"token packet length {len(packet)} != 4")
        tb0, tb1 = packet[2], packet[3]
        addr = tb0 & 0x7F
        endp = ((tb0 >> 7) & 1) | ((tb1 & 0x7) << 1)
        crc_rx = (tb1 >> 3) & 0x1F
        crc_want = crc5_token(addr, endp)
        if crc_rx != crc_want:
            raise USBProtocolError(
                f"USB CRC5 mismatch: wire 0x{crc_rx:02x} expected 0x{crc_want:02x}")
        result.update(type="token", addr=addr, endp=endp, crc=crc_rx)
    elif pid in DATA_PIDS:
        if len(packet) < 4:
            raise USBProtocolError("data packet too short for CRC16")
        payload = packet[2:-2]
        crc_rx = int.from_bytes(packet[-2:], "little")
        crc_want = crc16_data(payload)
        if crc_rx != crc_want:
            raise USBProtocolError(
                f"USB CRC16 mismatch: wire 0x{crc_rx:04x} expected 0x{crc_want:04x}")
        result.update(type="data", payload=payload, crc=crc_rx)
    elif pid in HANDSHAKE_PIDS:
        if len(packet) != 2:
            raise USBProtocolError("handshake packet length is not two bytes")
        result.update(type="handshake")
    else:
        raise USBProtocolError(f"unsupported USB PID 0x{pid:02x}")
    return result


def decode_wire_levels(levels):
    """NRZI decode, destuff, byte-pack, and validate captured data levels."""
    logical_stuffed = nrzi_decode(levels)
    logical, stuff_positions = destuff_bits(logical_stuffed)
    packet = bytes_lsb_first(logical)
    parsed = parse_packet_bytes(packet)
    parsed["stuff_positions"] = stuff_positions
    parsed["logical_bit_count"] = len(logical)
    return parsed


# ----------------------------------------------------------------------
# Two-pin host/device bus resolution model.
# ----------------------------------------------------------------------
def state_levels(state):
    if state == J:
        return 1, 0
    if state == K:
        return 0, 1
    if state == SE0:
        return 0, 0
    raise ValueError(f"bad USB bus state {state!r}")


class USBBus:
    """D+/D- digital pad resolver with pull-up idle-J and contention checks.

    Callbacks write the two external uio input bits and sample the DUT output
    and output-enable bits.  The host driver is tri-state; when nobody drives,
    the model's device-side pull-up creates the full-speed idle J state.
    """

    def __init__(self, clk, write_dp, write_dm, dut_dp, dut_dm, dut_dp_oe, dut_dm_oe):
        self.clk = clk
        self.write_dp = write_dp
        self.write_dm = write_dm
        self.dut_dp = dut_dp
        self.dut_dm = dut_dm
        self.dut_dp_oe = dut_dp_oe
        self.dut_dm_oe = dut_dm_oe
        self.host_state = HIZ
        self.line = J
        self.cycle = 0
        self.contention_errors = []
        self.events = []
        self._resolver = None
        self._write(J)

    def _write(self, state):
        dp, dm = state_levels(state)
        self.write_dp(dp)
        self.write_dm(dm)
        self.line = state

    def set_host(self, state):
        if state not in (J, K, SE0, HIZ):
            raise ValueError(f"bad host state {state!r}")
        self.host_state = state

    def dut_drive(self):
        oe_p, oe_n = self.dut_dp_oe() & 1, self.dut_dm_oe() & 1
        if oe_p != oe_n:
            msg = f"DUT USB output enables differ: dp={oe_p} dm={oe_n}"
            if msg not in self.contention_errors:
                self.contention_errors.append(msg)
        if not (oe_p or oe_n):
            return HIZ
        dp, dm = self.dut_dp() & 1, self.dut_dm() & 1
        if dp and dm:
            msg = "DUT drives illegal USB SE1 state"
            if msg not in self.contention_errors:
                self.contention_errors.append(msg)
            return SE0
        if dp:
            return J
        if dm:
            return K
        return SE0

    def start_resolver(self):
        if self._resolver is None:
            self._resolver = cocotb.start_soon(self._resolve_loop())

    async def _resolve_loop(self):
        while True:
            await RisingEdge(self.clk)
            self.cycle += 1
            host = self.host_state
            dev = self.dut_drive()
            if host != HIZ and dev != HIZ and host != dev:
                msg = (f"USB bus contention at cycle {self.cycle}: "
                       f"host {host}, device {dev}")
                if msg not in self.contention_errors:
                    self.contention_errors.append(msg)
            new = host if host != HIZ else (dev if dev != HIZ else J)
            if new != self.line:
                self.events.append((self.cycle, self.line, new))
            self._write(new)


# ----------------------------------------------------------------------
# Independent host transmitter and device packet receiver.
# ----------------------------------------------------------------------
@dataclass
class USBSendResult:
    packet: bytes
    stuffed_positions: list
    j_cycle: int
    end_cycle: int


@dataclass
class USBCaptureResult:
    packet: bytes
    pid: int
    kind: str
    payload: bytes = b""
    addr: int = 0
    endp: int = 0
    crc: int = 0
    start_cycle: int = 0
    first_edge_cycle: int = 0
    j_cycle: int = 0
    latency_cycles: int = 0
    stuff_positions: list = field(default_factory=list)


class USBHostBfm:
    """Independent USB host transmitter.  `bit_cycles` is 5 for FS or 40 LS."""

    def __init__(self, bus, bit_cycles=5):
        if bit_cycles not in (5, 40):
            raise ValueError("this BFM profile expects FS=5 or LS=40 cycles/bit")
        self.bus = bus
        self.clk = bus.clk
        self.bit_cycles = int(bit_cycles)

    async def send_levels(self, levels, jitter_pattern=None):
        pattern = list(jitter_pattern or [0])
        for index, level in enumerate(levels):
            offset = pattern[index % len(pattern)]
            duration = self.bit_cycles + int(offset)
            if duration < 2:
                raise ValueError("jitter collapses a USB bit cell")
            self.bus.set_host(level)
            await ClockCycles(self.clk, duration)
        # EOP: two bit times SE0, one bit time J, then release.
        self.bus.set_host(SE0)
        await ClockCycles(self.clk, 2 * self.bit_cycles)
        self.bus.set_host(J)
        j_cycle = self.bus.cycle
        await ClockCycles(self.clk, self.bit_cycles)
        self.bus.set_host(HIZ)
        return j_cycle

    async def send_packet(self, packet, stuff_error=False, jitter_pattern=None):
        logical = logical_packet_bits(packet)
        stuffed, positions = stuff_bits(logical)
        if stuff_error:
            if not positions:
                raise ValueError("packet has no stuff bit to corrupt")
            stuffed[positions[0]] = 1
        levels = nrzi_encode(stuffed)
        j_cycle = await self.send_levels(levels, jitter_pattern)
        return USBSendResult(packet=bytes(packet), stuffed_positions=positions,
                             j_cycle=j_cycle, end_cycle=self.bus.cycle)

    async def send_token(self, pid=PID_IN, addr=5, endp=0, crc_mutate=None,
                         stuff_error=False, jitter_pattern=None):
        return await self.send_packet(
            token_packet_bytes(pid, addr, endp, crc_mutate),
            stuff_error=stuff_error, jitter_pattern=jitter_pattern)

    async def send_data(self, payload=b"", pid=PID_DATA0, crc_mutate=None,
                        stuff_error=False, jitter_pattern=None):
        return await self.send_packet(
            data_packet_bytes(pid, payload, crc_mutate),
            stuff_error=stuff_error, jitter_pattern=jitter_pattern)


class USBDeviceReceiver:
    """Independent bit-level receiver for packets emitted by the DUT."""

    def __init__(self, bus, bit_cycles=5):
        if bit_cycles not in (5, 40):
            raise ValueError("this BFM profile expects FS=5 or LS=40 cycles/bit")
        self.bus = bus
        self.clk = bus.clk
        self.bit_cycles = int(bit_cycles)
        self.half_cycles = max(1, bit_cycles // 2)

    async def capture_packet(self, timeout_cycles=20000, reference_j_cycle=None):
        # Wait for the idle-J -> K start edge.  The DUT's invisible training
        # pulse is also J, so it cannot be mistaken for SOP.
        for _ in range(timeout_cycles):
            await ClockCycles(self.clk, 1)
            if self.bus.line == K:
                break
        else:
            raise USBProtocolError("timeout waiting for USB device SOP")
        first_edge = self.bus.cycle
        levels = []
        se0_count = 0
        await ClockCycles(self.clk, self.half_cycles)
        for _ in range(160):
            state = self.bus.line
            if state == SE0:
                se0_count += 1
                if se0_count >= 2:
                    break
            elif state in (J, K):
                if se0_count:
                    raise USBProtocolError("single-SE0 dribble inside USB packet body")
                levels.append(state)
            else:
                raise USBProtocolError(f"illegal state {state} in USB packet")
            await ClockCycles(self.clk, self.bit_cycles)
        else:
            raise USBProtocolError("USB packet did not reach SE0 EOP")

        # Finish at the following J bit; record its first observed cycle by
        # scanning the bus event list (falling back to the nominal schedule).
        parsed = decode_wire_levels(levels)
        j_cycle = None
        for cyc, old, new in self.bus.events:
            if cyc >= self.bus.cycle and old == SE0 and new == J:
                j_cycle = cyc
                break
        if j_cycle is None:
            j_cycle = self.bus.cycle + self.bit_cycles // 2
        latency = (first_edge - reference_j_cycle) if reference_j_cycle is not None else 0
        return USBCaptureResult(
            packet=parsed["raw"], pid=parsed["pid"], kind=parsed["type"],
            payload=parsed.get("payload", b""), addr=parsed.get("addr", 0),
            endp=parsed.get("endp", 0), crc=parsed.get("crc", 0),
            start_cycle=first_edge, first_edge_cycle=first_edge,
            j_cycle=j_cycle, latency_cycles=latency,
            stuff_positions=parsed.get("stuff_positions", []))


def mutate_first_stuff_packet(packet):
    """Return stuffed logical bits with the first inserted zero flipped to one.

    This helper is used by pure-BFM non-vacuity checks and documents the exact
    mutation independently of the chip firmware.
    """
    logical = logical_packet_bits(packet)
    stuffed, positions = stuff_bits(logical)
    if not positions:
        raise ValueError("packet has no stuff bit to mutate")
    stuffed[positions[0]] = 1
    return stuffed
