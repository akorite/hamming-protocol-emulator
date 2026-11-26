# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Independent IEEE 802.3 10BASE-T frame codec, wire model, and BFM.

The model is deliberately independent of the Hamming firmware.  It uses the
IEEE 802.3 Manchester convention in which the first half-cell is the
complement of the data bit and the second half-cell is the data bit.  On the
three-state twisted-pair digital interface used here, positive is
``(p,n)=(1,0)``, negative is ``(p,n)=(0,1)``, and idle/TP-idle is ``(0,0)``.
Data octets are transmitted least-significant bit first.  The FCS is the
complemented reflected CRC-32 (poly 0xEDB88320) appended little-endian; when
the FCS bits are included in the same unchecked CRC recurrence the pre-XOR
residue is 0xDEBB20E3.
"""

from dataclasses import dataclass, field

import cocotb
from cocotb.triggers import ClockCycles


class EthernetProtocolError(AssertionError):
    """Raised for Manchester, framing, FCS, TP_IDL, or timing violations."""


POSITIVE = 1
NEGATIVE = 0
IDLE = None
ETH_RESIDUE = 0xDEBB20E3
ETH_POLY = 0xEDB88320


# ----------------------------------------------------------------------
# Pure codec: independent bit-level reference implementation.
# ----------------------------------------------------------------------
def crc32_step(state, bit):
    """One reflected CRC-32 step, poly 0xEDB88320, with no init/final XOR."""
    fb = (state ^ bit) & 1
    state >>= 1
    if fb:
        state ^= ETH_POLY
    return state & 0xFFFFFFFF


def crc32_raw_bits(bits):
    """Raw reflected CRC state after `bits`, initialized to all ones."""
    state = 0xFFFFFFFF
    for bit in bits:
        state = crc32_step(state, bit)
    return state & 0xFFFFFFFF


def bits_lsb_first(data):
    """Expand bytes least-significant bit first, as on an 802.3 wire."""
    return [(byte >> bit) & 1 for byte in data for bit in range(8)]


def bytes_lsb_first(bits):
    """Pack least-significant-bit-first wire bits into bytes."""
    if len(bits) % 8:
        raise EthernetProtocolError("wire bit count is not octet aligned")
    out = bytearray()
    for pos in range(0, len(bits), 8):
        byte = 0
        for bitpos, bit in enumerate(bits[pos:pos + 8]):
            byte |= (bit & 1) << bitpos
        out.append(byte)
    return bytes(out)


def ethernet_fcs(frame):
    """The four FCS bytes in wire octet order (little-endian complemented)."""
    fcs = crc32_raw_bits(bits_lsb_first(frame)) ^ 0xFFFFFFFF
    return fcs.to_bytes(4, "little")


def frame_octets(frame):
    """Preamble/SFD + frame + FCS octets. `frame` excludes FCS."""
    return bytes([0x55] * 7 + [0xD5]) + bytes(frame) + ethernet_fcs(frame)


def frame_bits(frame):
    """Logical wire bits for preamble/SFD + frame + FCS, LSB-first per byte."""
    return bits_lsb_first(frame_octets(frame))


def manchester_cells(bits):
    """Encode logical bits as half-cell signs (POSITIVE/NEGATIVE)."""
    cells = []
    for bit in bits:
        data = POSITIVE if bit else NEGATIVE
        comp = NEGATIVE if bit else POSITIVE
        cells.extend((comp, data))
    return cells


def manchester_decode_cells(cells):
    """Decode half-cell signs, enforcing first-half complement pairing."""
    if len(cells) % 2:
        raise EthernetProtocolError("odd number of Manchester half-cells")
    bits = []
    for pos in range(0, len(cells), 2):
        first, second = cells[pos], cells[pos + 1]
        if first not in (POSITIVE, NEGATIVE) or second not in (POSITIVE, NEGATIVE):
            raise EthernetProtocolError("idle inside Manchester bit cell")
        if first == second:
            raise EthernetProtocolError(
                f"Manchester half-cells are not complementary at pair {pos // 2}")
        bits.append(1 if second == POSITIVE else 0)
    return bits


def decode_frame_bits(bits):
    """Validate and decode logical wire bits through the FCS.

    Returns a dictionary containing frame octets (without preamble/SFD or
    FCS), received/calculated FCS values, and the total logical bit count.
    """
    if len(bits) % 8:
        raise EthernetProtocolError("frame bit count is not octet aligned")
    octets = bytes_lsb_first(bits)
    if len(octets) < 8 + 60 + 4:
        raise EthernetProtocolError(
            f"short Ethernet frame: {len(octets)} octets on wire")
    if octets[:7] != bytes([0x55] * 7):
        raise EthernetProtocolError(
            f"bad Ethernet preamble: {octets[:7].hex()}")
    if octets[7] != 0xD5:
        raise EthernetProtocolError(f"bad Ethernet SFD: 0x{octets[7]:02x}")
    frame = octets[8:-4]
    fcs_rx = int.from_bytes(octets[-4:], "little")
    fcs_calc = int.from_bytes(ethernet_fcs(frame), "little")
    if fcs_rx != fcs_calc:
        raise EthernetProtocolError(
            f"Ethernet FCS mismatch: wire 0x{fcs_rx:08x} vs computed 0x{fcs_calc:08x}")
    residue = crc32_raw_bits(bits_lsb_first(frame) + bits_lsb_first(octets[-4:]))
    if residue != ETH_RESIDUE:
        raise EthernetProtocolError(
            f"Ethernet CRC residue 0x{residue:08x}, expected 0x{ETH_RESIDUE:08x}")
    return {
        "frame": bytes(frame),
        "fcs": fcs_rx,
        "bit_count": len(bits),
        "octets": octets,
        "residue": residue,
    }


# ----------------------------------------------------------------------
# Three-state digital twisted-pair wire abstraction.
# ----------------------------------------------------------------------
def pair_sign(p, n):
    """Map pin values to POSITIVE, NEGATIVE, or IDLE; reject simultaneous drive."""
    p, n = p & 1, n & 1
    if p and n:
        raise EthernetProtocolError("illegal simultaneous TX+ and TX- drive")
    if p:
        return POSITIVE
    if n:
        return NEGATIVE
    return IDLE


class EthWire:
    """10BASE-T three-state pin abstraction.

    `write_rxp`/`write_rxn` drive the external peer toward the DUT.
    `dut_txp`/`dut_txn` sample the DUT's independent output pins.  The model
    is intentionally simple because the unit tests transmit in one direction
    at a time; illegal simultaneous positive+negative drive is always an
    electrical violation.
    """

    def __init__(self, clk, write_rxp, write_rxn, dut_txp, dut_txn):
        self.clk = clk
        self.write_rxp = write_rxp
        self.write_rxn = write_rxn
        self.dut_txp = dut_txp
        self.dut_txn = dut_txn
        self.peer_sign = IDLE
        self.errors = []
        self.cycle = 0
        self.curr_dut = IDLE
        self._monitor = None
        self._set_peer(IDLE)

    def _set_peer(self, sign):
        self.peer_sign = sign
        if sign == POSITIVE:
            self.write_rxp(1)
            self.write_rxn(0)
        elif sign == NEGATIVE:
            self.write_rxp(0)
            self.write_rxn(1)
        elif sign is IDLE:
            self.write_rxp(0)
            self.write_rxn(0)
        else:
            raise ValueError(f"bad Ethernet wire sign {sign!r}")

    def set_peer_sign(self, sign):
        self._set_peer(sign)

    def dut_sign(self):
        try:
            return pair_sign(self.dut_txp(), self.dut_txn())
        except EthernetProtocolError as exc:
            if str(exc) not in self.errors:
                self.errors.append(str(exc))
            return IDLE

    def start_monitor(self):
        if self._monitor is None:
            self._monitor = cocotb.start_soon(self._monitor_loop())

    async def _monitor_loop(self):
        while True:
            await ClockCycles(self.clk, 1)
            self.cycle += 1
            self.curr_dut = self.dut_sign()


# ----------------------------------------------------------------------
# Independent peer transmitter and wire receiver.
# ----------------------------------------------------------------------
@dataclass
class EthFrameResult:
    frame: bytes
    fcs: int
    bit_count: int
    residue: int
    tp_idl_sign: int
    tp_idl_cycles: int
    halfcell_violations: list = field(default_factory=list)
    raw_octets: bytes = b""
    start_cycle: int = 0
    idle_start_cycle: int = 0


class Eth10BaseTNode:
    """Independent 10BASE-T peer/decoder.

    The transmitter emits exact Manchester half-cells with optional bit-center
    jitter.  The receiver captures the DUT waveform cycle-by-cycle, recovers
    half-cells from the frame-start edge, validates complement pairing,
    decodes octets LSB-first, verifies preamble/SFD/FCS/residue, and measures
    the last-bit-dependent TP_IDL pulse.
    """

    def __init__(self, wire, bit_cycles=6):
        if bit_cycles != 6:
            raise ValueError("this independent 60 MHz profile expects 6 cycles/bit")
        self.wire = wire
        self.clk = wire.clk
        self.bit_cycles = bit_cycles
        self.half_cycles = bit_cycles // 2

    async def send_frame(self, frame, fcs_mutate=None, jitter_pattern=None,
                         tp_idl_cycles=15, override_bits=None):
        """Transmit one conforming or deliberately mutated frame.

        `fcs_mutate` is an optional bit index (0..31) of the FCS to invert in
        the mutation space of the test; it is not part of the receiver logic.
        `jitter_pattern` is a repeatable sequence of center-edge offsets in
        cycles (-1, 0, +1 are meaningful at this 3-cycle half-cell profile).
        The bit period remains six cycles: the first half is lengthened or
        shortened by the center offset and the second half compensates.
        """
        if override_bits is not None:
            bits = list(override_bits)
        else:
            octets = bytes([0x55] * 7 + [0xD5]) + bytes(frame)
            fcs = int.from_bytes(ethernet_fcs(frame), "little")
            if fcs_mutate is not None:
                fcs ^= 1 << (fcs_mutate & 31)
            octets += fcs.to_bytes(4, "little")
            bits = bits_lsb_first(octets)
        cells = manchester_cells(bits)
        pattern = list(jitter_pattern or [0])
        final_second_duration = self.half_cycles
        for bit_index in range(len(bits)):
            center_offset = pattern[bit_index % len(pattern)]
            if center_offset not in (-1, 0, 1):
                raise ValueError("jitter offsets must be -1, 0, or +1 cycles")
            first_len = self.half_cycles + center_offset
            second_len = self.half_cycles - center_offset
            if first_len <= 0 or second_len <= 0:
                raise ValueError("jitter collapses a Manchester half-cell")
            pair = cells[2 * bit_index:2 * bit_index + 2]
            for half, duration in zip(pair, (first_len, second_len)):
                self.wire.set_peer_sign(half)
                await ClockCycles(self.clk, duration)
                if bit_index == len(bits) - 1 and half == pair[1]:
                    final_second_duration = duration
        # TP_IDL continues the final bit's second-half polarity, then both
        # pins fall to zero.  The requested width is the total final pulse,
        # including the final Manchester half-cell.
        final_sign = POSITIVE if bits[-1] else NEGATIVE
        extra = max(0, tp_idl_cycles - final_second_duration)
        self.wire.set_peer_sign(final_sign)
        await ClockCycles(self.clk, extra)
        self.wire.set_peer_sign(IDLE)
        return int.from_bytes(ethernet_fcs(frame), "little") if override_bits is None else 0

    async def capture_frame(self, timeout_cycles=200000, idle_cycles=24):
        """Capture and independently decode one DUT-transmitted frame."""
        wire = self.wire
        wire.start_monitor()
        # Wait for a non-idle transition.  Keep a short idle prefix so the
        # first active sample index is exact in the capture.
        for _ in range(timeout_cycles):
            await ClockCycles(self.clk, 1)
            if wire.dut_sign() is not IDLE:
                break
        else:
            raise EthernetProtocolError("timeout waiting for 10BASE-T frame start")

        start_cycle = wire.cycle
        samples = [wire.dut_sign()]
        idle_run = 0
        idle_start_cycle = 0
        previous = samples[0]
        for _ in range(timeout_cycles):
            await ClockCycles(self.clk, 1)
            sign = wire.dut_sign()
            samples.append(sign)
            if sign is IDLE:
                if previous is not IDLE and not idle_start_cycle:
                    idle_start_cycle = wire.cycle
                idle_run += 1
                if idle_run >= idle_cycles:
                    break
            else:
                idle_run = 0
            previous = sign
        else:
            raise EthernetProtocolError("timeout waiting for 10BASE-T TP_IDL end")
        result = self.decode_capture(samples)
        result.start_cycle = start_cycle
        result.idle_start_cycle = idle_start_cycle
        return result

    def decode_capture(self, samples):
        """Decode a cycle-by-cycle capture beginning at the first active sample."""
        # Trim trailing idle.  The first sample is defined to be the first
        # active half-cell sample visible at the pin.
        active = list(samples)
        while active and active[-1] is IDLE:
            active.pop()
        if len(active) < 2 * self.bit_cycles * 8:
            raise EthernetProtocolError("captured Ethernet waveform is too short")

        # Quantize each nominal half-cell by majority vote.  DUT TX is not
        # jittered in these tests; a constant one-cycle sampling offset is
        # harmless because the edge itself defines the grouping origin.
        cells = []
        halfcell_violations = []
        for start in range(0, len(active), self.half_cycles):
            chunk = active[start:start + self.half_cycles]
            if not chunk:
                break
            pos = sum(1 for x in chunk if x == POSITIVE)
            neg = sum(1 for x in chunk if x == NEGATIVE)
            if pos and neg:
                halfcell_violations.append(
                    (start // self.half_cycles, list(chunk)))
            if pos or neg:
                cells.append(POSITIVE if pos >= neg else NEGATIVE)
            else:
                cells.append(IDLE)

        # Consume strictly valid complementary pairs.  TP_IDL repeats the
        # final data polarity, so the first invalid pair marks end-of-frame.
        bits = []
        end_pair = 0
        pairing_error = None
        for pair_index in range(0, len(cells) - 1, 2):
            first, second = cells[pair_index], cells[pair_index + 1]
            if first in (POSITIVE, NEGATIVE) and second in (POSITIVE, NEGATIVE) \
                    and first != second:
                bits.append(1 if second == POSITIVE else 0)
                end_pair = pair_index + 2
            else:
                pairing_error = (pair_index // 2, first, second)
                break
        if pairing_error is None:
            # A frame must end through TP_IDL, not merely run out of capture.
            raise EthernetProtocolError("capture has no TP_IDL/end-of-frame boundary")
        if len(bits) % 8:
            raise EthernetProtocolError(
                f"end-of-frame is not octet aligned ({len(bits)} bits)")
        decoded = decode_frame_bits(bits)

        # Measure the final continuous pulse from the start of the final
        # Manchester second half-cell until the first idle sample.  The final
        # logical data polarity is the second half of the last valid pair.
        final_sign = cells[end_pair - 1]
        final_half_start = (end_pair - 1) * self.half_cycles
        tp_cycles = 0
        for sign in active[final_half_start:]:
            if sign != final_sign:
                break
            tp_cycles += 1
        return EthFrameResult(
            frame=decoded["frame"], fcs=decoded["fcs"],
            bit_count=decoded["bit_count"], residue=decoded["residue"],
            tp_idl_sign=final_sign, tp_idl_cycles=tp_cycles,
            halfcell_violations=halfcell_violations,
            raw_octets=decoded["octets"])


def mutate_fcs_bits(frame, fcs_bit_index=0):
    """Return complete logical wire bits with one FCS bit inverted.

    This mutation helper is used both by BFM non-vacuity tests and by the
    independent negative-transmitter path.  It preserves length and timing.
    """
    bits = frame_bits(frame)
    fcs_start = len(bits) - 32
    bits[fcs_start + (fcs_bit_index & 31)] ^= 1
    return bits
