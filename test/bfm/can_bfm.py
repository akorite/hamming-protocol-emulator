# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Independent CAN 2.0A bus model, node BFM, and frame codec.

The model knows nothing about the Hamming firmware.  It implements the
wired-AND dominant/recessive electrical rule, CAN 2.0A standard data-frame
layout, five-bit both-polarity stuffing, MSB-first CRC-15 (poly 0x4599),
in-slot ACK, and bitwise arbitration directly from the protocol
specification.

Bus electrical model: the wire is high (recessive) when released.  Any agent
pulling low makes it dominant.  No agent may ever actively drive the wire
high; an output-enabled high drive is flagged as an electrical violation.
"""

from dataclasses import dataclass, field

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge


class CANProtocolError(AssertionError):
    """Raised for CAN wire, stuffing, CRC, timing, or framing violations."""


DOMINANT = 0
RECESSIVE = 1


# ----------------------------------------------------------------------
# Pure frame codec (no timing): the independent reference implementation.
# ----------------------------------------------------------------------
def crc15_bits(bits):
    """MSB-first CRC-15 over a bit list, poly 0x4599, init 0, no final xor."""
    state = 0
    for bit in bits:
        top = (state >> 14) & 1
        fb = top ^ (bit & 1)
        state = ((state << 1) & 0x7FFF)
        if fb:
            state ^= 0x4599
    return state & 0x7FFF


def can_data_bits(identifier, data, rtr=0):
    """Unstuffed wire bits from SOF through the end of the data field.

    Standard (2.0A) data frame: SOF, 11-bit ID, RTR, IDE, r0, 4-bit DLC,
    then 8*dlc data bits.  All fields are MSB-first.
    """
    if not 0 <= identifier < (1 << 11):
        raise ValueError("CAN 2.0A identifier must fit in 11 bits")
    if len(data) > 8:
        raise ValueError("CAN data payload is at most 8 bytes")
    bits = [DOMINANT]  # SOF
    bits.extend((identifier >> i) & 1 for i in range(10, -1, -1))
    bits.append(rtr & 1)     # RTR (0 dominant = data frame)
    bits.append(DOMINANT)    # IDE (dominant = standard frame)
    bits.append(DOMINANT)    # r0
    dlc = len(data)
    bits.extend((dlc >> i) & 1 for i in range(3, -1, -1))
    for byte in data:
        bits.extend((byte >> i) & 1 for i in range(7, -1, -1))
    return bits


def encode_frame_bits(identifier, data, rtr=0):
    """Full wire bit sequence of one CAN 2.0A data frame, SOF through EOF.

    Returns (bits, crc, stuff_positions).  `bits` includes stuffing, the
    CRC field, CRC delimiter, ACK slot, ACK delimiter, and the 7-bit EOF.
    The ACK slot bit is recessive as transmitted (receivers overwrite it
    dominant).  stuff_positions marks wire positions inserted by stuffing.
    """
    head = can_data_bits(identifier, data, rtr)
    crc = crc15_bits(head)
    crc_bits = [(crc >> i) & 1 for i in range(14, -1, -1)]
    body = head + crc_bits

    out = []
    stuff_positions = []
    run_val = None
    run_len = 0
    for bit in body:
        out.append(bit)
        if run_val is not None and bit == run_val:
            run_len += 1
        else:
            run_val = bit
            run_len = 1
        if run_len == 5:
            out.append(1 - bit)
            stuff_positions.append(len(out) - 1)
            run_len = 0
            run_val = None
    # CRC delimiter, ACK slot, ACK delimiter, EOF x7: never stuffed.
    out.extend([RECESSIVE] * (3 + 7))
    return out, crc, stuff_positions


def decode_frame_bits(wire_bits):
    """Decode one stuffed wire-bit sequence (SOF first) into fields.

    Validates stuffing, extracts the identifier/data/CRC, verifies the
    MSB-first CRC-15, and checks the CRC delimiter.  Returns a dict;
    raises CANProtocolError on violations.
    """
    bits = list(wire_bits)
    if len(bits) < 20:
        raise CANProtocolError("CAN frame too short")
    if bits[0] != DOMINANT:
        raise CANProtocolError("CAN frame does not start with dominant SOF")

    pos = 0          # wire position
    run_val = None
    run_len = 0
    fields = []      # unstuffed bits in order

    def next_data_bit():
        nonlocal pos, run_val, run_len
        if pos >= len(bits):
            raise CANProtocolError("truncated CAN frame in stuffed region")
        bit = bits[pos]
        pos += 1
        fields.append(bit)
        if run_val is not None and bit == run_val:
            run_len += 1
        else:
            run_val = bit
            run_len = 1
        if run_len == 5:
            if pos >= len(bits):
                raise CANProtocolError("truncated CAN frame at stuff bit")
            stuff = bits[pos]
            pos += 1
            if stuff == run_val:
                raise CANProtocolError(
                    "CAN stuff error: sixth consecutive equal bit")
            run_len = 0
            run_val = None
        return bit

    # SOF(1) + ID(11) + RTR(1) + IDE(1) + r0(1) + DLC(4) = 19 fixed bits.
    head = [next_data_bit() for _ in range(19)]
    dlc = 0
    for bit in head[15:19]:
        dlc = (dlc << 1) | bit
    if dlc > 8:
        raise CANProtocolError(f"CAN DLC {dlc} out of range")
    for _ in range(8 * dlc + 15):
        next_data_bit()

    crc_rx = 0
    for bit in fields[-15:]:
        crc_rx = (crc_rx << 1) | bit
    body = fields[:-15]
    crc_calc = crc15_bits(body)
    if crc_calc != crc_rx:
        raise CANProtocolError(
            f"CAN CRC mismatch: wire 0x{crc_rx:04x} vs computed 0x{crc_calc:04x}")

    identifier = 0
    for bit in fields[1:12]:
        identifier = (identifier << 1) | bit
    rtr = fields[12]
    if fields[13] != DOMINANT:
        raise CANProtocolError("IDE bit not dominant: not a 2.0A frame")
    data = []
    payload = fields[19:-15]
    for posb in range(0, len(payload), 8):
        byte = 0
        for bit in payload[posb:posb + 8]:
            byte = (byte << 1) | bit
        data.append(byte)
    # The bit right after the CRC field must be the recessive CRC delimiter.
    if pos < len(bits) and bits[pos] != RECESSIVE:
        raise CANProtocolError("CRC delimiter not recessive")
    return {
        "identifier": identifier,
        "rtr": rtr,
        "dlc": dlc,
        "data": bytes(data),
        "crc": crc_rx,
        "wire_consumed": pos,
    }


# ----------------------------------------------------------------------
# Wired-AND single-wire bus model.
# ----------------------------------------------------------------------
class CANBus:
    """Dominant-wins wired-AND pad model for one CAN wire on a uio pin.

    `write_pin(level)` updates the external uio_in shadow.  `dut_out()` /
    `dut_oe()` sample the chip pad driver.  External agents pull dominant
    (low) by name; the recessive state is the released pull-up high.  Any
    output-enabled high drive by the DUT is an electrical violation, because
    a CAN transceiver may only ever pull the wire dominant or release it.
    """

    def __init__(self, clk, write_pin, dut_out, dut_oe):
        self.clk = clk
        self.write_pin = write_pin
        self.dut_out = dut_out
        self.dut_oe = dut_oe
        self.ext_dominant = set()
        self.line = RECESSIVE
        self.contention_errors = []
        self._resolver = None

    def start_resolver(self):
        if self._resolver is None:
            self._resolver = cocotb.start_soon(self._resolve_loop())

    def set_drive(self, owner, dominant):
        if dominant:
            self.ext_dominant.add(owner)
        else:
            self.ext_dominant.discard(owner)

    def dut_driving_dominant(self):
        out, oe = self.dut_out() & 1, self.dut_oe() & 1
        if oe and out:
            msg = "DUT actively drives CAN wire high (recessive) under OE"
            if msg not in self.contention_errors:
                self.contention_errors.append(msg)
        return oe and not out

    def sample(self):
        return DOMINANT if (self.ext_dominant or self.dut_driving_dominant()) \
            else RECESSIVE

    async def _resolve_loop(self):
        while True:
            await RisingEdge(self.clk)
            line = self.sample()
            self.line = line
            self.write_pin(line)


# ----------------------------------------------------------------------
# Independent node BFM.
# ----------------------------------------------------------------------
@dataclass
class CANFrameResult:
    identifier: int
    data: bytes
    crc: int
    rtr: int = 0
    acked: bool = False
    bit_periods: list = field(default_factory=list)
    events: list = field(default_factory=list)


class CANNodeBfm:
    """Independent CAN node: transmits conforming frames and receives/
    validates arbitrary frames off the wire, driving the ACK slot in-spec
    when the CRC validates.  All timing is generated independently.
    """

    def __init__(self, bus, owner="bfm", bit_cycles=60, sample_point=0.7):
        self.bus = bus
        self.clk = bus.clk
        self.owner = owner
        self.bit_cycles = int(bit_cycles)
        self.sample_at = max(2, int(round(bit_cycles * sample_point)))

    async def send_frame(self, identifier, data, rtr=0, wire_bits_override=None):
        """Transmit one frame and sample the ACK slot midpoint.

        `wire_bits_override` is a test-only mutation hook: it must be a
        complete encoded wire-bit sequence with the same trailer structure.
        The frame layout arguments still identify the nominal frame and the
        returned CRC, while the bits physically driven are exactly the
        override.  This lets negative tests corrupt CRC/stuffing without
        teaching the mutation or receiver logic anything about the DUT.

        Bits are driven for the full bit time; the wire is released during
        the ACK slot so a peer can acknowledge.  Implements bitwise
        arbitration release: if this node transmits recessive but observes
        dominant inside the arbitration field (SOF..RTR) it aborts at the
        next bit boundary, exactly as a real CAN node backs off.

        Returns (acked, lost_arbitration, crc).
        """
        wire_bits, crc, _ = encode_frame_bits(identifier, data, rtr)
        if wire_bits_override is not None:
            wire_bits = list(wire_bits_override)
        n = len(wire_bits)
        # The ten unstuffed trailer bits are CRC delimiter, ACK slot, ACK
        # delimiter, then EOF[6:0].  Hence the ACK slot is n-9 (not n-8).
        ack_index = n - 9
        # Unstuffed positions of the arbitration field: SOF+ID+RTR = 13
        # leading unstuffed bits.  Map each wire position to its unstuffed
        # index (stuff bits have no index).
        unstuffed_index = 0
        run_val = None
        run_len = 0
        lost = False
        acked = False
        for pos, bit in enumerate(wire_bits):
            is_stuff = False
            if run_len == 5:
                run_len = 0
                run_val = None
                is_stuff = True
            else:
                if run_val is not None and bit == run_val:
                    run_len += 1
                else:
                    run_val = bit
                    run_len = 1
                unstuffed_index += 1

            if pos == ack_index:
                self.bus.set_drive(self.owner, False)  # release for the ACK slot
            else:
                self.bus.set_drive(self.owner, bit == DOMINANT)

            if pos == ack_index:
                half = self.bit_cycles // 2
                await ClockCycles(self.clk, half)
                acked = self.bus.sample() == DOMINANT
                await ClockCycles(self.clk, self.bit_cycles - half)
            else:
                # Watch the wire during the second half of the bit: sending
                # recessive but reading dominant means another node won.
                half = self.bit_cycles // 2
                await ClockCycles(self.clk, half)
                wire_now = self.bus.sample()
                await ClockCycles(self.clk, self.bit_cycles - half)
                if (not is_stuff and bit == RECESSIVE
                        and unstuffed_index <= 13
                        and wire_now == DOMINANT):
                    lost = True
                    self.bus.set_drive(self.owner, False)
                    return acked, lost, crc
        self.bus.set_drive(self.owner, False)
        return acked, lost, crc

    async def receive_frame(self, timeout_cycles=30000, drive_ack=True):
        """Receive one frame with independent bit recovery and in-slot ACK.

        Synchronizes on the first dominant edge after recessive idle, then
        samples once per nominal bit time at the configured sample point.
        Destuffs on the fly, computes CRC-15 incrementally so the ACK
        decision is ready before the ACK slot, validates fixed frame
        structure, and records per-bit sample-to-sample periods.
        """
        bus = self.bus
        # Wait for SOF: a recessive -> dominant edge.
        prev = bus.sample()
        for _ in range(timeout_cycles):
            await ClockCycles(self.clk, 1)
            cur = bus.sample()
            if prev == RECESSIVE and cur == DOMINANT:
                break
            prev = cur
        else:
            raise CANProtocolError("CAN RX timeout waiting for SOF")

        crc = 0
        crc_rx = 0
        crc_bits = []
        run_val = None
        run_len = 0
        expect_stuff = False
        field_bits = []   # unstuffed bits in the CRC-covered region (incl SOF)
        dlc = None
        covered_total = None   # unstuffed count through end of data
        bit_periods = []
        acked_valid = False

        state = "data"    # data -> crc -> crc_delim -> ack_slot -> ack_delim -> eof
        crc_count = 0
        eof_count = 0
        periods = []

        async def advance_to_sample():
            """Advance exactly one bit time and sample at the sample point.

            Returns the sampled wire level.  The first sample is taken
            sample_at cycles after the SOF edge; subsequent samples are
            spaced bit_cycles apart.
            """
            await ClockCycles(self.clk, self.bit_cycles)
            return bus.sample()

        # Position at the SOF sample point.
        await ClockCycles(self.clk, self.sample_at - 1)
        first = True
        while True:
            if first:
                level = bus.sample()
                first = False
            else:
                level = await advance_to_sample()
            bit = level & 1

            if state in ("data", "crc"):
                if expect_stuff:
                    # This position must be the complement stuff bit.
                    if bit == run_val:
                        raise CANProtocolError(
                            "CAN stuff error: sixth consecutive equal bit")
                    expect_stuff = False
                    run_len = 0
                    run_val = None
                    continue  # stuff bit carries no data
                if state == "data":
                    crc = CANNodeBfm._crc_step(crc, bit)
                    field_bits.append(bit)
                    if len(field_bits) == 19:
                        dlc = 0
                        for b in field_bits[15:19]:
                            dlc = (dlc << 1) | b
                        if dlc > 8:
                            raise CANProtocolError(f"CAN DLC {dlc} out of range")
                        covered_total = 19 + 8 * dlc
                    if covered_total is not None and len(field_bits) >= covered_total:
                        state = "crc"
                        crc_count = 0
                else:
                    crc_bits.append(bit)
                    crc_count += 1
                    if crc_count == 15:
                        state = "crc_delim"
                # run tracking for stuffing (data and CRC bits count alike)
                if run_val is not None and bit == run_val:
                    run_len += 1
                else:
                    run_val = bit
                    run_len = 1
                if run_len == 5:
                    expect_stuff = True
            elif state == "crc_delim":
                if bit != RECESSIVE:
                    raise CANProtocolError("CAN CRC delimiter not recessive")
                crc_rx = 0
                for b in crc_bits:
                    crc_rx = (crc_rx << 1) | b
                acked_valid = (crc_rx == crc)
                state = "ack_slot"
                if drive_ack and acked_valid:
                    # The CRC delimiter sample is at bit center.  The ACK
                    # slot starts half a bit later.  Use a bounded 40-cycle
                    # pulse centered in that slot: it never intrudes into
                    # the recessive CRC delimiter, is unambiguously in-slot,
                    # and releases before the ACK delimiter.  Keeping the
                    # pulse in a child coroutine preserves the parent's
                    # exact one-sample-per-bit recovery schedule.
                    async def ack_pulse():
                        # At the CRC-delimiter sample instant there are
                        # bit_cycles-sample_at cycles left in that bit.  Use
                        # a two-cycle margin at both ends: dominant for 56
                        # cycles entirely inside the 60-cycle ACK slot.
                        margin = 2
                        await ClockCycles(
                            self.clk,
                            self.bit_cycles - self.sample_at + margin)
                        self.bus.set_drive(self.owner, True)
                        await ClockCycles(
                            self.clk, self.bit_cycles - 2 * margin)
                        self.bus.set_drive(self.owner, False)
                    cocotb.start_soon(ack_pulse())
            elif state == "ack_slot":
                # The independently scheduled pulse above owns drive/release
                # timing; this sample only advances the frame state.
                state = "ack_delim"
            elif state == "ack_delim":
                if bit != RECESSIVE:
                    raise CANProtocolError("CAN ACK delimiter not recessive")
                state = "eof"
                eof_count = 0
            elif state == "eof":
                if bit != RECESSIVE:
                    raise CANProtocolError("CAN EOF bit not recessive")
                eof_count += 1
                if eof_count >= 7:
                    break

        identifier = 0
        for b in field_bits[1:12]:
            identifier = (identifier << 1) | b
        rtr = field_bits[12]
        if field_bits[13] != DOMINANT:
            raise CANProtocolError("CAN IDE bit not dominant")
        payload = field_bits[19:]
        data = []
        for posb in range(0, len(payload), 8):
            byte = 0
            for b in payload[posb:posb + 8]:
                byte = (byte << 1) | b
            data.append(byte)
        return CANFrameResult(identifier=identifier, data=bytes(data),
                              crc=crc_rx, rtr=rtr, acked=acked_valid,
                              bit_periods=periods)

    @staticmethod
    def _crc_step(state, bit):
        top = (state >> 14) & 1
        fb = top ^ (bit & 1)
        state = ((state << 1) & 0x7FFF)
        if fb:
            state ^= 0x4599
        return state & 0x7FFF
