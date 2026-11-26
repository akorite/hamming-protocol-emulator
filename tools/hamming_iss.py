# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0
"""Cycle-accurate Golden ISS for one Hamming protocol core.

`HammingISS(sram0_words, sram1_words=None, ...)` models a single 16-bit
protocol core plus its 8 x 32-bit direct-mapped loop cache against the
shared 20-bit global timer.  `step(pin_inputs=None)` advances the core by
exactly one clock cycle; the global timer advances with it.

Timing model (per the KTD2/KTD3 bounds in docs/plans):

* Instruction fetch through the loop cache: hit = 1 cycle total
  (fetch+execute), miss = `imem_miss_cycles` (3..7, default 4) total.
* All non-memory instructions execute in the fetch-complete cycle
  (single-issue, 1.0 IPC inside a cached loop).
* `ld_dmem_*`/`st_dmem_*` add an SRAM 1 access stall after issue: reads
  `dmem_read_cycles` (3..8, default 5), write grant `dmem_write_cycles`
  (<= 6, default 3).  `ld_imem` uses the SRAM 0 path (`imem_miss_cycles`).
* `csr_rd`/`csr_wr` never add stalls.
* `wait_phase` stalls until `signed20(timer - deadline) >= 0`; a deadline
  strictly in the past sets sticky `deadline_missed` and does not stall.
* `sync_event` stalls until `(EV_FLAGS[3:0] & mask) != 0` or `timeout`
  cycles elapse, then re-anchors to the wakeup cycle.

Host-side helpers: `sram0_write()` models a host SRAM 0 commit (invalidates
the loop cache and drops an in-flight fill), `host_csr_write()` bypasses the
core's private-bank write restriction, `icache_inv_all()` is the RTL
`icache_inv_all` strobe.  `ev_flags`, `timer`, `sram0`, `sram1`, `csr`,
`lut_tt`, `lut_sel` and `bcu` are plain attributes the test may read/stub.
"""

from collections import deque

from . import hamming_isa as isa

TIMER_MASK = isa.TIMER_MASK
PC_MASK = isa.PC_MASK


class LoopCache:
    """8 x 32-bit direct-mapped instruction loop cache."""

    def __init__(self):
        self.tag = [None] * isa.ICACHE_LINES
        self.data = [0] * isa.ICACHE_LINES

    def index(self, waddr):
        return waddr & (isa.ICACHE_LINES - 1)

    def hit(self, waddr):
        return self.tag[self.index(waddr)] == waddr

    def read(self, waddr):
        return self.data[self.index(waddr)]

    def fill(self, waddr, word):
        idx = self.index(waddr)
        self.tag[idx] = waddr
        self.data[idx] = word & 0xFFFFFFFF

    def inv_all(self):
        self.tag = [None] * isa.ICACHE_LINES


class BcuModel:
    """Minimal BCU companion model: shadow buffers + status/CRC state.

    Tests stub these fields directly (e.g. push RX data into `rx_queue`,
    set `stat` bits, preload `crc_state`).
    """

    def __init__(self):
        self.tx_shadow = 0
        self.tx_pushed = 0
        self.rx_shadow = 0
        self.rx_queue = deque()
        self.ctrl = 0
        self.stat = 0                 # HAM_BCU_STAT_* bit positions
        self.crc_state = 0            # 32-bit CRC/LFSR state
        self.crc_poly = 0


class HammingISS:
    """Cycle-accurate instruction-set simulator for one Hamming core."""

    def __init__(self, sram0, sram1=None, *, core_id=0,
                 imem_miss_cycles=isa.IMEM_MISS_DFLT,
                 dmem_read_cycles=isa.DMEM_RD_DFLT,
                 dmem_write_cycles=isa.DMEM_WR_DFLT,
                 timer_init=0):
        if not isa.IMEM_MISS_MIN <= imem_miss_cycles <= isa.IMEM_MISS_MAX:
            raise ValueError("imem_miss_cycles must be 3..7")
        if not isa.DMEM_RD_MIN <= dmem_read_cycles <= isa.DMEM_RD_MAX:
            raise ValueError("dmem_read_cycles must be 3..8")
        if not isa.DMEM_WR_MIN <= dmem_write_cycles <= isa.DMEM_WR_MAX:
            raise ValueError("dmem_write_cycles must be <= 6")
        self.imem_miss_cycles = imem_miss_cycles
        self.dmem_read_cycles = dmem_read_cycles
        self.dmem_write_cycles = dmem_write_cycles
        self.core_id = core_id
        self.sram0 = ([w & 0xFFFFFFFF for w in sram0] +
                      [0] * (isa.SRAM0_WORDS - len(sram0)))[:isa.SRAM0_WORDS]
        if sram1 is None:
            sram1 = []
        self.sram1 = ([w & 0xFFFFFFFF for w in sram1] +
                      [0] * (isa.SRAM1_WORDS - len(sram1)))[:isa.SRAM1_WORDS]
        self.icache = LoopCache()
        self.bcu = BcuModel()
        self.reset()
        self.timer = timer_init & TIMER_MASK

    # ------------------------------------------------------------------ #
    # state                                                               #
    # ------------------------------------------------------------------ #

    def reset(self):
        self.regs = [0] * isa.N_REGS
        self.pc = 0
        self.z = 0
        self.c = 0
        self.eq = 0
        self.anchor = 0
        self.deadline_missed = 0
        self.core_out = 0
        self.core_oe = 0
        self.halted = False
        self.cycle = 0
        self.timer = 0
        self.ev_flags = 0
        self.pin_inputs = 0
        self.call_stack = []
        self.csr = [0] * 256
        self.lut_tt = [0] * 16
        self.lut_sel = [(0, 1, 2, 3)] * 16
        self.lut_oreg = 0
        self.icache.inv_all()
        self.retired = 0
        self.last_instr = None
        # fetch / stall micro-state
        self._fetch_left = 0
        self._miss_waddr = 0
        self._stall_kind = None          # "wait" | "sync" | "ld" | "st" | None
        self._stall_left = 0
        self._wait_deadline = 0
        self._sync_mask = 0
        self._sync_timeout = 0
        self._sync_elapsed = 0
        self._pending = None
        self._npc = 0

    # ------------------------------------------------------------------ #
    # host-side helpers                                                   #
    # ------------------------------------------------------------------ #

    def icache_inv_all(self):
        """RTL `icache_inv_all` strobe: drop lines and any in-flight fill."""
        self.icache.inv_all()
        self._fetch_left = 0

    def sram0_write(self, waddr, word):
        """Host SRAM 0 write commit: writes the word and invalidates caches."""
        self.sram0[waddr & 0x1FF] = word & 0xFFFFFFFF
        self.icache_inv_all()

    def host_csr_write(self, addr, value):
        """Host SPI CSR write (no core private-bank restriction)."""
        value &= 0xFFFF
        addr &= 0xFF
        self.csr[addr] = value
        if addr == isa.CSR["EV_SET"]:
            self.ev_flags = (self.ev_flags | value) & 0xFF
        elif addr == isa.CSR["EV_CLR"]:
            self.ev_flags &= (~value) & 0xFF
        elif addr == isa.CSR["EV_FLAGS"]:
            self.ev_flags = value & 0xFF
        elif isa.CSR_LUT_TT_BASE <= addr < isa.CSR_LUT_TT_BASE + 16:
            self.lut_tt[addr - isa.CSR_LUT_TT_BASE] = value
        elif isa.CSR_LUT_SEL_BASE <= addr < isa.CSR_LUT_SEL_BASE + 16:
            k = addr - isa.CSR_LUT_SEL_BASE
            self.lut_sel[k] = tuple((value >> (4 * j)) & 0xF for j in range(4))
        elif addr == isa.CSR_LUT_OREG:
            self.lut_oreg = value

    def lut_eval(self, inputs):
        """Evaluate all 16 LUT outputs against the 16-bit input vector.

        Returns a 16-bit vector where bit k is LUT k's output.  Tests may
        monkeypatch this method (it is consulted by `lut_read` and may be
        wired into the pin/event model of a larger sim).
        """
        inputs &= 0xFFFF
        out = 0
        for k in range(16):
            sel = self.lut_sel[k]
            idx = 0
            for j in range(4):
                idx |= ((inputs >> sel[j]) & 1) << j
            out |= ((self.lut_tt[k] >> idx) & 1) << k
        return out

    # ------------------------------------------------------------------ #
    # main cycle step                                                     #
    # ------------------------------------------------------------------ #

    def step(self, pin_inputs=None):
        """Advance the core by exactly one clock cycle."""
        if pin_inputs is not None:
            self.pin_inputs = pin_inputs & 0xFFFF
        if not self.halted:
            if self._stall_kind is not None:
                self._step_stall()
            elif self._fetch_left:
                # SRAM 0 miss in flight; the fill lands on the last cycle.
                self._fetch_left -= 1
                if self._fetch_left == 0:
                    self.icache.fill(self._miss_waddr,
                                     self.sram0[self._miss_waddr])
                    self._exec(self._fetch_halfword(self.pc))
            else:
                waddr = self.pc >> 1
                if self.icache.hit(waddr):
                    self._exec(self._fetch_halfword(self.pc))
                else:
                    self._miss_waddr = waddr
                    self._fetch_left = self.imem_miss_cycles - 1
        self.cycle += 1
        self.timer = (self.timer + 1) & TIMER_MASK

    def run(self, cycles, pin_inputs=None):
        for _ in range(cycles):
            self.step(pin_inputs)

    # ------------------------------------------------------------------ #
    # fetch helpers                                                       #
    # ------------------------------------------------------------------ #

    def _fetch_halfword(self, pc):
        waddr = (pc >> 1) & 0x1FF
        word = (self.icache.read(waddr) if self.icache.hit(waddr)
                else self.sram0[waddr])
        return (word >> (16 * (pc & 1))) & 0xFFFF

    # ------------------------------------------------------------------ #
    # stall engines                                                       #
    # ------------------------------------------------------------------ #

    def _step_stall(self):
        kind = self._stall_kind
        if kind == "wait":
            # Release the cycle the signed compare goes non-negative.
            if isa.signed20(self.timer - self._wait_deadline) >= 0:
                self._stall_kind = None
                self.pc = self._npc
        elif kind == "sync":
            self._sync_elapsed += 1
            if ((self.ev_flags & self._sync_mask) != 0 or
                    self._sync_elapsed >= self._sync_timeout):
                self.anchor = self.timer  # re-anchor to the wakeup cycle
                self._stall_kind = None
                self.pc = self._npc
        else:  # "ld" / "st": bounded SRAM 1 (or SRAM 0 table) access
            self._stall_left -= 1
            if self._stall_left <= 0:
                self._commit_mem()
                self._stall_kind = None
                self.pc = self._npc

    def _commit_mem(self):
        p = self._pending
        if p is None:
            return
        if p["kind"] == "ld":
            word = self.sram1[p["waddr"]]
            if p["lane"] is None:
                self.regs[p["rt"]] = word & 0xFFFF
            else:
                self.regs[p["rt"]] = (word >> (8 * p["lane"])) & 0xFF
        elif p["kind"] == "st":
            word = self.sram1[p["waddr"]]
            if p["lane"] is None:
                word = p["data"]
            else:
                mask = 0xFF << (8 * p["lane"])
                word = (word & ~mask & 0xFFFFFFFF) | \
                       ((p["data"] << (8 * p["lane"])) & mask)
            self.sram1[p["waddr"]] = word & 0xFFFFFFFF
        elif p["kind"] == "ld_imem":
            word = self.sram0[p["waddr"]]
            self.regs[p["rt"]] = (word >> (16 * p["lane"])) & 0xFFFF
        self._pending = None

    # ------------------------------------------------------------------ #
    # execute (runs in the fetch-complete cycle)                          #
    # ------------------------------------------------------------------ #

    def _exec(self, instr):
        self.retired += 1
        self.last_instr = (self.pc, instr)
        self._npc = (self.pc + 1) & PC_MASK
        dec = isa.decode(instr)
        if dec is None:
            self.pc = self._npc  # illegal/reserved words act as NOP
            return
        mnem, ops = dec
        handler = getattr(self, "_op_" + mnem, None)
        if handler is None:
            raise AssertionError(f"decoder/ISS gap for {mnem!r}")
        handler(ops)
        if self._stall_kind is None:
            self.pc = self._npc

    def _set_z(self, res):
        self.z = 1 if (res & 0xFFFF) == 0 else 0

    # ---- ALU --------------------------------------------------------- #

    def _alu_common(self, ops, res, carry=None):
        rd = ops["rd"]
        if carry is not None:
            self.c = carry
        self._set_z(res)
        self.regs[rd] = res & 0xFFFF

    def _op_mov(self, ops):
        self._alu_common(ops, self.regs[ops["rs"]])

    def _op_add(self, ops):
        a, b = self.regs[ops["rd"]], self.regs[ops["rs"]]
        self._alu_common(ops, a + b, carry=1 if a + b > 0xFFFF else 0)

    def _op_sub(self, ops):
        a, b = self.regs[ops["rd"]], self.regs[ops["rs"]]
        self._alu_common(ops, a - b, carry=1 if a < b else 0)

    def _op_and(self, ops):
        self._alu_common(ops, self.regs[ops["rd"]] & self.regs[ops["rs"]])

    def _op_or(self, ops):
        self._alu_common(ops, self.regs[ops["rd"]] | self.regs[ops["rs"]])

    def _op_xor(self, ops):
        self._alu_common(ops, self.regs[ops["rd"]] ^ self.regs[ops["rs"]])

    def _op_shl(self, ops):
        a = self.regs[ops["rd"]]
        self._alu_common(ops, (a << 1) & 0xFFFF, carry=(a >> 15) & 1)

    def _op_shr(self, ops):
        a = self.regs[ops["rd"]]
        self._alu_common(ops, a >> 1, carry=a & 1)

    def _op_cmp(self, ops):
        a, b = self.regs[ops["rd"]], self.regs[ops["rs"]]
        self.c = 1 if a < b else 0
        self._set_z(a - b)
        self.eq = 1 if a == b else 0

    def _op_not(self, ops):
        self._alu_common(ops, ~self.regs[ops["rs"]])

    def _op_inc(self, ops):
        self._alu_common(ops, self.regs[ops["rd"]] + 1)

    def _op_dec(self, ops):
        self._alu_common(ops, self.regs[ops["rd"]] - 1)

    # ---- LDI8 --------------------------------------------------------- #

    def _op_ldi8(self, ops):
        rd, imm = ops["rd"], ops["imm8"]
        if ops["half"]:
            self.regs[rd] = (self.regs[rd] & 0x00FF) | (imm << 8)
        else:
            self.regs[rd] = (self.regs[rd] & 0xFF00) | imm

    # ---- PIN ---------------------------------------------------------- #

    def _op_pin_out(self, ops):
        m, v = ops["mask4"], ops["val4"]
        self.core_out = (self.core_out & (~m & 0xF)) | (v & m)

    def _op_pin_oe(self, ops):
        m, v = ops["mask4"], ops["val4"]
        self.core_oe = (self.core_oe & (~m & 0xF)) | (v & m)

    def _op_pin_in(self, ops):
        self.regs[ops["rd"]] = (self.pin_inputs >> ops["pin"]) & 1

    def _op_lut_read(self, ops):
        self.regs[ops["rd"]] = (self.lut_eval(self.pin_inputs) >> ops["lut"]) & 1

    # ---- TIMING ------------------------------------------------------- #

    def _op_wait_phase(self, ops):
        delta = self.regs[ops["rs"]] if "rs" in ops else ops["imm8"]
        delta &= isa.MAX_WAIT_DELTA
        deadline = (self.anchor + delta) & TIMER_MASK
        diff = isa.signed20(self.timer - deadline)
        if diff > 0:
            self.deadline_missed = 1       # overshoot: no stall
        elif diff < 0:
            self._stall_kind = "wait"
            self._wait_deadline = deadline
        # diff == 0: exactly on time -> no stall, no miss.

    def _op_sync_event(self, ops):
        mask = ops["mask4"]
        timeout = self.regs[ops["rt"]]
        if (self.ev_flags & mask) != 0 or timeout == 0:
            self.anchor = self.timer       # immediate wake / pure poll
        else:
            self._stall_kind = "sync"
            self._sync_mask = mask
            self._sync_timeout = timeout
            self._sync_elapsed = 0

    def _op_set_anchor(self, ops):
        self.anchor = (self.timer + ops["imm8"]) & TIMER_MASK

    def _op_read_timer(self, ops):
        if ops["sel"] == isa.READ_TIMER_HI:
            self.regs[ops["rd"]] = ((self.deadline_missed << 15) |
                                    ((self.timer >> 16) & 0xF))
        else:
            self.regs[ops["rd"]] = self.timer & 0xFFFF

    # ---- BCU ---------------------------------------------------------- #

    def _op_bcu_tx(self, ops):
        self.bcu.tx_shadow = self.regs[ops["rs"]]
        self.bcu.tx_pushed += 1

    def _op_bcu_rx(self, ops):
        if self.bcu.rx_queue:
            self.regs[ops["rd"]] = self.bcu.rx_queue.popleft() & 0xFFFF
        else:
            self.regs[ops["rd"]] = self.bcu.rx_shadow & 0xFFFF

    def _op_bcu_ctrl(self, ops):
        self.bcu.ctrl = ops["imm8"]

    def _op_bcu_stat(self, ops):
        self.regs[ops["rd"]] = self.bcu.stat & 0xFFFF

    def _op_bcu_crc_rd(self, ops):
        if ops["sel"] == isa.BCU_CRC_HI:
            self.regs[ops["rd"]] = (self.bcu.crc_state >> 16) & 0xFFFF
        else:
            self.regs[ops["rd"]] = self.bcu.crc_state & 0xFFFF

    # ---- MEM ---------------------------------------------------------- #

    def _mem_common(self, ops):
        return (self.regs[ops["ra"]] + ops["imm2"]) & 0xFFFF

    def _op_ld_dmem_w(self, ops):
        waddr = self._mem_common(ops) & 0x1FF
        self._pending = {"kind": "ld", "waddr": waddr, "lane": None,
                         "rt": ops["rt"]}
        self._stall_kind = "ld"
        self._stall_left = self.dmem_read_cycles

    def _op_st_dmem_w(self, ops):
        waddr = self._mem_common(ops) & 0x1FF
        self._pending = {"kind": "st", "waddr": waddr, "lane": None,
                         "data": self.regs[ops["rt"]]}
        self._stall_kind = "st"
        self._stall_left = self.dmem_write_cycles

    def _op_ld_dmem_b(self, ops):
        ea = self._mem_common(ops)
        self._pending = {"kind": "ld", "waddr": (ea >> 2) & 0x1FF,
                         "lane": ea & 3, "rt": ops["rt"]}
        self._stall_kind = "ld"
        self._stall_left = self.dmem_read_cycles

    def _op_st_dmem_b(self, ops):
        ea = self._mem_common(ops)
        self._pending = {"kind": "st", "waddr": (ea >> 2) & 0x1FF,
                         "lane": ea & 3, "data": self.regs[ops["rt"]] & 0xFF}
        self._stall_kind = "st"
        self._stall_left = self.dmem_write_cycles

    def _op_ld_imem(self, ops):
        ea = self._mem_common(ops) & PC_MASK  # 16-bit entry index, 0..1023
        self._pending = {"kind": "ld_imem", "waddr": (ea >> 1) & 0x1FF,
                         "lane": ea & 1, "rt": ops["rt"]}
        self._stall_kind = "ld"
        self._stall_left = self.imem_miss_cycles

    # ---- CSR ---------------------------------------------------------- #

    def _op_csr_rd(self, ops):
        self.regs[ops["rd"]] = self._csr_read(ops["addr"])

    def _op_csr_wr(self, ops):
        self._csr_write(ops["addr"], self.regs[ops["rs"]])

    def _csr_read(self, addr):
        if addr == isa.CSR["EV_FLAGS"]:
            return self.ev_flags
        if addr == isa.CSR["TIMER_LO"]:
            return self.timer & 0xFFFF
        if addr == isa.CSR["TIMER_HI"]:
            return (self.timer >> 16) & 0xF
        if (isa.CSR_CORE_BANK <= addr < 0x80 and
                (addr & 0xF) == isa.CORE_CSR["CORE_ID"]):
            return self.core_id
        off = addr - (isa.CSR_CORE_BANK + 16 * self.core_id)
        if 0 <= off < 16:
            if off == isa.CORE_CSR["CORE_STATUS"]:
                return (self.z | (self.c << 1) | (self.eq << 2) |
                        (self.deadline_missed << 3) | (int(self.halted) << 4))
            if off == isa.CORE_CSR["CORE_PC"]:
                return self.pc
            if off == isa.CORE_CSR["CORE_OUT"]:
                return self.core_out
            if off == isa.CORE_CSR["CORE_OE"]:
                return self.core_oe
            if off == isa.CORE_CSR["BCU_STAT_SNAP"]:
                return self.bcu.stat & 0xFFFF
            if off == isa.CORE_CSR["BCU_RX_DATA"]:
                return self.bcu.rx_shadow
            if off == isa.CORE_CSR["BCU_CRC_STATE_LO"]:
                return self.bcu.crc_state & 0xFFFF
            if off == isa.CORE_CSR["BCU_CRC_STATE_HI"]:
                return (self.bcu.crc_state >> 16) & 0xFFFF
            if off == isa.CORE_CSR["BCU_CRC_POLY_LO"]:
                return self.bcu.crc_poly & 0xFFFF
            if off == isa.CORE_CSR["BCU_CRC_POLY_HI"]:
                return (self.bcu.crc_poly >> 16) & 0xFFFF
        return self.csr[addr]

    def _csr_write(self, addr, val):
        val &= 0xFFFF
        if addr == isa.CSR["EV_SET"]:
            self.ev_flags = (self.ev_flags | val) & 0xFF
            return
        if addr == isa.CSR["EV_CLR"]:
            self.ev_flags &= (~val) & 0xFF
            return
        base = isa.CSR_CORE_BANK + 16 * self.core_id
        if base <= addr < base + 16:
            off = addr - base
            if off == isa.CORE_CSR["BCU_CRC_STATE_LO"]:
                self.bcu.crc_state = (self.bcu.crc_state & 0xFFFF0000) | val
            elif off == isa.CORE_CSR["BCU_CRC_STATE_HI"]:
                self.bcu.crc_state = (self.bcu.crc_state & 0xFFFF) | (val << 16)
            elif off == isa.CORE_CSR["BCU_RX_DATA"]:
                self.bcu.rx_shadow = val
            elif off == isa.CORE_CSR["BCU_CRC_POLY_LO"]:
                self.bcu.crc_poly = (self.bcu.crc_poly & 0xFFFF0000) | val
            elif off == isa.CORE_CSR["BCU_CRC_POLY_HI"]:
                self.bcu.crc_poly = (self.bcu.crc_poly & 0xFFFF) | (val << 16)
            if off != isa.CORE_CSR["CORE_ID"]:
                self.csr[addr] = val
        # Writes outside the owned bank (other banks, LUT/router/global
        # config, read-only status) are dropped per the KTD11 ownership rule.

    # ---- control flow ------------------------------------------------- #

    def _op_jmp(self, ops):
        self._npc = ops["pc10"]

    def _op_jz(self, ops):
        if self.z:
            self._npc = ops["pc10"]

    def _op_jnz(self, ops):
        if not self.z:
            self._npc = ops["pc10"]

    def _op_jev(self, ops):
        if (self.ev_flags & ops["mask4"] & 0xF) != 0:
            self._npc = self.regs[ops["rs"]] & 0x3FF

    def _op_call(self, ops):
        self.call_stack.append(self._npc)
        if len(self.call_stack) > 2:
            self.call_stack.pop(0)          # 2-deep: drop oldest
        self._npc = ops["pc10"]

    def _op_ret(self, ops):
        self._npc = self.call_stack.pop() if self.call_stack else 0

    def _op_halt(self, ops):
        self.halted = True

    def _op_nop(self, ops):
        pass
