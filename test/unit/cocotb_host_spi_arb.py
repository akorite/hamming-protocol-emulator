# SPDX-FileCopyrightText: 2026 akorite
# SPDX-License-Identifier: Apache-2.0

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer

CMD_WR_SRAM0 = 0x01
CMD_WR_SRAM1 = 0x02
CMD_WR_CSR = 0x03
CMD_RD_SRAM0 = 0x81
CMD_RD_SRAM1 = 0x82
CMD_RD_CSR = 0x83

CSR_CTRL_RUN = 0x00
CSR_EV_FLAGS = 0x01
CSR_EV_SET = 0x02
CSR_EV_CLR = 0x03
CSR_RLE_BASE = 0x06
CSR_PIN_FILT_EN = 0x0A
CSR_OPEN_DRAIN_EN = 0x0B
CSR_PIN_ROUTER0 = 0x20
CSR_CORE_BANK = 0x40
CSR_LUT_TT_BASE = 0x80
CSR_LUT_SEL_BASE = 0x90
CSR_LUT_OREG = 0xA0


def u(sig):
    value = sig.value
    if hasattr(value, "to_unsigned"):
        return value.to_unsigned()
    return int(value)


def set_slice(word, index, width, value):
    mask = ((1 << width) - 1) << (index * width)
    return (word & ~mask) | ((value & ((1 << width) - 1)) << (index * width))


def get_slice(word, index, width):
    return (word >> (index * width)) & ((1 << width) - 1)


async def sample_after_rise(clk):
    await RisingEdge(clk)
    await Timer(1, unit="ns")


async def reset_direct_arb(dut):
    dut.req.value = 0
    dut.we.value = 0
    dut.addr_flat.value = 0
    dut.wdata_flat.value = 0
    dut.wmask_flat.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 6)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def drive_arb_request(dut, client, write, addr, data=0, mask=0xFFFFFFFF):
    nclients = len(dut.req.value)
    req = u(dut.req) | (1 << client)
    we = u(dut.we)
    if write:
        we |= 1 << client
    else:
        we &= ~(1 << client)
    addrs = set_slice(u(dut.addr_flat), client, 9, addr)
    wdata = set_slice(u(dut.wdata_flat), client, 32, data)
    wmask = set_slice(u(dut.wmask_flat), client, 32, mask)
    dut.req.value = req
    dut.we.value = we
    dut.addr_flat.value = addrs
    dut.wdata_flat.value = wdata
    dut.wmask_flat.value = wmask
    return nclients


async def clear_arb_request(dut, client):
    dut.req.value = u(dut.req) & ~(1 << client)


async def accept_arb_request(dut, client, write, addr, data=0, mask=0xFFFFFFFF, max_wait=12):
    """Hold one client request until its combinational grant, then accept it
    at the next rising edge. Returns the number of full cycles waited before
    the accepting edge (0 means grant was immediate)."""
    await FallingEdge(dut.clk)
    await drive_arb_request(dut, client, write, addr, data, mask)
    waited = 0
    for _ in range(max_wait):
        await Timer(1, unit="ns")
        if (u(dut.grant) >> client) & 1:
            await RisingEdge(dut.clk)
            await FallingEdge(dut.clk)
            await clear_arb_request(dut, client)
            return waited
        await RisingEdge(dut.clk)
        await FallingEdge(dut.clk)
        waited += 1
    raise AssertionError(f"client {client}: no grant within {max_wait} cycles")


async def arb_write(dut, client, addr, data, mask=0xFFFFFFFF):
    return await accept_arb_request(dut, client, 1, addr, data, mask)


async def arb_read(dut, client, addr):
    wait = await accept_arb_request(dut, client, 0, addr)
    latency = None
    data = None
    for cyc in range(1, 10):
        await sample_after_rise(dut.clk)
        if (u(dut.rvalid) >> client) & 1:
            latency = cyc
            data = u(dut.rdata)
            break
    assert latency is not None, f"client {client}: read never returned"
    return wait, latency, data


@cocotb.test()
async def test_arb_single_read_write_latency(dut):
    """Uncontended read latency is exactly three cycles from acceptance, and
    a write commit pulse appears at the macro write edge one cycle later."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_direct_arb(dut)

    await arb_write(dut, 0, 0x012, 0xDEADBEEF, 0xFFFFFFFF)
    await sample_after_rise(dut.clk)
    assert u(dut.write_commit) == 1, "write_commit did not pulse at macro write edge"
    await sample_after_rise(dut.clk)
    assert u(dut.write_commit) == 0, "write_commit is not a single-cycle pulse"

    wait, latency, data = await arb_read(dut, 0, 0x012)
    assert wait == 0
    assert latency == 3, f"uncontended read latency {latency}, want 3"
    assert data == 0xDEADBEEF, f"read {data:#010x}"


@cocotb.test()
async def test_arb_byte_lanes_and_end_addresses(dut):
    """Active-high bit masks update only selected byte lanes at the first and
    last logical SRAM addresses."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_direct_arb(dut)

    for addr in (0x000, 0x1FF):
        await arb_write(dut, 1, addr, 0xA5A5A5A5, 0xFFFFFFFF)
        await arb_write(dut, 1, addr, 0x3C000000, 0xFF000000)
        await arb_write(dut, 1, addr, 0x0000005A, 0x000000FF)
        _, _, got = await arb_read(dut, 1, addr)
        assert got == 0x3CA5A55A, f"addr {addr:#x}: got {got:#010x}"


@cocotb.test()
async def test_arb_round_robin_ii1_fairness(dut):
    """With every client continuously requesting, arbitration sustains one
    grant per cycle in strict round-robin order with no starvation."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_direct_arb(dut)

    nclients = len(dut.req.value)
    dut.we.value = (1 << nclients) - 1
    dut.req.value = (1 << nclients) - 1
    addr_flat = 0
    wdata_flat = 0
    wmask_flat = 0
    for client in range(nclients):
        addr_flat = set_slice(addr_flat, client, 9, client)
        wdata_flat = set_slice(wdata_flat, client, 32, 0x1000 + client)
        wmask_flat = set_slice(wmask_flat, client, 32, 0xFFFFFFFF)
    dut.addr_flat.value = addr_flat
    dut.wdata_flat.value = wdata_flat
    dut.wmask_flat.value = wmask_flat

    observed = []
    for _ in range(nclients * 5):
        await FallingEdge(dut.clk)
        await Timer(1, unit="ns")
        g = u(dut.grant)
        assert g != 0 and (g & (g - 1)) == 0, f"grant not one-hot: {g:#x}"
        observed.append(g.bit_length() - 1)
        await RisingEdge(dut.clk)

    first = observed[:nclients]
    assert first == list(range(nclients)), f"initial rotation {first}"
    for pos, client in enumerate(first):
        slots = [i for i, c in enumerate(observed) if c == client]
        gaps = [b - a for a, b in zip(slots, slots[1:])]
        assert gaps and all(g == nclients for g in gaps), (
            f"client {client} gaps {gaps}, want all {nclients}"
        )


@cocotb.test()
async def test_arb_random_exclusion_stress(dut):
    """Ten thousand randomized request cycles never assert WEN and REN
    together; the X-initialized macro model also hard-fails on a violation."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_direct_arb(dut)
    rng = random.Random(0xA321)
    nclients = len(dut.req.value)

    for cycle in range(10_000):
        await FallingEdge(dut.clk)
        dut.req.value = rng.randrange(1 << nclients)
        dut.we.value = rng.randrange(1 << nclients)
        dut.addr_flat.value = rng.getrandbits(nclients * 9)
        dut.wdata_flat.value = rng.getrandbits(nclients * 32)
        dut.wmask_flat.value = rng.getrandbits(nclients * 32)
        await Timer(1, unit="ns")
        if u(dut.men_r):
            assert not (u(dut.wen_r) and u(dut.ren_r)), (
                f"WEN/REN overlap at cycle {cycle}"
            )
        await RisingEdge(dut.clk)


async def init_harness_inputs(dut):
    for name in (
        "core_we", "core_waddr_flat", "core_wdata_flat", "core_raddr_flat",
        "core_status_flat", "core_pc_flat", "core_out_flat", "core_oe_flat",
        "bcu_rx_data_flat", "bcu_stat_flat", "rle_wr_ptr", "rle_status",
        "sram0_ext_req", "sram0_ext_we", "sram0_ext_addr_flat",
        "sram0_ext_wdata_flat", "sram0_ext_wmask_flat",
        "sram1_ext_req", "sram1_ext_we", "sram1_ext_addr_flat",
        "sram1_ext_wdata_flat", "sram1_ext_wmask_flat",
    ):
        getattr(dut, name).value = 0


async def reset_harness(dut, ena=1):
    await init_harness_inputs(dut)
    dut.host_cs_n.value = 1
    dut.host_sclk.value = 0
    dut.host_mosi.value = 0
    dut.ena.value = ena
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)


async def spi_cs_start(dut):
    dut.host_sclk.value = 0
    await ClockCycles(dut.clk, 2)
    dut.host_cs_n.value = 0
    await ClockCycles(dut.clk, 6)


async def spi_cs_end(dut):
    dut.host_sclk.value = 0
    await ClockCycles(dut.clk, 4)
    dut.host_cs_n.value = 1
    await ClockCycles(dut.clk, 10)


async def spi_byte(dut, byte, half=5):
    miso = 0
    for bitno in range(7, -1, -1):
        dut.host_mosi.value = (byte >> bitno) & 1
        await ClockCycles(dut.clk, half)
        # Mode 0 samples MISO on the rising edge. Sample at the end of the
        # low half-period, before driving that edge, so a late preload/shift
        # cannot be hidden by sampling near the following falling edge.
        miso = (miso << 1) | u(dut.host_miso)
        dut.host_sclk.value = 1
        await ClockCycles(dut.clk, half)
        dut.host_sclk.value = 0
    return miso


async def spi_write_burst(dut, cmd, addr, words, width, half=5, end=True):
    await spi_cs_start(dut)
    await spi_byte(dut, cmd, half)
    await spi_byte(dut, (addr >> 8) & 0xFF, half)
    await spi_byte(dut, addr & 0xFF, half)
    nbytes = width // 8
    for word in words:
        for byte_index in range(nbytes - 1, -1, -1):
            await spi_byte(dut, (word >> (byte_index * 8)) & 0xFF, half)
    if end:
        await spi_cs_end(dut)


async def spi_read_burst(dut, cmd, addr, count, width, half=5):
    await spi_cs_start(dut)
    await spi_byte(dut, cmd, half)
    await spi_byte(dut, (addr >> 8) & 0xFF, half)
    await spi_byte(dut, addr & 0xFF, half)
    await spi_byte(dut, 0x00, half)  # mandatory dummy byte
    nbytes = width // 8
    out = []
    for _ in range(count):
        word = 0
        for _byte in range(nbytes):
            word = (word << 8) | await spi_byte(dut, 0x00, half)
        out.append(word)
    await spi_cs_end(dut)
    return out


async def spi_abort_after_bits(dut, cmd, addr, bits, half=5):
    """Send command/address and then only the leading `bits` of a data word."""
    await spi_cs_start(dut)
    await spi_byte(dut, cmd, half)
    await spi_byte(dut, (addr >> 8) & 0xFF, half)
    await spi_byte(dut, addr & 0xFF, half)
    for bitno in range(bits - 1, -1, -1):
        dut.host_mosi.value = (0x55 >> (bitno % 8)) & 1
        await ClockCycles(dut.clk, half)
        dut.host_sclk.value = 1
        await ClockCycles(dut.clk, half)
        dut.host_sclk.value = 0
    await spi_cs_end(dut)


async def core_write(dut, core, addr, data):
    await FallingEdge(dut.clk)
    dut.core_we.value = 1 << core
    waddr = 0
    wdata = 0
    for c in range(4):
        waddr = set_slice(waddr, c, 8, addr if c == core else 0)
        wdata = set_slice(wdata, c, 16, data if c == core else 0)
    dut.core_waddr_flat.value = waddr
    dut.core_wdata_flat.value = wdata
    await ClockCycles(dut.clk, 1)
    await FallingEdge(dut.clk)
    dut.core_we.value = 0
    dut.core_waddr_flat.value = 0
    dut.core_wdata_flat.value = 0
    await ClockCycles(dut.clk, 1)


async def core_write_two(dut, core_a, addr_a, data_a, core_b, addr_b, data_b):
    await FallingEdge(dut.clk)
    dut.core_we.value = (1 << core_a) | (1 << core_b)
    waddr = 0
    wdata = 0
    waddr = set_slice(waddr, core_a, 8, addr_a)
    waddr = set_slice(waddr, core_b, 8, addr_b)
    wdata = set_slice(wdata, core_a, 16, data_a)
    wdata = set_slice(wdata, core_b, 16, data_b)
    dut.core_waddr_flat.value = waddr
    dut.core_wdata_flat.value = wdata
    await ClockCycles(dut.clk, 1)
    await FallingEdge(dut.clk)
    dut.core_we.value = 0
    dut.core_waddr_flat.value = 0
    dut.core_wdata_flat.value = 0
    await ClockCycles(dut.clk, 1)


@cocotb.test()
async def test_host_spi_csr_config_and_core_ownership(dut):
    """Host SPI writes and reads back global, router, and LUT CSRs; core write
    ownership is private-bank-only and simultaneous EV set beats clear."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_harness(dut)

    await spi_write_burst(dut, CMD_WR_CSR, CSR_PIN_FILT_EN, [0x00A5], 16)
    assert u(dut.pin_filt_en) == 0xA5
    got = await spi_read_burst(dut, CMD_RD_CSR, CSR_PIN_FILT_EN, 1, 16)
    assert got == [0x00A5], f"CSR readback {got}"

    await spi_write_burst(dut, CMD_WR_CSR, CSR_OPEN_DRAIN_EN, [0x003C], 16)
    assert u(dut.open_drain_en) == 0x3C

    await spi_write_burst(dut, CMD_WR_CSR, CSR_CTRL_RUN, [0x000B], 16)
    assert u(dut.ctrl_run) == 0xB
    await spi_write_burst(dut, CMD_WR_CSR, CSR_RLE_BASE, [0x0100, 0x00FF], 16)
    assert u(dut.rle_base) == 0x0100
    assert u(dut.rle_mask) == 0x00FF

    # Host writes also reach every private core/BCU configuration bank.
    await spi_write_burst(dut, CMD_WR_CSR, CSR_CORE_BANK + 2 * 16 + 4, [0xCAFE], 16)
    assert get_slice(u(dut.bcu_mode_flat), 2, 16) == 0xCAFE

    await spi_write_burst(dut, CMD_WR_CSR, CSR_PIN_ROUTER0, [0x3210, 0x7654], 16)
    assert get_slice(u(dut.pin_router_flat), 0, 16) == 0x3210
    assert get_slice(u(dut.pin_router_flat), 1, 16) == 0x7654

    await spi_write_burst(dut, CMD_WR_CSR, CSR_LUT_TT_BASE + 3, [0xBEEF], 16)
    assert get_slice(u(dut.lut_tt_flat), 3, 16) == 0xBEEF
    await spi_write_burst(dut, CMD_WR_CSR, CSR_LUT_SEL_BASE + 5, [0x4321], 16)
    assert get_slice(u(dut.lut_sel_flat), 5, 16) == 0x4321
    await spi_write_burst(dut, CMD_WR_CSR, CSR_LUT_OREG, [0x00AA], 16)
    assert u(dut.lut_oreg) == 0x00AA

    # Core 1 may write its own BCU_MODE at 0x50 + 4, but not core 0's bank
    # or the global pin-filter CSR.
    await core_write(dut, 1, 0x54, 0x1234)
    assert get_slice(u(dut.bcu_mode_flat), 1, 16) == 0x1234
    await core_write(dut, 1, 0x44, 0xFFFF)
    assert get_slice(u(dut.bcu_mode_flat), 0, 16) == 0x0000
    await core_write(dut, 1, CSR_PIN_FILT_EN, 0x0055)
    assert u(dut.pin_filt_en) == 0xA5

    # Core-to-core EV set/clear arbitration is bitwise and set wins ties.
    await core_write_two(dut, 0, CSR_EV_SET, 0x0003, 1, CSR_EV_CLR, 0x0001)
    assert u(dut.ev_flags) & 0x3 == 0x3, f"ev_flags {u(dut.ev_flags):#x}"
    await core_write(dut, 0, CSR_EV_CLR, 0x0003)
    assert (u(dut.ev_flags) & 0x3) == 0

    t0 = u(dut.timer)
    await ClockCycles(dut.clk, 5)
    assert (u(dut.timer) - t0) & 0xFFFFF >= 5


@cocotb.test()
async def test_host_spi_sram_bursts_and_abort(dut):
    """32-bit SRAM write/read bursts work through the real arbiters; the
    mandatory dummy byte preserves bit alignment, and a 16-bit abort commits
    nothing."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_harness(dut)

    # Exercise SRAM 1 host bursts while all five core/RLE clients continuously
    # write unrelated high memory. Their worst-case round-robin service still
    # leaves the host far inside the SPI bit-period budget.
    dut.sram1_ext_we.value = 0x1F
    dut.sram1_ext_req.value = 0x1F
    for c in range(5):
        dut.sram1_ext_addr_flat.value = set_slice(u(dut.sram1_ext_addr_flat), c, 9, 0x100 + c)
        dut.sram1_ext_wdata_flat.value = set_slice(u(dut.sram1_ext_wdata_flat), c, 32, 0x2222 * c)
        dut.sram1_ext_wmask_flat.value = set_slice(u(dut.sram1_ext_wmask_flat), c, 32, 0xFFFFFFFF)

    words1 = [0x80000001, 0x12345678, 0xFFFFFFFF, 0x00000001]
    await spi_write_burst(dut, CMD_WR_SRAM1, 0x020, words1, 32)
    got1 = await spi_read_burst(dut, CMD_RD_SRAM1, 0x020, 4, 32)
    assert got1 == words1, f"SRAM1 burst {[hex(x) for x in got1]}"

    words0 = [0xA5A50000, 0x5A5A0001]
    await spi_write_burst(dut, CMD_WR_SRAM0, 0x010, words0, 32)
    got0 = await spi_read_burst(dut, CMD_RD_SRAM0, 0x010, 2, 32)
    assert got0 == words0, f"SRAM0 burst {[hex(x) for x in got0]}"

    # Prepare a word, then abort a replacement after 16 bits. The old value
    # must survive because commits happen only on the final bit.
    await spi_write_burst(dut, CMD_WR_SRAM1, 0x030, [0xAAAAAAAA], 32)
    await spi_abort_after_bits(dut, CMD_WR_SRAM1, 0x030, 16)
    got_abort = await spi_read_burst(dut, CMD_RD_SRAM1, 0x030, 1, 32)
    assert got_abort == [0xAAAAAAAA], f"aborted write changed memory: {got_abort}"


@cocotb.test()
async def test_host_spi_write_commit_and_contention(dut):
    """An SRAM 0 host write produces the cache-invalidation commit strobe only
    when the write reaches the macro, even behind competing client requests."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_harness(dut)

    commits = []

    async def watch_commit():
        while True:
            await sample_after_rise(dut.clk)
            if u(dut.sram0_write_commit):
                commits.append(1)

    watcher = cocotb.start_soon(watch_commit())

    # Keep all four core clients continuously requesting reads while the
    # host loads SRAM 0. Reads contend for arbitration without producing
    # write-commit strobes, so an observed strobe is attributable to the host.
    dut.sram0_ext_we.value = 0x0
    dut.sram0_ext_req.value = 0xF
    for c in range(4):
        dut.sram0_ext_addr_flat.value = set_slice(u(dut.sram0_ext_addr_flat), c, 9, 0x40 + c)
        dut.sram0_ext_wdata_flat.value = set_slice(u(dut.sram0_ext_wdata_flat), c, 32, 0x1111 * c)
        dut.sram0_ext_wmask_flat.value = set_slice(u(dut.sram0_ext_wmask_flat), c, 32, 0xFFFFFFFF)

    await spi_write_burst(dut, CMD_WR_SRAM0, 0x018, [0xC0FFEE01], 32)
    await ClockCycles(dut.clk, 20)
    watcher.cancel()
    assert commits, "no SRAM 0 write commit strobe observed"
    got = await spi_read_burst(dut, CMD_RD_SRAM0, 0x018, 1, 32)
    assert got == [0xC0FFEE01]


@cocotb.test()
async def test_host_spi_ena_isolation(dut):
    """With ena low, even a perfectly formed Host SPI write frame is ignored
    and MISO remains inactive; after re-enable the same frame takes effect."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_harness(dut, ena=0)

    await spi_write_burst(dut, CMD_WR_CSR, CSR_PIN_FILT_EN, [0x005A], 16)
    assert u(dut.pin_filt_en) == 0
    assert u(dut.host_miso) == 0
    got = await spi_read_burst(dut, CMD_RD_CSR, CSR_PIN_FILT_EN, 1, 16)
    assert got == [0x0000]
    assert u(dut.pin_filt_en) == 0

    dut.ena.value = 1
    await ClockCycles(dut.clk, 8)
    await spi_write_burst(dut, CMD_WR_CSR, CSR_PIN_FILT_EN, [0x005A], 16)
    assert u(dut.pin_filt_en) == 0x5A
