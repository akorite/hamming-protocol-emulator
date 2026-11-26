# Hardening investigation — running notes

Evidence-driven log of the CMOS5L hardening failure chain. Update as runs land.

## The utilization wall (root problem)

- 8×4 IHP tile allocation: 202×155 µm/tile → 1616×620 µm die. GPL reports
  core area **1,208,172 µm²**.
- **Utilization 94.4%** — unroutable; needs ≤ ~80% (the tt-ihp-vga-trng
  example hardened DRC/LVS clean at 82.7%, so 80ish is achievable).
- Standard cells area (post-mapping): **986,837 µm²**; two SRAM macros add
  ~154k µm² more (79,674 µm² each per LEF, 237×336 µm).

## The area blowup (root cause of the utilization wall)

- Pre-map generic netlist: 42k logic gates incl. **11,243 `$_MUX_`**, zero buffers.
- Post-map: **940k µm² total, of which sequential (FF) area is 229k µm²**
  (4,677× dfrbpq ≈ 49 µm²/FF — cmos5l FFs are huge; every FF diet is worth double).
- **`sg13cmos5l_buf_1`: 27,776 instances** — over half of all cells are min-drive
  buffers (~2.5 per generic mux). Hypothesis: cmos5l mux cells are pass-gate
  style and cannot chain without buffer isolation, so mux trees explode into
  buf+mux structures. Mux-heavy RTL (CSR read muxes, LUT crossbar, barrel
  shifters, icache tag muxes) is therefore disproportionately expensive.
- Reference implementations prove the scope fits: smprather's entire logic is
  **170k µm²** (their wiki: cmos5l routing/fill inflation ×1.865).
  kdp1965/prism taped out with the same SRAMs and the same stripe plugin.

## Failure chain so far (oldest → newest)

1. Verilator lint: latch on `swaddr` in hamming_csr.v — FIXED (bb73202).
2. `No macro instance i_sram0.i_sram` — U6 moved SRAMs one level deeper:
   fixed MACROS instance paths to `u_arbX.i_sram.i_sram` — FIXED.
3. `No match found for regular expression` — PDN_MACRO_CONNECTIONS regexes
   needed the same instance-path update — FIXED (e24c2fb).
4. DPL-0036 detailed placement failed on ~106 instances near u_arb1 —
   FIXED by DPL/DIODE padding=2 + CELL_PAD_EXCLUDE + macro halos
   (copied verbatim from kdp1965/prism, commit f031802).
5. GRT-0010 instance not placed inside gpl.tcl — the utilization wall
   surfacing in global placement (padding inflation made it worse).
   PSM-0069/PSM-0038 power warnings are EXPECTED NOISE: pdngen's check runs
   before the stripe-extension step bridges the macro pins; LVS is the verdict
   (per prism's own config notes).

## Per-module mux census (local yosys synth -flatten, 2026-11-26)

Total design: 11,024 `$_MUX_` (matches CI's 11,243).

| module | muxes | notes |
|---|---|---|
| **hamming_csr** | **5,800** | **53% of the entire design, single instance** |
| hamming_lut_matrix | 856 | crossbar, irreducible-ish |
| hamming_core | 760 ea ×4 | operand/decode muxing |
| hamming_host_spi | 301 | |
| hamming_rle_engine | 302 | |
| hamming_icache | 263 | |
| hamming_bcu | 221 ea ×4 | |
| hamming_sram_arb | 30 | |

The CSR read path implements a full-width mux tree per read port over all
registers; multiple ports (host + cores) multiply the tree. THE diet target.
Options: narrow read width (8-bit reads, firmware composes), restrict per-port
register visibility (cores need only their own bank + few shared), share one
read mux across ports with arbitration. Must preserve the CSR contract in
docs/isa + ISS model + all firmware that reads CSRs (CORE_ID at 0xD!).

## Negative results (dead levers, do not retry)

- **SYNTH_STRATEGY AREA 3** (CI run 45835943761): mapped area DOUBLED to
  2,076,990 um2, utilization 106.5%, GPL-0301 hard fail. AREA 3 replicates
  logic for timing slack — smprather chose it for timing, not area. Default
  strategy is already the area-minimal one. Synthesis knobs cannot fix this;
  the mux/buffer structure is architectural.

## Levers, ranked by expected value

1. **Mux-count reduction in RTL** — the real fix. The original audit flagged
   the CSR read-mux restriction (~12k cells). Also candidates: LUT crossbar
   structure, barrel shifters, icache. Target ≤ ~300k µm² mapped logic.
2. SYNTH_STRATEGY AREA 3 — pushed in 9f97722, CI measuring. Probably not
   sufficient alone since buffers are structural, not optimization failure.
3. FF diet — each FF is 49 µm² plus its input mux logic. CSR block is the
   largest FF consumer (1,057 FFs).
4. Mouth keepouts (prism's odb_keepouts.py + Project.MouthKeepouts step)
   — needed IF placement near SRAM pin mouths congests after area is fixed;
   reference: /tmp clone of kdp1965/ihp-um-janestreet-prism.
5. Margin reduction LEFT/RIGHT_MARGIN_MULT=6 — grows core area; last resort,
   the margins exist for SRAM pin escape.

## References (cloned locally during investigation)

- `/tmp/ihp-um-janestreet-prism` — kdp1965's proven-taped-out project, source
  of the stripe plugin and the padding/halo settings.
- `/tmp/janestreet-ref` — smprather's competing implementation of THIS
  competition project. wiki/reference/floorplan-feasibility.md documents the
  ×1.865 cmos5l inflation factor, SRAM dims, and area budgeting.
  flow/pe_soc.json uses SYNTH_STRATEGY AREA 3 + max-buffer-pct 100 resizer
  settings.


## Measured results (2026-11-26 evening)

- CSR diet COMMITTED AND MEASURED: utilization 94.449% -> 86.978%
  (mapped cell area 940k -> 905k um2). Verified in gds CI run
  45858892850. Still fails GRT-0010 (unplaced instance) at GPL.
- AREA 3 synthesis strategy: REJECTED, doubled mapped area to 2.08M
  um2 / 106.5% utilization (logic replication for timing).
- LEFT/RIGHT_MARGIN_MULT 6 -> 2: REJECTED, utilization barely moved
  (86.98% -> 86.82%), core area only +2.7k um2. The die geometry is
  essentially fixed; margins are not the lever. Reverted.
- Placement becomes physically feasible only at roughly <= 82%
  utilization (tt-ihp-vga-trng hardened at 82.7% per reference notes).
  Current gap: ~5 points of utilization ~ 60k um2 of cell area.

## Remaining levers, ranked

1. Core barrel-shifter / ALU mux diet: cores hold 760 muxes each
   x4 = 3040 total. The ISA shl/shr/sha barrel paths are the densest
   known remaining mux clusters. Any restriction (e.g. iterative
   shift microcode) is an ISA-visible change - evaluate carefully.
2. lut_matrix crossbar: 856 muxes in the pin routing fabric. The
   crossbar is inherently muxy; a sparser routing pattern (fewer
   simultaneous source options) would cut it but reduces routing
   flexibility - check which source combinations the demos use.
3. FF diet: cmos5l FF = ~49 um2 vs ~4 in sg13g2. 4677 FFs ~ 229k um2.
   CSR bank storage and wide control registers are candidates.
4. The 28k minimum-buffer mapping explosion: buf_1 count 27,776 is
   still unexplained - investigate whether synthesis slew/fanout
   constraints are forcing buffer trees, and whether relaxing them
   (STA signoff is anyway a separate gate) recovers area.
