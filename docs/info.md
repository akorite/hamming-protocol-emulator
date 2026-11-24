<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

Hamming is a general-purpose, post-fabrication reprogrammable protocol emulator
and hardware debugging engine for the Tiny Tapeout IHP CMOS5L shuttle. The
silicon never changes between protocol deployments — only the bitstream in
instruction SRAM does.

At its heart are four deterministic, deadline-timed protocol cores executing a
custom 16-bit ISA. Each core owns a dedicated byte channel on the external pin
matrix, a small register file, a deadline timer, and an 8-entry direct-mapped
loop cache fed from code SRAM. Four bitstream-conditioning units (CRC engines,
DPLL clock recovery, digital filters) sit between the pin matrix and the cores
so that CRC generation/checking and clock recovery run in parallel with
instruction execution.

Two foundry SRAM banks (2 × `RM_IHPSG13_1P_512x32_c2_bm_bist`, 4 KiB total)
hold code (SRAM 0) and buffered packets/scratch (SRAM 1). A fixed two-tier,
16-LUT programmable pin matrix routes any protocol pin to any core channel, so
pinout never changes with firmware. A run-length-encoded capture engine
records pin transitions into SRAM 1 for protocol debugging, and a
man-in-the-middle wiring mode can splice two pins through a BCU channel for
fault-injection demos.

Host access is a standard SPI Mode 0 interface (CS_N/SCLK/MOSI on ui[7:5],
MISO on uo[7], up to 5 MHz) with a mandatory one-dummy-byte read protocol.
Through it the host loads firmware, pokes CSRs, reads RLE capture records, and
arms/disarms the pin matrix.

## How to test

Power-on reset puts all outputs in a safe, known state (protocol pins idle,
`uio_oe` all-input, SRAM enables low). Over the host SPI link:

1. Write a firmware image into instruction SRAM.
2. Release the target core(s) from reset via the control CSR.
3. Drive protocol activity on the PROTO pins, or read back captured data.

The U1 smoke build exercises both SRAM banks directly from the pins:
`ui_in[7:5]` selects a command (set 9-bit address, write byte lane, write both
banks, read), `ui_in[4]` picks the bank, `ui_in[3:2]` the byte lane, `uio_in`
carries the byte payload, and `uo_out` returns the selected byte of the last
word read.

## External hardware

Optional: any SPI-capable host (e.g. an RP2350 dev board) to load firmware
images and exercise protocol traffic. The design is fully self-contained —
protocol peers are other devices on the pin matrix (e.g. a UART source, SPI
flash, I2C sensor, CAN transceiver for physical layers requiring one).
