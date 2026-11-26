// hamming_bcu_dpll.v - edge_in-Priority Hard-Snap & Hold-Off Bit-Center DPLL
//
// Recovers bit-center sample strobes from wire edges for the per-core BCU.
// One bit cell spans cfg_period system clocks (0 encodes 256). The cell
// counter free-runs while `run` is asserted; edges restart or correct the
// cell phase according to the configured mode:
//
//   mode 0 (none):      edges are ignored; the counter free-runs.
//   mode 1 (hard_snap): any edge_in restarts the cell counter, so the sample
//                       strobe lands period/2 cycles after the edge_in. USB
//                       full-speed/low-speed style recovery.
//   mode 2 (hold_off):  an edge_in restarts the counter only when it arrives
//                       inside the acceptance window, i.e. within cfg_win+1
//                       cycles of the cell boundary (either side). Edges
//                       mid-cell are ignored, giving jitter tolerance for
//                       10BASE-T Manchester.
//
// `sample` is a single-cycle strobe at the bit-center crossing. `cell_end`
// strobes when the counter wraps. `miss_cnt` counts half-cell-resolution
// intervals without any accepted-or-observed wire edge_in, saturating at 15;
// Ethernet end-of-frame is signalled on the `eof` strobe when the miss
// count reaches 9 half-cells while running. Any observed edge_in (even a
// rejected one) resets the miss counter, because the wire is demonstrably
// still active.
//
// Strict Verilog-2005, default_nettype none.
`timescale 1ns/1ps
`default_nettype none

module hamming_bcu_dpll (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       run,          // RX armed: counter advances
    input  wire       edge_in,         // 1-cycle wire transition strobe
    input  wire [1:0] cfg_mode,     // 0 none, 1 hard_snap, 2 hold_off
    input  wire [3:0] cfg_win,      // acceptance window minus one
    input  wire [7:0] cfg_period,   // clocks per bit cell (0 => 256)

    output reg        sample,       // bit-center strobe
    output reg        cell_end,     // end-of-cell strobe
    output reg  [3:0] miss_cnt,     // half-cells without an edge_in
    output reg        eof           // no-edge_in EOF detect strobe
);

    // Effective period in the range 1..256.
    wire [8:0] period = (cfg_period == 8'd0) ? 9'd256 : {1'b0, cfg_period};
    wire [8:0] half   = period >> 1;

    reg [8:0] cnt;
    // Half-cell phase tracking for the miss counter. `half_cnt` counts
    // cycles within the current half-cell interval.
    reg [8:0] half_cnt;
    // Lock state: edge-recovered modes only emit sample strobes after the
    // first accepted edge, so free-running pre-lock samples cannot shift
    // spurious bits into the RX engine. Mode none is always locked.
    reg       locked;

    wire at_center = (cnt == (half - 9'd1));
    wire at_end    = (cnt == (period - 9'd1));

    // edge_in acceptance.
    wire [8:0] win = {5'd0, cfg_win} + 9'd1;
    wire near_start = (cnt <= (win - 9'd1));
    wire near_end   = (cnt >= (period - win));
    wire in_window  = near_start | near_end;

    wire accept_edge = edge_in && (cfg_mode == 2'd1 ||
                                (cfg_mode == 2'd2 && in_window));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt      <= 9'd0;
            half_cnt <= 9'd0;
            sample   <= 1'b0;
            cell_end <= 1'b0;
            miss_cnt <= 4'd0;
            eof      <= 1'b0;
            locked   <= 1'b0;
        end else begin
            sample   <= 1'b0;
            cell_end <= 1'b0;
            eof      <= 1'b0;
            if (!run) begin
                cnt      <= 9'd0;
                half_cnt <= 9'd0;
                miss_cnt <= 4'd0;
                locked   <= 1'b0;
            end else begin
                // Half-cell-resolution miss tracking. Any observed edge_in
                // proves line activity regardless of acceptance.
                if (edge_in) begin
                    miss_cnt <= 4'd0;
                    half_cnt <= 9'd0;
                end else if (half_cnt == (half - 9'd1)) begin
                    half_cnt <= 9'd0;
                    if (miss_cnt != 4'hF)
                        miss_cnt <= miss_cnt + 4'd1;
                    if (miss_cnt == 4'd8)
                        eof <= 1'b1;
                end else begin
                    half_cnt <= half_cnt + 9'd1;
                end

                // Cell counter with edge_in phase correction.
                if (accept_edge) begin
                    cnt    <= 9'd0;
                    locked <= 1'b1;
                end else if (at_end) begin
                    cnt      <= 9'd0;
                    cell_end <= 1'b1;
                end else begin
                    cnt <= cnt + 9'd1;
                    if (at_center && (locked || cfg_mode == 2'd0))
                        sample <= 1'b1;
                end
            end
        end
    end

`ifdef FORMAL
    // --------------------------------------------------------------------
    // Modular formal contract.  The DPLL configuration is a quasi-static CSR
    // setting: it may change while RX is disarmed, but not cycle-to-cycle
    // while the recovery engine remains armed.  This is the same software
    // discipline documented for the BCU CSR bank.
    // --------------------------------------------------------------------
    reg f_past_valid;
    initial f_past_valid = 1'b0;

    always @(posedge clk) begin
        f_past_valid <= 1'b1;
        if (!f_past_valid)
            assume (!rst_n);

        if (f_past_valid && $past(run)) begin
            // The edge on which run falls still belongs to the previous
            // armed configuration: registered sample/cell strobes visible in
            // that cycle were generated from it.  Software may reprogram the
            // CSR once one full disarmed clock has elapsed.
            assume ($stable(cfg_mode));
            assume ($stable(cfg_win));
            assume ($stable(cfg_period));
        end

        if (f_past_valid && rst_n) begin
            // Phase counter range is the central safety invariant.  With a
            // stable nonzero encoded period it makes both the center and end
            // comparisons exact and prevents wraparound/overflow artifacts.
            assert (cnt < period);
            if (half != 9'd0)
                assert (half_cnt < half);

            // The registered sample strobe marks the center crossing: in the
            // cycle in which it is visible the cell counter is exactly half.
            // Since an accepted edge forces cnt to zero, hard-snap therefore
            // places the first post-edge sample half a cell after that edge.
            if (sample) begin
                assert (cnt == half);
                assert (!cell_end);
                if (cfg_mode != 2'd0)
                    assert (locked);
            end
            if (cell_end)
                assert (cnt == 9'd0);

            // EOF is emitted only on the half-cell tick that increments the
            // miss count from eight to nine.  It is a one-shot cause/effect
            // relation, not merely a level association.
            if (eof) begin
                assert (miss_cnt == 4'd9);
                assert (!$past(edge_in));
                assert ($past(miss_cnt) == 4'd8);
                assert ($past(half_cnt) == (half - 9'd1));
            end

            // Any observed edge_in proves line activity and restarts miss
            // accumulation, whether or not the edge_in is accepted for phase
            // correction.  An accepted edge_in also hard-resets the cell phase
            // and establishes lock.
            if ($past(rst_n) && $past(edge_in)) begin
                assert (miss_cnt == 4'd0);
                assert (half_cnt == 9'd0);
            end
            if ($past(rst_n) && $past(run && accept_edge)) begin
                assert (cnt == 9'd0);
                assert (locked);
            end
            if ($past(rst_n) && $past(run && edge_in &&
                                          (cfg_mode == 2'd2) && !in_window && !locked))
                assert (!locked);

            // Documented edge_in acceptance profile.  Mode 0 ignores edges,
            // hard-snap accepts every edge_in, and hold-off accepts exactly
            // the configured boundary window.
            if (edge_in) begin
                if (cfg_mode == 2'd0)
                    assert (!accept_edge);
                if (cfg_mode == 2'd1)
                    assert (accept_edge);
                if (cfg_mode == 2'd2)
                    assert (accept_edge == in_window);
            end
        end

        if (f_past_valid && rst_n) begin
            // Non-vacuity: each recovery profile can produce samples, EOF is
            // reachable, and both sides of the hold-off acceptance window are
            // reachable.  Cover mode may choose a short constant period, so
            // these do not require a deep worst-case 256-cycle witness.
            cover (sample && (cfg_mode == 2'd0));
            cover (sample && (cfg_mode == 2'd1) && locked);
            cover (sample && (cfg_mode == 2'd2) && locked);
            cover (eof);
            cover (run && edge_in && (cfg_mode == 2'd2) && in_window);
            cover (run && edge_in && (cfg_mode == 2'd2) && !in_window);
        end
    end
`endif

endmodule
`default_nettype wire
