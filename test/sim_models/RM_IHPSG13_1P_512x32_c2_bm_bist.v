// Local behavioural model of the IHP RM_IHPSG13_1P_512x32_c2_bm_bist macro.
// The array starts X-filled like real foundry silicon, and the model hard-
// fails on the tie-offs Hamming's port contract forbids, so test failures
// show as simulation errors rather than silent mismatch.
`timescale 1ns/10ps
module RM_IHPSG13_1P_512x32_c2_bm_bist (
    A_CLK,
    A_MEN,
    A_WEN,
    A_REN,
    A_ADDR,
    A_DIN,
    A_DLY,
    A_DOUT,
    A_BM,
    A_BIST_CLK,
    A_BIST_EN,
    A_BIST_MEN,
    A_BIST_WEN,
    A_BIST_REN,
    A_BIST_ADDR,
    A_BIST_DIN,
    A_BIST_BM
);

    input A_CLK;
    input A_MEN;
    input A_WEN;
    input A_REN;
    input [8:0] A_ADDR;
    input [31:0] A_DIN;
    input A_DLY;
    output [31:0] A_DOUT;
    input [31:0] A_BM;
    input A_BIST_CLK;
    input A_BIST_EN;
    input A_BIST_MEN;
    input A_BIST_WEN;
    input A_BIST_REN;
    input [8:0] A_BIST_ADDR;
    input [31:0] A_BIST_DIN;
    input [31:0] A_BIST_BM;

    reg [31:0] memory [0:511];
    reg [31:0] dr_r;
    integer i;

    initial begin
        for (i = 0; i < 512; i = i + 1)
            memory[i] = 32'hxxxxxxxx;
        dr_r = 32'hxxxxxxxx;
        #0;
        if (A_DLY !== 1'b1)
            $fatal(1, "%m ERROR: A_DLY must be tied to 1'b1 (saw %b)", A_DLY);
    end

    always @(A_DLY) begin
        if (A_DLY !== 1'b1)
            $fatal(1, "%m ERROR: A_DLY must remain 1'b1 (observed %b)", A_DLY);
    end

    always @(posedge A_CLK) begin
        if (A_MEN == 1'b1) begin
            if (A_WEN == 1'b1 && A_REN == 1'b1)
                $fatal(1, "%m ERROR: A_WEN and A_REN asserted simultaneously");
            if (A_WEN == 1'b1)
                memory[A_ADDR] <= (memory[A_ADDR] & ~A_BM) | (A_DIN & A_BM);
            else if (A_REN == 1'b1)
                dr_r <= memory[A_ADDR];
        end
    end

    assign A_DOUT = dr_r;

    wire _unused = &{A_BIST_CLK, A_BIST_EN, A_BIST_MEN, A_BIST_WEN, A_BIST_REN,
                     A_BIST_ADDR, A_BIST_DIN, A_BIST_BM, 1'b0};

endmodule
