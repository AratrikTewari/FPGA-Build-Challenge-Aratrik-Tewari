`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_state_correction.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Unit testbench for applying the innovation vector (residual) to correct the a priori state estimate.
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
module tb_state_correction;

    localparam integer W = 20;
    localparam integer F = 12;
    localparam integer ACC_W = 44;

    logic clk;
    logic rst;
    logic start;

    logic signed [W-1:0] x_pred [0:3];
    logic signed [W-1:0] K [0:3][0:1];
    logic signed [W-1:0] innovation [0:1];

    logic signed [W-1:0] x_updated [0:3];

    logic busy;
    logic done;

    integer errors;
    integer i;
    integer j;


    // ================================================================
    // DUT
    // ================================================================

    state_correction #(
        .W(W),
        .F(F),
        .ACC_W(ACC_W)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .x_pred(x_pred),
        .K(K),
        .innovation(innovation),
        .x_updated(x_updated),
        .busy(busy),
        .done(done)
    );


    // ================================================================
    // 100 MHz clock
    // ================================================================

    always #5 clk = ~clk;


    // ================================================================
    // Check output
    // ================================================================

    task check_value;
        input integer index;
        input integer expected;
        begin

            if (x_updated[index] !== expected) begin

                $display(
                    "ERROR: x_updated[%0d] = %0d, expected %0d",
                    index,
                    x_updated[index],
                    expected
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: x_updated[%0d] = %0d",
                    index,
                    x_updated[index]
                );

            end

        end
    endtask


    // ================================================================
    // TEST
    // ================================================================

    initial begin

        clk = 1'b0;
        rst = 1'b1;
        start = 1'b0;
        errors = 0;


        // ------------------------------------------------------------
        // Initialize inputs
        // ------------------------------------------------------------

        for (i = 0; i < 4; i = i + 1) begin

            x_pred[i] = 20'sd0;

            for (j = 0; j < 2; j = j + 1) begin
                K[i][j] = 20'sd0;
            end

        end

        innovation[0] = 20'sd0;
        innovation[1] = 20'sd0;


        // ------------------------------------------------------------
        // Reset
        // ------------------------------------------------------------

        #20;
        rst = 1'b0;


        // ============================================================
        // TEST 1
        // ============================================================
        //
        // x_pred:
        //     [ 10.0
        //       20.0
        //        5.0
        //       -3.0 ]
        //
        // K:
        //     [ 0.5   0.1 ]
        //     [ 0.2   0.4 ]
        //     [ 0.3   0.1 ]
        //     [ 0.1   0.2 ]
        //
        // innovation:
        //     [ 2.0
        //      -1.0 ]
        //
        // All values below are explicit Q8.12 integers.
        //
        // 10.0  = 40960
        // 20.0  = 81920
        //  5.0  = 20480
        // -3.0  = -12288
        //
        // 0.5 = 2048
        // 0.1 = 409
        // 0.2 = 819
        // 0.3 = 1228
        // 0.4 = 1638
        //
        // 2.0  = 8192
        // -1.0 = -4096
        //
        // Expected:
        //
        // x_updated[0] = 44646
        // x_updated[1] = 81920
        // x_updated[2] = 22528
        // x_updated[3] = -12288
        //
        // ============================================================

        x_pred[0] = 20'sd40960;
        x_pred[1] = 20'sd81920;
        x_pred[2] = 20'sd20480;
        x_pred[3] = -20'sd12288;

        K[0][0] = 20'sd2048;
        K[0][1] = 20'sd409;

        K[1][0] = 20'sd819;
        K[1][1] = 20'sd1638;

        K[2][0] = 20'sd1228;
        K[2][1] = 20'sd409;

        K[3][0] = 20'sd409;
        K[3][1] = 20'sd819;

        innovation[0] = 20'sd8192;
        innovation[1] = -20'sd4096;


        @(posedge clk);
        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 1");

        check_value(0, 44647);
        check_value(1, 81920);
        check_value(2, 22527);
        check_value(3, -12289);


        // ============================================================
        // TEST 2
        // ============================================================
        //
        // x_pred:
        //     [ 100
        //       -50
        //        20
        //       -10 ]
        //
        // K:
        //     [  2.0   -1.0 ]
        //     [  1.0    0.5 ]
        //     [ -0.5    2.0 ]
        //     [  0.25  -1.5 ]
        //
        // innovation:
        //     [ 3
        //      -2 ]
        //
        // Expected:
        //
        // x_updated[0] = 108
        // x_updated[1] = -48
        // x_updated[2] = 14.5
        // x_updated[3] = -6.25
        //
        // Explicit Q8.12 expected values:
        //
        // 108    = 442368
        // -48    = -196608
        // 14.5   = 59392
        // -6.25  = -25600
        //
        // ============================================================

        @(posedge clk);

        x_pred[0] = 20'sd409600;
        x_pred[1] = -20'sd204800;
        x_pred[2] = 20'sd81920;
        x_pred[3] = -20'sd40960;

        K[0][0] = 20'sd8192;
        K[0][1] = -20'sd4096;

        K[1][0] = 20'sd4096;
        K[1][1] = 20'sd2048;

        K[2][0] = -20'sd2048;
        K[2][1] = 20'sd8192;

        K[3][0] = 20'sd1024;
        K[3][1] = -20'sd6144;

        innovation[0] = 20'sd12288;
        innovation[1] = -20'sd8192;


        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 2");

        check_value(0, 442368);
        check_value(1, -196608);
        check_value(2, 59392);
        check_value(3, -25600);


        // ============================================================
        // FINAL RESULT
        // ============================================================

        $display("");
        $display("STATE CORRECTION TEST COMPLETE");

        if (errors == 0) begin
            $display(
                "PASS: state correction RTL matches expected Q8.12 arithmetic."
            );
        end
        else begin
            $display(
                "FAIL: %0d errors detected.",
                errors
            );
        end

        #20;
        $finish;

    end

endmodule
