`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_innovation_covariance
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Advanced FPGA architecture implementation featuring Kalman filters,
//   collision detection, and multi-target tracking.
//
// Architectural Hierarchy:
//   Part of the Experiment 5 RTL / Simulation / Testbench ecosystem.
// ============================================================================

module tb_innovation_covariance;

    localparam integer W = 20;
    localparam integer F = 12;

    logic clk;
    logic rst;
    logic start;

    logic signed [W-1:0] P_pred [0:3][0:3];
    logic signed [W-1:0] R_diag [0:1];

    logic signed [W-1:0] S [0:1];

    logic busy;
    logic done;

    innovation_covariance #(
        .W(W)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .P_pred(P_pred),
        .R_diag(R_diag),
        .S(S),
        .busy(busy),
        .done(done)
    );

    // 100 MHz clock
    always #5 clk = ~clk;

    integer errors;

    task automatic check_S(
        input integer expected_0,
        input integer expected_1
    );
        begin
            if (S[0] !== expected_0) begin
                $display(
                    "ERROR: S[0] = %0d, expected %0d",
                    S[0], expected_0
                );
                errors = errors + 1;
            end else begin
                $display(
                    "PASS: S[0] = %0d",
                    S[0]
                );
            end

            if (S[1] !== expected_1) begin
                $display(
                    "ERROR: S[1] = %0d, expected %0d",
                    S[1], expected_1
                );
                errors = errors + 1;
            end else begin
                $display(
                    "PASS: S[1] = %0d",
                    S[1]
                );
            end
        end
    endtask

    initial begin

        clk = 1'b0;
        rst = 1'b1;
        start = 1'b0;
        errors = 0;

        for (int i = 0; i < 4; i = i + 1) begin
            for (int j = 0; j < 4; j = j + 1) begin
                P_pred[i][j] = '0;
            end
        end

        R_diag[0] = '0;
        R_diag[1] = '0;

        #20;
        rst = 1'b0;

        // ------------------------------------------------------------
        // Test 1
        //
        // S[0] = P_pred[0][0] + R[0]
        // S[1] = P_pred[1][1] + R[1]
        //
        // P00 = 2.0
        // P11 = 3.0
        // R0  = 0.5
        // R1  = 0.25
        //
        // Expected:
        // S0 = 2.5
        // S1 = 3.25
        // ------------------------------------------------------------

        P_pred[0][0] = 2.0 * (1 << F);
        P_pred[1][1] = 3.0 * (1 << F);

        R_diag[0] = 0.5  * (1 << F);
        R_diag[1] = 0.25 * (1 << F);

        @(posedge clk);
        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 1");
        check_S(
            2.5  * (1 << F),
            3.25 * (1 << F)
        );

        // ------------------------------------------------------------
        // Test 2
        //
        // P00 = 10.0
        // P11 = 7.5
        // R0  = 1.0
        // R1  = 2.0
        //
        // Expected:
        // S0 = 11.0
        // S1 = 9.5
        // ------------------------------------------------------------

        @(posedge clk);

        P_pred[0][0] = 10.0 * (1 << F);
        P_pred[1][1] = 7.5  * (1 << F);

        R_diag[0] = 1.0 * (1 << F);
        R_diag[1] = 2.0 * (1 << F);

        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 2");
        check_S(
            11.0 * (1 << F),
            9.5  * (1 << F)
        );

        // ------------------------------------------------------------
        // Test 3
        //
        // Fractional values:
        //
        // P00 = 4.125
        // P11 = 6.75
        // R0  = 0.125
        // R1  = 0.375
        //
        // Expected:
        // S0 = 4.25
        // S1 = 7.125
        // ------------------------------------------------------------

        @(posedge clk);

        P_pred[0][0] = 4.125 * (1 << F);
        P_pred[1][1] = 6.75  * (1 << F);

        R_diag[0] = 0.125 * (1 << F);
        R_diag[1] = 0.375 * (1 << F);

        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 3");
        check_S(
            4.25  * (1 << F),
            7.125 * (1 << F)
        );

        $display("");
        $display("INNOVATION COVARIANCE TEST COMPLETE");

        if (errors == 0) begin
            $display("PASS: innovation covariance RTL matches expected Q8.12 arithmetic.");
        end else begin
            $display("FAIL: %0d errors detected.", errors);
        end

        #20;
        $finish;

    end

endmodule
