`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_measurement_update
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

module tb_measurement_update;

    localparam integer W = 20;
    localparam integer F = 12;

    logic clk;
    logic rst;
    logic start;

    logic signed [W-1:0] x_pred [0:3];
    logic signed [W-1:0] z [0:1];

    logic signed [W-1:0] innovation [0:1];

    logic busy;
    logic done;

    measurement_update #(
        .W(W)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .x_pred(x_pred),
        .z(z),
        .innovation(innovation),
        .busy(busy),
        .done(done)
    );

    // 100 MHz clock
    always #5 clk = ~clk;

    task automatic check_innovation(
        input integer expected_x,
        input integer expected_y
    );
        begin
            if (innovation[0] !== expected_x) begin
                $display(
                    "ERROR: innovation[0] = %0d, expected %0d",
                    innovation[0], expected_x
                );
            end else begin
                $display(
                    "PASS: innovation[0] = %0d",
                    innovation[0]
                );
            end

            if (innovation[1] !== expected_y) begin
                $display(
                    "ERROR: innovation[1] = %0d, expected %0d",
                    innovation[1], expected_y
                );
            end else begin
                $display(
                    "PASS: innovation[1] = %0d",
                    innovation[1]
                );
            end
        end
    endtask

    initial begin

        clk = 1'b0;
        rst = 1'b1;
        start = 1'b0;

        for (int i = 0; i < 4; i = i + 1)
            x_pred[i] = '0;

        for (int i = 0; i < 2; i = i + 1)
            z[i] = '0;

        #20;
        rst = 1'b0;

        // ------------------------------------------------------------
        // Test 1
        //
        // x_pred = [10, 20, 1, -2]
        // z      = [12, 17]
        //
        // innovation = z - H*x_pred
        //            = [12-10, 17-20]
        //            = [2, -3]
        // ------------------------------------------------------------

        x_pred[0] = 10 <<< F;
        x_pred[1] = 20 <<< F;
        x_pred[2] = 1  <<< F;
        x_pred[3] = -2 <<< F;

        z[0] = 12 <<< F;
        z[1] = 17 <<< F;

        @(posedge clk);
        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("TEST 1");
        check_innovation(2 <<< F, -3 <<< F);

        // ------------------------------------------------------------
        // Test 2
        //
        // x_pred = [50, 75, -4, 3]
        // z      = [45, 80]
        //
        // innovation = [-5, 5]
        // ------------------------------------------------------------

        @(posedge clk);

        x_pred[0] = 50 <<< F;
        x_pred[1] = 75 <<< F;
        x_pred[2] = -4 <<< F;
        x_pred[3] = 3 <<< F;

        z[0] = 45 <<< F;
        z[1] = 80 <<< F;

        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("TEST 2");
        check_innovation(-5 <<< F, 5 <<< F);

        // ------------------------------------------------------------
        // Test 3
        //
        // Fractional values:
        //
        // x_pred = [10.25, 20.5, 0.5, -1.25]
        // z      = [11.75, 19.25]
        //
        // innovation = [1.5, -1.25]
        // ------------------------------------------------------------

        @(posedge clk);

        x_pred[0] = 10.25 * (1 << F);
        x_pred[1] = 20.5  * (1 << F);
        x_pred[2] = 0.5   * (1 << F);
        x_pred[3] = -1.25 * (1 << F);

        z[0] = 11.75 * (1 << F);
        z[1] = 19.25 * (1 << F);

        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("TEST 3");
        check_innovation(
            1.5  * (1 << F),
            -1.25 * (1 << F)
        );

        $display("");
        $display("MEASUREMENT UPDATE TEST COMPLETE");

        #20;
        $finish;

    end

endmodule
