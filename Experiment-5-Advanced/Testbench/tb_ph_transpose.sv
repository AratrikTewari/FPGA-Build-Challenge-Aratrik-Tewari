`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_ph_transpose
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

module tb_ph_transpose;

    localparam integer W = 20;
    localparam integer F = 12;

    logic clk;
    logic rst;
    logic start;

    logic signed [W-1:0] P_pred [0:3][0:3];
    logic signed [W-1:0] PHt    [0:3][0:1];

    logic busy;
    logic done;

    integer errors;

    ph_transpose #(
        .W(W)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .P_pred(P_pred),
        .PHt(PHt),
        .busy(busy),
        .done(done)
    );

    // 100 MHz clock
    always #5 clk = ~clk;

    task automatic check_value(
        input integer row_index,
        input integer col_index,
        input integer expected
    );
        begin
            if (PHt[row_index][col_index] !== expected) begin
                $display(
                    "ERROR: PHt[%0d][%0d] = %0d, expected %0d",
                    row_index,
                    col_index,
                    PHt[row_index][col_index],
                    expected
                );
                errors = errors + 1;
            end
            else begin
                $display(
                    "PASS: PHt[%0d][%0d] = %0d",
                    row_index,
                    col_index,
                    PHt[row_index][col_index]
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

        #20;
        rst = 1'b0;

        // ------------------------------------------------------------
        // Test 1
        //
        // P_pred =
        //
        // [ 2.0   0.2   0.5   0.1 ]
        // [ 0.2   3.0   0.1   0.4 ]
        // [ 0.5   0.1   1.5   0.3 ]
        // [ 0.1   0.4   0.3   2.0 ]
        //
        // PHt is simply the first two columns.
        // ------------------------------------------------------------

        P_pred[0][0] = 2.0 * (1 << F);
        P_pred[0][1] = 0.2 * (1 << F);
        P_pred[0][2] = 0.5 * (1 << F);
        P_pred[0][3] = 0.1 * (1 << F);

        P_pred[1][0] = 0.2 * (1 << F);
        P_pred[1][1] = 3.0 * (1 << F);
        P_pred[1][2] = 0.1 * (1 << F);
        P_pred[1][3] = 0.4 * (1 << F);

        P_pred[2][0] = 0.5 * (1 << F);
        P_pred[2][1] = 0.1 * (1 << F);
        P_pred[2][2] = 1.5 * (1 << F);
        P_pred[2][3] = 0.3 * (1 << F);

        P_pred[3][0] = 0.1 * (1 << F);
        P_pred[3][1] = 0.4 * (1 << F);
        P_pred[3][2] = 0.3 * (1 << F);
        P_pred[3][3] = 2.0 * (1 << F);

        @(posedge clk);
        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 1");

        check_value(0, 0, 2.0 * (1 << F));
        check_value(0, 1, 0.2 * (1 << F));

        check_value(1, 0, 0.2 * (1 << F));
        check_value(1, 1, 3.0 * (1 << F));

        check_value(2, 0, 0.5 * (1 << F));
        check_value(2, 1, 0.1 * (1 << F));

        check_value(3, 0, 0.1 * (1 << F));
        check_value(3, 1, 0.4 * (1 << F));

        // ------------------------------------------------------------
        // Test 2
        //
        // Use arbitrary signed Q8.12 values to verify that the
        // block copies the first two columns exactly.
        // ------------------------------------------------------------

        @(posedge clk);

        P_pred[0][0] = -5.25 * (1 << F);
        P_pred[0][1] = 7.125 * (1 << F);
        P_pred[0][2] = 11.0 * (1 << F);
        P_pred[0][3] = -2.0 * (1 << F);

        P_pred[1][0] = 3.5 * (1 << F);
        P_pred[1][1] = -8.75 * (1 << F);
        P_pred[1][2] = 4.0 * (1 << F);
        P_pred[1][3] = 6.0 * (1 << F);

        P_pred[2][0] = -1.25 * (1 << F);
        P_pred[2][1] = 9.5 * (1 << F);
        P_pred[2][2] = 2.0 * (1 << F);
        P_pred[2][3] = 3.0 * (1 << F);

        P_pred[3][0] = 12.0 * (1 << F);
        P_pred[3][1] = -4.5 * (1 << F);
        P_pred[3][2] = 1.0 * (1 << F);
        P_pred[3][3] = 5.0 * (1 << F);

        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 2");

        check_value(0, 0, -5.25 * (1 << F));
        check_value(0, 1, 7.125 * (1 << F));

        check_value(1, 0, 3.5 * (1 << F));
        check_value(1, 1, -8.75 * (1 << F));

        check_value(2, 0, -1.25 * (1 << F));
        check_value(2, 1, 9.5 * (1 << F));

        check_value(3, 0, 12.0 * (1 << F));
        check_value(3, 1, -4.5 * (1 << F));

        // ------------------------------------------------------------
        // Final result
        // ------------------------------------------------------------

        $display("");
        $display("PH TRANSPOSE TEST COMPLETE");

        if (errors == 0)
            $display("PASS: PHt RTL matches expected matrix.");
        else
            $display("FAIL: %0d errors detected.", errors);

        #20;
        $finish;

    end

endmodule
