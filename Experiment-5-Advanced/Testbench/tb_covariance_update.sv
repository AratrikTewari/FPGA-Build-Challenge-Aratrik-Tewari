`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_covariance_update.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Unit testbench for the Kalman filter covariance update step (P = (I - K*H)*P).
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
module tb_covariance_update;

    localparam integer W = 20;

    logic clk;
    logic rst;
    logic start;

    logic signed [W-1:0] I_KH [0:3][0:3];
    logic signed [W-1:0] P_pred [0:3][0:3];

    logic signed [W-1:0] P_updated [0:3][0:3];

    logic busy;
    logic done;

    integer errors;
    integer i;
    integer j;


    // ================================================================
    // DUT
    // ================================================================

    covariance_update dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .I_KH(I_KH),
        .P_pred(P_pred),
        .P_updated(P_updated),
        .busy(busy),
        .done(done)
    );


    // ================================================================
    // 100 MHz clock
    // ================================================================

    always #5 clk = ~clk;


    // ================================================================
    // Check matrix element
    // ================================================================

    task check_value;
        input integer row_index;
        input integer col_index;
        input integer expected;

        begin

            if (P_updated[row_index][col_index] !== expected) begin

                $display(
                    "ERROR: P_updated[%0d][%0d] = %0d, expected %0d",
                    row_index,
                    col_index,
                    P_updated[row_index][col_index],
                    expected
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: P_updated[%0d][%0d] = %0d",
                    row_index,
                    col_index,
                    P_updated[row_index][col_index]
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
        // Initialize matrices
        // ------------------------------------------------------------

        for (i = 0; i < 4; i = i + 1) begin

            for (j = 0; j < 4; j = j + 1) begin

                I_KH[i][j] = 20'sd0;
                P_pred[i][j] = 20'sd0;

            end

        end


        // ------------------------------------------------------------
        // Reset
        // ------------------------------------------------------------

        #20;
        rst = 1'b0;


        // ============================================================
        // TEST 1
        // ============================================================
        //
        // Use:
        //
        // I-KH =
        //
        // [2048  -409     0     0]
        // [-819  2458     0     0]
        // [-1228 -409  4096     0]
        // [-409  -819     0  4096]
        //
        // P_pred =
        //
        // [8192  819  2048  410]
        // [819  12288 410   1638]
        // [2048 410  6144  1229]
        // [410  1638 1229  8192]
        //
        // Expected result is calculated using:
        //
        // P_updated = (I-KH) * P_pred
        //
        // All arithmetic is Q8.12 with arithmetic >>> 12.
        //
        // ============================================================

        I_KH[0][0] = 20'sd2048;
        I_KH[0][1] = -20'sd409;
        I_KH[0][2] = 20'sd0;
        I_KH[0][3] = 20'sd0;

        I_KH[1][0] = -20'sd819;
        I_KH[1][1] = 20'sd2458;
        I_KH[1][2] = 20'sd0;
        I_KH[1][3] = 20'sd0;

        I_KH[2][0] = -20'sd1228;
        I_KH[2][1] = -20'sd409;
        I_KH[2][2] = 20'sd4096;
        I_KH[2][3] = 20'sd0;

        I_KH[3][0] = -20'sd409;
        I_KH[3][1] = -20'sd819;
        I_KH[3][2] = 20'sd0;
        I_KH[3][3] = 20'sd4096;


        P_pred[0][0] = 20'sd8192;
        P_pred[0][1] = 20'sd819;
        P_pred[0][2] = 20'sd2048;
        P_pred[0][3] = 20'sd410;

        P_pred[1][0] = 20'sd819;
        P_pred[1][1] = 20'sd12288;
        P_pred[1][2] = 20'sd410;
        P_pred[1][3] = 20'sd1638;

        P_pred[2][0] = 20'sd2048;
        P_pred[2][1] = 20'sd410;
        P_pred[2][2] = 20'sd6144;
        P_pred[2][3] = 20'sd1229;

        P_pred[3][0] = 20'sd410;
        P_pred[3][1] = 20'sd1638;
        P_pred[3][2] = 20'sd1229;
        P_pred[3][3] = 20'sd8192;


        @(posedge clk);
        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 1");


        // Expected values
        check_value(0, 0, 4014);
check_value(0, 1, -818);
check_value(0, 2, 983);
check_value(0, 3, 41);

check_value(1, 0, -1147);
check_value(1, 1, 7210);
check_value(1, 2, -164);
check_value(1, 3, 900);

check_value(2, 0, -490);
check_value(2, 1, -1063);
check_value(2, 2, 5489);
check_value(2, 3, 942);

check_value(3, 0, -572);
check_value(3, 1, -901);
check_value(3, 2, 942);
check_value(3, 3, 7823);


        // ============================================================
        // FINAL RESULT
        // ============================================================

        $display("");
        $display("COVARIANCE UPDATE TEST COMPLETE");

        if (errors == 0) begin

            $display(
                "PASS: covariance update RTL matches expected Q8.12 arithmetic."
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
