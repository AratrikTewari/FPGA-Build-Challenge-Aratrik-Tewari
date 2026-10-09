`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_state_predict.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Unit testbench for the forward state projection block (x = F*x) based on the kinematic model.
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
module tb_state_predict;

    logic clk = 0;
    logic rst = 1;
    logic start = 0;

    logic signed [19:0] x_in [0:3];
    logic signed [19:0] F_mat [0:3][0:3];
    logic signed [19:0] x_out [0:3];

    logic busy;
    logic done;

    integer i, j;
    integer errors;

    state_predict dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .x_in(x_in),
        .F_mat(F_mat),
        .x_out(x_out),
        .busy(busy),
        .done(done)
    );

    always #5 clk = ~clk;

    task automatic check_vector(
        input integer x0,
        input integer x1,
        input integer x2,
        input integer x3,
        input integer e0,
        input integer e1,
        input integer e2,
        input integer e3
    );
        begin
            @(negedge clk);

            x_in[0] = x0;
            x_in[1] = x1;
            x_in[2] = x2;
            x_in[3] = x3;

            start = 1'b1;

            @(negedge clk);
            start = 1'b0;

            while (!done)
    @(negedge clk);

            if (x_out[0] !== e0) begin
                $display("FAIL x_out[0]: expected=%0d got=%0d",
                         e0, x_out[0]);
                errors = errors + 1;
            end

            if (x_out[1] !== e1) begin
                $display("FAIL x_out[1]: expected=%0d got=%0d",
                         e1, x_out[1]);
                errors = errors + 1;
            end

            if (x_out[2] !== e2) begin
                $display("FAIL x_out[2]: expected=%0d got=%0d",
                         e2, x_out[2]);
                errors = errors + 1;
            end

            if (x_out[3] !== e3) begin
                $display("FAIL x_out[3]: expected=%0d got=%0d",
                         e3, x_out[3]);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        errors = 0;

        for (i = 0; i < 4; i = i + 1) begin
            x_in[i] = '0;
            for (j = 0; j < 4; j = j + 1)
                F_mat[i][j] = '0;
        end

        // Q8.12 constant-velocity F matrix
        F_mat[0][0] = 20'sd4096;
        F_mat[0][2] = 20'sd41;

        F_mat[1][1] = 20'sd4096;
        F_mat[1][3] = 20'sd41;

        F_mat[2][2] = 20'sd4096;
        F_mat[3][3] = 20'sd4096;

        repeat (4) @(negedge clk);
        rst = 1'b0;

        // --------------------------------------------------
        // Test 1
        // x = [10, 20, 3, -4]
        //
        // Expected raw Q8.12:
        // [41083, 81756, 12288, -16384]
        // --------------------------------------------------

        check_vector(
            20'sd40960,
            20'sd81920,
            20'sd12288,
            -20'sd16384,
            20'sd41083,
            20'sd81756,
            20'sd12288,
            -20'sd16384
        );

        // --------------------------------------------------
        // Test 2
        // x = [50, 80, -10, 15]
        //
        // Expected:
        // [204390, 328295, -40960, 61440]
        // --------------------------------------------------

        check_vector(
            20'sd204800,
            20'sd327680,
            -20'sd40960,
            20'sd61440,
            20'sd204390,
            20'sd328295,
            -20'sd40960,
            20'sd61440
        );

        // --------------------------------------------------
        // Test 3
        // x = [0, 100, 20, -20]
        //
        // Expected:
        // [820, 408780, 81920, -81920]
        // --------------------------------------------------

        check_vector(
            20'sd0,
            20'sd409600,
            20'sd81920,
            -20'sd81920,
            20'sd820,
            20'sd408780,
            20'sd81920,
            -20'sd81920
        );

        $display("");
        $display("STATE PREDICTION TESTS COMPLETE");
        $display("ERRORS: %0d", errors);

        if (errors == 0)
            $display("PASS: state prediction RTL matches expected Q8.12 arithmetic.");
        else
            $display("FAIL: state prediction RTL mismatch.");

        $finish;
    end

endmodule
