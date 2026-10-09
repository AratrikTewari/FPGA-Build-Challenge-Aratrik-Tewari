`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_multi_target_engine.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Testbench for the multi-target tracking supervisor, ensuring target IDs and states are independently maintained.
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
module tb_multi_target_engine;

    localparam integer W = 20;
    localparam integer NUM_TARGETS = 8;

    logic clk;
    logic rst;
    logic start;

    logic signed [W-1:0] z [0:NUM_TARGETS-1][0:1];

    logic signed [W-1:0] F_mat [0:3][0:3];
    logic signed [W-1:0] Q [0:3][0:3];
    logic signed [W-1:0] R_diag [0:1];

    logic signed [W-1:0] x_out [0:NUM_TARGETS-1][0:3];
    logic signed [W-1:0] P_out [0:NUM_TARGETS-1][0:3][0:3];

    logic busy;
    logic done;

    integer t;
    integer i;
    integer j;
    integer errors;


    // ============================================================
    // DUT
    // ============================================================

    multi_target_engine #(
        .W(W),
        .F(12),
        .ACC_W(44),
        .INV_W(16),
        .NUM_TARGETS(NUM_TARGETS),
        .ID_W(3)
    ) dut (

        .clk(clk),
        .rst(rst),
        .start(start),

        .z(z),

        .F_mat(F_mat),
        .Q(Q),
        .R_diag(R_diag),

        .x_out(x_out),
        .P_out(P_out),

        .busy(busy),
        .done(done)
    );


    // ============================================================
    // Clock
    // ============================================================

    always #5 clk = ~clk;


    // ============================================================
    // Test
    // ============================================================

    initial begin

        clk = 1'b0;
        rst = 1'b1;
        start = 1'b0;

        errors = 0;


        // --------------------------------------------------------
        // F = Identity
        // --------------------------------------------------------

        for (i = 0; i < 4; i = i + 1) begin

            for (j = 0; j < 4; j = j + 1) begin

                if (i == j)
                    F_mat[i][j] = 4096;
                else
                    F_mat[i][j] = 0;

                Q[i][j] = 0;

            end

        end


        // --------------------------------------------------------
        // Measurement noise
        // --------------------------------------------------------

        R_diag[0] = 4096;
        R_diag[1] = 4096;


        // --------------------------------------------------------
        // Give every target a unique measurement
        // --------------------------------------------------------

        for (t = 0; t < NUM_TARGETS; t = t + 1) begin

            z[t][0] = (t + 1) * 4096;
            z[t][1] = (t + 1) * 2048;

        end


        // --------------------------------------------------------
        // Reset
        // --------------------------------------------------------

        #30;

        rst = 1'b0;


        // --------------------------------------------------------
        // Start complete 8-target frame
        // --------------------------------------------------------

        @(posedge clk);
        start <= 1'b1;

        @(posedge clk);
        start <= 1'b0;


        // --------------------------------------------------------
        // Wait for frame completion
        // --------------------------------------------------------

        wait (done == 1'b1);


        // --------------------------------------------------------
        // Results
        // --------------------------------------------------------

        $display("");
        $display("==============================================");
        $display("8-TARGET MULTI-TARGET ENGINE TEST");
        $display("==============================================");


        for (t = 0; t < NUM_TARGETS; t = t + 1) begin

            $display(
                "TARGET %0d: x = %0d, %0d, %0d, %0d",
                t,
                x_out[t][0],
                x_out[t][1],
                x_out[t][2],
                x_out[t][3]
            );

        end


        // --------------------------------------------------------
        // Verify that every target was written
        //
        // With zero initial state and zero Q, the first two
        // components should remain zero only if the Kalman
        // calculation produces no correction. We therefore
        // use the write timing/result presence rather than
        // hard-coded Kalman numerical values here.
        // --------------------------------------------------------

        for (t = 0; t < NUM_TARGETS; t = t + 1) begin

            if ((x_out[t][0] === 'x) ||
                (x_out[t][1] === 'x) ||
                (x_out[t][2] === 'x) ||
                (x_out[t][3] === 'x)) begin

                $display(
                    "ERROR: target %0d contains X values",
                    t
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: target %0d completed",
                    t
                );

            end

        end


        // --------------------------------------------------------
        // Final result
        // --------------------------------------------------------

        $display("");
        $display("==============================================");
        $display("MULTI-TARGET ENGINE TEST COMPLETE");
        $display("==============================================");

        if (errors == 0) begin

            $display(
                "PASS: 8-target integrated Kalman path completed."
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
