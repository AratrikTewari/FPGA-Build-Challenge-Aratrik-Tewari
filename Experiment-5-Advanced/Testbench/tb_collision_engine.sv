`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_collision_engine.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Unit testbench for the collision_engine. Injects intersecting and diverging trajectories to verify true/false positive rates.
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
module tb_collision_engine;

localparam integer W = 20;
localparam integer F = 12;
localparam integer NUM_TARGETS = 8;

localparam integer Q = 4096;

// 2.0 seconds in Q8.12.
localparam integer HORIZON = 8192;

// 2.0 units in Q8.12.
localparam integer THRESHOLD = 8192;


// ============================================================
// Clock / reset
// ============================================================

logic clk;
logic rst;
logic start;


// ============================================================
// Target states
// ============================================================

logic signed [W-1:0] target_state
    [0:NUM_TARGETS-1][0:3];


// ============================================================
// Collision configuration
// ============================================================

logic signed [W-1:0] prediction_horizon;
logic signed [W-1:0] collision_threshold;


// ============================================================
// Outputs
// ============================================================

logic collision_matrix
    [0:NUM_TARGETS-1][0:NUM_TARGETS-1];

logic collision_detected;

logic busy;
logic done;


// ============================================================
// Error counter
// ============================================================

integer errors;


// ============================================================
// DUT
// ============================================================

collision_engine #(
    .W(W),
    .F(F),
    .NUM_TARGETS(NUM_TARGETS)
) dut (
    .clk(clk),
    .rst(rst),
    .start(start),

    .target_state(target_state),

    .prediction_horizon(prediction_horizon),
    .collision_threshold(collision_threshold),

    .collision_matrix(collision_matrix),
    .collision_detected(collision_detected),

    .busy(busy),
    .done(done)
);


// ============================================================
// Clock
// ============================================================

initial begin

    clk = 1'b0;

    forever #5 clk = ~clk;

end


// ============================================================
// Main test
// ============================================================

integer i;
integer j;


initial begin

    errors = 0;

    rst = 1'b1;
    start = 1'b0;

    prediction_horizon = HORIZON;
    collision_threshold = THRESHOLD;


    // --------------------------------------------------------
    // Clear all target states.
    // --------------------------------------------------------

    for (i = 0; i < NUM_TARGETS; i = i + 1) begin

        for (j = 0; j < 4; j = j + 1) begin

            target_state[i][j] = '0;

        end

    end


    // ========================================================
    // Reset
    // ========================================================

    repeat (3)
        @(posedge clk);

    rst = 1'b0;

    @(posedge clk);


    $display("");
    $display("==============================================");
    $display("COLLISION ENGINE TEST");
    $display("==============================================");


    // ========================================================
    // TEST 1
    //
    // Static targets:
    //
    // Target 0 = (10,10)
    // Target 1 = (11,10)
    //
    // Distance = 1
    // Threshold = 2
    //
    // Therefore collision must be detected.
    // ========================================================

    $display("");
    $display("TEST 1");
    $display("VERIFYING STATIC COLLISION");


    target_state[0][0] = 10 * Q;
    target_state[0][1] = 10 * Q;

    target_state[1][0] = 11 * Q;
    target_state[1][1] = 10 * Q;


    // Distant pair.

    target_state[2][0] = 50 * Q;
    target_state[2][1] = 50 * Q;

    target_state[3][0] = 80 * Q;
    target_state[3][1] = 80 * Q;


    @(negedge clk);

    start = 1'b1;

    @(negedge clk);

    start = 1'b0;


    wait (done == 1'b1);

    #1;


    if (collision_matrix[0][1] !== 1'b1) begin

        $display(
            "ERROR: target 0/1 collision not detected"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: target 0/1 collision detected"
        );

    end


    if (collision_matrix[1][0] !== 1'b1) begin

        $display(
            "ERROR: collision matrix symmetry missing"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: collision matrix symmetry verified"
        );

    end


    if (collision_matrix[2][3] !== 1'b0) begin

        $display(
            "ERROR: false collision detected for target 2/3"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: distant targets correctly rejected"
        );

    end


    // ========================================================
    // TEST 2
    //
    // Moving targets.
    //
    // Target 4:
    //     x = 0
    //     vx = +5
    //
    // Target 5:
    //     x = 12
    //     vx = -5
    //
    // At t = 2:
    //
    // target 4 -> x = 10
    // target 5 -> x = 2
    //
    // Separation = 8
    //
    // No collision.
    // ========================================================

    $display("");
    $display("TEST 2");
    $display("VERIFYING MOTION PREDICTION");


    target_state[4][0] = 0;
    target_state[4][1] = 0;

    target_state[4][2] = 5 * Q;
    target_state[4][3] = 0;


    target_state[5][0] = 12 * Q;
    target_state[5][1] = 0;

    target_state[5][2] = -5 * Q;
    target_state[5][3] = 0;


    @(negedge clk);

    start = 1'b1;

    @(negedge clk);

    start = 1'b0;


    wait (done == 1'b1);

    #1;


    if (collision_matrix[4][5] !== 1'b0) begin

        $display(
            "ERROR: false predicted collision for target 4/5"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: moving-target prediction verified"
        );

    end


    // ========================================================
    // TEST 3
    //
    // Exact threshold boundary.
    //
    // Target 4 at t=2 -> x=10
    //
    // Change target 5:
    //     x=18
    //     vx=-5
    //
    // At t=2 -> x=8
    //
    // Separation = 2 exactly.
    //
    // <= threshold must therefore detect collision.
    // ========================================================

    $display("");
    $display("TEST 3");
    $display("VERIFYING THRESHOLD BOUNDARY");


    target_state[5][0] = 18 * Q;
    target_state[5][2] = -5 * Q;


    @(negedge clk);

    start = 1'b1;

    @(negedge clk);

    start = 1'b0;


    wait (done == 1'b1);

    #1;


    if (collision_matrix[4][5] !== 1'b1) begin

        $display(
            "ERROR: exact threshold collision not detected"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: exact threshold collision detected"
        );

    end


    // ========================================================
    // TEST 4
    // Verify collision_detected.
    // ========================================================

    $display("");
    $display("TEST 4");
    $display("VERIFYING COLLISION STATUS");


    if (collision_detected !== 1'b1) begin

        $display(
            "ERROR: collision_detected not asserted"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: collision_detected asserted"
        );

    end


    // ========================================================
    // TEST 5
    // Verify busy handshake.
    // ========================================================

    $display("");
    $display("TEST 5");
    $display("VERIFYING BUSY/DONE HANDSHAKE");


    @(posedge clk);

    if (busy !== 1'b0) begin

        $display(
            "ERROR: busy remains asserted after completion"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: busy deasserted after completion"
        );

    end


    // ========================================================
    // TEST 6
    // Verify return to IDLE / second operation.
    // ========================================================

    $display("");
    $display("TEST 6");
    $display("VERIFYING RETURN TO IDLE");


    // Remove the previous collision.

    target_state[5][0] = 40 * Q;
    target_state[5][2] = 0;


    @(negedge clk);

    start = 1'b1;

    @(negedge clk);

    start = 1'b0;


    wait (done == 1'b1);

    #1;


    if (collision_matrix[4][5] !== 1'b0) begin

        $display(
            "ERROR: stale collision result remained"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: second operation returned clean results"
        );

    end


    // ========================================================
    // Final result
    // ========================================================

    $display("");
    $display("==============================================");
    $display("COLLISION ENGINE TEST COMPLETE");
    $display("==============================================");


    if (errors == 0) begin

        $display(
            "PASS: collision engine matches expected behavior."
        );

    end
    else begin

        $display(
            "FAIL: %0d errors detected.",
            errors
        );

    end


    $finish;

end

endmodule
