`timescale 1ns / 1ps

// ============================================================================
// Module:        system_corner_regression
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

module aei_system_corner_regression #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer ACC_W = 44,
    parameter integer INV_W = 16,
    parameter integer NUM_TARGETS = 8
)(
    input  logic clk,
    input logic rst,

    output logic regression_done,
    output logic regression_pass
);

localparam integer ID_W = $clog2(NUM_TARGETS);
localparam integer Q_SCALE = 4096;

localparam integer DEFAULT_HORIZON = 8192;
localparam integer DEFAULT_THRESHOLD = 8192;

integer errors;

logic frame_start;
logic measurement_valid;
logic signed [W-1:0] z_in [0:1];

logic load_enable;
logic [ID_W-1:0] load_target;

logic signed [W-1:0] x_load [0:3];
logic signed [W-1:0] P_load [0:3][0:3];

logic load_done;

logic write_enable;
logic signed [W-1:0] x_write [0:3];
logic signed [W-1:0] P_write [0:3][0:3];

logic signed [W-1:0] F_mat [0:3][0:3];
logic signed [W-1:0] Q_mat [0:3][0:3];
logic signed [W-1:0] R_diag [0:1];

logic signed [W-1:0] prediction_horizon;
logic signed [W-1:0] collision_threshold;

logic busy;
logic frame_done;

logic [ID_W-1:0] target_id;

logic signed [W-1:0] x_out [0:3];
logic signed [W-1:0] P_out [0:3][0:3];

logic collision_busy;
logic collision_done;
logic collision_detected;

logic collision_matrix
    [0:NUM_TARGETS-1][0:NUM_TARGETS-1];


// ============================================================
// DUT
// ============================================================

aei_engine_top #(
    .W(W),
    .F(F),
    .ACC_W(ACC_W),
    .INV_W(INV_W),
    .NUM_TARGETS(NUM_TARGETS)
) dut (
    .clk(clk),
    .rst(rst),

    .frame_start(frame_start),

    .measurement_valid(measurement_valid),
    .z_in(z_in),

    .load_enable(load_enable),
    .load_target(load_target),

    .x_load(x_load),
    .P_load(P_load),

    .load_done(load_done),

    .write_enable(write_enable),

    .x_write(x_write),
    .P_write(P_write),

    .F_mat(F_mat),
    .Q(Q_mat),
    .R_diag(R_diag),

    .prediction_horizon(prediction_horizon),
    .collision_threshold(collision_threshold),

    .busy(busy),
    .frame_done(frame_done),

    .target_id(target_id),

    .x_out(x_out),
    .P_out(P_out),

    .collision_busy(collision_busy),
    .collision_done(collision_done),

    .collision_detected(collision_detected),

    .collision_matrix(collision_matrix)
);


// ============================================================
// Configuration
// ============================================================

task automatic configure_default;

    integer i;
    integer j;

    begin

        for (i = 0; i < 4; i = i + 1) begin

            for (j = 0; j < 4; j = j + 1) begin

                F_mat[i][j] = '0;
                Q_mat[i][j] = '0;

            end

        end

        R_diag[0] = '0;
        R_diag[1] = '0;


        F_mat[0][0] = Q_SCALE;
        F_mat[1][1] = Q_SCALE;
        F_mat[2][2] = Q_SCALE;
        F_mat[3][3] = Q_SCALE;

        // dt = 0.01
        // 0.01 * 4096 ~= 41

        F_mat[0][2] = 41;
        F_mat[1][3] = 41;


        Q_mat[0][0] = 1;
        Q_mat[1][1] = 1;
        Q_mat[2][2] = 1;
        Q_mat[3][3] = 1;


        R_diag[0] = 1;
        R_diag[1] = 1;


        prediction_horizon = DEFAULT_HORIZON;
        collision_threshold = DEFAULT_THRESHOLD;

    end

endtask


// ============================================================
// Load one target
// ============================================================

task automatic load_target_state(
    input integer tid,
    input integer px,
    input integer py,
    input integer vx,
    input integer vy
);

    integer i;
    integer j;

    begin

        for (i = 0; i < 4; i = i + 1) begin

            x_load[i] = '0;

            for (j = 0; j < 4; j = j + 1) begin

                P_load[i][j] = '0;

            end

        end


        x_load[0] = px * Q_SCALE;
        x_load[1] = py * Q_SCALE;
        x_load[2] = vx * Q_SCALE;
        x_load[3] = vy * Q_SCALE;


        P_load[0][0] = Q_SCALE;
        P_load[1][1] = Q_SCALE;
        P_load[2][2] = Q_SCALE;
        P_load[3][3] = Q_SCALE;


        @(negedge clk);

        load_target = tid;

        load_enable = 1'b1;

        @(negedge clk);

        load_enable = 1'b0;

        wait (load_done == 1'b1);

        #1;

    end

endtask


// ============================================================
// Start one frame
// ============================================================

task automatic start_frame;

    begin

        @(negedge clk);

        frame_start = 1'b1;

        @(negedge clk);

        frame_start = 1'b0;

    end

endtask


// ============================================================
// Generic measurement sequence
// ============================================================

task automatic supply_measurements(
    input integer offset
);

    integer t;

    begin

        for (t = 0; t < NUM_TARGETS; t = t + 1) begin

            wait (busy == 1'b1);

            wait (target_id == t);


            z_in[0] =
                ((t + 1) * 10 + offset) * Q_SCALE;

            z_in[1] =
                ((t + 1) * 10 + offset) * Q_SCALE;


            @(negedge clk);

            measurement_valid = 1'b1;

            @(negedge clk);

            measurement_valid = 1'b0;

        end

    end

endtask


// ============================================================
// Collision corner-case measurement sequence
//
// These tests intentionally measure each target at its loaded
// position.  The Kalman update therefore does not move the
// zero-velocity test targets away from the positions being tested.
//
// test_case = 5 : target 2 = 50, target 3 = 52
// test_case = 6 : target 2 = 50, target 3 = 53
// test_case = 7 : target 0 = 10, target 1 = 11,
//                  target 2 = 50, target 3 = 51
// test_case = 8 : target 4 = 70, target 5 = 73
// ============================================================

task automatic supply_collision_test_measurements(
    input integer test_case
);

    integer t;
    integer mx;
    integer my;

    begin

        for (t = 0; t < NUM_TARGETS; t = t + 1) begin

            wait (busy == 1'b1);
            wait (target_id == t);

            // Default: measurement at the normal loaded position.
            mx = (t + 1) * 10;
            my = (t + 1) * 10;

            case (test_case)

                5: begin
                    if (t == 2) begin
                        mx = 50;
                        my = 50;
                    end
                    else if (t == 3) begin
                        mx = 52;
                        my = 50;
                    end
                end

                6: begin
                    if (t == 2) begin
                        mx = 50;
                        my = 50;
                    end
                    else if (t == 3) begin
                        mx = 53;
                        my = 50;
                    end
                end

                7: begin
                    if (t == 0) begin
                        mx = 10;
                        my = 10;
                    end
                    else if (t == 1) begin
                        mx = 11;
                        my = 10;
                    end
                    else if (t == 2) begin
                        mx = 50;
                        my = 50;
                    end
                    else if (t == 3) begin
                        mx = 51;
                        my = 50;
                    end
                end

                8: begin
                    if (t == 4) begin
                        mx = 70;
                        my = 70;
                    end
                    else if (t == 5) begin
                        mx = 73;
                        my = 70;
                    end
                end

                default: begin
                end

            endcase

            z_in[0] = mx * Q_SCALE;
            z_in[1] = my * Q_SCALE;

            @(negedge clk);
            measurement_valid = 1'b1;

            @(negedge clk);
            measurement_valid = 1'b0;

        end

    end

endtask


// ============================================================
// TEST 9 measurement sequence
//
// IMPORTANT:
//
// The Kalman engine performs a prediction before the measurement
// update. Therefore the measurement must not introduce an
// artificial velocity correction.
//
// Target 6:
//
//     initial x  = 0
//     initial vx = +5
//
// With the configured fixed-point F matrix:
//
//     x_pred = 0 + (5 * 41 >>> 12)
//            = 0
//
// Therefore measurement = 0 produces zero position innovation.
//
//
// Target 7:
//
//     initial x  = 12
//     initial vx = -5
//
// Fixed-point prediction:
//
//     x_pred = 12 + (-5 * 41 >>> 12)
//            = 12 - 1
//            = 11
//
// Therefore measurement = 11 produces zero position innovation.
//
// This preserves the intended loaded velocities:
//
//     target 6: vx = +5
//     target 7: vx = -5
//
// The collision engine subsequently applies:
//
//     horizon = 1 second
//
// giving:
//
//     target 6 -> x = 0 + 5  = 5
//     target 7 -> x = 11 - 5 = 6
//
// Separation = 1 <= threshold 2.
//
// Thus TEST 9 verifies that the configured prediction horizon
// actually affects collision prediction rather than accidentally
// testing a velocity modified by the Kalman measurement update.
// ============================================================

task automatic supply_horizon_test_measurements;

    integer t;

    begin

        for (t = 0; t < NUM_TARGETS; t = t + 1) begin

            wait (busy == 1'b1);

            wait (target_id == t);


            if (t == 6) begin

                // Predicted fixed-point position = 0

                z_in[0] = 0;
                z_in[1] = 0;

            end
            else if (t == 7) begin

                // Predicted fixed-point position = 11

                z_in[0] = 11 * Q_SCALE;
                z_in[1] = 0;

            end
            else begin

                z_in[0] =
                    ((t + 1) * 10) * Q_SCALE;

                z_in[1] =
                    ((t + 1) * 10) * Q_SCALE;

            end


            @(negedge clk);

            measurement_valid = 1'b1;

            @(negedge clk);

            measurement_valid = 1'b0;

        end

    end

endtask


// ============================================================
// Wait for frame completion
// ============================================================

task automatic wait_frame_done;

    begin

        // frame_done only means the Kalman frame has completed.
        // Collision prediction starts after that boundary and runs
        // asynchronously from the frame transaction.
        wait (frame_done == 1'b1);

        // The collision matrix and collision_detected are not valid
        // for checking until collision_done is asserted.
        wait (collision_done == 1'b1);

        #1;

    end

endtask


// ============================================================
// Check collision outputs for X
// ============================================================

task automatic check_collision_outputs;

    integer i;
    integer j;

    begin

        if (^collision_detected === 1'bx) begin

            $display(
                "ERROR: collision_detected contains X"
            );

            errors = errors + 1;

        end


        for (i = 0; i < NUM_TARGETS; i = i + 1) begin

            for (j = 0; j < NUM_TARGETS; j = j + 1) begin

                if (collision_matrix[i][j] === 1'bx) begin

                    $display(
                        "ERROR: collision_matrix[%0d][%0d] contains X",
                        i,
                        j
                    );

                    errors = errors + 1;

                end

            end

        end

    end

endtask


// ============================================================
// Initial regression
// ============================================================

integer i;
integer j;

initial begin

    errors = 0;

    regression_done = 1'b0;
    regression_pass = 1'b0;

    frame_start = 1'b0;
    measurement_valid = 1'b0;

    load_enable = 1'b0;
    load_target = '0;

    write_enable = 1'b0;

    z_in[0] = '0;
    z_in[1] = '0;


    for (i = 0; i < 4; i = i + 1) begin

        x_load[i] = '0;
        x_write[i] = '0;

        for (j = 0; j < 4; j = j + 1) begin

            P_load[i][j] = '0;
            P_write[i][j] = '0;

        end

    end


    configure_default();


    // ========================================================
    // TEST 1
    // ========================================================

    wait (rst == 1'b1);

    @(posedge clk);

    #1;

    if (busy !== 1'b0) begin

        $display(
            "ERROR: busy asserted during reset"
        );

        errors = errors + 1;

    end


    if (frame_done !== 1'b0) begin

        $display(
            "ERROR: frame_done asserted during reset"
        );

        errors = errors + 1;

    end


    if (collision_done !== 1'b0) begin

        $display(
            "ERROR: collision_done asserted during reset"
        );

        errors = errors + 1;

    end


    if (collision_detected !== 1'b0) begin

        $display(
            "ERROR: collision_detected not cleared by reset"
        );

        errors = errors + 1;

    end


    $display("");
    $display("==============================================");
    $display("AEI RESET / CONFIGURATION / CORNER REGRESSION");
    $display("==============================================");

    $display("");
    $display("TEST 1");
    $display("VERIFYING RESET");


    if (errors == 0) begin

        $display(
            "PASS: reset cleared system status"
        );

    end


    // ========================================================
    // TEST 2
    // ========================================================

    $display("");
    $display("TEST 2");
    $display("VERIFYING TARGET RELOAD AFTER RESET");


    wait (rst == 1'b0);

    @(posedge clk);


    for (i = 0; i < NUM_TARGETS; i = i + 1) begin

        load_target_state(
            i,
            (i + 1) * 10,
            (i + 1) * 10,
            0,
            0
        );

        $display(
            "PASS: target %0d reloaded",
            i
        );

    end


    // ========================================================
    // TEST 3
    // ========================================================

    $display("");
    $display("TEST 3");
    $display("VERIFYING ZERO-VELOCITY TARGETS");


    start_frame();

    supply_measurements(0);

    wait_frame_done();


    if (target_id !== 7) begin

        $display(
            "ERROR: zero-velocity frame final target incorrect"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: zero-velocity frame completed"
        );

    end


    // ========================================================
    // TEST 4
    // ========================================================

    $display("");
    $display("TEST 4");
    $display("VERIFYING NEGATIVE VELOCITY");


    load_target_state(0, 20, 20, -5, -5);
    load_target_state(1, 40, 40, 5, 5);


    start_frame();

    supply_measurements(1);

    wait_frame_done();


    $display(
        "PASS: negative-velocity frame completed"
    );


    // ========================================================
    // TEST 5
    // ========================================================

    $display("");
    $display("TEST 5");
    $display("VERIFYING EXACT COLLISION THRESHOLD");


    load_target_state(2, 50, 50, 0, 0);
    load_target_state(3, 52, 50, 0, 0);


    start_frame();

    supply_collision_test_measurements(5);

    wait_frame_done();


    if (collision_matrix[2][3] !== 1'b1 ||
        collision_matrix[3][2] !== 1'b1) begin

        $display(
            "ERROR: exact threshold collision missing"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: exact threshold collision detected"
        );

    end


    // ========================================================
    // TEST 6
    // ========================================================

    $display("");
    $display("TEST 6");
    $display("VERIFYING JUST-OUTSIDE THRESHOLD");


    load_target_state(2, 50, 50, 0, 0);
    load_target_state(3, 53, 50, 0, 0);


    start_frame();

    supply_collision_test_measurements(6);

    wait_frame_done();


    if (collision_matrix[2][3] !== 1'b0 ||
        collision_matrix[3][2] !== 1'b0) begin

        $display(
            "ERROR: false collision just outside threshold"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: just-outside threshold rejected"
        );

    end


    // ========================================================
    // TEST 7
    // ========================================================

    $display("");
    $display("TEST 7");
    $display("VERIFYING MULTIPLE COLLISIONS");


    load_target_state(0, 10, 10, 0, 0);
    load_target_state(1, 11, 10, 0, 0);

    load_target_state(2, 50, 50, 0, 0);
    load_target_state(3, 51, 50, 0, 0);


    start_frame();

    supply_collision_test_measurements(7);

    wait_frame_done();


    if (collision_matrix[0][1] !== 1'b1 ||
        collision_matrix[1][0] !== 1'b1 ||
        collision_matrix[2][3] !== 1'b1 ||
        collision_matrix[3][2] !== 1'b1) begin

        $display(
            "ERROR: simultaneous collisions not detected"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: multiple simultaneous collisions detected"
        );

    end


    // ========================================================
    // TEST 8
    // ========================================================

    $display("");
    $display("TEST 8");
    $display("VERIFYING COLLISION THRESHOLD CONFIGURATION");


    collision_threshold = 4 * Q_SCALE;


    load_target_state(4, 70, 70, 0, 0);
    load_target_state(5, 73, 70, 0, 0);


    start_frame();

    supply_collision_test_measurements(8);

    wait_frame_done();


    if (collision_matrix[4][5] !== 1'b1) begin

        $display(
            "ERROR: configured threshold was not applied"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: changed collision threshold applied"
        );

    end


    // ========================================================
    // TEST 9
    // ========================================================

    $display("");
    $display("TEST 9");
    $display("VERIFYING PREDICTION-HORIZON CONFIGURATION");


    collision_threshold = DEFAULT_THRESHOLD;

    prediction_horizon = 4096;


    load_target_state(6, 0, 0, 5, 0);
    load_target_state(7, 12, 0, -5, 0);


    start_frame();

    supply_horizon_test_measurements();

    wait_frame_done();


    if (collision_matrix[6][7] !== 1'b1) begin

        $display(
            "ERROR: configured prediction horizon was not applied"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: changed prediction horizon applied"
        );

    end


    // ========================================================
    // TEST 10
    // ========================================================

    $display("");
    $display("TEST 10");
    $display("VERIFYING NO STALE COLLISION RESULT");


    prediction_horizon = DEFAULT_HORIZON;
    collision_threshold = DEFAULT_THRESHOLD;


    load_target_state(6, 0, 0, 0, 0);
    load_target_state(7, 40, 0, 0, 0);


    start_frame();

    supply_measurements(0);

    wait_frame_done();


    if (collision_detected !== 1'b0) begin

        $display(
            "ERROR: stale collision_detected remained asserted"
        );

        errors = errors + 1;

    end


    if (collision_matrix[6][7] !== 1'b0 ||
        collision_matrix[7][6] !== 1'b0) begin

        $display(
            "ERROR: stale collision matrix entry remained"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: collision results cleared correctly"
        );

    end


    // ========================================================
    // TEST 11
    // ========================================================

    $display("");
    $display("TEST 11");
    $display("VERIFYING OUTPUT SANITY");


    check_collision_outputs();


    if (^x_out[0] === 1'bx ||
        ^x_out[1] === 1'bx ||
        ^x_out[2] === 1'bx ||
        ^x_out[3] === 1'bx) begin

        $display(
            "ERROR: X detected in x_out"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: externally visible state outputs contain no X"
        );

    end


    // ========================================================
    // TEST 12
    // ========================================================

    $display("");
    $display("TEST 12");
    $display("VERIFYING FINAL IDLE STATE");


    @(posedge clk);

    if (busy !== 1'b0) begin

        $display(
            "ERROR: system remains busy"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: system returned to IDLE"
        );

    end


    // ========================================================
    // Final result
    // ========================================================

    $display("");
    $display("==============================================");
    $display("AEI CORNER REGRESSION COMPLETE");
    $display("==============================================");


    if (errors == 0) begin

        regression_pass = 1'b1;

        $display(
            "PASS: reset/configuration/corner-case regression passed."
        );

    end
    else begin

        regression_pass = 1'b0;

        $display(
            "FAIL: %0d errors detected.",
            errors
        );

    end


    regression_done = 1'b1;

end

endmodule
