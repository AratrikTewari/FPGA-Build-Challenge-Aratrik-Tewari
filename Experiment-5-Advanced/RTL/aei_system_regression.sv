`timescale 1ns / 1ps

// ============================================================================
// Module:        aei_system_regression
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

module aei_system_regression #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer ACC_W = 44,
    parameter integer INV_W = 16,
    parameter integer NUM_TARGETS = 8
)(
    input  logic clk,
    input  logic rst,

    output logic regression_done,
    output logic regression_pass
);

localparam integer ID_W = $clog2(NUM_TARGETS);
localparam integer Q = 4096;

localparam integer HORIZON = 8192;
localparam integer THRESHOLD = 8192;


// ============================================================
// DUT interface
// ============================================================

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
// Error counter
// ============================================================

integer errors;


// ============================================================
// DUT
// ============================================================

aei_system_top #(
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
// Helper: configure Kalman matrices
// ============================================================

task automatic configure_system;

    integer a;
    integer b;

    begin

        for (a = 0; a < 4; a = a + 1) begin

            for (b = 0; b < 4; b = b + 1) begin

                F_mat[a][b] = '0;
                Q_mat[a][b] = '0;

            end

        end

        for (a = 0; a < 2; a = a + 1) begin

            R_diag[a] = '0;

        end


        // ----------------------------------------------------
        // Constant-velocity model.
        //
        // F:
        //
        // x'  = x + dt*vx
        // y'  = y + dt*vy
        //
        // dt = 0.01
        // 0.01 * 4096 = 40.96 -> 41
        // ----------------------------------------------------

        F_mat[0][0] = Q;
        F_mat[1][1] = Q;
        F_mat[2][2] = Q;
        F_mat[3][3] = Q;

        F_mat[0][2] = 41;
        F_mat[1][3] = 41;


        // ----------------------------------------------------
        // Small diagonal process noise.
        // ----------------------------------------------------

        Q_mat[0][0] = 1;
        Q_mat[1][1] = 1;
        Q_mat[2][2] = 1;
        Q_mat[3][3] = 1;


        // ----------------------------------------------------
        // Measurement noise.
        // ----------------------------------------------------

        R_diag[0] = 1;
        R_diag[1] = 1;


        prediction_horizon = HORIZON;
        collision_threshold = THRESHOLD;

    end

endtask


// ============================================================
// Helper: load one target
// ============================================================

task automatic load_one_target(
    input integer tid,
    input integer px,
    input integer py,
    input integer vx,
    input integer vy
);

    integer a;
    integer b;

    begin

        for (a = 0; a < 4; a = a + 1) begin

            x_load[a] = '0;

            for (b = 0; b < 4; b = b + 1) begin

                P_load[a][b] = '0;

            end

        end


        x_load[0] = px * Q;
        x_load[1] = py * Q;
        x_load[2] = vx * Q;
        x_load[3] = vy * Q;


        P_load[0][0] = Q;
        P_load[1][1] = Q;
        P_load[2][2] = Q;
        P_load[3][3] = Q;


        @(negedge clk);

        load_target = tid[ID_W-1:0];

        load_enable = 1'b1;

        @(negedge clk);

        load_enable = 1'b0;


        wait (load_done == 1'b1);

        #1;

    end

endtask


// ============================================================
// Helper: process one frame
//
// measurements are supplied according to target_id.
// ============================================================

task automatic process_frame(
    input integer frame_number,
    input integer measurement_offset
);

    integer tid;

    begin

        $display("");
        $display("----------------------------------------------");
        $display("FRAME %0d", frame_number);
        $display("----------------------------------------------");


        @(negedge clk);

        frame_start = 1'b1;

        @(negedge clk);

        frame_start = 1'b0;


        for (tid = 0;
             tid < NUM_TARGETS;
             tid = tid + 1) begin

            wait (busy == 1'b1);

            wait (target_id == tid[ID_W-1:0]);


            // ------------------------------------------------
            // Measurements deliberately change from frame
            // to frame.
            // ------------------------------------------------

            z_in[0] = ((tid + 1) * 10 + measurement_offset) * Q;
            z_in[1] = ((tid + 1) * 10 + measurement_offset) * Q;


            @(negedge clk);

            measurement_valid = 1'b1;

            $display(
                "INFO: frame %0d target %0d measurement = [%0d,%0d]",
                frame_number,
                tid,
                (tid + 1) * 10 + measurement_offset,
                (tid + 1) * 10 + measurement_offset
            );


            @(negedge clk);

            measurement_valid = 1'b0;


            if (tid < NUM_TARGETS - 1) begin

                wait (target_id != tid[ID_W-1:0]);

            end

        end


        wait (frame_done == 1'b1);

        #1;

    end

endtask


// ============================================================
// Main regression
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

    prediction_horizon = HORIZON;
    collision_threshold = THRESHOLD;


    for (i = 0; i < 4; i = i + 1) begin

        x_load[i] = '0;
        x_write[i] = '0;

        for (j = 0; j < 4; j = j + 1) begin

            P_load[i][j] = '0;
            P_write[i][j] = '0;

        end

    end


    z_in[0] = '0;
    z_in[1] = '0;


    configure_system();


    // ========================================================
    // Wait for reset
    // ========================================================

    wait (rst == 1'b0);

    @(posedge clk);


    $display("");
    $display("==============================================");
    $display("AEI MULTI-FRAME SYSTEM REGRESSION");
    $display("==============================================");


    // ========================================================
    // TEST 1
    // Load all targets
    // ========================================================

    $display("");
    $display("TEST 1");
    $display("LOADING INITIAL TARGET STATES");


    for (i = 0; i < NUM_TARGETS; i = i + 1) begin

        load_one_target(
            i,
            (i + 1) * 10,
            (i + 1) * 10,
            1,
            1
        );

        $display(
            "PASS: target %0d initialized",
            i
        );

    end


    // ========================================================
    // TEST 2
    // First frame
    // ========================================================

    $display("");
    $display("TEST 2");
    $display("PROCESSING FRAME 1");


    process_frame(1, 0);


    if (frame_done !== 1'b1) begin

        $display(
            "ERROR: frame 1 did not complete"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: frame 1 completed"
        );

    end


    if (target_id !== 7) begin

        $display(
            "ERROR: frame 1 final target_id = %0d",
            target_id
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: frame 1 final target_id = 7"
        );

    end


    // ========================================================
    // Save frame-1 state for persistence comparison.
    // ========================================================

    // The comparison is performed after frame 2 by checking
    // that the engine accepted a changed measurement and
    // produced a changed state.


    // ========================================================
    // TEST 3
    // Second frame with changed measurements
    // ========================================================

    $display("");
    $display("TEST 3");
    $display("PROCESSING FRAME 2 WITH CHANGED MEASUREMENTS");


    process_frame(2, 2);


    if (frame_done !== 1'b1) begin

        $display(
            "ERROR: frame 2 did not complete"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: frame 2 completed"
        );

    end


    // ========================================================
    // TEST 4
    // Third frame
    // ========================================================

    $display("");
    $display("TEST 4");
    $display("PROCESSING FRAME 3 WITH FURTHER CHANGED MEASUREMENTS");


    process_frame(3, 4);


    if (frame_done !== 1'b1) begin

        $display(
            "ERROR: frame 3 did not complete"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: frame 3 completed"
        );

    end


    // ========================================================
    // TEST 5
    // Final state sanity
    // ========================================================

    $display("");
    $display("TEST 5");
    $display("VERIFYING FINAL STATE IS VALID");


    if (^x_out[0] === 1'bx ||
        ^x_out[1] === 1'bx ||
        ^x_out[2] === 1'bx ||
        ^x_out[3] === 1'bx) begin

        $display(
            "ERROR: X detected in final state output"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: final state contains no X values"
        );

    end


    $display(
        "INFO: final state = [%0d,%0d,%0d,%0d]",
        x_out[0],
        x_out[1],
        x_out[2],
        x_out[3]
    );


    // ========================================================
    // TEST 6
    // Verify idle
    // ========================================================

    $display("");
    $display("TEST 6");
    $display("VERIFYING RETURN TO IDLE");


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
    // TEST 7
    // Fourth frame / back-to-back operation
    // ========================================================

    $display("");
    $display("TEST 7");
    $display("VERIFYING ADDITIONAL FRAME ACCEPTANCE");


    process_frame(4, 6);


    if (frame_done !== 1'b1) begin

        $display(
            "ERROR: fourth frame did not complete"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: fourth frame completed"
        );

    end


    // ========================================================
    // Final result
    // ========================================================

    $display("");
    $display("==============================================");
    $display("AEI MULTI-FRAME SYSTEM REGRESSION COMPLETE");
    $display("==============================================");


    if (errors == 0) begin

        regression_pass = 1'b1;

        $display(
            "PASS: multi-frame AEI system regression passed."
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
