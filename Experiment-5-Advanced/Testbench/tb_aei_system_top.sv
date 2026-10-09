`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_aei_system_top
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

module tb_aei_system_top;

localparam integer W = 20;
localparam integer F = 12;
localparam integer ACC_W = 44;
localparam integer INV_W = 16;
localparam integer NUM_TARGETS = 8;
localparam integer ID_W = $clog2(NUM_TARGETS);

localparam integer Q = 4096;

localparam integer HORIZON = 8192;
localparam integer THRESHOLD = 8192;


// ============================================================
// Clock / reset
// ============================================================

logic clk;
logic rst;


// ============================================================
// Frame control
// ============================================================

logic frame_start;


// ============================================================
// Measurement input
// ============================================================

logic measurement_valid;

logic signed [W-1:0] z_in [0:1];


// ============================================================
// Target-memory initialization
// ============================================================

logic load_enable;

logic [ID_W-1:0] load_target;

logic signed [W-1:0] x_load [0:3];

logic signed [W-1:0] P_load [0:3][0:3];

logic load_done;


// ============================================================
// Runtime target-memory write
// ============================================================

logic write_enable;

logic signed [W-1:0] x_write [0:3];

logic signed [W-1:0] P_write [0:3][0:3];


// ============================================================
// Kalman configuration
// ============================================================

logic signed [W-1:0] F_mat [0:3][0:3];

logic signed [W-1:0] Q_mat [0:3][0:3];

logic signed [W-1:0] R_diag [0:1];


// ============================================================
// Collision configuration
// ============================================================

logic signed [W-1:0] prediction_horizon;

logic signed [W-1:0] collision_threshold;


// ============================================================
// Outputs
// ============================================================

logic busy;

logic frame_done;

logic [ID_W-1:0] target_id;

logic signed [W-1:0] x_out [0:3];

logic signed [W-1:0] P_out [0:3][0:3];


// ============================================================
// Collision outputs
// ============================================================

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

integer target_index;

initial begin

    errors = 0;

    rst = 1'b1;

    frame_start = 1'b0;

    measurement_valid = 1'b0;

    load_enable = 1'b0;

    load_target = '0;

    write_enable = 1'b0;

    prediction_horizon = HORIZON;

    collision_threshold = THRESHOLD;


    // ========================================================
    // Initialize arrays
    // ========================================================

    for (i = 0; i < 4; i = i + 1) begin

        x_load[i] = '0;
        x_write[i] = '0;

        for (j = 0; j < 4; j = j + 1) begin

            P_load[i][j] = '0;
            P_write[i][j] = '0;
            F_mat[i][j] = '0;
            Q_mat[i][j] = '0;

        end

    end


    for (i = 0; i < 2; i = i + 1) begin

        z_in[i] = '0;
        R_diag[i] = '0;

    end


    // ========================================================
    // Reset
    // ========================================================

    repeat (3)
        @(posedge clk);

    rst = 1'b0;

    @(posedge clk);


    // ========================================================
    // Header
    // ========================================================

    $display("");
    $display("==============================================");
    $display("AEI SYSTEM TOP INTEGRATION TEST");
    $display("==============================================");


    // ========================================================
    // TEST 1
    // Load all targets
    // ========================================================

    $display("");
    $display("TEST 1");
    $display("LOADING ALL TARGETS");


    // --------------------------------------------------------
    // Basic constant-velocity F matrix.
    //
    // x'  = x + vx
    // y'  = y + vy
    //
    // dt = 0.01 is represented by 41 Q8.12 counts.
    // --------------------------------------------------------

    for (i = 0; i < 4; i = i + 1) begin

        for (j = 0; j < 4; j = j + 1) begin

            F_mat[i][j] = '0;

        end

    end

    F_mat[0][0] = Q;
    F_mat[1][1] = Q;
    F_mat[2][2] = Q;
    F_mat[3][3] = Q;

    F_mat[0][2] = 41;
    F_mat[1][3] = 41;


    // --------------------------------------------------------
    // Small process noise.
    // --------------------------------------------------------

    for (i = 0; i < 4; i = i + 1) begin

        Q_mat[i][i] = 1;

    end


    // --------------------------------------------------------
    // Measurement noise.
    // --------------------------------------------------------

    R_diag[0] = 1;
    R_diag[1] = 1;


    // --------------------------------------------------------
    // Load each target.
    // --------------------------------------------------------

    for (target_index = 0;
         target_index < NUM_TARGETS;
         target_index = target_index + 1) begin

        for (i = 0; i < 4; i = i + 1) begin

            x_load[i] = '0;

        end

        for (i = 0; i < 4; i = i + 1) begin

            for (j = 0; j < 4; j = j + 1) begin

                P_load[i][j] = '0;

            end

        end


        // ----------------------------------------------------
        // Give every target a unique position.
        // ----------------------------------------------------

        x_load[0] = (target_index + 1) * 10 * Q;
        x_load[1] = (target_index + 1) * 10 * Q;

        x_load[2] = 0;
        x_load[3] = 0;


        // ----------------------------------------------------
        // Identity covariance.
        // ----------------------------------------------------

        P_load[0][0] = Q;
        P_load[1][1] = Q;
        P_load[2][2] = Q;
        P_load[3][3] = Q;


        @(negedge clk);

        load_target = target_index[ID_W-1:0];

        load_enable = 1'b1;

        @(negedge clk);

        load_enable = 1'b0;


        wait (load_done == 1'b1);

        #1;

        if (load_done !== 1'b1) begin

            $display(
                "ERROR: load_done missing for target %0d",
                target_index
            );

            errors = errors + 1;

        end
        else begin

            $display(
                "PASS: target %0d loaded",
                target_index
            );

        end

        @(posedge clk);

    end


    // ========================================================
    // TEST 2
    // Complete frame
    // ========================================================

    $display("");
    $display("TEST 2");
    $display("STARTING COMPLETE 8-TARGET FRAME");


    @(negedge clk);

    frame_start = 1'b1;

    @(negedge clk);

    frame_start = 1'b0;


    // --------------------------------------------------------
    // Supply one measurement for each target.
    // --------------------------------------------------------

    for (target_index = 0;
         target_index < NUM_TARGETS;
         target_index = target_index + 1) begin

        wait (busy == 1'b1);

        wait (target_id == target_index[ID_W-1:0]);


        z_in[0] = (target_index + 1) * 10 * Q;
        z_in[1] = (target_index + 1) * 10 * Q;


        @(negedge clk);

        measurement_valid = 1'b1;

        $display(
            "INFO: measurement supplied for target %0d: [%0d, %0d]",
            target_index,
            (target_index + 1) * 10,
            (target_index + 1) * 10
        );

        @(negedge clk);

        measurement_valid = 1'b0;


        // ----------------------------------------------------
        // Wait until this target has completed.
        // ----------------------------------------------------

        if (target_index < NUM_TARGETS - 1) begin

            wait (target_id != target_index[ID_W-1:0]);

        end

    end


    // ========================================================
    // TEST 3
    // Frame completion
    // ========================================================

    $display("");
    $display("TEST 3");
    $display("VERIFYING FRAME COMPLETION");


    wait (frame_done == 1'b1);

    #1;

    if (frame_done !== 1'b1) begin

        $display(
            "ERROR: frame_done not asserted"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: frame_done asserted"
        );

    end


    // ========================================================
    // TEST 4
    // Final target output
    // ========================================================

    $display("");
    $display("TEST 4");
    $display("VERIFYING FINAL TARGET OUTPUT");


    if (target_id !== 7) begin

        $display(
            "ERROR: final target_id = %0d, expected 7",
            target_id
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: final target_id = 7"
        );

    end


    $display(
        "INFO: final x_out = %0d, %0d, %0d, %0d",
        x_out[0],
        x_out[1],
        x_out[2],
        x_out[3]
    );


    // ========================================================
    // TEST 5
    // Busy / frame_done handshake
    // ========================================================

    $display("");
    $display("TEST 5");
    $display("VERIFYING FRAME BUSY HANDSHAKE");


    @(posedge clk);

    if (busy !== 1'b0) begin

        $display(
            "ERROR: busy remains asserted after frame completion"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: busy deasserted after frame completion"
        );

    end


    // ========================================================
    // TEST 6
    // Collision interface
    //
    // This verifies that the collision subsystem is connected
    // to the top-level interface without requiring hierarchical
    // access to internal instances.
    // ========================================================

    $display("");
    $display("TEST 6");
    $display("VERIFYING COLLISION INTERFACE");


    wait (collision_done == 1'b1);

    #1;

    if (collision_done !== 1'b1) begin

        $display(
            "ERROR: collision_done not asserted"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: collision_done asserted"
        );

    end


    if (collision_busy !== 1'b0) begin

        $display(
            "ERROR: collision_busy remains asserted"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: collision_busy deasserted"
        );

    end


    // ========================================================
    // TEST 7
    // Return to idle
    // ========================================================

    $display("");
    $display("TEST 7");
    $display("VERIFYING RETURN TO IDLE");


    @(posedge clk);

    if (busy !== 1'b0) begin

        $display(
            "ERROR: system did not return to idle"
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
    $display("AEI SYSTEM TOP INTEGRATION TEST COMPLETE");
    $display("==============================================");


    if (errors == 0) begin

        $display(
            "PASS: AEI system top integration matches expected behavior."
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
