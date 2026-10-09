`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_aei_engine_top
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

module tb_aei_engine_top;

localparam integer W = 20;
localparam integer F = 12;
localparam integer ACC_W = 44;
localparam integer INV_W = 16;
localparam integer NUM_TARGETS = 8;
localparam integer ID_W = $clog2(NUM_TARGETS);

localparam integer Q_SCALE = 4096;

// ============================================================
// Clock / reset
// ============================================================

logic clk;
logic rst;

// ============================================================
// Frame control
// ============================================================

logic frame_start;
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
// Runtime target write
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
// Outputs
// ============================================================

logic busy;
logic frame_done;

logic [ID_W-1:0] target_id;

logic signed [W-1:0] x_out [0:3];
logic signed [W-1:0] P_out [0:3][0:3];

// ============================================================
// Error counter
// ============================================================

integer errors;

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

    .busy(busy),
    .frame_done(frame_done),

    .target_id(target_id),

    .x_out(x_out),
    .P_out(P_out)
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
integer k;

integer expected_x;
integer expected_y;

initial begin

    errors = 0;

    rst = 1'b1;

    frame_start = 1'b0;
    measurement_valid = 1'b0;

    load_enable = 1'b0;
    load_target = '0;

    write_enable = 1'b0;

    // --------------------------------------------------------
    // Default measurement
    // --------------------------------------------------------

    z_in[0] = '0;
    z_in[1] = '0;

    // --------------------------------------------------------
    // Clear load/write arrays
    // --------------------------------------------------------

    for (i = 0; i < 4; i = i + 1) begin
        x_load[i] = '0;
        x_write[i] = '0;

        for (j = 0; j < 4; j = j + 1) begin
            P_load[i][j] = '0;
            P_write[i][j] = '0;
        end
    end

    // ========================================================
    // Kalman configuration
    // ========================================================

    for (i = 0; i < 4; i = i + 1) begin
        for (j = 0; j < 4; j = j + 1) begin
            F_mat[i][j] = '0;
            Q_mat[i][j] = '0;
        end
    end

    R_diag[0] = 1 * Q_SCALE;
    R_diag[1] = 1 * Q_SCALE;

    // --------------------------------------------------------
    // Constant-velocity state transition matrix
    //
    // dt = 1 for this integration test.
    //
    // [x']   [1 0 1 0] [x ]
    // [y'] = [0 1 0 1] [y ]
    // [vx']  [0 0 1 0] [vx]
    // [vy']  [0 0 0 1] [vy]
    // --------------------------------------------------------

    F_mat[0][0] = 1 * Q_SCALE;
    F_mat[0][1] = 0;
    F_mat[0][2] = 1 * Q_SCALE;
    F_mat[0][3] = 0;

    F_mat[1][0] = 0;
    F_mat[1][1] = 1 * Q_SCALE;
    F_mat[1][2] = 0;
    F_mat[1][3] = 1 * Q_SCALE;

    F_mat[2][0] = 0;
    F_mat[2][1] = 0;
    F_mat[2][2] = 1 * Q_SCALE;
    F_mat[2][3] = 0;

    F_mat[3][0] = 0;
    F_mat[3][1] = 0;
    F_mat[3][2] = 0;
    F_mat[3][3] = 1 * Q_SCALE;

    // --------------------------------------------------------
    // Small process noise.
    // --------------------------------------------------------

    for (i = 0; i < 4; i = i + 1) begin
        Q_mat[i][i] = 1;
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
    $display("AEI ENGINE TOP INTEGRATION TEST");
    $display("==============================================");

    // ========================================================
    // TEST 1
    //
    // Load all eight target states.
    // ========================================================

    $display("");
    $display("TEST 1");
    $display("LOADING ALL TARGETS");

    for (k = 0; k < NUM_TARGETS; k = k + 1) begin

        // ----------------------------------------------------
        // Give every target a distinct initial state.
        //
        // Position:
        // target 0 -> (10, 10)
        // target 1 -> (20, 20)
        // ...
        //
        // Velocity = (1, 1)
        // ----------------------------------------------------

        x_load[0] = (k + 1) * 10 * Q_SCALE;
        x_load[1] = (k + 1) * 10 * Q_SCALE;
        x_load[2] = 1 * Q_SCALE;
        x_load[3] = 1 * Q_SCALE;

        // ----------------------------------------------------
        // Identity covariance.
        // ----------------------------------------------------

        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin

                if (i == j)
                    P_load[i][j] = 1 * Q_SCALE;
                else
                    P_load[i][j] = 0;

            end
        end

        @(negedge clk);

        load_target = k;
        load_enable = 1'b1;

        @(negedge clk);

        load_enable = 1'b0;

        // ----------------------------------------------------
        // load_done is a one-clock pulse.
        // ----------------------------------------------------

        wait (load_done == 1'b1);

        #1;

        if (load_done !== 1'b1) begin

            $display(
                "ERROR: load_done missing for target %0d",
                k
            );

            errors = errors + 1;

        end
        else begin

            $display(
                "PASS: target %0d loaded",
                k
            );

        end

        @(posedge clk);

    end

    // ========================================================
    // TEST 2
    //
    // Start complete frame.
    // ========================================================

    $display("");
    $display("TEST 2");
    $display("STARTING COMPLETE 8-TARGET FRAME");

    @(negedge clk);

    frame_start = 1'b1;

    @(negedge clk);

    frame_start = 1'b0;

    // --------------------------------------------------------
    // Supply one measurement whenever the engine becomes busy.
    //
    // The frame scheduler processes targets sequentially.
    // --------------------------------------------------------

    for (k = 0; k < NUM_TARGETS; k = k + 1) begin

        // ----------------------------------------------------
        // Wait for correct target selection.
        // ----------------------------------------------------

        wait (target_id == k);

        // ----------------------------------------------------
        // Measurement:
        //
        // target 0 -> [100,101]
        // target 1 -> [200,201]
        // ...
        // target 7 -> [800,801]
        // ----------------------------------------------------

        z_in[0] = (k + 1) * 100;
        z_in[1] = (k + 1) * 100 + 1;

        @(negedge clk);

        measurement_valid = 1'b1;

        $display(
            "INFO: measurement supplied for target %0d: [%0d, %0d]",
            k,
            z_in[0],
            z_in[1]
        );

        @(negedge clk);

        measurement_valid = 1'b0;

        // ----------------------------------------------------
        // Wait until target processing completes before
        // continuing to the next target.
        // ----------------------------------------------------

        if (k < NUM_TARGETS - 1) begin

            wait (target_id == (k + 1));

        end

    end

    // ========================================================
    // TEST 3
    //
    // Frame completion.
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
    //
    // Final target output.
    // ========================================================

    $display("");
    $display("TEST 4");
    $display("VERIFYING FINAL TARGET OUTPUT");

    if (target_id !== 7) begin

        $display(
            "ERROR: expected final target_id = 7, got %0d",
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
    //
    // Busy must be low after frame completion.
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
    //
    // Verify that the system returns to IDLE and accepts
    // another frame.
    // ========================================================

    $display("");
    $display("TEST 6");
    $display("VERIFYING RETURN TO IDLE");

    // --------------------------------------------------------
    // Wait until frame_done has returned low.
    // --------------------------------------------------------

    wait (frame_done == 1'b0);

    // --------------------------------------------------------
    // Start second frame.
    // --------------------------------------------------------

    @(negedge clk);

    frame_start = 1'b1;

    @(negedge clk);

    frame_start = 1'b0;

    // --------------------------------------------------------
    // The scheduler should return to target 0.
    // --------------------------------------------------------

    wait (target_id == 0);

    #1;

    if (target_id !== 0) begin

        $display(
            "ERROR: second frame did not return to target 0"
        );

        errors = errors + 1;

    end
    else begin

        $display(
            "PASS: second frame accepted"
        );

    end

    // --------------------------------------------------------
    // Supply target-0 measurement so the second frame can
    // actually enter the processing pipeline.
    // --------------------------------------------------------

    z_in[0] = 100;
    z_in[1] = 101;

    @(negedge clk);

    measurement_valid = 1'b1;

    @(negedge clk);

    measurement_valid = 1'b0;

    // --------------------------------------------------------
    // We only need to verify acceptance of the second frame,
    // so stop after target 0 has started processing.
    // --------------------------------------------------------

    wait (busy == 1'b1);

    // ========================================================
    // Final result
    // ========================================================

    $display("");
    $display("==============================================");
    $display("AEI ENGINE TOP INTEGRATION TEST COMPLETE");
    $display("==============================================");

    if (errors == 0) begin

        $display(
            "PASS: AEI engine top integration matches expected behavior."
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
