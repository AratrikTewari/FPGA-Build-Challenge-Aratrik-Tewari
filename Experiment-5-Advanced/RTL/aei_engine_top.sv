`timescale 1ns / 1ps

// ============================================================================
// Module:        aei_engine_top.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Top-level wrapper for the AEI (Algebraic Equivalent Indicator) engine. Manages AXI interfaces and orchestrates data flow into the core tracking engines.
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
module aei_engine_top #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer ACC_W = 44,
    parameter integer INV_W = 16,
    parameter integer NUM_TARGETS = 8
)(
    input logic clk,
    input logic rst,

    // ============================================================
    // Frame control
    // ============================================================

    input logic frame_start,

    // ============================================================
    // Measurement input
    // ============================================================

    input logic measurement_valid,

    input logic signed [W-1:0] z_in [0:1],

    // ============================================================
    // Target-memory initialization
    // ============================================================

    input logic load_enable,

    input logic [$clog2(NUM_TARGETS)-1:0] load_target,

    input logic signed [W-1:0] x_load [0:3],
    input logic signed [W-1:0] P_load [0:3][0:3],

    output logic load_done,

    // ============================================================
    // Runtime target-memory write
    // ============================================================

    input logic write_enable,

    input logic signed [W-1:0] x_write [0:3],
    input logic signed [W-1:0] P_write [0:3][0:3],

    // ============================================================
    // Kalman configuration
    // ============================================================

    input logic signed [W-1:0] F_mat [0:3][0:3],
    input logic signed [W-1:0] Q [0:3][0:3],
    input logic signed [W-1:0] R_diag [0:1],

    // ============================================================
    // Frame status
    // ============================================================

    output logic busy,
    output logic frame_done,

    // ============================================================
    // Last processed target
    // ============================================================

    output logic [$clog2(NUM_TARGETS)-1:0] target_id,

    // ============================================================
    // Last Kalman output
    // ============================================================

    output logic signed [W-1:0] x_out [0:3],
    output logic signed [W-1:0] P_out [0:3][0:3],

    // ============================================================
    // Collision prediction configuration
    // ============================================================

    input logic signed [W-1:0] prediction_horizon,
    input logic signed [W-1:0] collision_threshold,

    // ============================================================
    // Collision outputs
    // ============================================================

    output logic collision_matrix
        [0:NUM_TARGETS-1][0:NUM_TARGETS-1],

    output logic collision_detected,

    output logic collision_busy,
    output logic collision_done
);

    localparam integer ID_W = $clog2(NUM_TARGETS);


    // ============================================================
    // FRAME ENGINE
    // ============================================================

    logic frame_busy;
    logic frame_complete;
    logic frame_target_done;

    logic [ID_W-1:0] frame_target_id;

    logic signed [W-1:0] frame_x_out [0:3];
    logic signed [W-1:0] frame_P_out [0:3][0:3];


    frame_engine #(
        .W(W),
        .F(F),
        .ACC_W(ACC_W),
        .INV_W(INV_W),
        .NUM_TARGETS(NUM_TARGETS)
    ) u_frame_engine (
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
        .Q(Q),
        .R_diag(R_diag),

        .busy(frame_busy),
        .frame_done(frame_complete),
        .target_done(frame_target_done),

        .target_id(frame_target_id),

        .x_out(frame_x_out),
        .P_out(frame_P_out)
    );


    // ============================================================
    // COMPLETED TARGET TRANSACTION BUFFER
    //
    // This buffer contains exactly one completed frame:
    //
    //     final_target_state[t][0] = x
    //     final_target_state[t][1] = y
    //     final_target_state[t][2] = vx
    //     final_target_state[t][3] = vy
    //
    // collision_engine NEVER reads frame_x_out directly.
    // ============================================================

    logic signed [W-1:0] final_target_state
        [0:NUM_TARGETS-1][0:3];


    // ============================================================
    // LATCHED COLLISION CONFIGURATION
    //
    // Configuration belongs to the frame transaction.
    //
    // It is captured when the frame completes and remains constant
    // for the entire collision transaction.
    // ============================================================

    logic signed [W-1:0] collision_prediction_horizon;

    logic signed [W-1:0] collision_threshold_latched;


    // ============================================================
    // Target completion tracking
    //
    // frame_engine provides an explicit target_done pulse.
    // This is the authoritative indication that target_id/x_out
    // belong to a completed Kalman transaction.
    // ============================================================

    logic [ID_W-1:0] previous_target_id;


    // ============================================================
    // COLLISION ENGINE
    // ============================================================

    logic collision_start;

    logic collision_engine_busy;
    logic collision_engine_done;
    logic collision_engine_detected;

    logic collision_matrix_internal
        [0:NUM_TARGETS-1][0:NUM_TARGETS-1];


    collision_engine #(
        .W(W),
        .F(F),
        .NUM_TARGETS(NUM_TARGETS)
    ) u_collision_engine (
        .clk(clk),
        .rst(rst),

        .start(collision_start),

        .target_state(final_target_state),

        .prediction_horizon(collision_prediction_horizon),
        .collision_threshold(collision_threshold_latched),

        .collision_matrix(collision_matrix_internal),

        .collision_detected(collision_engine_detected),

        .busy(collision_engine_busy),
        .done(collision_engine_done)
    );


    // ============================================================
    // TRANSACTION CONTROLLER
    //
    // The transaction is:
    //
    //     FRAME
    //       |
    //       v
    //     CAPTURE
    //       |
    //       v
    //     HOLD
    //       |
    //       v
    //     START COLLISION
    //
    // There is deliberately a complete clock boundary between
    // final state capture and collision_engine start.
    // ============================================================

    typedef enum logic [1:0] {
        TX_IDLE,
        TX_CAPTURED,
        TX_START
    } tx_state_t;

    tx_state_t tx_state;


    // ============================================================
    // Completed-target capture
    //
    // Capture every target ONLY on target_done.  Do not infer
    // completion from target_id changes: the scheduler may change
    // target_id before the previous target's Kalman transaction
    // has actually completed.
    // ============================================================

    integer i;
    integer j;

    always_ff @(posedge clk) begin

        if (rst) begin

            previous_target_id <= '0;

            collision_prediction_horizon <= '0;
            collision_threshold_latched <= '0;

            for (i = 0; i < NUM_TARGETS; i = i + 1) begin

                for (j = 0; j < 4; j = j + 1) begin

                    final_target_state[i][j] <= '0;

                end

            end

        end
        else begin

            // Explicit target completion boundary.
            if (frame_target_done) begin

                for (j = 0; j < 4; j = j + 1) begin

                    final_target_state[frame_target_id][j]
                        <= frame_x_out[j];

                end

            end

            // The complete frame configuration is captured only
            // when the frame itself completes.
            if (frame_complete) begin

                collision_prediction_horizon
                    <= prediction_horizon;

                collision_threshold_latched
                    <= collision_threshold;

            end

            previous_target_id <= frame_target_id;

        end

    end


    // ============================================================
    // Transaction controller
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            tx_state <= TX_IDLE;

            collision_start <= 1'b0;

        end
        else begin

            // One-clock pulse.

            collision_start <= 1'b0;


            case (tx_state)


                // =================================================
                // IDLE
                //
                // Waiting for complete Kalman frame.
                // =================================================

                TX_IDLE: begin

                    if (frame_complete) begin

                        tx_state <= TX_CAPTURED;

                    end

                end


                // =================================================
                // CAPTURED
                //
                // The frame completion edge has already registered:
                //
                //     final_target_state[last]
                //     collision_prediction_horizon
                //     collision_threshold_latched
                //
                // Therefore this entire cycle is a synchronization
                // boundary.
                // =================================================

                TX_CAPTURED: begin

                    tx_state <= TX_START;

                end


                // =================================================
                // START
                //
                // All collision inputs are now stable.
                // =================================================

                TX_START: begin

                    collision_start <= 1'b1;

                    tx_state <= TX_IDLE;

                end


                // =================================================
                // Recovery
                // =================================================

                default: begin

                    tx_state <= TX_IDLE;

                    collision_start <= 1'b0;

                end

            endcase

        end

    end


    // ============================================================
    // FRAME OUTPUTS
    // ============================================================

    assign busy =
        frame_busy |
        collision_engine_busy;

    assign frame_done =
        frame_complete;

    assign target_id =
        frame_target_id;

    assign x_out =
        frame_x_out;

    assign P_out =
        frame_P_out;


    // ============================================================
    // COLLISION OUTPUTS
    // ============================================================

    assign collision_busy =
        collision_engine_busy;

    assign collision_done =
        collision_engine_done;

    assign collision_detected =
        collision_engine_detected;


    // ============================================================
    // Collision matrix forwarding
    // ============================================================

    genvar gi;
    genvar gj;

    generate

        for (
            gi = 0;
            gi < NUM_TARGETS;
            gi = gi + 1
        ) begin : GEN_MATRIX_I

            for (
                gj = 0;
                gj < NUM_TARGETS;
                gj = gj + 1
            ) begin : GEN_MATRIX_J

                assign collision_matrix[gi][gj]
                    = collision_matrix_internal[gi][gj];

            end

        end

    endgenerate

endmodule
