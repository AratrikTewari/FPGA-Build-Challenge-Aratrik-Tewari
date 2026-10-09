`timescale 1ns / 1ps

// ============================================================================
// Module:        frame_engine
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

module frame_engine #(
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
    // Target-memory initialization/load interface
    // ============================================================

    input logic load_enable,

    input logic [$clog2(NUM_TARGETS)-1:0] load_target,

    input logic signed [W-1:0] x_load [0:3],
    input logic signed [W-1:0] P_load [0:3][0:3],

    output logic load_done,

    // ============================================================
    // Normal runtime target-memory write
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
    // Target completion
    //
    // One-clock pulse generated exactly when the currently
    // processed target completes.
    //
    // target_id identifies the completed target.
    // x_out/P_out contain that target's completed state.
    // ============================================================

    output logic target_done,

    // ============================================================
    // Last processed target
    // ============================================================

    output logic [$clog2(NUM_TARGETS)-1:0] target_id,

    // ============================================================
    // Kalman output
    //
    // These are the outputs produced by the most recently
    // completed target.
    // ============================================================

    output logic signed [W-1:0] x_out [0:3],
    output logic signed [W-1:0] P_out [0:3][0:3]
);


    localparam integer ID_W = $clog2(NUM_TARGETS);


    // ============================================================
    // Measurement scheduler signals
    // ============================================================

    logic [ID_W-1:0] scheduler_target_id;

    logic signed [W-1:0] scheduler_z [0:1];

    logic scheduler_target_start;

    logic scheduler_target_busy;
    logic scheduler_target_done;

    logic scheduler_busy;
    logic scheduler_frame_done;


    // ============================================================
    // AEI engine signals
    // ============================================================

    logic aei_busy;
    logic aei_done;

    logic signed [W-1:0] aei_x_out [0:3];
    logic signed [W-1:0] aei_P_out [0:3][0:3];


    // ============================================================
    // Measurement scheduler
    // ============================================================

    measurement_scheduler #(
        .NUM_TARGETS(NUM_TARGETS)
    ) u_measurement_scheduler (
        .clk(clk),
        .rst(rst),

        .frame_start(frame_start),

        .measurement_valid(measurement_valid),
        .z_in(z_in),

        .target_id(scheduler_target_id),
        .z_out(scheduler_z),

        .target_start(scheduler_target_start),

        .target_busy(scheduler_target_busy),
        .target_done(scheduler_target_done),

        .busy(scheduler_busy),
        .frame_done(scheduler_frame_done)
    );


    // ============================================================
    // AEI engine
    // ============================================================

    aei_engine #(
        .W(W),
        .F(F),
        .ACC_W(ACC_W),
        .INV_W(INV_W),
        .NUM_TARGETS(NUM_TARGETS)
    ) u_aei_engine (
        .clk(clk),
        .rst(rst),

        .target_id(scheduler_target_id),

        .load_enable(load_enable),
        .load_target(load_target),

        .x_load(x_load),
        .P_load(P_load),

        .load_done(load_done),

        .write_enable(write_enable),

        .x_write(x_write),
        .P_write(P_write),

        .z(scheduler_z),

        .F_mat(F_mat),
        .Q(Q),
        .R_diag(R_diag),

        .start(scheduler_target_start),

        .busy(aei_busy),
        .done(aei_done),

        .x_out(aei_x_out),
        .P_out(aei_P_out)
    );


    // ============================================================
    // Target completion handshake
    //
    // aei_done is the authoritative completion indication for
    // one Kalman target transaction.
    //
    // Therefore:
    //
    //     target_done = aei_done
    //
    // At the same transaction boundary:
    //
    //     target_id = scheduler_target_id
    //     x_out     = aei_x_out
    //     P_out     = aei_P_out
    //
    // aei_engine_top can now capture the completed target using
    // target_done rather than trying to infer completion from
    // target_id changes.
    // ============================================================

    assign scheduler_target_busy = aei_busy;

    assign scheduler_target_done = aei_done;


    // ============================================================
    // Frame-level outputs
    // ============================================================

    assign busy =
        scheduler_busy;

    assign frame_done =
        scheduler_frame_done;


    // ============================================================
    // Target completion output
    //
    // This is intentionally the AEI completion pulse rather than
    // scheduler_target_done being reconstructed indirectly.
    // ============================================================

    assign target_done =
        aei_done;


    // ============================================================
    // Target ID
    // ============================================================

    assign target_id =
        scheduler_target_id;


    // ============================================================
    // Kalman outputs
    // ============================================================

    assign x_out =
        aei_x_out;

    assign P_out =
        aei_P_out;


endmodule
