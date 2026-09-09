`timescale 1ns / 1ps

module aei_engine #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer ACC_W = 44,
    parameter integer INV_W = 16,
    parameter integer NUM_TARGETS = 8
)(
    input logic clk,
    input logic rst,

    // ============================================================
    // Target selection
    // ============================================================

    input logic [$clog2(NUM_TARGETS)-1:0] target_id,

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
    // Kalman measurement/configuration
    // ============================================================

    input logic signed [W-1:0] z [0:1],

    input logic signed [W-1:0] F_mat [0:3][0:3],
    input logic signed [W-1:0] Q [0:3][0:3],
    input logic signed [W-1:0] R_diag [0:1],

    // ============================================================
    // Kalman control
    // ============================================================

    input logic start,

    output logic busy,
    output logic done,

    // ============================================================
    // Kalman outputs
    // ============================================================

    output logic signed [W-1:0] x_out [0:3],
    output logic signed [W-1:0] P_out [0:3][0:3]
);


    localparam integer ID_W = $clog2(NUM_TARGETS);


    // ============================================================
    // Target memory read interface
    // ============================================================

    logic signed [W-1:0] x_mem_read [0:3];
    logic signed [W-1:0] P_mem_read [0:3][0:3];

    logic mem_read_valid;


    // ============================================================
    // Kalman start control
    // ============================================================

    logic kalman_start;

    logic kalman_busy;
    logic kalman_done;


    // ============================================================
    // Latched Kalman inputs
    //
    // The target-memory read is synchronous. Therefore, when the
    // external start request arrives, target_id is captured first.
    // On the following cycle the memory output is valid, and the
    // Kalman engine can be started using that target's state.
    // ============================================================

    logic start_pending;

    logic [ID_W-1:0] selected_target;


    // ============================================================
    // Target memory
    // ============================================================

    target_memory #(
        .W(W),
        .NUM_TARGETS(NUM_TARGETS)
    ) u_target_memory (
        .clk(clk),
        .rst(rst),

        .target_id(selected_target),

        .write_enable(write_enable),

        .x_write(x_write),
        .P_write(P_write),

        .x_read(x_mem_read),
        .P_read(P_mem_read),

        .read_valid(mem_read_valid),

        .load_enable(load_enable),

        .load_target(load_target),

        .x_load(x_load),
        .P_load(P_load),

        .load_done(load_done)
    );


    // ============================================================
    // Kalman engine
    // ============================================================

    kalman_engine #(
        .W(W),
        .F(F),
        .ACC_W(ACC_W),
        .INV_W(INV_W)
    ) u_kalman_engine (
        .clk(clk),
        .rst(rst),
        .start(kalman_start),

        .x_in(x_mem_read),
        .P_in(P_mem_read),
        .z(z),

        .F_mat(F_mat),
        .Q(Q),
        .R_diag(R_diag),

        .x_out(x_out),
        .P_out(P_out),

        .busy(kalman_busy),
        .done(kalman_done)
    );


    // ============================================================
    // Integration controller
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            selected_target <= '0;

            start_pending <= 1'b0;

            kalman_start <= 1'b0;

            busy <= 1'b0;
            done <= 1'b0;

        end
        else begin

            // ----------------------------------------------------
            // One-clock pulse outputs
            // ----------------------------------------------------

            kalman_start <= 1'b0;
            done <= 1'b0;


            // ----------------------------------------------------
            // External start request
            //
            // Only accept a new request when the Kalman engine is
            // idle and no previous memory-read request is pending.
            // ----------------------------------------------------

            if (start && !busy && !start_pending) begin

                selected_target <= target_id;

                start_pending <= 1'b1;

                busy <= 1'b1;

            end


            // ----------------------------------------------------
            // Wait for synchronous target-memory read
            //
            // target_memory updates x_mem_read/P_mem_read on the
            // clock edge after selected_target has been changed.
            //
            // mem_read_valid is therefore used as the indication
            // that the memory read has completed.
            // ----------------------------------------------------

            if (start_pending && mem_read_valid) begin

                kalman_start <= 1'b1;

                start_pending <= 1'b0;

            end


            // ----------------------------------------------------
            // Wait for Kalman engine completion
            // ----------------------------------------------------

            if (kalman_done) begin

                busy <= 1'b0;

                done <= 1'b1;

            end

        end

    end

endmodule