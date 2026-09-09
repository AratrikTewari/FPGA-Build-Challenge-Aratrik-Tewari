`timescale 1ns / 1ps

module measurement_scheduler #(
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
    //
    // One measurement is presented for the currently selected
    // target when measurement_valid is asserted.
    // ============================================================

    input logic measurement_valid,

    input logic signed [19:0] z_in [0:1],

    // ============================================================
    // Target processing interface
    //
    // The scheduler selects one target at a time and starts the
    // AEI/Kalman processing for that target.
    // ============================================================

    output logic [$clog2(NUM_TARGETS)-1:0] target_id,

    output logic signed [19:0] z_out [0:1],

    output logic target_start,

    input logic target_busy,
    input logic target_done,

    // ============================================================
    // Frame status
    // ============================================================

    output logic busy,
    output logic frame_done
);


    localparam integer ID_W = $clog2(NUM_TARGETS);


    // ============================================================
    // Internal state
    // ============================================================

    typedef enum logic [2:0] {
        IDLE,
        WAIT_MEASUREMENT,
        START_TARGET,
        WAIT_TARGET,
        FRAME_FINISH
    } state_t;

    state_t state;


    logic [ID_W-1:0] current_target;


    integer i;


    // ============================================================
    // Scheduler
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            state <= IDLE;

            current_target <= '0;

            target_id <= '0;

            z_out[0] <= '0;
            z_out[1] <= '0;

            target_start <= 1'b0;

            busy <= 1'b0;
            frame_done <= 1'b0;

        end
        else begin

            // ----------------------------------------------------
            // One-clock pulse outputs
            // ----------------------------------------------------

            target_start <= 1'b0;
            frame_done <= 1'b0;


            case (state)


                // =================================================
                // IDLE
                // =================================================

                IDLE: begin

                    busy <= 1'b0;

                    if (frame_start) begin

                        busy <= 1'b1;

                        current_target <= '0;
                        target_id <= '0;

                        state <= WAIT_MEASUREMENT;

                    end

                end


                // =================================================
                // WAIT_MEASUREMENT
                //
                // Wait until the measurement for the current target
                // is presented.
                // =================================================

                WAIT_MEASUREMENT: begin

                    busy <= 1'b1;

                    if (measurement_valid) begin

                        z_out[0] <= z_in[0];
                        z_out[1] <= z_in[1];

                        state <= START_TARGET;

                    end

                end


                // =================================================
                // START_TARGET
                //
                // Generate a one-clock target_start pulse.
                // =================================================

                START_TARGET: begin

                    busy <= 1'b1;

                    target_id <= current_target;

                    target_start <= 1'b1;

                    state <= WAIT_TARGET;

                end


                // =================================================
                // WAIT_TARGET
                //
                // Wait until the AEI/Kalman processing for the
                // selected target has completed.
                // =================================================

                WAIT_TARGET: begin

                    busy <= 1'b1;

                    if (target_done) begin

                        if (current_target == NUM_TARGETS - 1) begin

                            state <= FRAME_FINISH;

                        end
                        else begin

                            current_target <= current_target + 1'b1;

                            target_id <= current_target + 1'b1;

                            state <= WAIT_MEASUREMENT;

                        end

                    end

                end


                // =================================================
                // FRAME_FINISH
                // =================================================

                FRAME_FINISH: begin

                    busy <= 1'b0;

                    frame_done <= 1'b1;

                    state <= IDLE;

                end


                // =================================================
                // Default recovery
                // =================================================

                default: begin

                    state <= IDLE;

                    busy <= 1'b0;

                    target_start <= 1'b0;
                    frame_done <= 1'b0;

                end

            endcase

        end

    end

endmodule
