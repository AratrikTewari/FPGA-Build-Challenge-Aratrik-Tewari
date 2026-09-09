`timescale 1ns / 1ps

module target_scheduler #(
    parameter integer NUM_TARGETS = 8,
    parameter integer ID_W = 3
)(
    input  logic clk,
    input  logic rst,

    input  logic start,

    // ------------------------------------------------------------
    // Memory interface
    // ------------------------------------------------------------

    output logic [ID_W-1:0] target_id,

    output logic memory_read_request,
    input  logic memory_read_valid,

    output logic memory_write_enable,

    // ------------------------------------------------------------
    // Kalman engine interface
    // ------------------------------------------------------------

    output logic kalman_start,
    input  logic kalman_busy,
    input  logic kalman_done,

    // ------------------------------------------------------------
    // Status
    // ------------------------------------------------------------

    output logic busy,
    output logic frame_done
);

    typedef enum logic [3:0] {
        S_IDLE,
        S_READ,
        S_WAIT_READ,
        S_START_KALMAN,
        S_WAIT_KALMAN,
        S_WRITE,
        S_NEXT,
        S_DONE
    } state_t;

    state_t state;

    logic [ID_W-1:0] current_target;


    // ============================================================
    // State / output control
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            state               <= S_IDLE;
            current_target      <= '0;

            target_id           <= '0;

            memory_read_request <= 1'b0;
            memory_write_enable <= 1'b0;

            kalman_start        <= 1'b0;

            busy                <= 1'b0;
            frame_done          <= 1'b0;

        end
        else begin

            // ----------------------------------------------------
            // Default pulse signals
            // ----------------------------------------------------

            memory_read_request <= 1'b0;
            memory_write_enable <= 1'b0;
            kalman_start        <= 1'b0;
            frame_done          <= 1'b0;


            case (state)

                // ------------------------------------------------
                // IDLE
                // ------------------------------------------------

                S_IDLE: begin

                    busy <= 1'b0;

                    if (start) begin

                        current_target <= '0;
                        target_id      <= '0;

                        busy <= 1'b1;

                        state <= S_READ;

                    end

                end


                // ------------------------------------------------
                // Request current target from memory
                // ------------------------------------------------

                S_READ: begin

                    busy <= 1'b1;

                    target_id <= current_target;

                    memory_read_request <= 1'b1;

                    state <= S_WAIT_READ;

                end


                // ------------------------------------------------
                // Wait for memory read
                // ------------------------------------------------

                S_WAIT_READ: begin

                    busy <= 1'b1;

                    if (memory_read_valid) begin

                        state <= S_START_KALMAN;

                    end

                end


                // ------------------------------------------------
                // Start Kalman engine
                // ------------------------------------------------

                S_START_KALMAN: begin

                    busy <= 1'b1;

                    if (!kalman_busy) begin

                        kalman_start <= 1'b1;

                        state <= S_WAIT_KALMAN;

                    end

                end


                // ------------------------------------------------
                // Wait for Kalman engine
                // ------------------------------------------------

                S_WAIT_KALMAN: begin

                    busy <= 1'b1;

                    if (kalman_done) begin

                        state <= S_WRITE;

                    end

                end


                // ------------------------------------------------
                // Write updated target back to memory
                // ------------------------------------------------

                S_WRITE: begin

                    busy <= 1'b1;

                    target_id <= current_target;

                    memory_write_enable <= 1'b1;

                    state <= S_NEXT;

                end


                // ------------------------------------------------
                // Advance target
                // ------------------------------------------------

                S_NEXT: begin

                    busy <= 1'b1;

                    if (current_target == NUM_TARGETS-1) begin

                        state <= S_DONE;

                    end
                    else begin

                        current_target <= current_target + 1'b1;

                        target_id <= current_target + 1'b1;

                        state <= S_READ;

                    end

                end


                // ------------------------------------------------
                // Complete frame
                // ------------------------------------------------

                S_DONE: begin

                    busy <= 1'b0;
                    frame_done <= 1'b1;

                    state <= S_IDLE;

                end


                default: begin

                    state <= S_IDLE;
                    busy <= 1'b0;

                end

            endcase

        end

    end

endmodule
