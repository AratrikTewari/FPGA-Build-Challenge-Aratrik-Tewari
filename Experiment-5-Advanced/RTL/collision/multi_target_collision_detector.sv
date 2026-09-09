`timescale 1ns / 1ps

module multi_target_collision_detector #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer NUM_TARGETS = 8
)(
    input  logic clk,
    input  logic rst,
    input  logic start,

    // [0]=x, [1]=y, [2]=vx, [3]=vy (Q8.12)
    input  logic signed [W-1:0] states [0:NUM_TARGETS-1][0:3],

    output logic [NUM_TARGETS*(NUM_TARGETS-1)/2-1:0] collision_pairs,
    output logic [6:0] max_coarse_risk, // NEW: Aggregated highest risk
    output logic busy,
    output logic done
);

    localparam integer NUM_PAIRS = NUM_TARGETS * (NUM_TARGETS - 1) / 2;

    // Target indices
    integer pair_i;
    integer pair_j;
    integer pair_index;

    // collision_predictor instance signals
    logic cp_start;
    logic cp_coll;
    logic cp_early;
    logic [6:0] cp_risk;
    logic cp_busy;
    logic cp_done;

    // ============================================================
    // Singleton Predictor Instance
    // ============================================================
    collision_predictor #(
        .W(W),
        .F(F)
    ) cp_inst (
        .clk(clk),
        .rst(rst),
        .start(cp_start),
        
        // Mux inputs based on current FSM pair indices
        .state_a_x(states[pair_i][0]),
        .state_a_y(states[pair_i][1]),
        .state_a_vx(states[pair_i][2]),
        .state_a_vy(states[pair_i][3]),
        
        .state_b_x(states[pair_j][0]),
        .state_b_y(states[pair_j][1]),
        .state_b_vx(states[pair_j][2]),
        .state_b_vy(states[pair_j][3]),
        
        .collision_predicted(cp_coll),
        .early_exit(cp_early),
        .coarse_risk(cp_risk),
        .busy(cp_busy),
        .done(cp_done)
    );

    // ============================================================
    // Orchestrator FSM
    // ============================================================
    typedef enum logic [2:0] {
        S_IDLE,
        S_START_CP,
        S_WAIT_CP,
        S_EVALUATE,
        S_DONE
    } state_t;

    state_t state;

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            collision_pairs <= '0;
            max_coarse_risk <= '0;
            busy <= 1'b0;
            done <= 1'b0;
            cp_start <= 1'b0;
            pair_i <= 0;
            pair_j <= 1;
            pair_index <= 0;
        end else begin
            done <= 1'b0;
            cp_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        collision_pairs <= '0;
                        max_coarse_risk <= '0;
                        pair_i <= 0;
                        pair_j <= 1;
                        pair_index <= 0;
                        state <= S_START_CP;
                    end
                end

                S_START_CP: begin
                    cp_start <= 1'b1;
                    state <= S_WAIT_CP;
                end

                S_WAIT_CP: begin
                    // The fast-tracking happens natively here.
                    // If objects are moving apart, cp_inst triggers 'early_exit'
                    // and immediately pulses 'cp_done', skipping the heavy math cycles.
                    if (cp_done) begin
                        state <= S_EVALUATE;
                    end
                end

                S_EVALUATE: begin
                    // Record collision prediction for this specific pair
                    collision_pairs[pair_index] <= cp_coll;

                    // Latch the highest risk seen in this frame
                    if (cp_risk > max_coarse_risk) begin
                        max_coarse_risk <= cp_risk;
                    end

                    // Advance the pair indices
                    if (pair_j == NUM_TARGETS - 1) begin
                        if (pair_i == NUM_TARGETS - 2) begin
                            state <= S_DONE; // All 28 pairs finished
                        end else begin
                            pair_i <= pair_i + 1;
                            pair_j <= pair_i + 2;
                            pair_index <= pair_index + 1;
                            state <= S_START_CP;
                        end
                    end else begin
                        pair_j <= pair_j + 1;
                        pair_index <= pair_index + 1;
                        state <= S_START_CP;
                    end
                end

                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
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