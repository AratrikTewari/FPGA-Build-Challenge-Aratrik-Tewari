`timescale 1ns / 1ps

module collision_predictor #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter signed [19:0] THRESHOLD_SQ = 20'sd16384, 
    parameter signed [19:0] HORIZON      = 20'sd8192   
)(
    input  logic clk, input  logic rst, input  logic start,
    input  logic signed [W-1:0] state_a_x, input  logic signed [W-1:0] state_a_y,
    input  logic signed [W-1:0] state_a_vx, input  logic signed [W-1:0] state_a_vy,
    input  logic signed [W-1:0] state_b_x, input  logic signed [W-1:0] state_b_y,
    input  logic signed [W-1:0] state_b_vx, input  logic signed [W-1:0] state_b_vy,
    
    output logic collision_predicted, output logic early_exit,
    output logic [6:0] coarse_risk, output logic busy, output logic done
);

    typedef enum logic [3:0] {
        IDLE, DELTAS, SQUARES_MULT, SQUARES_ADD, CHECK_EARLY, 
        CALC_BOUNDS_MULT, CALC_BOUNDS_ADD, EVALUATE, FINISH
    } state_t;
    state_t state;

    logic signed [W:0] dp_x, dp_y, dv_x, dv_y; 
    logic signed [W-1:0] dp_x_reg, dp_y_reg, dv_x_reg, dv_y_reg;

    logic signed [40:0] dp2_raw_x, dp2_raw_y, dv2_raw_x, dv2_raw_y, dot_raw_x, dot_raw_y;
    logic signed [W-1:0] dp2, dv2, dot;
    
    logic signed [40:0] dot_sq_raw, thresh_dv2_raw, dp2_dv2_raw, horizon_dv2_raw;
    logic signed [W-1:0] dot_sq, thresh_dv2, dp2_dv2, horizon_dv2;

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE; collision_predicted <= 1'b0; early_exit <= 1'b0; coarse_risk <= '0;
            busy <= 1'b0; done <= 1'b0;
            dp_x_reg <= '0; dp_y_reg <= '0; dv_x_reg <= '0; dv_y_reg <= '0;
            dp2 <= '0; dv2 <= '0; dot <= '0;
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1; early_exit <= 1'b0; collision_predicted <= 1'b0;
                        coarse_risk <= '0; state <= DELTAS;
                    end
                end
                
                DELTAS: begin
                    dp_x = state_b_x - state_a_x; dp_y = state_b_y - state_a_y;
                    dv_x = state_b_vx - state_a_vx; dv_y = state_b_vy - state_a_vy;
                    dp_x_reg <= dp_x[W-1:0]; dp_y_reg <= dp_y[W-1:0];
                    dv_x_reg <= dv_x[W-1:0]; dv_y_reg <= dv_y[W-1:0];
                    state <= SQUARES_MULT;
                end
                
                SQUARES_MULT: begin
                    dp2_raw_x <= dp_x_reg * dp_x_reg; dp2_raw_y <= dp_y_reg * dp_y_reg;
                    dv2_raw_x <= dv_x_reg * dv_x_reg; dv2_raw_y <= dv_y_reg * dv_y_reg;
                    dot_raw_x <= dp_x_reg * dv_x_reg; dot_raw_y <= dp_y_reg * dv_y_reg;
                    state <= SQUARES_ADD;
                end

                SQUARES_ADD: begin
                    dp2 <= (dp2_raw_x + dp2_raw_y) >>> F;
                    dv2 <= (dv2_raw_x + dv2_raw_y) >>> F;
                    dot <= (dot_raw_x + dot_raw_y) >>> F;
                    state <= CHECK_EARLY;
                end
                
                CHECK_EARLY: begin
                    if (dot >= 0) begin
                        early_exit <= 1'b1; collision_predicted <= 1'b0;
                        coarse_risk <= 7'd4; state <= FINISH;
                    end else begin
                        early_exit <= 1'b0; state <= CALC_BOUNDS_MULT;
                    end
                end
                
                CALC_BOUNDS_MULT: begin
                    dot_sq_raw      <= dot * dot;
                    dp2_dv2_raw     <= dp2 * dv2;
                    thresh_dv2_raw  <= THRESHOLD_SQ * dv2;
                    horizon_dv2_raw <= HORIZON * dv2;
                    state <= CALC_BOUNDS_ADD;
                end

                CALC_BOUNDS_ADD: begin
                    dot_sq      <= dot_sq_raw >>> F;
                    dp2_dv2     <= dp2_dv2_raw >>> F;
                    thresh_dv2  <= thresh_dv2_raw >>> F;
                    horizon_dv2 <= horizon_dv2_raw >>> F;
                    state <= EVALUATE;
                end
                
                EVALUATE: begin
                    if ((dp2_dv2 - dot_sq) < thresh_dv2) begin
                        if (-dot < horizon_dv2) begin 
                            collision_predicted <= 1'b1; coarse_risk <= 7'd92; 
                        end else begin
                            collision_predicted <= 1'b0; coarse_risk <= 7'd45; 
                        end
                    end else begin
                        collision_predicted <= 1'b0; coarse_risk <= 7'd15; 
                    end
                    state <= FINISH;
                end
                
                FINISH: begin done <= 1'b1; busy <= 1'b0; state <= IDLE; end
                default: begin state <= IDLE; busy <= 1'b0; end
            endcase
        end
    end
endmodule