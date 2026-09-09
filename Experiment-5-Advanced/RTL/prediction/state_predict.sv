`timescale 1ns / 1ps

module state_predict #(
    parameter integer W = 20,
    parameter integer F = 12,
    parameter integer ACC_W = 44
)(
    input  logic clk,
    input  logic rst,
    input  logic start,

    input  logic signed [W-1:0] x_in [0:3],
    input  logic signed [W-1:0] F_mat [0:3][0:3],

    output logic signed [W-1:0] x_out [0:3],

    output logic busy,
    output logic done
);

    typedef enum logic [1:0] {
        IDLE,
        MULT_STAGE,
        ADD_STAGE_1,
        ADD_STAGE_2
    } state_t;

    state_t state;

    logic signed [W-1:0] x_reg [0:3];
    logic signed [W-1:0] F_reg [0:3][0:3];
    
    (* max_fanout = "8" *) logic [1:0] row;

    logic signed [39:0] p0, p1, p2, p3;
    logic signed [ACC_W-1:0] sum0, sum1;
    logic signed [ACC_W-1:0] shifted_result;

    integer i;

    always_ff @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; done <= 1'b0; row <= '0;
            p0 <= '0; p1 <= '0; p2 <= '0; p3 <= '0; sum0 <= '0; sum1 <= '0;
            for (i = 0; i < 4; i = i + 1) begin
                x_reg[i] <= '0; x_out[i] <= '0;
            end
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1; row <= 2'd0;
                        for (i = 0; i < 4; i = i + 1) begin
                            x_reg[i] <= x_in[i];
                            F_reg[i][0] <= F_mat[i][0];
                            F_reg[i][1] <= F_mat[i][1];
                            F_reg[i][2] <= F_mat[i][2];
                            F_reg[i][3] <= F_mat[i][3];
                        end
                        state <= MULT_STAGE;
                    end
                end

                MULT_STAGE: begin
                    p0 <= $signed(F_reg[row][0]) * $signed(x_reg[0]);
                    p1 <= $signed(F_reg[row][1]) * $signed(x_reg[1]);
                    p2 <= $signed(F_reg[row][2]) * $signed(x_reg[2]);
                    p3 <= $signed(F_reg[row][3]) * $signed(x_reg[3]);
                    state <= ADD_STAGE_1;
                end

                ADD_STAGE_1: begin
                    sum0 <= p0 + p1;
                    sum1 <= p2 + p3;
                    state <= ADD_STAGE_2;
                end

                ADD_STAGE_2: begin
                    shifted_result = (sum0 + sum1) >>> F;
                    x_out[row] <= shifted_result[W-1:0];

                    if (row == 3) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        state <= IDLE;
                    end else begin
                        row <= row + 1'b1;
                        state <= MULT_STAGE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule