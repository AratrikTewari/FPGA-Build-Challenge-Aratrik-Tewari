`timescale 1ns / 1ps

module covariance_predict #(
    parameter integer W     = 20,
    parameter integer F     = 12,
    parameter integer ACC_W = 44
)(
    input  logic clk,
    input  logic rst,
    input  logic start,

    input  logic maneuver_flag,

    input  logic signed [W-1:0] F_mat [0:3][0:3],
    input  logic signed [W-1:0] P_in  [0:3][0:3],
    input  logic signed [W-1:0] Q_in  [0:3][0:3],

    output logic signed [W-1:0] P_pred   [0:3][0:3],
    output logic signed [W-1:0] FPFt_out [0:3][0:3],

    output logic busy,
    output logic done
);

    typedef enum logic [3:0] {
        IDLE,
        FP_FETCH,
        FP_MULT,
        FP_ADD_1,
        FP_ADD_2,
        FPFT_FETCH,
        FPFT_MULT,
        FPFT_ADD_1,
        FPFT_ADD_2,
        COMPUTE_Q
    } state_t;

    state_t state;

    logic signed [W-1:0] F_reg [0:3][0:3];
    logic signed [W-1:0] P_reg [0:3][0:3];
    logic signed [W-1:0] Q_reg [0:3][0:3];

    logic signed [W-1:0] FP_reg   [0:3][0:3];
    logic signed [W-1:0] FPFt_reg [0:3][0:3];

    (* max_fanout = "8" *) logic [1:0] row;
    (* max_fanout = "8" *) logic [1:0] col;

    // Registered operand isolation to sever mux-to-DSP timing paths
    logic signed [W-1:0] op_a0, op_a1, op_a2, op_a3;
    logic signed [W-1:0] op_b0, op_b1, op_b2, op_b3;

    logic signed [39:0] p0, p1, p2, p3;
    logic signed [ACC_W-1:0] sum0, sum1;
    logic signed [ACC_W-1:0] shifted_result;
    logic signed [W:0] q_sum;

    integer i, j;

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
            row   <= '0;
            col   <= '0;

            op_a0 <= '0; op_a1 <= '0; op_a2 <= '0; op_a3 <= '0;
            op_b0 <= '0; op_b1 <= '0; op_b2 <= '0; op_b3 <= '0;

            p0 <= '0; p1 <= '0; p2 <= '0; p3 <= '0;
            sum0 <= '0; sum1 <= '0;

            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    F_reg[i][j]    <= '0;
                    P_reg[i][j]    <= '0;
                    Q_reg[i][j]    <= '0;
                    FP_reg[i][j]   <= '0;
                    FPFt_reg[i][j] <= '0;
                    P_pred[i][j]   <= '0;
                    FPFt_out[i][j] <= '0;
                end
            end
        end else begin
            done <= 1'b0;

            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        row  <= 2'd0;
                        col  <= 2'd0;

                        for (i = 0; i < 4; i = i + 1) begin
                            for (j = 0; j < 4; j = j + 1) begin
                                F_reg[i][j] <= F_mat[i][j];
                                P_reg[i][j] <= P_in[i][j];
                                Q_reg[i][j] <= Q_in[i][j];
                            end
                        end
                        state <= FP_FETCH;
                    end
                end

                // --- PASS 1: FP = F * P ---
                FP_FETCH: begin
                    op_a0 <= F_reg[row][0]; op_b0 <= P_reg[0][col];
                    op_a1 <= F_reg[row][1]; op_b1 <= P_reg[1][col];
                    op_a2 <= F_reg[row][2]; op_b2 <= P_reg[2][col];
                    op_a3 <= F_reg[row][3]; op_b3 <= P_reg[3][col];
                    state <= FP_MULT;
                end

                FP_MULT: begin
                    p0 <= $signed(op_a0) * $signed(op_b0);
                    p1 <= $signed(op_a1) * $signed(op_b1);
                    p2 <= $signed(op_a2) * $signed(op_b2);
                    p3 <= $signed(op_a3) * $signed(op_b3);
                    state <= FP_ADD_1;
                end

                FP_ADD_1: begin
                    sum0 <= p0 + p1;
                    sum1 <= p2 + p3;
                    state <= FP_ADD_2;
                end

                FP_ADD_2: begin
                    shifted_result = (sum0 + sum1) >>> F;
                    FP_reg[row][col] <= shifted_result[W-1:0];

                    if (col == 3) begin
                        col <= 2'd0;
                        if (row == 3) begin
                            row   <= 2'd0;
                            state <= FPFT_FETCH;
                        end else begin
                            row   <= row + 1'b1;
                            state <= FP_FETCH;
                        end
                    end else begin
                        col   <= col + 1'b1;
                        state <= FP_FETCH;
                    end
                end

                // --- PASS 2: FPFt = FP * F^T ---
                FPFT_FETCH: begin
                    op_a0 <= FP_reg[row][0]; op_b0 <= F_reg[col][0];
                    op_a1 <= FP_reg[row][1]; op_b1 <= F_reg[col][1];
                    op_a2 <= FP_reg[row][2]; op_b2 <= F_reg[col][2];
                    op_a3 <= FP_reg[row][3]; op_b3 <= F_reg[col][3];
                    state <= FPFT_MULT;
                end

                FPFT_MULT: begin
                    p0 <= $signed(op_a0) * $signed(op_b0);
                    p1 <= $signed(op_a1) * $signed(op_b1);
                    p2 <= $signed(op_a2) * $signed(op_b2);
                    p3 <= $signed(op_a3) * $signed(op_b3);
                    state <= FPFT_ADD_1;
                end

                FPFT_ADD_1: begin
                    sum0 <= p0 + p1;
                    sum1 <= p2 + p3;
                    state <= FPFT_ADD_2;
                end

                FPFT_ADD_2: begin
                    shifted_result = (sum0 + sum1) >>> F;
                    FPFt_reg[row][col] <= shifted_result[W-1:0];
                    FPFt_out[row][col] <= shifted_result[W-1:0];

                    if (col == 3) begin
                        col <= 2'd0;
                        if (row == 3) begin
                            row   <= 2'd0;
                            state <= COMPUTE_Q;
                        end else begin
                            row   <= row + 1'b1;
                            state <= FPFT_FETCH;
                        end
                    end else begin
                        col   <= col + 1'b1;
                        state <= FPFT_FETCH;
                    end
                end

                // --- PASS 3: P_pred = FPFt + Q ---
                COMPUTE_Q: begin
                    if (maneuver_flag) begin
                        q_sum = FPFt_reg[row][col] + (Q_reg[row][col] <<< 2);
                    end else begin
                        q_sum = FPFt_reg[row][col] + Q_reg[row][col];
                    end
                    P_pred[row][col] <= q_sum[W-1:0];

                    if (col == 3) begin
                        col <= 2'd0;
                        if (row == 3) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= IDLE;
                        end else begin
                            row <= row + 1'b1;
                        end
                    end else begin
                        col <= col + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule