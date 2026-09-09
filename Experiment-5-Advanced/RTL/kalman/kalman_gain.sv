`timescale 1ns / 1ps

module kalman_gain #(
    parameter integer W = 20,
    parameter integer INV_W = 16,
    parameter integer F = 12
)(
    input logic clk,
    input logic rst,
    input logic start,

    input logic signed [W-1:0] PHt [0:3][0:1],
    input logic signed [INV_W-1:0] invS,

    output logic signed [W-1:0] K [0:3][0:1],

    output logic busy,
    output logic done
);

    logic signed [W-1:0] PHt_reg [0:3][0:1];
    logic signed [INV_W-1:0] invS_reg;

    logic [2:0] row;
    logic        col;

    logic signed [W+INV_W-1:0] product;
    logic signed [W+INV_W-1:0] shifted_result;

    integer i;
    integer j;

    always_ff @(posedge clk) begin

        if (rst) begin

            busy <= 1'b0;
            done <= 1'b0;

            row <= '0;
            col <= 1'b0;

            invS_reg <= '0;

            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 2; j = j + 1) begin
                    PHt_reg[i][j] <= '0;
                    K[i][j] <= '0;
                end
            end

        end else begin

            done <= 1'b0;

            if (!busy) begin

                if (start) begin

                    busy <= 1'b1;
                    row <= 3'd0;
                    col <= 1'b0;

                    invS_reg <= invS;

                    for (i = 0; i < 4; i = i + 1) begin
                        for (j = 0; j < 2; j = j + 1) begin
                            PHt_reg[i][j] <= PHt[i][j];
                        end
                    end

                end

            end else begin

                // 20-bit Q8.12 × 16-bit Q0.12
                // = 36-bit raw product.
                product = PHt_reg[row][col] * invS_reg;

                // Return to Q8.12.
                shifted_result = product >>> F;

                K[row][col] <= shifted_result[W-1:0];

                if (col == 1'b1) begin

                    col <= 1'b0;

                    if (row == 3) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                    end else begin
                        row <= row + 1'b1;
                    end

                end else begin
                    col <= 1'b1;
                end

            end

        end
    end

endmodule