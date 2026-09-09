`timescale 1ns / 1ps

module tb_kalman_gain;

    localparam integer W = 20;
    localparam integer INV_W = 16;
    localparam integer F = 12;

    logic clk;
    logic rst;
    logic start;

    logic signed [W-1:0] PHt [0:3][0:1];
    logic signed [INV_W-1:0] invS;

    logic signed [W-1:0] K [0:3][0:1];

    logic busy;
    logic done;

    integer errors;

    kalman_gain #(
        .W(W),
        .INV_W(INV_W),
        .F(F)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .PHt(PHt),
        .invS(invS),
        .K(K),
        .busy(busy),
        .done(done)
    );

    // 100 MHz clock
    always #5 clk = ~clk;

    task automatic check_value(
        input integer row_index,
        input integer col_index,
        input integer expected
    );
        begin

            if (K[row_index][col_index] !== expected) begin

                $display(
                    "ERROR: K[%0d][%0d] = %0d, expected %0d",
                    row_index,
                    col_index,
                    K[row_index][col_index],
                    expected
                );

                errors = errors + 1;

            end else begin

                $display(
                    "PASS: K[%0d][%0d] = %0d",
                    row_index,
                    col_index,
                    K[row_index][col_index]
                );

            end

        end
    endtask

    initial begin

        clk = 1'b0;
        rst = 1'b1;
        start = 1'b0;
        errors = 0;

        for (int i = 0; i < 4; i = i + 1) begin
            for (int j = 0; j < 2; j = j + 1) begin
                PHt[i][j] = '0;
            end
        end

        invS = '0;

        #20;
        rst = 1'b0;

        // ============================================================
        // TEST 1
        //
        // invS = 0.5
        //
        // PHt =
        //
        // [ 2.0   0.2 ]
        // [ 0.2   3.0 ]
        // [ 0.5   0.1 ]
        // [ 0.1   0.4 ]
        //
        // K = PHt * invS
        //
        // Important:
        // All arithmetic is performed using the actual Q8.12
        // integer representations and arithmetic >>> 12.
        //
        // 0.2 is represented as 819, not 819.2.
        //
        // Therefore:
        // (819 * 2048) >>> 12 = 409
        // ============================================================

        PHt[0][0] = 2.0 * (1 << F);
        PHt[0][1] = 0.2 * (1 << F);

        PHt[1][0] = 0.2 * (1 << F);
        PHt[1][1] = 3.0 * (1 << F);

        PHt[2][0] = 0.5 * (1 << F);
        PHt[2][1] = 0.1 * (1 << F);

        PHt[3][0] = 0.1 * (1 << F);
        PHt[3][1] = 0.4 * (1 << F);

        invS = 0.5 * (1 << F);

        @(posedge clk);
        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 1");

        check_value(0, 0, 4096);
        check_value(0, 1, 409);

        check_value(1, 0, 409);
        check_value(1, 1, 6144);

        check_value(2, 0, 1024);
        check_value(2, 1, 205);

        check_value(3, 0, 205);
        check_value(3, 1, 819);

        // ============================================================
        // TEST 2
        //
        // invS = 372
        //
        // Q0.12 value:
        //
        // 372 / 4096 = 0.0908203125
        //
        // PHt values:
        //
        // [ 5.0   2.0 ]
        // [ 1.5   4.0 ]
        // [-2.0   3.0 ]
        // [ 0.5  -1.5 ]
        //
        // Expected results are calculated directly from:
        //
        // (PHt_integer * invS_integer) >>> 12
        // ============================================================

        @(posedge clk);

        PHt[0][0] = 5.0 * (1 << F);
        PHt[0][1] = 2.0 * (1 << F);

        PHt[1][0] = 1.5 * (1 << F);
        PHt[1][1] = 4.0 * (1 << F);

        PHt[2][0] = -2.0 * (1 << F);
        PHt[2][1] = 3.0 * (1 << F);

        PHt[3][0] = 0.5 * (1 << F);
        PHt[3][1] = -1.5 * (1 << F);

        invS = 372;

        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 2");

        check_value(0, 0, 1860);
        check_value(0, 1, 744);

        check_value(1, 0, 558);
        check_value(1, 1, 1488);

        check_value(2, 0, -744);
        check_value(2, 1, 1116);

        check_value(3, 0, 186);
        check_value(3, 1, -558);

        // ============================================================
        // FINAL RESULT
        // ============================================================

        $display("");
        $display("KALMAN GAIN TEST COMPLETE");

        if (errors == 0)
            $display("PASS: Kalman gain RTL matches expected Q8.12 arithmetic.");
        else
            $display("FAIL: %0d errors detected.", errors);

        #20;
        $finish;

    end

endmodule