`timescale 1ns / 1ps

module tb_kh_matrix;

    localparam integer W = 20;

    logic clk;
    logic rst;
    logic start;

    logic signed [W-1:0] K [0:3][0:1];

    logic signed [W-1:0] KH [0:3][0:3];

    logic busy;
    logic done;

    integer errors;
    integer i;
    integer j;


    // ================================================================
    // DUT
    // ================================================================

    kh_matrix #(
        .W(W)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .K(K),
        .KH(KH),
        .busy(busy),
        .done(done)
    );


    // ================================================================
    // 100 MHz clock
    // ================================================================

    always #5 clk = ~clk;


    // ================================================================
    // Check one matrix element
    // ================================================================

    task check_value;
        input integer row_index;
        input integer col_index;
        input integer expected;

        begin

            if (KH[row_index][col_index] !== expected) begin

                $display(
                    "ERROR: KH[%0d][%0d] = %0d, expected %0d",
                    row_index,
                    col_index,
                    KH[row_index][col_index],
                    expected
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: KH[%0d][%0d] = %0d",
                    row_index,
                    col_index,
                    KH[row_index][col_index]
                );

            end

        end
    endtask


    // ================================================================
    // TESTS
    // ================================================================

    initial begin

        clk = 1'b0;
        rst = 1'b1;
        start = 1'b0;
        errors = 0;


        // ------------------------------------------------------------
        // Initialize inputs
        // ------------------------------------------------------------

        for (i = 0; i < 4; i = i + 1) begin

            for (j = 0; j < 2; j = j + 1) begin
                K[i][j] = 20'sd0;
            end

        end


        // ------------------------------------------------------------
        // Reset
        // ------------------------------------------------------------

        #20;
        rst = 1'b0;


        // ============================================================
        // TEST 1
        // ============================================================
        //
        // K =
        //
        // [ 2048   409 ]
        // [  819  1638 ]
        // [ 1228   409 ]
        // [  409   819 ]
        //
        // Corresponding approximately to:
        //
        // [ 0.5   0.1 ]
        // [ 0.2   0.4 ]
        // [ 0.3   0.1 ]
        // [ 0.1   0.2 ]
        //
        // Expected:
        //
        // KH =
        //
        // [2048  409   0   0]
        // [ 819 1638   0   0]
        // [1228  409   0   0]
        // [ 409  819   0   0]
        //
        // ============================================================

        K[0][0] = 20'sd2048;
        K[0][1] = 20'sd409;

        K[1][0] = 20'sd819;
        K[1][1] = 20'sd1638;

        K[2][0] = 20'sd1228;
        K[2][1] = 20'sd409;

        K[3][0] = 20'sd409;
        K[3][1] = 20'sd819;


        @(posedge clk);
        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 1");

        check_value(0, 0, 2048);
        check_value(0, 1, 409);
        check_value(0, 2, 0);
        check_value(0, 3, 0);

        check_value(1, 0, 819);
        check_value(1, 1, 1638);
        check_value(1, 2, 0);
        check_value(1, 3, 0);

        check_value(2, 0, 1228);
        check_value(2, 1, 409);
        check_value(2, 2, 0);
        check_value(2, 3, 0);

        check_value(3, 0, 409);
        check_value(3, 1, 819);
        check_value(3, 2, 0);
        check_value(3, 3, 0);


        // ============================================================
        // TEST 2
        // ============================================================
        //
        // Exercise positive and negative values.
        //
        // K =
        //
        // [  8192  -4096 ]
        // [ -2048   6144 ]
        // [ 12288  -8192 ]
        // [ -4096   2048 ]
        //
        // Expected KH is identical in first two columns,
        // with the final two columns equal to zero.
        //
        // ============================================================

        @(posedge clk);

        K[0][0] = 20'sd8192;
        K[0][1] = -20'sd4096;

        K[1][0] = -20'sd2048;
        K[1][1] = 20'sd6144;

        K[2][0] = 20'sd12288;
        K[2][1] = -20'sd8192;

        K[3][0] = -20'sd4096;
        K[3][1] = 20'sd2048;


        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 2");

        check_value(0, 0, 8192);
        check_value(0, 1, -4096);
        check_value(0, 2, 0);
        check_value(0, 3, 0);

        check_value(1, 0, -2048);
        check_value(1, 1, 6144);
        check_value(1, 2, 0);
        check_value(1, 3, 0);

        check_value(2, 0, 12288);
        check_value(2, 1, -8192);
        check_value(2, 2, 0);
        check_value(2, 3, 0);

        check_value(3, 0, -4096);
        check_value(3, 1, 2048);
        check_value(3, 2, 0);
        check_value(3, 3, 0);


        // ============================================================
        // FINAL RESULT
        // ============================================================

        $display("");
        $display("KH MATRIX TEST COMPLETE");

        if (errors == 0) begin

            $display(
                "PASS: KH RTL matches expected K*H structure."
            );

        end
        else begin

            $display(
                "FAIL: %0d errors detected.",
                errors
            );

        end


        #20;
        $finish;

    end

endmodule