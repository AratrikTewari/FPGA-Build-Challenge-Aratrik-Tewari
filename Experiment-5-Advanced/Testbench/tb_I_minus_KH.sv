`timescale 1ns / 1ps

module tb_I_minus_KH;

    localparam integer W = 20;
    localparam integer IDENTITY = 4096;

    logic clk;
    logic rst;
    logic start;

    logic signed [W-1:0] KH [0:3][0:3];

    logic signed [W-1:0] I_KH [0:3][0:3];

    logic busy;
    logic done;

    integer errors;
    integer i;
    integer j;


    // ================================================================
    // DUT
    // ================================================================

    I_minus_KH #(
        .W(W),
        .IDENTITY(IDENTITY)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .KH(KH),
        .I_KH(I_KH),
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

            if (I_KH[row_index][col_index] !== expected) begin

                $display(
                    "ERROR: I_KH[%0d][%0d] = %0d, expected %0d",
                    row_index,
                    col_index,
                    I_KH[row_index][col_index],
                    expected
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: I_KH[%0d][%0d] = %0d",
                    row_index,
                    col_index,
                    I_KH[row_index][col_index]
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

            for (j = 0; j < 4; j = j + 1) begin
                KH[i][j] = 20'sd0;
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
        // KH =
        //
        // [2048  409  0  0]
        // [ 819 1638  0  0]
        // [1228  409  0  0]
        // [ 409  819  0  0]
        //
        // I =
        //
        // [4096 0    0    0]
        // [0    4096 0    0]
        // [0    0    4096 0]
        // [0    0    0    4096]
        //
        // Expected I-KH:
        //
        // [2048  -409     0     0]
        // [-819  2458     0     0]
        // [-1228 -409  4096     0]
        // [-409  -819     0  4096]
        //
        // ============================================================

        KH[0][0] = 20'sd2048;
        KH[0][1] = 20'sd409;
        KH[0][2] = 20'sd0;
        KH[0][3] = 20'sd0;

        KH[1][0] = 20'sd819;
        KH[1][1] = 20'sd1638;
        KH[1][2] = 20'sd0;
        KH[1][3] = 20'sd0;

        KH[2][0] = 20'sd1228;
        KH[2][1] = 20'sd409;
        KH[2][2] = 20'sd0;
        KH[2][3] = 20'sd0;

        KH[3][0] = 20'sd409;
        KH[3][1] = 20'sd819;
        KH[3][2] = 20'sd0;
        KH[3][3] = 20'sd0;


        @(posedge clk);
        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 1");

        check_value(0, 0, 2048);
        check_value(0, 1, -409);
        check_value(0, 2, 0);
        check_value(0, 3, 0);

        check_value(1, 0, -819);
        check_value(1, 1, 2458);
        check_value(1, 2, 0);
        check_value(1, 3, 0);

        check_value(2, 0, -1228);
        check_value(2, 1, -409);
        check_value(2, 2, 4096);
        check_value(2, 3, 0);

        check_value(3, 0, -409);
        check_value(3, 1, -819);
        check_value(3, 2, 0);
        check_value(3, 3, 4096);


        // ============================================================
        // TEST 2
        // ============================================================
        //
        // Exercise positive and negative KH values.
        //
        // KH =
        //
        // [ 8192 -4096  1024 -2048]
        // [-2048 6144 -4096  2048]
        // [12288 -8192 2048 -1024]
        // [-4096  2048 -2048 1024]
        //
        // Expected:
        //
        // [-4096  4096 -1024  2048]
        // [ 2048 -2048  4096 -2048]
        // [-12288 8192  2048? wait]
        //
        // For diagonal:
        //
        // row 2 col 2 = 4096 - 2048 = 2048
        // row 3 col 3 = 4096 - 1024 = 3072
        //
        // ============================================================

        @(posedge clk);

        KH[0][0] = 20'sd8192;
        KH[0][1] = -20'sd4096;
        KH[0][2] = 20'sd1024;
        KH[0][3] = -20'sd2048;

        KH[1][0] = -20'sd2048;
        KH[1][1] = 20'sd6144;
        KH[1][2] = -20'sd4096;
        KH[1][3] = 20'sd2048;

        KH[2][0] = 20'sd12288;
        KH[2][1] = -20'sd8192;
        KH[2][2] = 20'sd2048;
        KH[2][3] = -20'sd1024;

        KH[3][0] = -20'sd4096;
        KH[3][1] = 20'sd2048;
        KH[3][2] = -20'sd2048;
        KH[3][3] = 20'sd1024;


        start = 1'b1;

        @(posedge clk);
        start = 1'b0;

        wait(done);

        #1;

        $display("");
        $display("TEST 2");

        check_value(0, 0, -4096);
        check_value(0, 1, 4096);
        check_value(0, 2, -1024);
        check_value(0, 3, 2048);

        check_value(1, 0, 2048);
        check_value(1, 1, -2048);
        check_value(1, 2, 4096);
        check_value(1, 3, -2048);

        check_value(2, 0, -12288);
        check_value(2, 1, 8192);
        check_value(2, 2, 2048);
        check_value(2, 3, 1024);

        check_value(3, 0, 4096);
        check_value(3, 1, -2048);
        check_value(3, 2, 2048);
        check_value(3, 3, 3072);


        // ============================================================
        // FINAL RESULT
        // ============================================================

        $display("");
        $display("I-KH MATRIX TEST COMPLETE");

        if (errors == 0) begin

            $display(
                "PASS: I-KH RTL matches expected Q8.12 arithmetic."
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