`timescale 1ns / 1ps

module tb_target_memory;

    localparam integer W = 20;
    localparam integer NUM_TARGETS = 8;
    localparam integer ID_W = $clog2(NUM_TARGETS);

    logic clk;
    logic rst;

    logic [ID_W-1:0] target_id;
    logic write_enable;

    logic signed [W-1:0] x_write [0:3];
    logic signed [W-1:0] P_write [0:3][0:3];

    logic signed [W-1:0] x_read [0:3];
    logic signed [W-1:0] P_read [0:3][0:3];

    logic read_valid;

    integer errors;
    integer target;
    integer i;
    integer j;


    // ================================================================
    // DUT
    // ================================================================

    target_memory #(
        .W(W),
        .NUM_TARGETS(NUM_TARGETS)
    ) dut (
        .clk(clk),
        .rst(rst),

        .target_id(target_id),
        .write_enable(write_enable),

        .x_write(x_write),
        .P_write(P_write),

        .x_read(x_read),
        .P_read(P_read),

        .read_valid(read_valid)
    );


    // ================================================================
    // Clock
    // ================================================================

    always #5 clk = ~clk;


    // ================================================================
    // Write one target
    // ================================================================

    task write_target;
        input integer t;

        begin

            target_id = t;

            for (i = 0; i < 4; i = i + 1) begin

                x_write[i] = (t + 1) * 1000 + i * 100;

                for (j = 0; j < 4; j = j + 1) begin

                    P_write[i][j] =
                        (t + 1) * 100 +
                        i * 10 +
                        j;

                end

            end

            write_enable = 1'b1;

            @(posedge clk);
            #1;

            write_enable = 1'b0;

        end
    endtask


    // ================================================================
    // Read/check one target
    // ================================================================

    task check_target;
        input integer t;

        integer expected_x;
        integer expected_p;

        begin

            target_id = t;

            // Synchronous read
            @(posedge clk);
            #1;

            expected_x = (t + 1) * 1000;

            for (i = 0; i < 4; i = i + 1) begin

                if (x_read[i] !== expected_x + i * 100) begin

                    $display(
                        "ERROR: target %0d x[%0d] = %0d, expected %0d",
                        t,
                        i,
                        x_read[i],
                        expected_x + i * 100
                    );

                    errors = errors + 1;

                end
                else begin

                    $display(
                        "PASS: target %0d x[%0d] = %0d",
                        t,
                        i,
                        x_read[i]
                    );

                end

            end


            for (i = 0; i < 4; i = i + 1) begin

                for (j = 0; j < 4; j = j + 1) begin

                    expected_p =
                        (t + 1) * 100 +
                        i * 10 +
                        j;

                    if (P_read[i][j] !== expected_p) begin

                        $display(
                            "ERROR: target %0d P[%0d][%0d] = %0d, expected %0d",
                            t,
                            i,
                            j,
                            P_read[i][j],
                            expected_p
                        );

                        errors = errors + 1;

                    end

                end

            end

        end
    endtask


    // ================================================================
    // Test
    // ================================================================

    initial begin

        clk = 1'b0;
        rst = 1'b1;
        write_enable = 1'b0;
        target_id = '0;
        errors = 0;


        // ------------------------------------------------------------
        // Reset
        // ------------------------------------------------------------

        #20;
        rst = 1'b0;


        // ------------------------------------------------------------
        // Write all eight targets
        // ------------------------------------------------------------

        $display("");
        $display("==============================================");
        $display("WRITING 8 TARGETS");
        $display("==============================================");

        for (target = 0; target < NUM_TARGETS; target = target + 1) begin
            write_target(target);
        end


        // ------------------------------------------------------------
        // Read all eight targets
        // ------------------------------------------------------------

        $display("");
        $display("==============================================");
        $display("READING 8 TARGETS");
        $display("==============================================");

        for (target = 0; target < NUM_TARGETS; target = target + 1) begin
            check_target(target);
        end


        // ------------------------------------------------------------
        // Overwrite target 3
        // ------------------------------------------------------------

        $display("");
        $display("OVERWRITING TARGET 3");

        target_id = 3;

        for (i = 0; i < 4; i = i + 1) begin

            x_write[i] = 50000 + i;

            for (j = 0; j < 4; j = j + 1) begin
                P_write[i][j] = 60000 + i * 10 + j;
            end

        end

        write_enable = 1'b1;

        @(posedge clk);
        #1;

        write_enable = 1'b0;


        // ------------------------------------------------------------
        // Verify target 3
        // ------------------------------------------------------------

        @(posedge clk);
        #1;

        for (i = 0; i < 4; i = i + 1) begin

            if (x_read[i] !== 50000 + i) begin

                $display(
                    "ERROR: overwritten target x[%0d] = %0d",
                    i,
                    x_read[i]
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: overwritten target x[%0d] = %0d",
                    i,
                    x_read[i]
                );

            end

        end


        // ------------------------------------------------------------
        // Verify another target remains unchanged
        // ------------------------------------------------------------

        target_id = 2;

        @(posedge clk);
        #1;

        for (i = 0; i < 4; i = i + 1) begin

            if (x_read[i] !== 3000 + i * 100) begin

                $display(
                    "ERROR: target 2 corrupted, x[%0d] = %0d",
                    i,
                    x_read[i]
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: target 2 preserved, x[%0d] = %0d",
                    i,
                    x_read[i]
                );

            end

        end


        // ------------------------------------------------------------
        // Final result
        // ------------------------------------------------------------

        $display("");
        $display("==============================================");
        $display("TARGET MEMORY TEST COMPLETE");
        $display("==============================================");

        if (errors == 0) begin
            $display("PASS: 8-target state/covariance memory is correct.");
        end
        else begin
            $display("FAIL: %0d errors detected.", errors);
        end

        #20;
        $finish;

    end

endmodule
