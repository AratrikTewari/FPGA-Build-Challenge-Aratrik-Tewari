`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_target_memory_load
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Advanced FPGA architecture implementation featuring Kalman filters,
//   collision detection, and multi-target tracking.
//
// Architectural Hierarchy:
//   Part of the Experiment 5 RTL / Simulation / Testbench ecosystem.
// ============================================================================

module tb_target_memory_load;

    localparam integer W = 20;
    localparam integer NUM_TARGETS = 8;
    localparam integer ID_W = $clog2(NUM_TARGETS);

    logic clk;
    logic rst;

    // ============================================================
    // Normal runtime interface
    // ============================================================

    logic [ID_W-1:0] target_id;

    logic write_enable;

    logic signed [W-1:0] x_write [0:3];
    logic signed [W-1:0] P_write [0:3][0:3];

    // ============================================================
    // Read interface
    // ============================================================

    logic signed [W-1:0] x_read [0:3];
    logic signed [W-1:0] P_read [0:3][0:3];

    logic read_valid;

    // ============================================================
    // Load interface
    // ============================================================

    logic load_enable;

    logic [ID_W-1:0] load_target;

    logic signed [W-1:0] x_load [0:3];
    logic signed [W-1:0] P_load [0:3][0:3];

    logic load_done;

    integer errors;

    integer i;
    integer j;
    integer k;


    // ============================================================
    // DUT
    // ============================================================

    target_memory #(
        .W(W),
        .NUM_TARGETS(NUM_TARGETS)
    ) dut (
        .clk(clk),
        .rst(rst),

        // Runtime write
        .target_id(target_id),
        .write_enable(write_enable),
        .x_write(x_write),
        .P_write(P_write),

        // Read
        .x_read(x_read),
        .P_read(P_read),
        .read_valid(read_valid),

        // Load
        .load_enable(load_enable),
        .load_target(load_target),
        .x_load(x_load),
        .P_load(P_load),
        .load_done(load_done)
    );


    // ============================================================
    // Clock
    // ============================================================

    always #5 clk = ~clk;


    // ============================================================
    // Clear load inputs
    // ============================================================

    task automatic clear_load_inputs;

        integer a;
        integer b;

        begin

            load_enable = 1'b0;
            load_target = '0;

            for (a = 0; a < 4; a = a + 1) begin

                x_load[a] = '0;

                for (b = 0; b < 4; b = b + 1) begin
                    P_load[a][b] = '0;
                end

            end

        end

    endtask


    // ============================================================
    // Clear runtime write inputs
    // ============================================================

    task automatic clear_write_inputs;

        integer a;
        integer b;

        begin

            write_enable = 1'b0;
            target_id = '0;

            for (a = 0; a < 4; a = a + 1) begin

                x_write[a] = '0;

                for (b = 0; b < 4; b = b + 1) begin
                    P_write[a][b] = '0;
                end

            end

        end

    endtask


    // ============================================================
    // Clear all inputs
    // ============================================================

    task automatic clear_inputs;

        begin

            clear_load_inputs();
            clear_write_inputs();

        end

    endtask


    // ============================================================
    // Load one complete target
    //
    // Data pattern:
    //
    // Target 0:
    //   x = 100,101,102,103
    //
    // Target 1:
    //   x = 200,201,202,203
    //
    // ...
    //
    // Target 7:
    //   x = 800,801,802,803
    //
    // P[i][j] = base + i*10 + j
    // ============================================================

    task automatic load_one_target(
        input integer id
    );

        integer base;
        integer a;
        integer b;

        begin

            base = (id + 1) * 100;

            // ----------------------------------------------------
            // Prepare all load inputs BEFORE asserting load_enable.
            // ----------------------------------------------------

            load_target = id;

            for (a = 0; a < 4; a = a + 1) begin

                x_load[a] = base + a;

                for (b = 0; b < 4; b = b + 1) begin
                    P_load[a][b] = base + a * 10 + b;
                end

            end

            // ----------------------------------------------------
            // Make sure inputs are stable before active edge.
            // ----------------------------------------------------

            @(negedge clk);

            load_enable = 1'b1;

            // ----------------------------------------------------
            // DUT performs the load at this rising edge.
            // ----------------------------------------------------

            @(posedge clk);

            // Allow nonblocking assignments to update.
            #1;

            // ----------------------------------------------------
            // Check acknowledgement.
            // ----------------------------------------------------

            if (load_done !== 1'b1) begin

                $display(
                    "ERROR: load_done missing for target %0d",
                    id
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: load_done received for target %0d",
                    id
                );

            end

            // ----------------------------------------------------
            // Deassert load_enable before the next clock.
            // ----------------------------------------------------

            load_enable = 1'b0;

            @(negedge clk);

        end

    endtask


    // ============================================================
    // Read one target
    //
    // target_id is applied before a rising edge.
    //
    // The memory performs a synchronous read.
    // ============================================================

    task automatic read_one_target(
        input integer id
    );

        begin

            target_id = id;

            @(posedge clk);

            #1;

        end

    endtask


    // ============================================================
    // Verify one loaded target
    // ============================================================

    task automatic verify_target(
        input integer id
    );

        integer base;
        integer a;
        integer b;
        integer expected_x;
        integer expected_p;

        begin

            base = (id + 1) * 100;

            // ----------------------------------------------------
            // Request synchronous read.
            // ----------------------------------------------------

            read_one_target(id);

            // ----------------------------------------------------
            // Verify read-valid.
            // ----------------------------------------------------

            if (read_valid !== 1'b1) begin

                $display(
                    "ERROR: read_valid missing for target %0d",
                    id
                );

                errors = errors + 1;

            end

            // ----------------------------------------------------
            // Verify state vector.
            // ----------------------------------------------------

            for (a = 0; a < 4; a = a + 1) begin

                expected_x = base + a;

                if ($signed(x_read[a]) !== expected_x) begin

                    $display(
                        "ERROR: target %0d x[%0d] = %0d, expected %0d",
                        id,
                        a,
                        $signed(x_read[a]),
                        expected_x
                    );

                    errors = errors + 1;

                end
                else begin

                    $display(
                        "PASS: target %0d x[%0d] = %0d",
                        id,
                        a,
                        $signed(x_read[a])
                    );

                end

            end

            // ----------------------------------------------------
            // Verify covariance matrix.
            // ----------------------------------------------------

            for (a = 0; a < 4; a = a + 1) begin

                for (b = 0; b < 4; b = b + 1) begin

                    expected_p = base + a * 10 + b;

                    if ($signed(P_read[a][b]) !== expected_p) begin

                        $display(
                            "ERROR: target %0d P[%0d][%0d] = %0d, expected %0d",
                            id,
                            a,
                            b,
                            $signed(P_read[a][b]),
                            expected_p
                        );

                        errors = errors + 1;

                    end
                    else begin

                        $display(
                            "PASS: target %0d P[%0d][%0d] = %0d",
                            id,
                            a,
                            b,
                            $signed(P_read[a][b]),
                            expected_p
                        );

                    end

                end

            end

        end

    endtask


    // ============================================================
    // Runtime write test
    // ============================================================

    task automatic runtime_write_test;

        integer a;
        integer b;

        begin

            $display("");
            $display("VERIFYING NORMAL RUNTIME WRITE");

            // ----------------------------------------------------
            // Write target 0.
            // ----------------------------------------------------

            target_id = 0;

            for (a = 0; a < 4; a = a + 1) begin

                x_write[a] = 900 + a;

                for (b = 0; b < 4; b = b + 1) begin
                    P_write[a][b] = 900 + a * 10 + b;
                end

            end

            @(negedge clk);

            write_enable = 1'b1;

            @(posedge clk);

            #1;

            write_enable = 1'b0;

            // ----------------------------------------------------
            // Read target 0 back.
            // ----------------------------------------------------

            @(negedge clk);

            target_id = 0;

            @(posedge clk);

            #1;

            // ----------------------------------------------------
            // Verify state.
            // ----------------------------------------------------

            for (a = 0; a < 4; a = a + 1) begin

                if ($signed(x_read[a]) !== (900 + a)) begin

                    $display(
                        "ERROR: runtime write x[%0d] = %0d, expected %0d",
                        a,
                        $signed(x_read[a]),
                        900 + a
                    );

                    errors = errors + 1;

                end
                else begin

                    $display(
                        "PASS: runtime write x[%0d] = %0d",
                        a,
                        $signed(x_read[a])
                    );

                end

            end

            // ----------------------------------------------------
            // Verify covariance as well.
            // ----------------------------------------------------

            for (a = 0; a < 4; a = a + 1) begin

                for (b = 0; b < 4; b = b + 1) begin

                    if (
                        $signed(P_read[a][b])
                        !==
                        (900 + a * 10 + b)
                    ) begin

                        $display(
                            "ERROR: runtime write P[%0d][%0d] = %0d, expected %0d",
                            a,
                            b,
                            $signed(P_read[a][b]),
                            900 + a * 10 + b
                        );

                        errors = errors + 1;

                    end
                    else begin

                        $display(
                            "PASS: runtime write P[%0d][%0d] = %0d",
                            a,
                            b,
                            $signed(P_read[a][b])
                        );

                    end

                end

            end

        end

    endtask


    // ============================================================
    // Main test sequence
    // ============================================================

    initial begin

        clk = 1'b0;
        rst = 1'b1;

        errors = 0;

        clear_inputs();

        // --------------------------------------------------------
        // Reset
        //
        // Keep reset asserted for complete clock cycles.
        // --------------------------------------------------------

        repeat (3)
            @(posedge clk);

        #1;

        rst = 1'b0;

        // Give reset release its own setup interval.
        @(negedge clk);


        // ========================================================
        // Header
        // ========================================================

        $display("");
        $display("==============================================");
        $display("TARGET MEMORY LOAD TEST");
        $display("==============================================");


        // ========================================================
        // LOAD ALL 8 TARGETS
        // ========================================================

        for (i = 0; i < NUM_TARGETS; i = i + 1) begin

            load_one_target(i);

        end


        // ========================================================
        // VERIFY ALL LOADED TARGETS
        // ========================================================

        $display("");
        $display("VERIFYING LOADED TARGETS");

        for (i = 0; i < NUM_TARGETS; i = i + 1) begin

            verify_target(i);

        end


        // ========================================================
        // NORMAL RUNTIME WRITE
        // ========================================================

        runtime_write_test();


        // ========================================================
        // Final result
        // ========================================================

        $display("");
        $display("==============================================");
        $display("TARGET MEMORY LOAD TEST COMPLETE");
        $display("==============================================");

        if (errors == 0) begin

            $display(
                "PASS: target memory load interface matches expected behavior."
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
