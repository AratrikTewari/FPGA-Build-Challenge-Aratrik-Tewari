`timescale 1ns / 1ps

// ============================================================================
// Module:        tb_frame_engine.sv
// Project:       FPGA Build Challenge - Experiment 5 (Advanced)
// Target Device: AMD Xilinx Zynq-7000 SoC (PYNQ-Z2)
//
// System Context & Top-Level Integration:
//   Unit testbench for the frame transformation engine, injecting known polar/Cartesian coordinates and verifying rotation matrices.
//
// Architectural Hierarchy:
//   - Instantiated within the broader Experiment 5 RTL/Verification ecosystem.
//   - Synthesizable for PL (Programmable Logic) deployment unless marked as TB.
// ============================================================================
module tb_frame_engine;

    localparam integer W = 20;
    localparam integer F = 12;
    localparam integer ACC_W = 44;
    localparam integer INV_W = 16;
    localparam integer NUM_TARGETS = 8;
    localparam integer ID_W = $clog2(NUM_TARGETS);


    // ============================================================
    // Clock / reset
    // ============================================================

    logic clk;
    logic rst;


    // ============================================================
    // Frame control
    // ============================================================

    logic frame_start;


    // ============================================================
    // Measurement input
    // ============================================================

    logic measurement_valid;
    logic signed [W-1:0] z_in [0:1];


    // ============================================================
    // Target-memory load interface
    // ============================================================

    logic load_enable;
    logic [ID_W-1:0] load_target;

    logic signed [W-1:0] x_load [0:3];
    logic signed [W-1:0] P_load [0:3][0:3];

    logic load_done;


    // ============================================================
    // Runtime target-memory write interface
    // ============================================================

    logic write_enable;

    logic signed [W-1:0] x_write [0:3];
    logic signed [W-1:0] P_write [0:3][0:3];


    // ============================================================
    // Kalman configuration
    // ============================================================

    logic signed [W-1:0] F_mat [0:3][0:3];
    logic signed [W-1:0] Q [0:3][0:3];
    logic signed [W-1:0] R_diag [0:1];


    // ============================================================
    // Frame outputs
    // ============================================================

    logic busy;
    logic frame_done;

    logic [ID_W-1:0] target_id;

    logic signed [W-1:0] x_out [0:3];
    logic signed [W-1:0] P_out [0:3][0:3];


    // ============================================================
    // Error counter
    // ============================================================

    integer errors;


    // ============================================================
    // DUT
    // ============================================================

    frame_engine #(
        .W(W),
        .F(F),
        .ACC_W(ACC_W),
        .INV_W(INV_W),
        .NUM_TARGETS(NUM_TARGETS)
    ) dut (
        .clk(clk),
        .rst(rst),

        .frame_start(frame_start),

        .measurement_valid(measurement_valid),
        .z_in(z_in),

        .load_enable(load_enable),
        .load_target(load_target),
        .x_load(x_load),
        .P_load(P_load),
        .load_done(load_done),

        .write_enable(write_enable),
        .x_write(x_write),
        .P_write(P_write),

        .F_mat(F_mat),
        .Q(Q),
        .R_diag(R_diag),

        .busy(busy),
        .frame_done(frame_done),

        .target_id(target_id),

        .x_out(x_out),
        .P_out(P_out)
    );


    // ============================================================
    // Clock
    // ============================================================

    initial begin

        clk = 1'b0;

        forever #5 clk = ~clk;

    end


    // ============================================================
    // Load one target
    //
    // load_done is generated on the rising edge that performs the
    // memory load. Therefore the testbench samples it immediately
    // after that rising edge.
    // ============================================================

    task automatic load_one_target(
        input integer tid
    );

        integer i;
        integer j;

        begin

            load_target = tid[ID_W-1:0];

            for (i = 0; i < 4; i = i + 1) begin

                x_load[i] =
                    (tid + 1) * 100 +
                    i;

                for (j = 0; j < 4; j = j + 1) begin

                    P_load[i][j] =
                        (tid + 1) * 100 +
                        i * 10 +
                        j;

                end

            end


            // ----------------------------------------------------
            // Assert load_enable before the active clock edge.
            // ----------------------------------------------------

            @(negedge clk);

            load_enable = 1'b1;


            // ----------------------------------------------------
            // This is the clock edge at which target_memory
            // performs the load and asserts load_done.
            // ----------------------------------------------------

            @(posedge clk);

            #1;

            if (load_done !== 1'b1) begin

                $display(
                    "ERROR: load_done missing for target %0d",
                    tid
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: target %0d loaded",
                    tid
                );

            end


            // ----------------------------------------------------
            // Deassert load_enable after the load edge.
            // ----------------------------------------------------

            @(negedge clk);

            load_enable = 1'b0;

        end

    endtask


    // ============================================================
    // Provide measurement
    // ============================================================

    task automatic provide_measurement(
        input integer tid
    );

        integer expected_z0;
        integer expected_z1;

        begin

            expected_z0 = (tid + 1) * 100;
            expected_z1 = (tid + 1) * 100 + 1;

            z_in[0] = expected_z0;
            z_in[1] = expected_z1;


            @(negedge clk);

            measurement_valid = 1'b1;

            @(negedge clk);

            measurement_valid = 1'b0;


            $display(
                "INFO: measurement supplied for target %0d: [%0d, %0d]",
                tid,
                expected_z0,
                expected_z1
            );

        end

    endtask


    // ============================================================
    // Main test
    // ============================================================

    integer i;
    integer j;
    integer t;

    integer timeout_count;


    initial begin

        errors = 0;

        rst = 1'b1;

        frame_start = 1'b0;

        measurement_valid = 1'b0;

        load_enable = 1'b0;
        load_target = '0;

        write_enable = 1'b0;

        z_in[0] = '0;
        z_in[1] = '0;


        // --------------------------------------------------------
        // Initialize load/write arrays
        // --------------------------------------------------------

        for (i = 0; i < 4; i = i + 1) begin

            x_load[i] = '0;
            x_write[i] = '0;

            for (j = 0; j < 4; j = j + 1) begin

                P_load[i][j] = '0;
                P_write[i][j] = '0;

            end

        end


        // --------------------------------------------------------
        // Initialize Kalman configuration
        // --------------------------------------------------------

        for (i = 0; i < 4; i = i + 1) begin

            for (j = 0; j < 4; j = j + 1) begin

                F_mat[i][j] = 0;
                Q[i][j] = 0;

            end

        end


        // Constant-velocity model.
        //
        // State:
        // [x, y, vx, vy]
        //
        // dt = 0.01
        // Q8.12:
        // 1.0  = 4096
        // 0.01 = 41 approximately

        F_mat[0][0] = 4096;
        F_mat[1][1] = 4096;
        F_mat[2][2] = 4096;
        F_mat[3][3] = 4096;

        F_mat[0][2] = 41;
        F_mat[1][3] = 41;


        // Process noise.

        Q[0][0] = 1;
        Q[1][1] = 1;
        Q[2][2] = 1;
        Q[3][3] = 1;


        // Measurement noise.

        R_diag[0] = 4096;
        R_diag[1] = 4096;


        // ========================================================
        // RESET
        // ========================================================

        repeat (3)
            @(posedge clk);

        rst = 1'b0;

        @(posedge clk);


        // ========================================================
        // HEADER
        // ========================================================

        $display("");
        $display("==============================================");
        $display("FRAME ENGINE SYSTEM INTEGRATION TEST");
        $display("==============================================");


        // ========================================================
        // TEST 1
        // Load all targets
        // ========================================================

        $display("");
        $display("TEST 1");
        $display("LOADING ALL TARGETS");


        for (t = 0; t < NUM_TARGETS; t = t + 1) begin

            load_one_target(t);

        end


        // ========================================================
        // TEST 2
        // Start complete frame
        // ========================================================

        $display("");
        $display("TEST 2");
        $display("STARTING COMPLETE 8-TARGET FRAME");


        @(negedge clk);

        frame_start = 1'b1;

        @(negedge clk);

        frame_start = 1'b0;


        // ========================================================
        // Process all targets
        // ========================================================

        for (t = 0; t < NUM_TARGETS; t = t + 1) begin


            // ----------------------------------------------------
            // Wait for frame engine to become active.
            // ----------------------------------------------------

            timeout_count = 0;

            while (busy !== 1'b1) begin

                @(posedge clk);

                timeout_count = timeout_count + 1;

                if (timeout_count > 1000) begin

                    $display(
                        "ERROR: timeout waiting for frame busy for target %0d",
                        t
                    );

                    errors = errors + 1;

                    disable process_frame_targets;

                end

            end


            // ----------------------------------------------------
            // Wait for expected target selection.
            // ----------------------------------------------------

            timeout_count = 0;

            while (target_id !== t[ID_W-1:0]) begin

                @(posedge clk);

                timeout_count = timeout_count + 1;

                if (timeout_count > 1000) begin

                    $display(
                        "ERROR: timeout waiting for target %0d selection",
                        t
                    );

                    errors = errors + 1;

                    disable process_frame_targets;

                end

            end


            $display(
                "PASS: frame selected target %0d",
                target_id
            );


            // ----------------------------------------------------
            // Supply measurement.
            // ----------------------------------------------------

            provide_measurement(t);


            // ----------------------------------------------------
            // Wait for target completion.
            //
            // target_done is internal to the scheduler and is used
            // here only for integration-test observation.
            // ----------------------------------------------------

            timeout_count = 0;

            while (dut.u_measurement_scheduler.target_done !== 1'b1) begin

                @(posedge clk);

                timeout_count = timeout_count + 1;

                if (timeout_count > 10000) begin

                    $display(
                        "ERROR: timeout waiting for target %0d completion",
                        t
                    );

                    errors = errors + 1;

                    disable process_frame_targets;

                end

            end


            $display(
                "PASS: target %0d completed",
                t
            );

        end


        process_frame_targets:


        // ========================================================
        // TEST 3
        // Frame completion
        // ========================================================

        $display("");
        $display("TEST 3");
        $display("VERIFYING FRAME COMPLETION");


        timeout_count = 0;

        while (frame_done !== 1'b1) begin

            @(posedge clk);

            timeout_count = timeout_count + 1;

            if (timeout_count > 10000) begin

                $display(
                    "ERROR: timeout waiting for frame_done"
                );

                errors = errors + 1;

                break;

            end

        end


        if (frame_done === 1'b1) begin

            $display(
                "PASS: frame_done asserted"
            );

        end


        // ========================================================
        // TEST 4
        // Final target
        // ========================================================

        $display("");
        $display("TEST 4");
        $display("VERIFYING FINAL TARGET OUTPUT");


        if (target_id !== (NUM_TARGETS - 1)) begin

            $display(
                "ERROR: final target_id = %0d, expected %0d",
                target_id,
                NUM_TARGETS - 1
            );

            errors = errors + 1;

        end
        else begin

            $display(
                "PASS: final target_id = %0d",
                target_id
            );

        end


        $display(
            "INFO: final x_out = %0d, %0d, %0d, %0d",
            x_out[0],
            x_out[1],
            x_out[2],
            x_out[3]
        );


        // ========================================================
        // TEST 5
        // Busy handshake
        // ========================================================

        $display("");
        $display("TEST 5");
        $display("VERIFYING FRAME BUSY HANDSHAKE");


        @(posedge clk);

        if (busy !== 1'b0) begin

            $display(
                "ERROR: busy remains asserted after frame completion"
            );

            errors = errors + 1;

        end
        else begin

            $display(
                "PASS: busy deasserted after frame completion"
            );

        end


        // ========================================================
        // TEST 6
        // Return to IDLE / second frame acceptance
        // ========================================================

        $display("");
        $display("TEST 6");
        $display("VERIFYING RETURN TO IDLE");


        @(negedge clk);

        frame_start = 1'b1;

        @(negedge clk);

        frame_start = 1'b0;


        @(posedge clk);

        #1;

        if (busy !== 1'b1) begin

            $display(
                "ERROR: second frame was not accepted"
            );

            errors = errors + 1;

        end
        else begin

            $display(
                "PASS: second frame accepted"
            );

        end


        // --------------------------------------------------------
        // Reset to terminate the second frame cleanly.
        // --------------------------------------------------------

        @(negedge clk);

        rst = 1'b1;

        repeat (2)
            @(posedge clk);

        rst = 1'b0;


        // ========================================================
        // FINAL RESULT
        // ========================================================

        $display("");
        $display("==============================================");
        $display("FRAME ENGINE SYSTEM INTEGRATION TEST COMPLETE");
        $display("==============================================");


        if (errors == 0) begin

            $display(
                "PASS: frame engine integration matches expected behavior."
            );

        end
        else begin

            $display(
                "FAIL: %0d errors detected.",
                errors
            );

        end


        $finish;

    end

endmodule
