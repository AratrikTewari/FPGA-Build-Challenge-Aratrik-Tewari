`timescale 1ns / 1ps

module tb_measurement_scheduler;

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
    logic signed [19:0] z_in [0:1];

    // ============================================================
    // Target processing interface
    // ============================================================

    logic [ID_W-1:0] target_id;

    logic signed [19:0] z_out [0:1];

    logic target_start;

    logic target_busy;
    logic target_done;

    // ============================================================
    // Frame status
    // ============================================================

    logic busy;
    logic frame_done;

    // ============================================================
    // Error counter
    // ============================================================

    integer errors;

    // ============================================================
    // DUT
    // ============================================================

    measurement_scheduler #(
        .NUM_TARGETS(NUM_TARGETS)
    ) dut (
        .clk(clk),
        .rst(rst),

        .frame_start(frame_start),

        .measurement_valid(measurement_valid),
        .z_in(z_in),

        .target_id(target_id),
        .z_out(z_out),

        .target_start(target_start),

        .target_busy(target_busy),
        .target_done(target_done),

        .busy(busy),
        .frame_done(frame_done)
    );

    // ============================================================
    // Clock
    // ============================================================

    initial begin
        clk = 1'b0;

        forever #5 clk = ~clk;
    end

    // ============================================================
    // Simulated target processor
    //
    // target_start is sampled synchronously.
    //
    // Once started:
    //   cycle 1 -> processing
    //   cycle 2 -> processing
    //   cycle 3 -> target_done pulse
    //
    // target_busy is included to model the real AEI interface.
    // ============================================================

    integer processing_count;

    always_ff @(posedge clk) begin

        if (rst) begin

            processing_count <= 0;

            target_busy <= 1'b0;
            target_done <= 1'b0;

        end
        else begin

            target_done <= 1'b0;

            if (target_start && !target_busy) begin

                target_busy <= 1'b1;
                processing_count <= 2;

            end
            else if (processing_count > 0) begin

                processing_count <= processing_count - 1;

                if (processing_count == 1) begin

                    target_busy <= 1'b0;
                    target_done <= 1'b1;

                end

            end

        end

    end

    // ============================================================
    // Task: send one measurement
    //
    // measurement_valid is asserted across one complete rising
    // clock edge, guaranteeing that the DUT samples it.
    // ============================================================

    task automatic send_measurement(
        input integer tid
    );

        begin

            z_in[0] = (tid + 1) * 100;
            z_in[1] = (tid + 1) * 100 + 1;

            @(negedge clk);

            measurement_valid = 1'b1;

            @(negedge clk);

            measurement_valid = 1'b0;

        end

    endtask

    // ============================================================
    // Task: wait for a one-cycle pulse
    //
    // This samples the signal only after a rising edge, avoiding
    // races between the testbench and DUT always_ff blocks.
    // ============================================================

    task automatic wait_for_target_start(
        input integer expected
    );

        integer timeout;

        begin

            timeout = 0;

            forever begin

                @(posedge clk);
                #1;

                if (target_start === 1'b1)
                    break;

                timeout = timeout + 1;

                if (timeout > 100) begin

                    $display(
                        "ERROR: timeout waiting for target_start for target %0d",
                        expected
                    );

                    errors = errors + 1;

                    $finish;

                end

            end

        end

    endtask

    // ============================================================
    // Task: wait for target_done
    // ============================================================

    task automatic wait_for_target_done(
        input integer tid
    );

        integer timeout;

        begin

            timeout = 0;

            forever begin

                @(posedge clk);
                #1;

                if (target_done === 1'b1)
                    break;

                timeout = timeout + 1;

                if (timeout > 100) begin

                    $display(
                        "ERROR: timeout waiting for target_done for target %0d",
                        tid
                    );

                    errors = errors + 1;

                    $finish;

                end

            end

        end

    endtask

    // ============================================================
    // Main test
    // ============================================================

    integer t;
    integer i;

    initial begin

        errors = 0;

        rst = 1'b1;

        frame_start = 1'b0;

        measurement_valid = 1'b0;

        z_in[0] = '0;
        z_in[1] = '0;

        // ========================================================
        // Reset
        // ========================================================

        repeat (3)
            @(posedge clk);

        rst = 1'b0;

        @(posedge clk);

        // ========================================================
        // Header
        // ========================================================

        $display("");
        $display("==============================================");
        $display("MEASUREMENT SCHEDULER TEST");
        $display("==============================================");

        // ========================================================
        // TEST 1
        // ========================================================

        $display("");
        $display("TEST 1");
        $display("STARTING FRAME");

        // --------------------------------------------------------
        // Start frame.
        // --------------------------------------------------------

        @(negedge clk);

        frame_start = 1'b1;

        @(negedge clk);

        frame_start = 1'b0;

        // --------------------------------------------------------
        // Process all eight targets.
        // --------------------------------------------------------

        for (t = 0; t < NUM_TARGETS; t = t + 1) begin

            // ----------------------------------------------------
            // Wait until the scheduler is in the measurement
            // phase for this target.
            //
            // target_id is updated by the scheduler before the
            // measurement is consumed.
            // ----------------------------------------------------

            while (target_id !== t[ID_W-1:0]) begin
                @(posedge clk);
                #1;
            end

            // ----------------------------------------------------
            // Send measurement.
            // ----------------------------------------------------

            send_measurement(t);

            // ----------------------------------------------------
            // Wait for target_start.
            // ----------------------------------------------------

            wait_for_target_start(t);

            // ----------------------------------------------------
            // Verify selected target.
            // ----------------------------------------------------

            if (target_id !== t[ID_W-1:0]) begin

                $display(
                    "ERROR: expected target %0d, got target %0d",
                    t,
                    target_id
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: target %0d selected",
                    t
                );

            end

            // ----------------------------------------------------
            // Verify z[0].
            // ----------------------------------------------------

            if (z_out[0] !== ((t + 1) * 100)) begin

                $display(
                    "ERROR: target %0d z[0] = %0d, expected %0d",
                    t,
                    z_out[0],
                    (t + 1) * 100
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: target %0d z[0] = %0d",
                    t,
                    z_out[0]
                );

            end

            // ----------------------------------------------------
            // Verify z[1].
            // ----------------------------------------------------

            if (z_out[1] !== ((t + 1) * 100 + 1)) begin

                $display(
                    "ERROR: target %0d z[1] = %0d, expected %0d",
                    t,
                    z_out[1],
                    (t + 1) * 100 + 1
                );

                errors = errors + 1;

            end
            else begin

                $display(
                    "PASS: target %0d z[1] = %0d",
                    t,
                    z_out[1]
                );

            end

            // ----------------------------------------------------
            // Wait for simulated target processor to complete.
            // ----------------------------------------------------

            wait_for_target_done(t);

            $display(
                "PASS: target %0d processing completed",
                t
            );

        end

        // ========================================================
        // TEST 2
        // Verify frame_done.
        //
        // frame_done is a one-clock pulse generated after target 7
        // completes. We sample it synchronously.
        // ========================================================

        $display("");
        $display("TEST 2");
        $display("VERIFYING FRAME COMPLETION");

        begin : frame_done_check

            integer timeout;
            logic frame_done_seen;

            timeout = 0;
            frame_done_seen = 1'b0;

            forever begin

                @(posedge clk);
                #1;

                if (frame_done === 1'b1) begin

                    frame_done_seen = 1'b1;

                    $display(
                        "PASS: frame_done asserted"
                    );

                    break;

                end

                timeout = timeout + 1;

                if (timeout > 100) begin

                    $display(
                        "ERROR: timeout waiting for frame_done"
                    );

                    errors = errors + 1;

                    break;

                end

            end

        end

        // ========================================================
        // TEST 3
        // Verify busy deassertion.
        //
        // The scheduler enters IDLE on the clock following
        // FRAME_FINISH.
        // ========================================================

        $display("");
        $display("TEST 3");
        $display("VERIFYING FRAME HANDSHAKE");

        @(posedge clk);
        #1;

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
        // TEST 4
        // Verify return to IDLE by starting another frame.
        //
        // Only frame acceptance is tested here; TEST 1 already
        // verifies complete eight-target processing.
        // ========================================================

        $display("");
        $display("TEST 4");
        $display("VERIFYING RETURN TO IDLE");

        @(negedge clk);

        frame_start = 1'b1;

        @(negedge clk);

        frame_start = 1'b0;

        // --------------------------------------------------------
        // Verify scheduler accepts the new frame.
        // --------------------------------------------------------

        begin : second_frame_check

            integer timeout;

            timeout = 0;

            while (busy !== 1'b1) begin

                @(posedge clk);
                #1;

                timeout = timeout + 1;

                if (timeout > 20) begin

                    $display(
                        "ERROR: scheduler did not accept second frame"
                    );

                    errors = errors + 1;

                    break;

                end

            end

            if (busy === 1'b1) begin

                $display(
                    "PASS: scheduler returned to IDLE"
                );

            end

        end

        // ========================================================
        // Final result
        // ========================================================

        $display("");
        $display("==============================================");
        $display("MEASUREMENT SCHEDULER TEST COMPLETE");
        $display("==============================================");

        if (errors == 0) begin

            $display(
                "PASS: measurement scheduler matches expected behavior."
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