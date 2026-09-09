`timescale 1ns / 1ps

module tb_multi_target_collision_detector;

    localparam integer W = 20;
    localparam integer F = 12;
    localparam integer NUM_TARGETS = 8;
    localparam integer NUM_PAIRS = NUM_TARGETS * (NUM_TARGETS - 1) / 2;

    logic clk;
    logic rst;
    logic start;

    // [0]=x, [1]=y, [2]=vx, [3]=vy (Q8.12)
    logic signed [W-1:0] states [0:NUM_TARGETS-1][0:3];

    logic [NUM_PAIRS-1:0] collision_pairs;
    logic [6:0] max_coarse_risk;
    logic busy;
    logic done;

    // ============================================================
    // DUT Instantiation
    // ============================================================
    multi_target_collision_detector #(
        .W(W),
        .F(F),
        .NUM_TARGETS(NUM_TARGETS)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .states(states),
        .collision_pairs(collision_pairs),
        .max_coarse_risk(max_coarse_risk),
        .busy(busy),
        .done(done)
    );

    // ============================================================
    // Clock Generation (100 MHz)
    // ============================================================
    always #5 clk = ~clk;

    // ============================================================
    // Helper function to convert floats to Q8.12 format
    // ============================================================
    function automatic logic signed [W-1:0] q(input real val);
        return int'(val * 4096.0);
    endfunction

    // ============================================================
    // Helper task to clear all target states
    // ============================================================
    task clear_states();
        for (int i = 0; i < NUM_TARGETS; i++) begin
            // Space them out so they don't collide at the origin
            states[i][0] = q(100.0 * i); 
            states[i][1] = q(100.0 * i);
            states[i][2] = q(0.0);
            states[i][3] = q(0.0);
        end
    endtask

    // ============================================================
    // Test Sequence
    // ============================================================
    initial begin
        clk = 0;
        rst = 1;
        start = 0;
        clear_states();

        #20;
        rst = 0;

        $display("==================================================");
        $display(" MULTI-TARGET COLLISION DETECTOR (28-PAIR SWEEP)  ");
        $display("==================================================");

        // --------------------------------------------------------
        // TEST 1: No Collisions (All targets distant and stationary)
        // --------------------------------------------------------
        #10;
        start = 1;
        #10;
        start = 0;
        wait(done);
        #1;
        
        $display("TEST 1: No Collisions");
        if (collision_pairs == 28'd0 && max_coarse_risk < 20)
            $display(" -> PASS: Pairs flag = %b, Max Risk = %0d%%", collision_pairs, max_coarse_risk);
        else
            $display(" -> FAIL: Pairs flag = %b, Max Risk = %0d%%", collision_pairs, max_coarse_risk);

        // --------------------------------------------------------
        // TEST 2: Single Head-On Collision (Target 0 and Target 1)
        // pair index 0 = (0,1)
        // --------------------------------------------------------
        clear_states();
        states[0][0] = q(0.0); states[0][1] = q(0.0); states[0][2] = q(1.0);  states[0][3] = q(0.0);
        states[1][0] = q(3.0); states[1][1] = q(0.0); states[1][2] = q(-1.0); states[1][3] = q(0.0);
        
        #10;
        start = 1;
        #10;
        start = 0;
        wait(done);
        #1;

        $display("\nTEST 2: Target 0 vs Target 1 (Index 0)");
        if (collision_pairs[0] == 1'b1 && max_coarse_risk > 90)
            $display(" -> PASS: T0-T1 Collide! Pairs flag = %b, Max Risk = %0d%%", collision_pairs, max_coarse_risk);
        else
            $display(" -> FAIL: T0-T1 didn't register. Pairs flag = %b", collision_pairs);

        // --------------------------------------------------------
        // TEST 3: Multi-Pair & Early Exit Fast-Track
        // Target 2 & Target 5 collide (Index 15)
        // Target 6 & Target 7 move away (Early exit internally)
        // --------------------------------------------------------
        clear_states();
        // T2 vs T5 (Collision course on Y-axis)
        states[2][0] = q(10.0); states[2][1] = q(10.0); states[2][2] = q(0.0); states[2][3] = q(1.0);
        states[5][0] = q(10.0); states[5][1] = q(12.0); states[5][2] = q(0.0); states[5][3] = q(-1.0);
        
        // T6 vs T7 (Moving away from each other on X-axis)
        states[6][0] = q(20.0); states[6][1] = q(20.0); states[6][2] = q(-1.0); states[6][3] = q(0.0);
        states[7][0] = q(30.0); states[7][1] = q(20.0); states[7][2] = q(1.0);  states[7][3] = q(0.0);

        #10;
        start = 1;
        #10;
        start = 0;
        wait(done);
        #1;

        $display("\nTEST 3: Target 2 vs Target 5 (Index 15) + T6/T7 Diverging");
        if (collision_pairs[15] == 1'b1 && collision_pairs[27] == 1'b0)
            $display(" -> PASS: T2-T5 Collide! T6-T7 Diverged safely. Max Risk = %0d%%", max_coarse_risk);
        else
            $display(" -> FAIL: Mask mismatch. Pairs flag = %b", collision_pairs);

        $display("\n==================================================");
        $display(" TESTS COMPLETE");
        $display("==================================================");
        
        #20;
        $finish;
    end
endmodule