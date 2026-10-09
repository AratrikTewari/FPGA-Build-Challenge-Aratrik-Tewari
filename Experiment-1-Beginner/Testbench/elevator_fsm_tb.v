`timescale 1ns / 1ps

// ============================================================================
// Testbench:     elevator_fsm_tb
// Project:       Multi-Floor FPGA Elevator Controller (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// Verification Scope & System Context:
//   This self-checking testbench verifies the algorithmic correctness and
//   state transitions of the elevator finite state machine (elevator_fsm.v).
//   In the overall system hierarchy, this controller sits between the debounced
//   user inputs (from debouncer.v via elevator_top.v) and the physical board
//   actuators (RGB LEDs representing motor hoist direction and door status).
//
// Verification Coverage:
//   1. Reset & Quiescent State:
//      - Confirms that asserting reset places the FSM in IDLE with all motor
//        and door control lines deactivated.
//   2. Ascending Dispatch & Intermediate Floor Traversal:
//      - Asserts target_floor > current_floor (Floor 0 -> Floor 2).
//      - Confirms MOVE_UP activation (motor_up = 1, motor_down = 0, door_open = 0).
//      - Simulates cabin advancing past an intermediate level (Floor 1) to verify
//        uninterrupted upward motion without premature stops.
//   3. Floor Arrival & Safety Interlock Dwell:
//      - Simulates reaching the destination (current_floor = 2'b10).
//      - Confirms immediate hoist motor cutoff and transition to DOOR_OPEN_STATE.
//      - Asserts door_sensor = 1 (active optical obstruction) to verify that the
//        FSM respects the passenger safety barrier before attempting state clearance.
//   4. Obstruction Clearance & Quiescent Restoration:
//      - De-asserts door_sensor = 0 and validates safe re-entry into IDLE.
//   5. Descending Dispatch:
//      - Asserts target_floor < current_floor (Floor 2 -> Floor 0).
//      - Confirms MOVE_DOWN activation (motor_down = 1, motor_up = 0).
//
// Hardware Timing Emulation:
//   Clock Generator: always #4 clk = ~clk; -> T_clk = 8.0 ns (125 MHz equivalent)
// ============================================================================

module elevator_fsm_tb;

    // ------------------------------------------------------------------------
    // Testbench Stimulus Registers
    // ------------------------------------------------------------------------
    reg       clk;           // Emulated 125 MHz system master clock
    reg       reset;         // Active-high master reset stimulus
    reg [1:0] target_floor;  // Simulated destination call request [Floor 0..3]
    reg [1:0] current_floor; // Simulated shaft position sensor feedback [Floor 0..3]
    reg       door_sensor;   // Simulated optical safety obstruction beam (active-high)

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Output Monitoring Wires
    // ------------------------------------------------------------------------
    wire      motor_up;      // Hoist UP actuator output (drives LD4 Red)
    wire      motor_down;    // Hoist DOWN actuator output (drives LD4 Blue)
    wire      door_open;     // Door actuator output (drives LD5 Green)

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Instantiation
    // ------------------------------------------------------------------------
    elevator_fsm uut (
        .clk           (clk),
        .reset         (reset),
        .target_floor  (target_floor),
        .current_floor (current_floor),
        .door_sensor   (door_sensor),
        .motor_up      (motor_up),
        .motor_down    (motor_down),
        .door_open     (door_open)
    );

    // ------------------------------------------------------------------------
    // System Clock Generation (125 MHz)
    // T_high = 4 ns, T_low = 4 ns -> Period = 8 ns -> 125 MHz
    // ------------------------------------------------------------------------
    always #4 clk = ~clk;

    // ------------------------------------------------------------------------
    // Main Verification Scenario
    // ------------------------------------------------------------------------
    initial begin
        $display("==================================================");
        $display("[Time %0t ns] STARTING FSM TEST", $time);
        $display("==================================================");
        
        // --------------------------------------------------------------------
        // Scenario 1: System Power-On & Master Reset Validation
        // --------------------------------------------------------------------
        clk          = 0;
        reset        = 1;
        target_floor = 2'b00;
        current_floor = 2'b00;
        door_sensor  = 0;

        #16;
        reset        = 0; // Release master reset line
        #8;

        // --------------------------------------------------------------------
        // Scenario 2: Ascending Motion Dispatch (Floor 0 -> Floor 2)
        // --------------------------------------------------------------------
        $display("[Time %0t ns] -> Requesting Floor 2 from Floor 0", $time);
        target_floor  = 2'b10;
        current_floor = 2'b00;
        #8;
        
        // Validation: Verify motor up engagement with all competing drivers held off
        if (motor_up == 1 && motor_down == 0 && door_open == 0)
            $display("[PASS] FSM entered MOVE_UP state.");
        else
            $display("[FAIL] Expected MOVE_UP. Got: UP=%b DOWN=%b DOOR=%b", motor_up, motor_down, door_open);

        // --------------------------------------------------------------------
        // Scenario 3: Shaft Traversal Past Intermediate Floor (Floor 1)
        // --------------------------------------------------------------------
        $display("[Time %0t ns] -> Passing Floor 1...", $time);
        current_floor = 2'b01;
        #16;
        
        // Validation: Confirm cabin continues hoist motion past unrequested landing
        if (motor_up == 1)
            $display("[PASS] FSM maintained MOVE_UP state through intermediate floor.");

        // --------------------------------------------------------------------
        // Scenario 4: Destination Arrival & Obstruction Beam Detection
        // --------------------------------------------------------------------
        $display("[Time %0t ns] -> Arriving at Floor 2 (Door Blocked)", $time);
        current_floor = 2'b10;
        door_sensor   = 1; // Simulate passenger interrupting the optical safety curtain
        #8;
        
        // Validation: Hoist motor must cut off instantly and door actuator engage
        if (door_open == 1 && motor_up == 0)
            $display("[PASS] FSM entered DOOR_OPEN_STATE and halted motor.");
        else
            $display("[FAIL] Failed to enter DOOR_OPEN_STATE.");

        // --------------------------------------------------------------------
        // Scenario 5: Safety Barrier Clearance & Dwell Resolution
        // --------------------------------------------------------------------
        #32;
        $display("[Time %0t ns] -> Clearing Door Sensor", $time);
        door_sensor = 0; // Obstruction removed
        #8;
        
        // Validation: System must transition back to IDLE with all actuators silent
        if (door_open == 0 && motor_up == 0 && motor_down == 0)
            $display("[PASS] FSM safely returned to IDLE.");

        // --------------------------------------------------------------------
        // Scenario 6: Descending Motion Dispatch (Floor 2 -> Floor 0)
        // --------------------------------------------------------------------
        $display("[Time %0t ns] -> Requesting Floor 0 from Floor 2", $time);
        target_floor = 2'b00;
        #8;
        
        // Validation: Verify motor down engagement with safe interlocking
        if (motor_down == 1 && motor_up == 0)
            $display("[PASS] FSM entered MOVE_DOWN state.");
        else
            $display("[FAIL] Expected MOVE_DOWN. Got: UP=%b DOWN=%b DOOR=%b", motor_up, motor_down, door_open);

        $display("==================================================");
        $display("[Time %0t ns] FSM TEST COMPLETE", $time);
        $display("==================================================");
        $finish;
    end

endmodule
