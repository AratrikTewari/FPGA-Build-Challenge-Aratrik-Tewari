`timescale 1ns / 1ps

// ============================================================================
// Testbench:     elevator_tb
// Project:       Multi-Floor FPGA Elevator Controller (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// Verification Scope & System Context:
//   This testbench serves as the complete top-level hardware integration test
//   for the elevator control system ('elevator_top.v'). It evaluates the unified
//   interaction of:
//     1. Input conditioning filters ('debouncer.v') across asynchronous manual inputs.
//     2. Synchronous floor-latching registers capturing momentary arrival pulses.
//     3. The central finite state machine ('elevator_fsm.v') coordinating motion logic.
//     4. Board-level peripheral driving (RGB LEDs indicating hoist direction and door dwell).
//
// Verification Methodology & Parameter Scaling:
//   In physical hardware:
//     - Debounce window = 10 ms (1,250,000 clock cycles at 125 MHz).
//     - Door dwell interval = 2.0 s (250,000,000 clock cycles at 125 MHz).
//   For cycle-accurate HDL simulation efficiency without altering state mechanics:
//     - 'DB_LIMIT' is overridden to 125,000 cycles (1.0 ms).
//     - 'DOOR_LIMIT' is overridden to 250,000 cycles (2.0 ms).
//   This allows thorough multi-millisecond end-to-end verification within seconds
//   of simulator runtime.
//
// Emulated Hardware Timing:
//   Clock Generator: always #4 clk = ~clk; -> T_clk = 8.0 ns (125 MHz system clock)
// ============================================================================

module elevator_tb;

    // ------------------------------------------------------------------------
    // Testbench Stimulus Registers (Driving Physical Board Inputs)
    // ------------------------------------------------------------------------
    reg       clk;           // 125 MHz master clock oscillator (Pin H16)
    reg       rst_btn;       // Asynchronous master reset push button (BTN0, Pin D19)
    reg [1:0] target_sw;     // Target floor call switches (SW0: Pin M20, SW1: Pin M19)
    reg [1:0] current_fl_sw; // Momentary floor arrival buttons (BTN1: Pin D20, BTN2: Pin L20)
    reg       door_sens_btn; // Optical safety beam / obstacle sensor (BTN3, Pin L19)

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Output Monitoring Wires (RGB LED Status)
    // ------------------------------------------------------------------------
    wire      motor_up_led;   // Hoist UP active indicator (LD4 Red, Pin N15)
    wire      motor_down_led; // Hoist DOWN active indicator (LD4 Blue, Pin L15)
    wire      door_open_led;  // Door open dwell indicator (LD5 Green, Pin L14)

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Instantiation
    // Overriding hardware counter limits for rapid 1 ms / 2 ms simulation timing
    // ------------------------------------------------------------------------
    elevator_top #(
        .DB_LIMIT   (125000), // Scaled 1.0 ms debounce window (125,000 cycles * 8 ns)
        .DOOR_LIMIT (250000)  // Scaled 2.0 ms door open interval (250,000 cycles * 8 ns)
    ) uut (
        .clk            (clk),
        .rst_btn        (rst_btn),
        .target_sw      (target_sw),
        .current_fl_sw  (current_fl_sw),
        .door_sens_btn  (door_sens_btn),
        .motor_up_led   (motor_up_led),
        .motor_down_led (motor_down_led),
        .door_open_led  (door_open_led)
    );

    // ------------------------------------------------------------------------
    // System Clock Generation (125 MHz)
    // 4 ns high / 4 ns low toggling -> Period = 8.0 ns
    // ------------------------------------------------------------------------
    always #4 clk = ~clk;

    // ------------------------------------------------------------------------
    // Main Verification Scenario
    // ------------------------------------------------------------------------
    initial begin
        $display("==================================================");
        $display("[Time %0t ns] STARTING TOP LEVEL INTEGRATION TEST", $time);
        $display("==================================================");
        
        // Continuous event monitor tracking LED actuator states across simulation time
        $monitor("[EVENT Time: %0t ns] LED STATUS -> UP(Red): %b | DOWN(Blue): %b | DOOR(Green): %b", 
                 $time, motor_up_led, motor_down_led, door_open_led);

        // --------------------------------------------------------------------
        // Phase 1: Quiescent State Initialization & Power-On Reset
        // --------------------------------------------------------------------
        clk           = 0;
        rst_btn       = 1;     // Assert active-high master reset (BTN0)
        target_sw     = 2'b00; // Default target call to Ground Floor (Floor 0)
        current_fl_sw = 2'b00; // Default shaft feedback at Floor 0
        door_sens_btn = 0;     // Optical curtain unblocked

        $display("[Time %0t ns] System Reset Held...", $time);
        #2000000; // Hold reset active for 2.0 ms (exceeds 1.0 ms debounce limit)
        rst_btn       = 0;     // De-assert reset
        
        $display("[Time %0t ns] System Reset Released. Waiting 2ms for debounce...", $time);
        #2000000; // Allow 2.0 ms for reset de-assertion to filter through debouncer chain
        
        // --------------------------------------------------------------------
        // Phase 2: Upward Dispatch Request (Floor 0 -> Floor 2)
        // --------------------------------------------------------------------
        $display("[Time %0t ns] User sets target switch to Floor 2 (10)...", $time);
        target_sw = 2'b10; // Request destination Floor 2 via slide switches (SW1=1, SW0=0)
        
        #3000000; // Allow 3.0 ms for target switch sampling and hoist UP activation (LD4 Red)

        // --------------------------------------------------------------------
        // Phase 3: Shaft Traversal Past Intermediate Level (Floor 1)
        // --------------------------------------------------------------------
        $display("[Time %0t ns] Elevator passes physical Floor 1 (01)...", $time);
        current_fl_sw = 2'b01; // Momentary landing sensor trip at Floor 1 (BTN1)
        
        #3000000; // Hold 3.0 ms to qualify debouncer, latch position, and verify continuous hoist UP

        // --------------------------------------------------------------------
        // Phase 4: Target Destination Arrival (Floor 2)
        // --------------------------------------------------------------------
        $display("[Time %0t ns] Elevator reaches physical Floor 2 (10)...", $time);
        current_fl_sw = 2'b10; // Floor 2 landing sensor triggered (BTN2)

        #1000; // Brief phase offset to emulate real-world sensor-to-actuator physical latency
        
        // --------------------------------------------------------------------
        // Phase 5: Passenger Safety Interlock & Door Obstruction Evaluation
        // --------------------------------------------------------------------
        $display("[Time %0t ns] Passenger trips the door sensor...", $time);
        door_sens_btn = 1; // Trip optical door obstruction beam (BTN3)
        
        // Hold obstruction for 5.0 ms: outlasts the 2.0 ms door dwell timer and 1.0 ms debounce.
        // Verifies that door actuator remains forced open (LD5 Green) while beam is blocked.
        #5000000; 
        
        $display("[Time %0t ns] Passenger clears the door...", $time);
        door_sens_btn = 0; // Obstruction removed
        
        // Allow 5.0 ms for door sensor de-assertion debounce filter and subsequent FSM return to IDLE
        #5000000; 

        $display("==================================================");
        $display("[Time %0t ns] TOP LEVEL TEST COMPLETE", $time);
        $display("==================================================");
        $finish;
    end

endmodule
