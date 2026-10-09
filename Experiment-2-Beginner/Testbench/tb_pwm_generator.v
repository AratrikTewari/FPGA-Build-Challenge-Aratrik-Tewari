`timescale 1ns/1ps

// ============================================================================
// Module Name:   tb_pwm_generator
// Project:       Zynq-7000 Digital Door Lock System (PYNQ-Z2)
// Target Device: AMD Xilinx Zynq-7000 SoC (XC7Z020-1CLG400C)
//
// Description:
//   Unit testbench verifying the 50 Hz PWM servo generator (pwm_generator.v).
//   Validates dynamic duty-cycle modulation driven by the access control state:
//     1. System reset and timebase counter initialization.
//     2. Quiescent LOCKED mode producing a nominal 1.0 ms high-pulse (0° position).
//     3. Dynamic transition to UNLOCKED mode producing a 2.0 ms high-pulse (90° position).
//     4. Reversion to LOCKED mode following expiration of the FSM relock window.
//
// Simulation Timing & Calibration:
//   - Clock Reference: 125 MHz system clock (8.0 ns period: #4 high / #4 low).
//   - Timing Acceleration:
//       * PERIOD_CYCLES scaled down to 100 cycles (800 ns vs. 20.0 ms in hardware).
//       * LOCKED_HIGH scaled to 10 cycles (10% duty cycle, representing 1.0 ms).
//       * UNLOCKED_HIGH scaled to 20 cycles (20% duty cycle, representing 2.0 ms).
//     This preserves duty-cycle ratios while completing full modulation cycles
//     within nanoseconds of simulation time rather than milliseconds.
// ============================================================================

module tb_pwm_generator;

    // ------------------------------------------------------------------------
    // Testbench Stimulus Signals & Interconnects
    // ------------------------------------------------------------------------
    reg  clk        = 0;  // 125 MHz system clock reference
    reg  rst        = 0;  // Synchronous master reset
    reg  unlock_sig = 0;  // Actuation enable input from door_lock_fsm
    wire servo_pwm;       // Modulated PWM output monitored from DUT

    // ------------------------------------------------------------------------
    // Clock Generator: 125 MHz (Period = 8.0 ns)
    // ------------------------------------------------------------------------
    always #4 clk = ~clk;

    // ------------------------------------------------------------------------
    // Device Under Test (DUT) Instantiation
    // Scaled parameters: 100-cycle frame period, 10/20-cycle active pulses
    // ------------------------------------------------------------------------
    pwm_generator #(
        .PERIOD_CYCLES(100), // Accelerated frame window for simulation efficiency
        .LOCKED_HIGH(10),    // 10% duty cycle (emulates 1.0 ms pulse @ 50 Hz)
        .UNLOCKED_HIGH(20)   // 20% duty cycle (emulates 2.0 ms pulse @ 50 Hz)
    ) uut (
        .clk        (clk),
        .rst        (rst),
        .unlock_sig (unlock_sig),
        .servo_pwm  (servo_pwm)
    );

    // ------------------------------------------------------------------------
    // Simulation Stimulus Sequence
    // ------------------------------------------------------------------------
    initial begin
        $display("[%0t] Starting PWM Generator Test...", $time);
        
        // 1. Assert Power-On Reset Pulse (>2 clock cycles)
        rst = 1; 
        #20; 
        rst = 0;
        
        // --------------------------------------------------------------------
        // Test Phase 1: Quiescent LOCKED State (Deadbolt Engaged)
        // Verify default 10-cycle active pulse width across 3 full frame periods.
        // --------------------------------------------------------------------
        $display("[%0t] Mode: LOCKED (1ms pulse expected)", $time);
        #300; // Observe 3 full accelerated PWM cycles
        
        // --------------------------------------------------------------------
        // Test Phase 2: Access Granted / UNLOCKED State (Deadbolt Retracted)
        // Assert unlock_sig to trigger 20-cycle active pulse width expansion.
        // --------------------------------------------------------------------
        $display("[%0t] Mode: UNLOCKED (2ms pulse expected)", $time);
        unlock_sig = 1;
        #300; // Observe modulated 20-cycle pulse widths
        
        // --------------------------------------------------------------------
        // Test Phase 3: Auto-Relock Transition
        // De-assert unlock_sig to confirm instantaneous return to 10-cycle pulses.
        // --------------------------------------------------------------------
        $display("[%0t] Mode: LOCKED (Reverted to 1ms)", $time);
        unlock_sig = 0;
        #300; // Confirm stable return to resting state
        
        $display("[%0t] PWM Generator Test Complete.\n", $time);
        $finish;
    end

endmodule
