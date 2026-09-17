`timescale 1ns/1ps

module tb_pwm_generator;
    reg clk = 0;
    reg rst = 0;
    reg unlock_sig = 0;
    wire servo_pwm;

    always #4 clk = ~clk;

    pwm_generator #(
        .PERIOD_CYCLES(100), 
        .LOCKED_HIGH(10), 
        .UNLOCKED_HIGH(20)
    ) uut (
        .clk(clk),
        .rst(rst),
        .unlock_sig(unlock_sig),
        .servo_pwm(servo_pwm)
    );

    initial begin
        $display("[%0t] Starting PWM Generator Test...", $time);
        rst = 1; #20 rst = 0;
        
        $display("[%0t] Mode: LOCKED (1ms pulse expected)", $time);
        #300; 
        
        $display("[%0t] Mode: UNLOCKED (2ms pulse expected)", $time);
        unlock_sig = 1;
        #300; 
        
        $display("[%0t] Mode: LOCKED (Reverted to 1ms)", $time);
        unlock_sig = 0;
        #300;
        
        $display("[%0t] PWM Generator Test Complete.\n", $time);
        $finish;
    end
endmodule