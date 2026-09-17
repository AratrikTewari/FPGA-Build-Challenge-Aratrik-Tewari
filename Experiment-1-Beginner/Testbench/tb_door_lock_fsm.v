`timescale 1ns/1ps

module tb_door_lock_fsm;
    reg clk = 0;
    reg rst = 0;
    reg key_valid = 0;
    reg [3:0] key_data = 0;
    
    wire unlock_sig, alarm_sig;
    wire [2:0] rgb_status;

    always #4 clk = ~clk;

    door_lock_fsm #(.UNLOCK_HOLD_CYCLES(20)) uut (
        .clk(clk),
        .rst(rst),
        .key_valid(key_valid),
        .key_data(key_data),
        .unlock_sig(unlock_sig),
        .rgb_status(rgb_status),
        .alarm_sig(alarm_sig)
    );

    task press_key(input [3:0] k);
    begin
        $display("[%0t] Pressing Key: %h", $time, k);
        key_data = k; 
        key_valid = 1;
        #8 key_valid = 0; 
        #16;
    end
    endtask

    always @(posedge unlock_sig) $display("\n[%0t] *** SYSTEM UNLOCKED *** (rgb_status: %b)", $time, rgb_status);
    always @(posedge alarm_sig) $display("\n[%0t] !!! ALARM TRIGGERED !!! (rgb_status: %b)", $time, rgb_status);

    initial begin
        $display("[%0t] Starting FSM Test...", $time);
        rst = 1; #20 rst = 0;

        $display("\n[%0t] --- Testing Correct PIN (1234) ---", $time);
        press_key(4'h1); press_key(4'h2); press_key(4'h3); press_key(4'h4);
        
        #300; 
        $display("[%0t] Auto-lock timer expired. System locked.", $time);

        $display("\n[%0t] --- Testing Incorrect PINs (Trigger Alarm) ---", $time);
        repeat(3) begin
            press_key(4'hF); press_key(4'hF); press_key(4'hF); press_key(4'hF);
            #50;
        end
        
        #100;
        $display("[%0t] FSM Test Complete.\n", $time);
        $finish;
    end
endmodule