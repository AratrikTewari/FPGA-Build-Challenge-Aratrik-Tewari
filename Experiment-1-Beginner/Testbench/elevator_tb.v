`timescale 1ns / 1ps

module elevator_tb;
    reg clk;
    reg rst_btn;
    reg [1:0] target_sw;
    reg [1:0] current_fl_sw;
    reg door_sens_btn;

    wire motor_up_led;
    wire motor_down_led;
    wire door_open_led;

    // Override limits for rapid 1ms/2ms simulation
    elevator_top #(
        .DB_LIMIT(125000), 
        .DOOR_LIMIT(250000)
    ) uut (
        .clk(clk),
        .rst_btn(rst_btn),
        .target_sw(target_sw),
        .current_fl_sw(current_fl_sw),
        .door_sens_btn(door_sens_btn),
        .motor_up_led(motor_up_led),
        .motor_down_led(motor_down_led),
        .door_open_led(door_open_led)
    );

    always #4 clk = ~clk;

    initial begin
        $display("==================================================");
        $display("[Time %0t ns] STARTING TOP LEVEL INTEGRATION TEST", $time);
        $display("==================================================");
        
        $monitor("[EVENT Time: %0t ns] LED STATUS -> UP(Red): %b | DOWN(Blue): %b | DOOR(Green): %b", 
                 $time, motor_up_led, motor_down_led, door_open_led);

        clk = 0;
        rst_btn = 1;         
        target_sw = 2'b00;   
        current_fl_sw = 2'b00; 
        door_sens_btn = 0;   

        $display("[Time %0t ns] System Reset Held...", $time);
        #2000000; // Wait 2 ms
        rst_btn = 0;         
        
        $display("[Time %0t ns] System Reset Released. Waiting 2ms for debounce...", $time);
        #2000000;      
        
        $display("[Time %0t ns] User sets target switch to Floor 2 (10)...", $time);
        target_sw = 2'b10;
        
        #3000000; // Wait 3 ms for movement to start

        $display("[Time %0t ns] Elevator passes physical Floor 1 (01)...", $time);
        current_fl_sw = 2'b01;
        
        #3000000; // Wait 3 ms for movement

        $display("[Time %0t ns] Elevator reaches physical Floor 2 (10)...", $time);
        current_fl_sw = 2'b10;

        #1000; // Slight delay to mimic physical misalignment
        $display("[Time %0t ns] Passenger trips the door sensor...", $time);
        door_sens_btn = 1;
        
        // Wait 5 ms to outlast the 2ms door timer + 1ms debounce
        #5000000; 
        
        $display("[Time %0t ns] Passenger clears the door...", $time);
        door_sens_btn = 0;
        
        // Wait 5 ms to allow debounced signal to drop and FSM to return IDLE
        #5000000; 

        $display("==================================================");
        $display("[Time %0t ns] TOP LEVEL TEST COMPLETE", $time);
        $display("==================================================");
        $finish;
    end
endmodule