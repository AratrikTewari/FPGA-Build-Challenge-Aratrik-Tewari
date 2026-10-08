`timescale 1ns / 1ps

module elevator_fsm_tb;
    reg clk;
    reg reset;
    reg [1:0] target_floor;
    reg [1:0] current_floor;
    reg door_sensor;

    wire motor_up;
    wire motor_down;
    wire door_open;

    elevator_fsm uut (
        .clk(clk),
        .reset(reset),
        .target_floor(target_floor),
        .current_floor(current_floor),
        .door_sensor(door_sensor),
        .motor_up(motor_up),
        .motor_down(motor_down),
        .door_open(door_open)
    );

    always #4 clk = ~clk;

    initial begin
        $display("==================================================");
        $display("[Time %0t ns] STARTING FSM TEST", $time);
        $display("==================================================");
        
        clk = 0;
        reset = 1;
        target_floor = 2'b00;
        current_floor = 2'b00;
        door_sensor = 0;

        #16;
        reset = 0;
        #8;

        $display("[Time %0t ns] -> Requesting Floor 2 from Floor 0", $time);
        target_floor = 2'b10;
        current_floor = 2'b00;
        #8;
        if (motor_up == 1 && motor_down == 0 && door_open == 0)
            $display("[PASS] FSM entered MOVE_UP state.");
        else
            $display("[FAIL] Expected MOVE_UP. Got: UP=%b DOWN=%b DOOR=%b", motor_up, motor_down, door_open);

        $display("[Time %0t ns] -> Passing Floor 1...", $time);
        current_floor = 2'b01;
        #16;
        if (motor_up == 1)
            $display("[PASS] FSM maintained MOVE_UP state through intermediate floor.");

        $display("[Time %0t ns] -> Arriving at Floor 2 (Door Blocked)", $time);
        current_floor = 2'b10;
        door_sensor = 1; 
        #8;
        if (door_open == 1 && motor_up == 0)
            $display("[PASS] FSM entered DOOR_OPEN_STATE and halted motor.");
        else
            $display("[FAIL] Failed to enter DOOR_OPEN_STATE.");

        #32;
        $display("[Time %0t ns] -> Clearing Door Sensor", $time);
        door_sensor = 0;
        #8;
        if (door_open == 0 && motor_up == 0 && motor_down == 0)
            $display("[PASS] FSM safely returned to IDLE.");

        $display("[Time %0t ns] -> Requesting Floor 0 from Floor 2", $time);
        target_floor = 2'b00;
        #8;
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