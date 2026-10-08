module elevator_fsm #(
    parameter DOOR_LIMIT = 250000000 // 2 seconds at 125 MHz
)(
    input clk,
    input reset,
    input [1:0] target_floor,   
    input [1:0] current_floor,  
    input door_sensor,          
    output reg motor_up,
    output reg motor_down,
    output reg door_open
);

    localparam IDLE            = 2'b00;
    localparam MOVE_UP         = 2'b01;
    localparam MOVE_DOWN       = 2'b10;
    localparam DOOR_OPEN_STATE = 2'b11;

    reg [1:0] state, next_state;
    reg [27:0] door_timer;

    // Sequential State Register & Timer
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state      <= IDLE;
            door_timer <= 0;
        end else begin
            state <= next_state;
            
            if (state == DOOR_OPEN_STATE)
                door_timer <= door_timer + 1'b1;
            else
                door_timer <= 0;
        end
    end

    // Combinational Next-State and Output Logic
    always @(*) begin
        motor_up   = 1'b0; 
        motor_down = 1'b0; 
        door_open  = 1'b0;
        next_state = state;

        case (state)
            IDLE: begin
                if (target_floor > current_floor)
                    next_state = MOVE_UP;
                else if (target_floor < current_floor)
                    next_state = MOVE_DOWN;
                else
                    next_state = IDLE;
            end

            MOVE_UP: begin
                motor_up = 1'b1;
                // Transition to door open only when target is reached
                if (current_floor >= target_floor)
                    next_state = DOOR_OPEN_STATE;
            end

            MOVE_DOWN: begin
                motor_down = 1'b1;
                // Transition to door open only when target is reached
                if (current_floor <= target_floor)
                    next_state = DOOR_OPEN_STATE;
            end

            DOOR_OPEN_STATE: begin
                door_open = 1'b1;
                // Hold door open until timer completes AND door sensor (BTN3) is clear
                if ((door_timer >= DOOR_LIMIT) && !door_sensor) begin
                    next_state = IDLE;
                end
            end

            default: next_state = IDLE;
        endcase
    end
endmodule