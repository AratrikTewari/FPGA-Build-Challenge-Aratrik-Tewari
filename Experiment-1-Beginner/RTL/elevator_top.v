module elevator_top #(
    parameter DB_LIMIT = 1250000,     // 10 ms at 125 MHz
    parameter DOOR_LIMIT = 250000000  // 2 sec at 125 MHz
)(
    input clk,
    input rst_btn,               
    input [1:0] target_sw,       
    input [1:0] current_fl_sw,   // BTN1 (bit 0), BTN2 (bit 1)
    input door_sens_btn,         // BTN3
    output motor_up_led,         // LD4 Red
    output motor_down_led,       // LD4 Blue
    output door_open_led         // LD5 Green
);

    wire clean_rst;
    wire clean_door;
    wire [1:0] clean_btn_floor;
    reg [1:0] latched_current_floor = 2'b00;

    // Debounce reset button (BTN0)
    debouncer #( .DEBOUNCE_LIMIT(DB_LIMIT) ) rst_db (
        .clk(clk), .rst(1'b0), .btn_in(rst_btn), .btn_out(clean_rst)
    );

    // Debounce door sensor button (BTN3)
    debouncer #( .DEBOUNCE_LIMIT(DB_LIMIT) ) door_db (
        .clk(clk), .rst(clean_rst), .btn_in(door_sens_btn), .btn_out(clean_door)
    );

    // Debounce Floor Bit 0 (BTN1)
    debouncer #( .DEBOUNCE_LIMIT(DB_LIMIT) ) curr_fl_db0 (
        .clk(clk), .rst(clean_rst), .btn_in(current_fl_sw[0]), .btn_out(clean_btn_floor[0])
    );

    // Debounce Floor Bit 1 (BTN2)
    debouncer #( .DEBOUNCE_LIMIT(DB_LIMIT) ) curr_fl_db1 (
        .clk(clk), .rst(clean_rst), .btn_in(current_fl_sw[1]), .btn_out(clean_btn_floor[1])
    );

    // Latch floor arrival: Updates cabin position on button press and holds it
    always @(posedge clk or posedge clean_rst) begin
        if (clean_rst) begin
            latched_current_floor <= 2'b00; // Floor 0 on reset
        end else begin
            case (clean_btn_floor)
                2'b01: latched_current_floor <= 2'b01; // BTN1 pressed -> Floor 1
                2'b10: latched_current_floor <= 2'b10; // BTN2 pressed -> Floor 2
                2'b11: latched_current_floor <= 2'b11; // Both pressed -> Floor 3
                default: latched_current_floor <= latched_current_floor; // Hold state
            endcase
        end
    end

    // FSM instance
    elevator_fsm #( .DOOR_LIMIT(DOOR_LIMIT) ) u_fsm (
        .clk(clk),
        .reset(clean_rst),        
        .target_floor(target_sw),
        .current_floor(latched_current_floor), 
        .door_sensor(clean_door), 
        .motor_up(motor_up_led),
        .motor_down(motor_down_led),
        .door_open(door_open_led)
    );
endmodule