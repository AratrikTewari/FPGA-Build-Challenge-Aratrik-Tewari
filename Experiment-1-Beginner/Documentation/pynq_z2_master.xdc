## Clock Signal (125 MHz system clock on PYNQ-Z2)
set_property -dict { PACKAGE_PIN H16   IOSTANDARD LVCMOS33 } [get_ports { clk }];
create_clock -add -name sys_clk_pin -period 8.000 -waveform {0 4} [get_ports { clk }];

## Reset Button (Mapped to on-board BTN0)
set_property -dict { PACKAGE_PIN D19   IOSTANDARD LVCMOS33 } [get_ports { rst_btn }];

## Target Floor Switches (Mapped to on-board SW0 and SW1)
set_property -dict { PACKAGE_PIN M20   IOSTANDARD LVCMOS33 } [get_ports { target_sw[0] }];
set_property -dict { PACKAGE_PIN M19   IOSTANDARD LVCMOS33 } [get_ports { target_sw[1] }];

## Current Floor Buttons (Mapped to on-board BTN1 and BTN2)
set_property -dict { PACKAGE_PIN D20   IOSTANDARD LVCMOS33 } [get_ports { current_fl_sw[0] }]; # BTN1
set_property -dict { PACKAGE_PIN L20   IOSTANDARD LVCMOS33 } [get_ports { current_fl_sw[1] }]; # BTN2

## Door Sensor Button (Mapped to on-board BTN3)
set_property -dict { PACKAGE_PIN L19   IOSTANDARD LVCMOS33 } [get_ports { door_sens_btn }];   # BTN3

## Motor & Door Outputs (Mapped to on-board RGB LEDs LD4 and LD5)
set_property -dict { PACKAGE_PIN N15   IOSTANDARD LVCMOS33 } [get_ports { motor_up_led }];    # LD4 Red
set_property -dict { PACKAGE_PIN L15   IOSTANDARD LVCMOS33 } [get_ports { motor_down_led }];  # LD4 Blue
set_property -dict { PACKAGE_PIN L14   IOSTANDARD LVCMOS33 } [get_ports { door_open_led }];   # LD5 Green