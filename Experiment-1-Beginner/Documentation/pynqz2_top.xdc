## pynqz2_top.xdc
## Pin constraints for design_1_wrapper / top_module on PYNQ-Z2 (xc7z020clg400-1)
## Reference: PYNQ-Z2 Master XDC (v1.0)

## ---- Clock: MANAGED INTERNALLY BY ZYNQ PS (FCLK_CLK0) ----
## Do not un-comment external H16 clock constraints when using the Zynq Processing System.
## set_property -dict {PACKAGE_PIN H16 IOSTANDARD LVCMOS33} [get_ports clk]
## create_clock -period 8.000 -name sys_clk_pin -waveform {0 4} [get_ports clk]

## ---- Reset: BTN0 ----
set_property -dict {PACKAGE_PIN D19 IOSTANDARD LVCMOS33} [get_ports btn_rst]

## ---- Keypad rows (outputs) -- Pmod A, JA1_P..JA4_P used as ROW0..ROW3 ----
set_property -dict {PACKAGE_PIN Y18 IOSTANDARD LVCMOS33} [get_ports {kp_row[0]}]
set_property -dict {PACKAGE_PIN Y16 IOSTANDARD LVCMOS33} [get_ports {kp_row[1]}]
set_property -dict {PACKAGE_PIN U18 IOSTANDARD LVCMOS33} [get_ports {kp_row[2]}]
set_property -dict {PACKAGE_PIN W18 IOSTANDARD LVCMOS33} [get_ports {kp_row[3]}]

## ---- Keypad cols (inputs, pull-ups enabled) -- Pmod A, JA1_N..JA4_N ----
set_property -dict {PACKAGE_PIN Y19 IOSTANDARD LVCMOS33 PULLUP true} [get_ports {kp_col[0]}]
set_property -dict {PACKAGE_PIN Y17 IOSTANDARD LVCMOS33 PULLUP true} [get_ports {kp_col[1]}]
set_property -dict {PACKAGE_PIN U19 IOSTANDARD LVCMOS33 PULLUP true} [get_ports {kp_col[2]}]
set_property -dict {PACKAGE_PIN W19 IOSTANDARD LVCMOS33 PULLUP true} [get_ports {kp_col[3]}]

## ---- Servo PWM -- Pmod B, JB1_P ----
set_property -dict {PACKAGE_PIN W14 IOSTANDARD LVCMOS33} [get_ports servo_pwm]

## ---- Tri-color LED LD4 ----
set_property -dict {PACKAGE_PIN N15 IOSTANDARD LVCMOS33} [get_ports led_r]
set_property -dict {PACKAGE_PIN G17 IOSTANDARD LVCMOS33} [get_ports led_g]
set_property -dict {PACKAGE_PIN L15 IOSTANDARD LVCMOS33} [get_ports led_b]