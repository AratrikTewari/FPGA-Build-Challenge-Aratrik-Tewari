# ============================================================================
# Physical Constraints File: pynq_z2_master.xdc
# Project:       Multi-Floor FPGA Elevator Controller
# Target Board:  PYNQ-Z2 (XC7Z020-1CLG400C)
# Tool Version:  AMD Xilinx Vivado 2025.2
#
# System Integration Overview:
#   This file maps top-level digital I/O ports from 'elevator_top.v' to the
#   physical package pins, logic levels (LVCMOS33), and timing specifications
#   of the PYNQ-Z2 evaluation platform.
#
# Hardware Mapping Summary:
#   - Clock:             125 MHz system oscillator via Bank 35 (Pin H16)
#   - Global Reset:      Push button BTN0 (Pin D19)
#   - Floor Targets:     Slide switches SW0 (Pin M20) and SW1 (Pin M19)
#   - Floor Feedback:    Momentary buttons BTN1 (Pin D20) and BTN2 (Pin L20)
#   - Door Safety:       Optical beam / obstruction sensor on BTN3 (Pin L19)
#   - Actuator Displays: Dual tricolor LEDs LD4 (Motor Up/Down) and LD5 (Door)
# ============================================================================

# ----------------------------------------------------------------------------
# 1. Primary Timing Constraints
# ----------------------------------------------------------------------------
# Connects to the on-board 125 MHz clock oscillator.
# Period = 8.000 ns, 50% duty cycle (0.0 ns rise, 4.0 ns fall).
set_property -dict { PACKAGE_PIN H16   IOSTANDARD LVCMOS33 } [get_ports { clk }];
create_clock -add -name sys_clk_pin -period 8.000 -waveform {0 4} [get_ports { clk }];

# ----------------------------------------------------------------------------
# 2. Master Asynchronous Reset Control
# ----------------------------------------------------------------------------
# Push button BTN0 provides an active-high reset to initialize internal registers
# and force the cabin position latch to Ground Floor (Floor 0).
set_property -dict { PACKAGE_PIN D19   IOSTANDARD LVCMOS33 } [get_ports { rst_btn }];

# ----------------------------------------------------------------------------
# 3. User Target Destination Inputs (Slide Switches)
# ----------------------------------------------------------------------------
# SW0 and SW1 represent the 2-bit binary requested destination landing [0..3]:
#   2'b00 = Floor 0, 2'b01 = Floor 1, 2'b10 = Floor 2, 2'b11 = Floor 3.
set_property -dict { PACKAGE_PIN M20   IOSTANDARD LVCMOS33 } [get_ports { target_sw[0] }];
set_property -dict { PACKAGE_PIN M19   IOSTANDARD LVCMOS33 } [get_ports { target_sw[1] }];

# ----------------------------------------------------------------------------
# 4. Cabin Shaft Arrival Feedback (Push Buttons)
# ----------------------------------------------------------------------------
# Momentary push buttons simulate landing position limit sensors in the shaft.
# Pulses are filtered through 'debouncer.v' and held in 'latched_current_floor':
#   BTN1 (Pin D20) -> Bit 0 arrival trigger
#   BTN2 (Pin L20) -> Bit 1 arrival trigger
set_property -dict { PACKAGE_PIN D20   IOSTANDARD LVCMOS33 } [get_ports { current_fl_sw[0] }]; # BTN1
set_property -dict { PACKAGE_PIN L20   IOSTANDARD LVCMOS33 } [get_ports { current_fl_sw[1] }]; # BTN2

# ----------------------------------------------------------------------------
# 5. Door Safety Curtain Interlock (Push Button)
# ----------------------------------------------------------------------------
# BTN3 emulates an active-high optical barrier obstruction sensor.
# While held high, it prevents the door timer from timing out, keeping the cabin
# safely stationary in DOOR_OPEN_STATE.
set_property -dict { PACKAGE_PIN L19   IOSTANDARD LVCMOS33 } [get_ports { door_sens_btn }];   # BTN3

# ----------------------------------------------------------------------------
# 6. Actuator Status Displays (On-board RGB LEDs LD4 and LD5)
# ----------------------------------------------------------------------------
# LD4 indicates active hoist traction motor drive:
#   - LD4 Red  (Pin N15): Hoist UP active
#   - LD4 Blue (Pin L15): Hoist DOWN active
# LD5 indicates passenger door actuator state:
#   - LD5 Green (Pin L14): Door open dwell state
set_property -dict { PACKAGE_PIN N15   IOSTANDARD LVCMOS33 } [get_ports { motor_up_led }];    # LD4 Red
set_property -dict { PACKAGE_PIN L15   IOSTANDARD LVCMOS33 } [get_ports { motor_down_led }];  # LD4 Blue
set_property -dict { PACKAGE_PIN L14   IOSTANDARD LVCMOS33 } [get_ports { door_open_led }];   # LD5 Green
