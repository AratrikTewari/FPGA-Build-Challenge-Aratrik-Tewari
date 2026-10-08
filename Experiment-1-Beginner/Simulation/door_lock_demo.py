from pynq import Overlay
import time

class DoorLockDemo:
    def __init__(self, bitfile_path="door_lock.bit"):
        self.overlay = Overlay(bitfile_path)
        
        # axi_gpio_0 must be configured as an INPUT in Vivado (reads ps_status_out)
        self.status_gpio = self.overlay.axi_gpio_0
        
        # axi_gpio_1 must be configured as an OUTPUT in Vivado (writes ps_key_inject)
        self.key_gpio = self.overlay.axi_gpio_1

    def read_hardware_leds(self):
        """Reads the physical LED status directly from the FPGA fabric."""
        val = self.status_gpio.channel1.read()
        alarm = (val >> 3) & 1
        rgb = val & 0b111
        
        colors = {
            0b100: "🔴 RED (Locked / Idle)",
            0b001: "🔵 BLUE (Receiving Input)",
            0b010: "🟢 GREEN (Unlocked)",
            0b111: "⚪ WHITE (Alarm Latched)"
        }
        
        current_color = colors.get(rgb, "UNKNOWN")
        print(f"Hardware LED State: {current_color} | Alarm Active: {bool(alarm)}")

    def inject_keystroke(self, digit_hex):
        """Synthesizes a hardware keystroke from Python."""
        print(f"Typing digit: {digit_hex}")
        
        # Assert key_valid (bit 4) and set key_data (bits 3:0)
        payload = (1 << 4) | (digit_hex & 0xF)
        self.key_gpio.channel1.write(payload)
        
        # Drop key_valid to create a 1-cycle pulse equivalent
        self.key_gpio.channel1.write(0)
        time.sleep(0.2) 

    def test_password(self, pin_list):
        print("\n--- Starting Software Password Demo ---")
        self.read_hardware_leds()
        
        for digit in pin_list:
            self.inject_keystroke(digit)
            self.read_hardware_leds()
            time.sleep(0.5)