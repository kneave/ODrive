#!/usr/bin/env python3
"""
Hoverboard motor configuration script.

Applies the ODrive configuration steps from the hoverboard setup guide:
https://docs.odriverobotics.com/v/0.5.6/hoverboard.html#odrive-configuration

Connects to an ODrive and configures the axes listed in AXES_TO_CONFIG for
standard 6.5" hoverboard hub motors with hall sensors, then runs calibration.

Troubleshooting:
If you get 'MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE' then try increasing the following:
'motor.config.resistance_calib_max_voltage'

Manual steps:

odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION


Edit the constants below to match your motor and setup.
"""

import sys
import time
import math
import threading

import odrive
import fibre
from odrive.enums import (
    ENCODER_MODE_HALL,
    CONTROL_MODE_VELOCITY_CONTROL,
    AXIS_ERROR_NONE,
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    GPIO_MODE_DIGITAL,
)
from odrive.utils import dump_errors, get_serial_number_str_sync


# ============================================================================
# Configuration constants - edit these to match your motor and setup
# ============================================================================

# Which axes to configure, in order (0-based)
AXES_TO_CONFIG = [0, 1]
POLE_PAIRS = 15               # 6.5" hoverboard hub motors have 30 poles = 15 pole pairs
MOTOR_KV = 16.0               # Measured via the "drill test" - see hoverboard guide
BRAKE_RESISTANCE = 2.0        # Ohms - set to match your brake resistor
ENABLE_BRAKE_RESISTOR = True  # Set to False only if no brake resistor is connected
DC_MAX_NEGATIVE_CURRENT = -10.0    # Amps allowed back into the power supply (negative = regen to source)
REQUESTED_CURRENT_RANGE = 25.0     # Motor current sense range (requires save + reboot)
RESISTANCE_CALIB_MAX_VOLTAGE = 4.0    # Volts - DO NOT EXCEED 4V! Values above this can permanently damage the DRV8301 gate driver chips
CALIBRATION_CURRENT = 10.0     # Amps used during motor resistance calibration (lower if getting UNBALANCED_PHASES, raise CALIB_MAX_VOLTAGE if getting PHASE_RESISTANCE_OUT_OF_RANGE)
CURRENT_CONTROL_BANDWIDTH = 100.0  # Hz - reduced for high-inductance hoverboard motors
ENCODER_BANDWIDTH = 100.0          # Hz - reduced for low-resolution hall feedback
CALIB_SCAN_DISTANCE = 150.0
VEL_LIMIT = 10.0              # turns/s

# Which axes to configure, in order (0-based)
AXES_TO_CONFIG = [0, 1]


class HoverboardConfigurator:
    """Configures ODrive axes for hoverboard hub motors with hall sensors."""

    def __init__(self, odrv=None):
        if odrv is not None:
            self.odrv = odrv
        else:
            self.odrv = odrive.find_any()

        self.serial_number = format(self.odrv.serial_number, 'X')
        print("Connected to " + self.serial_number)

    def reconnect(self):
        """Re-connect to an ODrive (e.g. after save + reboot).
        Retries with a delay until the requested device is available again."""
        while True:
            self.odrv = odrive.find_any(serial_number=self.serial_number)
            if self.odrv is not None:
                break
            print("ODrive not found yet, retrying in 1 seconds...")
            time.sleep(1)
        print("Reconnected.")

    def get_axis(self, index):
        """Return the axis object for the given index (0-based).
        Raises ValueError if the index is out of range for this ODrive."""
        try:
            return getattr(self.odrv, f"axis{index}")
        except AttributeError:
            raise ValueError(f"ODrive {self.serial_number} has no axis{index}")

    def get_axis_index(self, axis):
        """Return the index (0-based) of the given axis object on this ODrive.
        Raises ValueError if axis doesn't belong to this ODrive."""
        for i in range(2):
            name = f"axis{i}"
            if hasattr(self.odrv, name) and getattr(self.odrv, name) is axis:
                return i
        raise ValueError(f"axis not found on ODrive {self.serial_number}")

    def save_config(self):
        try:
            self.odrv.save_configuration()
        except fibre.libfibre.ObjectLostError:
            pass  # Saving configuration makes the device reboot
        print("Configuration saved.")
        self.reconnect()



    def initial_config(self, index):
        """Perform initial configuration for the axis with the given index.
        Returns True on success, False on failure."""
        try:
            axis_obj = self.get_axis(index)
            self.odrv.config.enable_brake_resistor = ENABLE_BRAKE_RESISTOR
            self.odrv.config.brake_resistance = BRAKE_RESISTANCE
            self.odrv.config.dc_max_negative_current = DC_MAX_NEGATIVE_CURRENT
            axis_obj.motor.config.pole_pairs = POLE_PAIRS
            axis_obj.motor.config.calibration_current = CALIBRATION_CURRENT
            axis_obj.motor.config.resistance_calib_max_voltage = RESISTANCE_CALIB_MAX_VOLTAGE
            axis_obj.motor.config.requested_current_range = 25
            axis_obj.motor.config.current_control_bandwidth = 100
            axis_obj.motor.config.torque_constant = 8.27 / MOTOR_KV
            print(f"Calib voltage: {axis_obj.motor.config.resistance_calib_max_voltage}")
            self.save_config()
            print(f"Initial config for {index} complete")
            return True
        except Exception as e:
            print(f"initial_config failed for axis {index}: {e}")
            return False

    def encoder_config(self, index):
        """Configure the encoder for the axis with the given index.
        Returns True on success, False on failure."""
        try:
            print(f"Configuring encoder for axis {index}")
            axis_obj = self.get_axis(index)
            axis_obj.encoder.config.mode = ENCODER_MODE_HALL
            axis_obj.encoder.config.cpr = 90
            axis_obj.encoder.config.calib_scan_distance = 150
            if index == 0:
                self.odrv.config.gpio9_mode = GPIO_MODE_DIGITAL
                self.odrv.config.gpio10_mode = GPIO_MODE_DIGITAL
                self.odrv.config.gpio11_mode = GPIO_MODE_DIGITAL
            else:
                self.odrv.config.gpio12_mode = GPIO_MODE_DIGITAL
                self.odrv.config.gpio13_mode = GPIO_MODE_DIGITAL
                self.odrv.config.gpio14_mode = GPIO_MODE_DIGITAL
            self.save_config()
            return True
        except Exception as e:
            print(f"encoder_config failed for axis {index}: {e}")
            return False

    def config_controller(self, index):
        """Configure the controller for the axis with the given index.
        Returns True on success, False if any step fails."""
        try:
            print(f"Configuring controller for axis {index}")
            axis_obj = self.get_axis(index)
            axis_obj.encoder.config.bandwidth = 100
            axis_obj.controller.config.pos_gain = 1
            axis_obj.controller.config.vel_gain = 0.02 * axis_obj.motor.config.torque_constant * axis_obj.encoder.config.cpr
            axis_obj.controller.config.vel_integrator_gain = 0.1 * axis_obj.motor.config.torque_constant * axis_obj.encoder.config.cpr
            axis_obj.controller.config.vel_limit = 10
            axis_obj.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.save_config()
            return True
        except Exception as e:
            print(f"config_controller failed for axis {index}: {e}")
            return False

    def wait_for_state(self, index, axis, target_state, timeout=10):
        """Poll the ODrive until the given axis reaches target_state or timeout (seconds).
        Returns True if the target state is reached, False on timeout.
        """
        start = time.time()
        while True:
            if axis.current_state == target_state:
                return True
            if time.time() - start > timeout:
                print(f"Timeout waiting for axis {index} to reach state {target_state}")
                return False
            time.sleep(0.1)

    def calibrate(self, index):
        """Run motor, encoder hall polarity and encoder offset calibration
        for the axis with the given index.
        Returns True on success, False on failure."""
        try:
            axis = self.get_axis(index)

            print(f"Calibrating motor for axis {index}")
            axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            # Wait for calibration to start, then for it to finish (back to idle)
            if not self.wait_for_state(index, axis, AXIS_STATE_MOTOR_CALIBRATION, timeout=5):
                return False
            if not self.wait_for_state(index, axis, AXIS_STATE_IDLE, timeout=60):
                return False
            if axis.error != AXIS_ERROR_NONE or axis.motor.error != 0:
                print(f"Motor calibration failed for axis {index}:")
                dump_errors(self.odrv, True)
                return False
            axis.motor.config.pre_calibrated = True

            print(f"Calibrating encoder hall polarity for axis {index}")
            axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
            if not self.wait_for_state(index, axis, AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION, timeout=5):
                return False
            if not self.wait_for_state(index, axis, AXIS_STATE_IDLE, timeout=60):
                return False
            if axis.error != AXIS_ERROR_NONE or axis.encoder.error != 0:
                print(f"Encoder hall polarity calibration failed for axis {index}:")
                dump_errors(self.odrv, True)
                return False

            print(f"Calibrating encoder offset for axis {index}")
            axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            if not self.wait_for_state(index, axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, timeout=5):
                return False  
            print("offset mode set")                     
            if not self.wait_for_state(index, axis, AXIS_STATE_IDLE, timeout=60):
                return False
            
            print("idle")                     
                        
            if axis.error != AXIS_ERROR_NONE or axis.encoder.error != 0:
                print(f"Encoder offset calibration failed for axis {index}:")
                dump_errors(self.odrv, True)
                return False

            axis.encoder.config.pre_calibrated = True
            self.save_config()
            return True
        except Exception as e:
            print(f"calibrate failed for axis {index}: {e}")
            return False

    def config_axis(self, index):
        """Configure the axis with the given index.
        If a step fails the remaining steps for that axis are skipped."""
        print(f"Configuring axis {index}")
        axis = self.get_axis(index)
        if not self.initial_config(index):
            print(f"initial_config failed for axis {index} – skipping remaining steps for this axis.")
        else:
            axis = self.get_axis(index)                
            self.wait_for_state(index, axis, AXIS_STATE_IDLE)
            if not self.encoder_config(index):
                print(f"encoder_config failed for axis {index} – skipping remaining steps for this axis.")
            else:            
                axis = self.get_axis(index)      
                self.wait_for_state(index, axis, AXIS_STATE_IDLE)
                if not self.config_controller(index):
                    print(f"config_controller failed for axis {index} – skipping remaining steps for this axis.")
                else:                
                    axis = self.get_axis(index) 
                    self.wait_for_state(index, axis, AXIS_STATE_IDLE)
                    return True
        return False

    def config_axes(self):
        """Configure the axes listed in AXES_TO_CONFIG, sequentially."""
        for index in AXES_TO_CONFIG:
            print(f"Configuring axis {index}...")
            self.config_axis(index)


def main():
    configurator = HoverboardConfigurator()
    configured = configurator.config_axis(1)
    if configured:
        configurator.calibrate(1)


if __name__ == "__main__":
    main()