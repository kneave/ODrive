#!/usr/bin/env python3
"""
Hoverboard motor configuration script.

Applies the ODrive configuration steps from the hoverboard setup guide:
https://docs.odriverobotics.com/v/0.5.6/hoverboard.html#odrive-configuration

Connects to an ODrive and configures axis0 for a standard 6.5" hoverboard
hub motor with hall sensors, then runs calibration.

Troubleshooting:
If you get 'MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE' then try increasing the following:
'motor.config.resistance_calib_max_voltage'


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

AXIS_INDEX = 0                # Which axis to configure (0 or 1)
POLE_PAIRS = 15               # 6.5" hoverboard hub motors have 30 poles = 15 pole pairs
MOTOR_KV = 16.0               # Measured via the "drill test" - see hoverboard guide
BRAKE_RESISTANCE = 2.0        # Ohms - set to match your brake resistor
ENABLE_BRAKE_RESISTOR = True  # Set to False only if no brake resistor is connected
DC_MAX_NEGATIVE_CURRENT = -10.0    # Amps allowed back into the power supply (negative = regen to source)
REQUESTED_CURRENT_RANGE = 25.0     # Motor current sense range (requires save + reboot)
RESISTANCE_CALIB_MAX_VOLTAGE = 22.0
CALIBRATION_CURRENT = 10.0     # Amps used during motor resistance calibration (lower if getting UNBALANCED_PHASES, raise CALIB_MAX_VOLTAGE if getting PHASE_RESISTANCE_OUT_OF_RANGE)
CURRENT_CONTROL_BANDWIDTH = 100.0  # Hz - reduced for high-inductance hoverboard motors
ENCODER_BANDWIDTH = 100.0          # Hz - reduced for low-resolution hall feedback
CALIB_SCAN_DISTANCE = 150.0
VEL_LIMIT = 10.0              # turns/s


odrv0 = None

def save_config():
    global odrv0
    try:
        odrv0.save_configuration()
    except fibre.libfibre.ObjectLostError:
        pass  # Saving configuration makes the device reboot
    print("Configuration saved.")

def initial_config(axis = 0):
    global odrv0
    odrv0 = odrive.find_any()

    if axis == 0:
        axis = odrv0.axis0
    else:
        axis = odrv0.axis1

    odrv0.config.enable_brake_resistor = ENABLE_BRAKE_RESISTOR
    odrv0.config.brake_resistance = BRAKE_RESISTANCE
    odrv0.config.dc_max_negative_current = DC_MAX_NEGATIVE_CURRENT

    axis.motor.config.pole_pairs = POLE_PAIRS
    axis.motor.config.calibration_current = CALIBRATION_CURRENT
    axis.motor.config.resistance_calib_max_voltage = RESISTANCE_CALIB_MAX_VOLTAGE
    axis.motor.config.requested_current_range = 25 #Requires config save and reboot
    axis.motor.config.current_control_bandwidth = 100
    axis.motor.config.torque_constant = 8.27 / MOTOR_KV

    print(f"Calib voltage: {axis.motor.config.resistance_calib_max_voltage}")
    save_config()

def encoder_config(axis = 0):
    global odrv0
    odrv0 = odrive.find_any()

    if axis == 0:
        axis = odrv0.axis0
    else:
        axis = odrv0.axis1

    axis.encoder.config.mode = ENCODER_MODE_HALL
    axis.encoder.config.cpr = 90
    axis.encoder.config.calib_scan_distance = 150
    odrv0.config.gpio9_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio10_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio11_mode = GPIO_MODE_DIGITAL

    save_config()

def config_controller(axis = 0):
    global odrv0
    odrv0 = odrive.find_any()
    
    if axis == 0:
        axis = odrv0.axis0
    else:
        axis = odrv0.axis1

    axis.encoder.config.bandwidth = 100
    axis.controller.config.pos_gain = 1
    axis.controller.config.vel_gain = 0.02 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
    axis.controller.config.vel_integrator_gain = 0.1 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
    axis.controller.config.vel_limit = 10
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    save_config()

def config_axes():
    initial_config(0)
    time.sleep(5)
    encoder_config(0)
    time.sleep(5)
    config_controller(0)
    # time.sleep(3)
    # initial_config(1)
    # time.sleep(3)
    # encoder_config(1)
    # time.sleep(3)
    # config_controller(1)

config_axes()