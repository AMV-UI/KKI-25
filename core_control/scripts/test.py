#!/usr/bin/env python3

from pymavlink import mavutil
import time

def set_rc_channel_pwm(id, pwm=1500):
    """Set RC channel pwm value
    Args:
        id (int): Channel ID (1-8)
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if id < 1:
        print("Channel does not exist.")
        return

    # We only have 8 channels
    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]  # Default values for channels
        rc_channel_values[id - 1] = pwm  # Set the desired channel
        master.mav.rc_channels_override_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            *rc_channel_values,
        )

def set_drone_mode(pwm_value):
    """Set drone mode based on channel 8 PWM value
    Args:
        pwm_value (int): PWM value from channel 8 (1100 - 1900)
    """
    if pwm_value <= 1300:
        mode = "HOLD"
    elif 1301 <= pwm_value <= 1700:
        mode = "MANUAL"
    else:
        mode = "GUIDED"

    # Print selected mode
    print(f"Selected mode: {mode}")

    # Check if mode is available in the drone's mode mapping
    if mode not in master.mode_mapping():
        print(f"Unknown mode: {mode}")
        print("Available modes:", list(master.mode_mapping().keys()))
        return False

    # Get mode ID and send the set_mode command
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
    )
    print(f"Mode set to {mode}")
    return True

def check_and_override(channel_8_pwm):
    """Check the PWM value and override channel 3 if necessary
    Args:
        channel_8_pwm (int): PWM value from channel 8
    """
    if channel_8_pwm > 1700:
        # Autonomous mode: force channel 3 PWM to 1300
        print("Autonomous mode detected, overriding channel 3 to PWM 1300")
        set_rc_channel_pwm(3, 1300)
    elif 1301 <= channel_8_pwm <= 1700:
        # Manual mode: no override, allow manual control
        print("Manual mode detected, override disabled")
        set_rc_channel_pwm(3, 65535)  # Disable override by setting to neutral (65535)

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=115200)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received!")

# Arm the drone
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # 1 to arm, 0 to disarm
    0, 0, 0, 0, 0, 0
)
print("Waiting to arm...")
master.motors_armed_wait()
print("Armed successfully!")

# Main loop
while True:
    # Get the latest RC channels' values
    message = master.recv_match(type='RC_CHANNELS', blocking=True)
    if not message:
        continue

    # Extract channel 8 PWM value
    channel_8_pwm = message.chan8_raw
    print(f"Channel 8 PWM value: {channel_8_pwm}")

    # Set mode based on channel 8 PWM
    if set_drone_mode(channel_8_pwm):
        print("Mode change successful")

    # Check and apply channel 3 override if necessary
    check_and_override(channel_8_pwm)

    # Sleep for a bit before the next iteration
    # time.sleep(0.5)