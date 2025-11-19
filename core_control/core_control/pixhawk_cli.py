#!/usr/bin/env python3
"""
Pixhawk Digital Channel Controller - Interactive CLI
Independent script for real-time Pixhawk control via MAVLink
"""

from pymavlink import mavutil
from time import sleep
import sys
import traceback


class PixhawkCLI:
    def __init__(self, connection_string="/dev/ttyUSB0", baud=57600):
        """
        Initialize Pixhawk CLI controller.
        
        Args:
            connection_string (str): Serial port or connection string
            baud (int): Baud rate for serial connection
        """
        print(f"Connecting to Pixhawk on {connection_string} @ {baud} baud...")
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        
        # Digital channel control state
        self.armed = False
        self.manual_channels = [1500] * 18  # 18 channels, all at neutral (1500)
        self.channel_override_enabled = False
        
        # Channel value limits
        self.PWM_MIN = 1000
        self.PWM_MAX = 2000
        self.PWM_NEUTRAL = 1500
        
        self._wait_for_heartbeat()
    
    def _wait_for_heartbeat(self):
        """Wait for heartbeat from Pixhawk to ensure connection"""
        print("Waiting for heartbeat from Pixhawk...")
        self.master.wait_heartbeat()
        print(f"✓ Heartbeat received from system {self.master.target_system}, component {self.master.target_component}")
        print()
    
    def get_pixhawk_status(self):
        """
        Get comprehensive Pixhawk status information.
        
        Returns:
            dict: Status information including mode, armed state, GPS, battery, etc.
        """
        status = {}
        
        # Get heartbeat
        heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if heartbeat:
            status['system_status'] = heartbeat.system_status
            status['base_mode'] = heartbeat.base_mode
            status['custom_mode'] = heartbeat.custom_mode
            status['armed'] = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            status['mode'] = self.master.mode_mapping().get(heartbeat.custom_mode, 'UNKNOWN')
        
        # Get GPS status
        gps = self.master.recv_match(type='GPS_RAW_INT', blocking=False, timeout=1)
        if gps:
            status['gps_fix'] = gps.fix_type
            status['gps_satellites'] = gps.satellites_visible
            status['gps_lat'] = gps.lat / 1e7
            status['gps_lon'] = gps.lon / 1e7
            status['gps_alt'] = gps.alt / 1000.0
        
        # Get battery status
        battery = self.master.recv_match(type='BATTERY_STATUS', blocking=False, timeout=1)
        if battery:
            status['battery_voltage'] = battery.voltages[0] / 1000.0 if battery.voltages else 0
            status['battery_current'] = battery.current_battery / 100.0 if battery.current_battery > 0 else 0
            status['battery_remaining'] = battery.battery_remaining
        
        # Get RC channels
        rc = self.master.recv_match(type='RC_CHANNELS', blocking=False, timeout=1)
        if rc:
            status['rc_channels'] = [
                rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
                rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw
            ]
        
        return status
    
    def print_status(self):
        """Print formatted Pixhawk status"""
        status = self.get_pixhawk_status()
        
        print("=" * 60)
        print("PIXHAWK STATUS")
        print("=" * 60)
        
        if 'armed' in status:
            armed_str = "✓ ARMED" if status['armed'] else "✗ DISARMED"
            print(f"State: {armed_str}")
            print(f"Mode: {status.get('mode', 'UNKNOWN')}")
        
        if 'gps_fix' in status:
            fix_types = {0: 'NO FIX', 1: 'NO FIX', 2: '2D FIX', 3: '3D FIX', 4: 'DGPS', 5: 'RTK FLOAT', 6: 'RTK FIXED'}
            fix_str = fix_types.get(status['gps_fix'], 'UNKNOWN')
            print(f"\nGPS Fix: {fix_str} | Satellites: {status['gps_satellites']}")
            print(f"Position: {status['gps_lat']:.6f}, {status['gps_lon']:.6f}")
            print(f"Altitude: {status['gps_alt']:.1f}m")
        
        if 'battery_voltage' in status:
            print(f"\nBattery: {status['battery_voltage']:.2f}V | Current: {status['battery_current']:.2f}A | Remaining: {status['battery_remaining']}%")
        
        if 'rc_channels' in status:
            print("\nRC Channels:")
            for i, v in enumerate(status['rc_channels'], 1):
                print(f"  CH{i}: {v}", end="")
                if i % 4 == 0:
                    print()
            print()
        
        print("=" * 60)
    
    def arm_pixhawk(self, force=False):
        """
        Arm the Pixhawk.
        
        Args:
            force (bool): Force arming even if pre-arm checks fail
        
        Returns:
            bool: True if arming command sent successfully
        """
        print("Attempting to arm Pixhawk...")
        
        # Set mode to GUIDED first (usually required for arming)
        self.set_mode('GUIDED')
        sleep(1)
        
        # Send arm command
        arm_param = 1 if not force else 21196  # Magic number for force arm
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_param,  # 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0
        )
        
        # Wait for acknowledgment
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.armed = True
                print("✓ Pixhawk armed successfully")
                return True
            else:
                print(f"✗ Arming failed with result: {ack.result}")
                return False
        else:
            print("⚠ No acknowledgment received for arm command")
            return False
    
    def disarm_pixhawk(self, force=False):
        """
        Disarm the Pixhawk.
        
        Args:
            force (bool): Force disarm
        
        Returns:
            bool: True if disarming command sent successfully
        """
        print("Attempting to disarm Pixhawk...")
        
        disarm_param = 0 if not force else 21196  # Magic number for force disarm
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            disarm_param,  # 0 to disarm
            0, 0, 0, 0, 0, 0
        )
        
        # Wait for acknowledgment
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.armed = False
                print("✓ Pixhawk disarmed successfully")
                return True
            else:
                print(f"✗ Disarming failed with result: {ack.result}")
                return False
        else:
            print("⚠ No acknowledgment received for disarm command")
            return False
    
    def set_mode(self, mode_name):
        """
        Set Pixhawk flight mode.
        
        Args:
            mode_name (str): Mode name (e.g., 'MANUAL', 'GUIDED', 'AUTO', 'HOLD', 'STABILIZE')
        
        Returns:
            bool: True if mode set successfully
        """
        # Get mode mapping
        mode_mapping = self.master.mode_mapping()
        
        # Reverse lookup to get mode ID from name
        mode_id = None
        for id, name in mode_mapping.items():
            if name == mode_name.upper():
                mode_id = id
                break
        
        if mode_id is None:
            print(f"✗ Unknown mode: {mode_name}")
            print(f"Available modes: {list(mode_mapping.values())}")
            return False
        
        print(f"Setting mode to {mode_name}...")
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        # Wait for acknowledgment
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"✓ Mode set to {mode_name}")
            return True
        else:
            print(f"✗ Failed to set mode to {mode_name}")
            return False
    
    def set_channel(self, channel_num, pwm_value):
        """
        Set a specific RC channel to a PWM value.
        
        Args:
            channel_num (int): Channel number (1-18)
            pwm_value (int): PWM value (1000-2000, typically 1500 is neutral)
        
        Returns:
            bool: True if value is valid and set
        """
        if not (1 <= channel_num <= 18):
            print(f"✗ Invalid channel number: {channel_num}. Must be 1-18")
            return False
        
        if not (self.PWM_MIN <= pwm_value <= self.PWM_MAX):
            print(f"✗ Invalid PWM value: {pwm_value}. Must be {self.PWM_MIN}-{self.PWM_MAX}")
            return False
        
        self.manual_channels[channel_num - 1] = pwm_value
        print(f"✓ Channel {channel_num} set to {pwm_value}")
        return True
    
    def set_channels(self, channel_dict):
        """
        Set multiple channels at once.
        
        Args:
            channel_dict (dict): Dictionary of {channel_num: pwm_value}
        
        Example:
            set_channels({1: 1500, 2: 1600, 3: 1400})
        """
        for channel_num, pwm_value in channel_dict.items():
            self.set_channel(channel_num, pwm_value)
    
    def reset_all_channels(self):
        """Reset all channels to neutral (1500)"""
        self.manual_channels = [self.PWM_NEUTRAL] * 18
        print("✓ All channels reset to neutral (1500)")
    
    def send_manual_channels(self):
        """
        Send the manually set channel values to Pixhawk.
        Only sends the first 8 channels via RC override.
        """
        if not self.channel_override_enabled:
            print("⚠ Channel override not enabled. Call 'enable' first.")
            return
        
        # Prepare RC channel values (65535 = ignore/release)
        rc_channel_values = [65535] * 8
        for idx in range(8):
            rc_channel_values[idx] = self.manual_channels[idx]
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values,
        )
        
        channels_str = ', '.join([f"CH{i+1}:{v}" for i, v in enumerate(self.manual_channels[:8])])
        print(f"✓ Sent: {channels_str}")
    
    def enable_channel_override(self):
        """Enable manual channel override mode"""
        self.channel_override_enabled = True
        print("✓ Channel override ENABLED")
        print("  You can now set and send manual channel values")
    
    def disable_channel_override(self):
        """Disable manual channel override mode and release control"""
        self.channel_override_enabled = False
        # Send release command (all channels to 65535)
        rc_channel_values = [65535] * 8
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values,
        )
        print("✓ Channel override DISABLED")
        print("  Control released to RC transmitter")
    
    def get_channel_value(self, channel_num):
        """
        Get the current manual channel value.
        
        Args:
            channel_num (int): Channel number (1-18)
        
        Returns:
            int: PWM value or None if invalid channel
        """
        if not (1 <= channel_num <= 18):
            return None
        return self.manual_channels[channel_num - 1]
    
    def get_all_channels(self):
        """Get all manual channel values"""
        return self.manual_channels.copy()
    
    def print_help(self):
        """Print help information"""
        print("\n" + "="*60)
        print("AVAILABLE COMMANDS")
        print("="*60)
        print("\n📊 Status & Information:")
        print("  status / s          - Show Pixhawk status (GPS, battery, mode)")
        print("  help / h            - Show this help message")
        print("\n🔧 Arming & Mode:")
        print("  arm [force]         - Arm Pixhawk (optional: force)")
        print("  disarm [force]      - Disarm Pixhawk (optional: force)")
        print("  mode <name>         - Set flight mode (MANUAL, GUIDED, AUTO, HOLD, etc.)")
        print("\n📡 Channel Override:")
        print("  enable              - Enable manual channel override")
        print("  disable             - Disable manual channel override")
        print("\n🎛️  Channel Control:")
        print("  set <ch> <pwm>      - Set channel to PWM value")
        print("                        Example: set 1 1600")
        print("  setmulti <ch:pwm>   - Set multiple channels")
        print("                        Example: setmulti 1:1500 2:1600 3:1400")
        print("  send                - Send manual channels to Pixhawk")
        print("  get <ch>            - Get channel value")
        print("  getall              - Get all channel values")
        print("  reset               - Reset all channels to neutral (1500)")
        print("\n🚪 Exit:")
        print("  quit / q / exit     - Exit program")
        print("="*60 + "\n")
    
    def run(self):
        """Run the interactive CLI"""
        print("\n" + "="*60)
        print("PIXHAWK DIGITAL CHANNEL CONTROLLER - INTERACTIVE MODE")
        print("="*60)
        print("\nType 'help' or 'h' to see available commands")
        print("Type 'quit', 'q', or 'exit' to quit")
        print("="*60 + "\n")
        
        while True:
            try:
                user_input = input("\npixhawk> ").strip()
                
                if not user_input:
                    continue
                
                parts = user_input.split()
                cmd = parts[0].lower()
                
                # Help
                if cmd in ['help', 'h', '?']:
                    self.print_help()
                
                # Status
                elif cmd in ['status', 's']:
                    self.print_status()
                
                # Arming
                elif cmd == 'arm':
                    force = len(parts) > 1 and parts[1].lower() == 'force'
                    self.arm_pixhawk(force=force)
                
                elif cmd == 'disarm':
                    force = len(parts) > 1 and parts[1].lower() == 'force'
                    self.disarm_pixhawk(force=force)
                
                # Mode setting
                elif cmd == 'mode':
                    if len(parts) < 2:
                        print("Usage: mode <name>")
                        print("Example: mode GUIDED")
                    else:
                        self.set_mode(parts[1].upper())
                
                # Channel override control
                elif cmd == 'enable':
                    self.enable_channel_override()
                
                elif cmd == 'disable':
                    self.disable_channel_override()
                
                # Set single channel
                elif cmd == 'set':
                    if len(parts) < 3:
                        print("Usage: set <channel> <pwm>")
                        print("Example: set 1 1600")
                    else:
                        try:
                            ch = int(parts[1])
                            pwm = int(parts[2])
                            self.set_channel(ch, pwm)
                        except ValueError:
                            print("✗ Error: Channel and PWM must be integers")
                
                # Set multiple channels
                elif cmd == 'setmulti':
                    if len(parts) < 2:
                        print("Usage: setmulti <ch:pwm> <ch:pwm> ...")
                        print("Example: setmulti 1:1500 2:1600 3:1400")
                    else:
                        try:
                            for pair in parts[1:]:
                                ch, pwm = pair.split(':')
                                self.set_channel(int(ch), int(pwm))
                        except ValueError:
                            print("✗ Error: Format should be ch:pwm (e.g., 1:1500)")
                
                # Send channels
                elif cmd == 'send':
                    self.send_manual_channels()
                
                # Get channel value
                elif cmd == 'get':
                    if len(parts) < 2:
                        print("Usage: get <channel>")
                        print("Example: get 1")
                    else:
                        try:
                            ch = int(parts[1])
                            value = self.get_channel_value(ch)
                            if value is not None:
                                print(f"Channel {ch}: {value}")
                            else:
                                print(f"✗ Invalid channel: {ch}")
                        except ValueError:
                            print("✗ Error: Channel must be an integer")
                
                # Get all channels
                elif cmd == 'getall':
                    channels = self.get_all_channels()
                    print("\n📊 Manual Channel Values:")
                    for i in range(8):
                        print(f"  CH{i+1}: {channels[i]}")
                    print()
                
                # Reset channels
                elif cmd == 'reset':
                    self.reset_all_channels()
                
                # Quit
                elif cmd in ['quit', 'q', 'exit']:
                    print("\n👋 Exiting Pixhawk CLI...")
                    # Make sure to release override before exiting
                    if self.channel_override_enabled:
                        self.disable_channel_override()
                    break
                
                # Unknown command
                else:
                    print(f"✗ Unknown command: {cmd}")
                    print("Type 'help' to see available commands")
            
            except KeyboardInterrupt:
                print("\n\n👋 Interrupted by user. Exiting...")
                if self.channel_override_enabled:
                    self.disable_channel_override()
                break
            except Exception as e:
                print(f"✗ Error: {e}")
                traceback.print_exc()


def main():
    """Main entry point"""
    print("\n" + "="*60)
    print("🚁 PIXHAWK DIGITAL CHANNEL CONTROLLER")
    print("="*60)
    
    # Get connection parameters
    if len(sys.argv) > 1:
        connection_string = sys.argv[1]
    else:
        connection_string = "/dev/ttyUSB0"
    
    if len(sys.argv) > 2:
        baud = int(sys.argv[2])
    else:
        baud = 57600
    
    print(f"\nConnection: {connection_string}")
    print(f"Baud Rate: {baud}")
    print("="*60)
    
    try:
        cli = PixhawkCLI(connection_string, baud)
        cli.run()
    except KeyboardInterrupt:
        print("\n\n👋 Interrupted by user. Exiting...")
    except Exception as e:
        print(f"\n✗ Fatal error: {e}")
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
