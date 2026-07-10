#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from enum import Enum
import sys
import threading
import termios
import tty
from core.utils.config import Topic
from core_msgs.msg import Pixhawk

class ChannelState(Enum):
    LOW = 1
    MID = 2
    HIGH = 3

class ChannelSimulator(Node):
    def __init__(self):
        super().__init__('channel_simulator')
        self.rc5_state = ChannelState.LOW
        self.rc6_state = ChannelState.LOW
        self.rc7_state = ChannelState.LOW
        self.rc8_state = ChannelState.LOW
        self.rc5_pub = Topic.rc5.createPublisher(self)
        self.rc6_pub = Topic.rc6.createPublisher(self)
        self.rc7_pub = Topic.rc7.createPublisher(self)
        self.rc8_pub = Topic.pixhawk.createPublisher(self)
        
        self.running = True
        
        # Save terminal settings to restore later
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.read_input, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info('Channel Simulator Node Started')
        self.get_logger().info('Press 5 to toggle RC5 (Docking), Press 6 to toggle RC6 (Recording)')
        self.get_logger().info('Press 8 to toggle RC8 (GPS Position), Press q to quit')
        self.print_status()

    def publish_rc(self, pub, value):
        pub.publish(Float64(value))

    def read_input(self):
        """Read keyboard input from stdin in a separate thread"""
        try:
            # Set terminal to raw mode for character-by-character input
            tty.setraw(sys.stdin.fileno())
            
            while self.running:
                try:
                    # Read one character at a time (non-blocking with timeout)
                    key = sys.stdin.read(1)
                    
                    if key == '5':
                        self.toggle_rc5()
                    elif key == '6':
                        self.toggle_rc6()
                    elif key == '7':
                        self.toggle_rc7()
                    elif key == '8':
                        self.toggle_rc8()
                    elif key == 'q' or key == 'Q':
                        self.get_logger().info('Quit command received — shutting down')
                        self.running = False
                        rclpy.shutdown()
                        break
                except Exception as e:
                    if self.running:
                        self.get_logger().error(f'Input error: {e}')
                    break
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def print_status(self):
        """Print current channel states"""
        self.get_logger().info(f'RC5: {self.rc5_state.name} | RC6: {self.rc6_state.name} | RC7: {self.rc7_state.name} | RC8: {self.rc8_state.name}')

    def toggle_rc(self, channel):
        curr = getattr(self, f"{channel}_state")

        if curr == ChannelState.LOW:
            next_state = ChannelState.MID
            pwm = 1495

        elif curr == ChannelState.MID:
            prev_state = getattr(self, f"{channel}_prev_state", ChannelState.LOW)

            if prev_state == ChannelState.LOW:
                next_state = ChannelState.HIGH
                pwm = 2006
            else:
                next_state = ChannelState.LOW
                pwm = 983

        elif curr == ChannelState.HIGH:
            next_state = ChannelState.MID
            pwm = 1495

        setattr(self, f"{channel}_prev_state", curr)
        setattr(self, f"{channel}_state", next_state)

        getattr(self, f"{channel}_pub").publish(Float64(data=float(pwm)))

        print(f"✓ {channel}: {curr.name} -> {next_state.name}")
        self.print_status()

    
    def toggle_rc5(self):
        self.toggle_rc("rc5")

    def toggle_rc6(self):
        self.toggle_rc("rc6")

    def toggle_rc7(self):
        self.toggle_rc("rc7")

    
    def toggle_rc8(self):
        past = self.rc8_state
        if past == ChannelState.LOW:
            self.rc8_state = ChannelState.MID
            mock_idle_coord = Pixhawk()
            mock_idle_coord.lat = -6.362495
            mock_idle_coord.lon = 106.827153
            mock_idle_coord.msg_heading = 45
            self.rc8_pub.publish(mock_idle_coord)
            self.get_logger().info(f'RC8 MID: Start position (lat={mock_idle_coord.lat}, lon={mock_idle_coord.lon})')
            
        elif past == ChannelState.MID:
            self.rc8_state = ChannelState.HIGH
            mock_moving_coord = Pixhawk()
            mock_moving_coord.lat = -6.361595
            mock_moving_coord.lon = 106.828053
            mock_moving_coord.msg_heading = 45
            self.rc8_pub.publish(mock_moving_coord)
            self.get_logger().info(f'RC8 HIGH: Moved ~100m NE (lat={mock_moving_coord.lat}, lon={mock_moving_coord.lon})')
            
        elif past == ChannelState.HIGH:
            self.rc8_state = ChannelState.LOW
            mock_moving_coord = Pixhawk()
            mock_moving_coord.lat = 6.360695 
            mock_moving_coord.lon = -106.828953
            mock_moving_coord.msg_heading = 45
            self.rc8_pub.publish(mock_moving_coord)
            self.get_logger().info(f'RC8 LOW: Moved ~200m NE (lat={mock_moving_coord.lat}, lon={mock_moving_coord.lon})')

        self.print_status()


def main(args=None):
    rclpy.init(args=args)
    node = ChannelSimulator()
    while rclpy.ok():
        rclpy.spin(node)
    try:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)
    except:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()