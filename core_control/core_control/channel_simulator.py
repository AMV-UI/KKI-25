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

class ChannelState(Enum):
    LOW = 1
    MID = 2
    HIGH = 3

class ChannelSimulator(Node):
    def __init__(self):
        super().__init__('channel_simulator')
        self.rc5_state = ChannelState.LOW
        self.rc6_state = ChannelState.LOW
        self.rc5_pub = Topic.rc5.createPublisher(self)
        self.rc6_pub = Topic.rc6.createPublisher(self)
        
        self.running = True
        
        # Save terminal settings to restore later
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.read_input, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info('Channel Simulator Node Started')
        self.get_logger().info('Press 5 to toggle RC5 (Docking), Press 6 to toggle RC6 (Recording)')
        self.get_logger().info('Press q to quit')
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
        self.get_logger().info(f'RC5: {self.rc5_state.name} | RC6: {self.rc6_state.name}')

    def toggle_rc5(self):
        past = self.rc5_state
        if past == ChannelState.LOW:
            self.rc5_state = ChannelState.MID
            self.rc5_pub.publish(Float64(data=float(1495)))
        elif past == ChannelState.MID:
            self.rc5_state = ChannelState.HIGH
            self.rc5_pub.publish(Float64(data=float(2006)))
        elif past == ChannelState.HIGH:
            self.rc5_state = ChannelState.LOW
            self.rc5_pub.publish(Float64(data=float(983)))

        self.print_status()

    def toggle_rc6(self):
        past = self.rc6_state
        if past == ChannelState.LOW:
            self.rc6_state = ChannelState.MID
            self.rc6_pub.publish(Float64(data=float(1495)))
        elif past == ChannelState.MID:
            self.rc6_state = ChannelState.HIGH
            self.rc6_pub.publish(Float64(data=float(2006)))
        elif past == ChannelState.HIGH:
            self.rc6_state = ChannelState.LOW
            self.rc6_pub.publish(Float64(data=float(983)))

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
