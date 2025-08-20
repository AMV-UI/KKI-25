#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import sys
from threading import Thread

try:
    from core.utils.motor import Motor
except ImportError:
    print("Error: Cannot import Motor class. Make sure motor.py is in the same directory or in your Python path.")
    sys.exit(1)

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        
        # Create motor instance with some test parameters
        self.motor = Motor(offset_horizontal=150, motor_adjust=40)
        
        # Test configuration
        self.test_duration = 2.0  # seconds for each test
        self.pause_duration = 1.0  # seconds between tests
        
        self.get_logger().info("Motor Tester initialized")
        self.get_logger().info("=" * 60)
        self.display_pwm_status()

    def display_pwm_status(self):
        """Display current PWM configuration"""
        status = self.motor.get_pwm_status()
        self.get_logger().info("PWM Configuration:")
        self.get_logger().info(f"  - Range: {status['pwm_range']['min']} to {status['pwm_range']['max']}")
        self.get_logger().info(f"  - Standby: {status['pwm_range']['standby']}")
        self.get_logger().info(f"  - Forward: {status['pwm_range']['forward']}")
        self.get_logger().info(f"  - Backward: {status['pwm_range']['backward']}")
        self.get_logger().info(f"  - Motor Adjust: {status['motor_adjust']}")
        self.get_logger().info("=" * 60)

    def test_action(self, action_name, action_func, *args, **kwargs):
        """Test a single motor action"""
        self.get_logger().info(f"Testing: {action_name.upper()}")
        self.get_logger().info("-" * 40)
        
        try:
            # Execute the action
            result = action_func(*args, **kwargs)
            
            # Display results
            if isinstance(result, dict):
                for channel, pwm in result.items():
                    motor_name = "Left Motor (X)" if channel == self.motor.channel.MOTOR_X else "Right Motor (Y)"
                    self.get_logger().info(f"  {motor_name}: PWM = {pwm}")
            
            # Simulate holding the command for test duration
            self.get_logger().info(f"  Holding command for {self.test_duration} seconds...")
            time.sleep(self.test_duration)
            
        except Exception as e:
            self.get_logger().error(f"Error testing {action_name}: {str(e)}")
        
        # Pause between tests
        self.get_logger().info(f"  Pausing for {self.pause_duration} seconds...\n")
        time.sleep(self.pause_duration)

    def run_basic_movement_tests(self):
        """Test basic movement functions"""
        self.get_logger().info("STARTING BASIC MOVEMENT TESTS")
        self.get_logger().info("=" * 60)
        
        # Test idle first (safe starting position)
        self.test_action("Idle/Stop", self.motor.idle)
        
        # Test forward movement
        self.test_action("Forward", self.motor.forward)
        
        # Test backward movement
        self.test_action("Backward", self.motor.backward)
        
        # Test turning
        self.test_action("Turn Left", self.motor.turnLeft)
        self.test_action("Turn Right", self.motor.turnRight)
        
        # Test rotation
        self.test_action("Rotate Left", self.motor.rotateLeft)
        self.test_action("Rotate Right", self.motor.rotateRight)
        
        # Return to idle
        self.test_action("Return to Idle", self.motor.idle)

    def run_advanced_movement_tests(self):
        """Test advanced movement functions"""
        self.get_logger().info("STARTING ADVANCED MOVEMENT TESTS")
        self.get_logger().info("=" * 60)
        
        # Test straight with confidence
        self.test_action("Straight with Confidence (default)", 
                        self.motor.straight_with_conf)
        
        self.test_action("Straight with Confidence (yaw=50)", 
                        self.motor.straight_with_conf, 50)
        
        self.test_action("Straight with Confidence (yaw=200)", 
                        self.motor.straight_with_conf, 200)
        
        # Test autonomous movement
        self.test_action("Autonomous (default)", 
                        self.motor.autonomous)
        
        self.test_action("Autonomous (x=100, y=150)", 
                        self.motor.autonomous, 100, 150)
        
        self.test_action("Autonomous (x=25, y=300)", 
                        self.motor.autonomous, 25, 300)

    def run_speed_profile_tests(self):
        """Test different speed profiles"""
        self.get_logger().info("STARTING SPEED PROFILE TESTS")
        self.get_logger().info("=" * 60)
        
        profiles = [
            'reset',
            'half_detected',
            'one_is_closer_detected', 
            'full_detected',
            'not_detected',
            'searching'
        ]
        
        for profile in profiles:
            self.get_logger().info(f"Testing Speed Profile: {profile.upper()}")
            self.get_logger().info("-" * 40)
            
            # Set the speed profile
            self.motor.set_speed_profile(profile)
            
            # Test autonomous movement with this profile
            self.test_action(f"Autonomous with {profile}", 
                           self.motor.autonomous, 75, 100)

    def run_edge_case_tests(self):
        """Test edge cases and potential issues"""
        self.get_logger().info("STARTING EDGE CASE TESTS")
        self.get_logger().info("=" * 60)
        
        # Test with extreme values
        self.test_action("Autonomous with high values", 
                        self.motor.autonomous, 500, 800)
        
        self.test_action("Autonomous with low values", 
                        self.motor.autonomous, -100, -200)
        
        self.test_action("Straight with high yaw", 
                        self.motor.straight_with_conf, 500)
        
        self.test_action("Straight with negative yaw", 
                        self.motor.straight_with_conf, -200)

    def run_interactive_test(self):
        """Interactive test mode"""
        self.get_logger().info("INTERACTIVE TEST MODE")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Available commands:")
        self.get_logger().info("  1 - Forward")
        self.get_logger().info("  2 - Backward") 
        self.get_logger().info("  3 - Turn Left")
        self.get_logger().info("  4 - Turn Right")
        self.get_logger().info("  5 - Rotate Left")
        self.get_logger().info("  6 - Rotate Right")
        self.get_logger().info("  7 - Idle")
        self.get_logger().info("  8 - Straight with confidence")
        self.get_logger().info("  9 - Autonomous")
        self.get_logger().info("  s - Show PWM status")
        self.get_logger().info("  q - Quit")
        
        actions = {
            '1': ('Forward', self.motor.forward),
            '2': ('Backward', self.motor.backward),
            '3': ('Turn Left', self.motor.turnLeft),
            '4': ('Turn Right', self.motor.turnRight),
            '5': ('Rotate Left', self.motor.rotateLeft),
            '6': ('Rotate Right', self.motor.rotateRight),
            '7': ('Idle', self.motor.idle),
            '8': ('Straight with Confidence', lambda: self.motor.straight_with_conf(100)),
            '9': ('Autonomous', lambda: self.motor.autonomous(50, 200)),
        }
        
        while rclpy.ok():
            try:
                choice = input("\nEnter command: ").strip().lower()
                
                if choice == 'q':
                    break
                elif choice == 's':
                    self.display_pwm_status()
                elif choice in actions:
                    name, func = actions[choice]
                    self.test_action(name, func)
                else:
                    print("Invalid choice. Try again.")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break

    def run_all_tests(self):
        """Run all automated tests"""
        try:
            self.run_basic_movement_tests()
            self.run_advanced_movement_tests() 
            self.run_speed_profile_tests()
            self.run_edge_case_tests()
            
            self.get_logger().info("ALL TESTS COMPLETED")
            self.get_logger().info("=" * 60)
            
        except KeyboardInterrupt:
            self.get_logger().info("Tests interrupted by user")
        except Exception as e:
            self.get_logger().error(f"Test failed with error: {str(e)}")


def main():
    """Main function with command line options"""
    rclpy.init()
    
    # Parse command line arguments
    test_mode = 'all'  # default
    if len(sys.argv) > 1:
        test_mode = sys.argv[1].lower()
    
    try:
        tester = MotorTester()
        
        if test_mode == 'basic':
            tester.run_basic_movement_tests()
        elif test_mode == 'advanced':
            tester.run_advanced_movement_tests()
        elif test_mode == 'profiles':
            tester.run_speed_profile_tests()
        elif test_mode == 'edge':
            tester.run_edge_case_tests()
        elif test_mode == 'interactive':
            tester.run_interactive_test()
        elif test_mode == 'all':
            tester.run_all_tests()
        else:
            print("Usage: python motor_test.py [basic|advanced|profiles|edge|interactive|all]")
            print("  basic      - Test basic movement functions")
            print("  advanced   - Test advanced movement functions")  
            print("  profiles   - Test speed profile changes")
            print("  edge       - Test edge cases and extreme values")
            print("  interactive- Interactive testing mode")
            print("  all        - Run all automated tests (default)")
            
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        try:
            tester.motor.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()