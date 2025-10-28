#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math
import threading
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from core_msgs.msg import KillSwitch, Config
import tf_transformations

from pid_controller import YawPIDController, PIDController


class PIDControllerTester(Node):
    """
    Test node for PID Controller functionality
    """
    
    def __init__(self):
        super().__init__('pid_controller_tester')
        
        # Test publishers
        self.dsc_pub = self.create_publisher(Float64, 'dsc', 10)
        self.yaw_setpoint_pub = self.create_publisher(Float64, 'yaw_setpoint', 10)
        self.imu_pub = self.create_publisher(Imu, '/mavros/imu/data', 10)
        self.config_pub = self.create_publisher(Config, 'pid_config', 10)
        self.killswitch_pub = self.create_publisher(KillSwitch, 'killswitch', 10)
        
        # Test subscriber to monitor output
        self.yaw_effort_sub = self.create_subscription(
            Float64, 'yaw_effort', self._yaw_effort_callback, 10
        )
        
        self.received_efforts = []
        self.test_results = {}
        
        # Wait for connections
        time.sleep(1.0)
        
    def _yaw_effort_callback(self, msg: Float64):
        """Record received yaw efforts for analysis"""
        self.received_efforts.append({
            'timestamp': time.time(),
            'effort': msg.data
        })
        
    def publish_vision_error(self, error_value):
        """Publish vision error (DSC) for testing"""
        msg = Float64()
        msg.data = error_value
        self.dsc_pub.publish(msg)
        
    def publish_yaw_setpoint(self, setpoint):
        """Publish yaw setpoint for testing"""
        msg = Float64()
        msg.data = setpoint
        self.yaw_setpoint_pub.publish(msg)
        
    def publish_imu_data(self, yaw_angle):
        """Publish simulated IMU data with specific yaw angle"""
        msg = Imu()
        
        # Convert yaw angle to quaternion
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw_angle)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        
        # Add some fake angular velocity and linear acceleration
        msg.angular_velocity.z = 0.1
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81
        
        self.imu_pub.publish(msg)
        
    def publish_pid_config(self, kp, ki, kd):
        """Publish PID configuration for testing"""
        msg = Config()
        # Assuming Config message has these fields
        # Adjust based on your actual Config message structure
        msg.kp = kp
        msg.ki = ki
        msg.kd = kd
        self.config_pub.publish(msg)
        
    def publish_killswitch(self, state):
        """Publish killswitch state"""
        msg = KillSwitch()
        msg.data = state
        self.killswitch_pub.publish(msg)
        
    def clear_effort_history(self):
        """Clear recorded effort history"""
        self.received_efforts = []
        
    def wait_for_efforts(self, duration=2.0, min_samples=10):
        """Wait for PID efforts and return collected data"""
        start_time = time.time()
        initial_count = len(self.received_efforts)
        
        while (time.time() - start_time) < duration:
            if len(self.received_efforts) - initial_count >= min_samples:
                break
            time.sleep(0.1)
            
        return self.received_efforts[initial_count:]
        
    def test_vision_control(self):
        """Test vision-based control mode"""
        self.get_logger().info("=== Testing Vision Control ===")
        
        test_cases = [
            {"error": 0, "expected": "~0", "description": "No error (centered)"},
            {"error": 100, "expected": ">0", "description": "Positive error (turn right)"},
            {"error": -100, "expected": "<0", "description": "Negative error (turn left)"},
            {"error": 10, "expected": "~0", "description": "Small error (deadzone)"},
            {"error": 320, "expected": ">0", "description": "Maximum error"},
        ]
        
        results = []
        
        for case in test_cases:
            self.get_logger().info(f"Testing: {case['description']}")
            self.clear_effort_history()
            
            # Publish vision error
            self.publish_vision_error(case["error"])
            
            # Wait for response
            efforts = self.wait_for_efforts(duration=1.0)
            
            if efforts:
                avg_effort = sum([e['effort'] for e in efforts]) / len(efforts)
                
                # Analyze result
                if case["expected"] == "~0":
                    passed = abs(avg_effort) < 10
                elif case["expected"] == ">0":
                    passed = avg_effort > 10
                elif case["expected"] == "<0":
                    passed = avg_effort < -10
                else:
                    passed = False
                    
                results.append({
                    'case': case['description'],
                    'error': case['error'],
                    'effort': avg_effort,
                    'passed': passed
                })
                
                status = "PASS" if passed else "FAIL"
                self.get_logger().info(f"  Result: effort={avg_effort:.2f} [{status}]")
            else:
                results.append({
                    'case': case['description'],
                    'error': case['error'],
                    'effort': 0.0,
                    'passed': False
                })
                self.get_logger().error("  No response received!")
                
        self.test_results['vision_control'] = results
        return results
        
    def test_heading_control(self):
        """Test heading-based control mode"""
        self.get_logger().info("=== Testing Heading Control ===")
        
        test_cases = [
            {"setpoint": 0.0, "current": 0.0, "expected": "~0", "description": "On target"},
            {"setpoint": 1.57, "current": 0.0, "expected": ">0", "description": "Turn to 90 degrees"},
            {"setpoint": 0.0, "current": 1.57, "expected": "<0", "description": "Turn back to 0"},
            {"setpoint": 3.14, "current": 0.0, "expected": ">0", "description": "Turn to 180 degrees"},
        ]
        
        results = []
        
        for case in test_cases:
            self.get_logger().info(f"Testing: {case['description']}")
            self.clear_effort_history()
            
            # Publish current heading via IMU
            self.publish_imu_data(case["current"])
            time.sleep(0.1)
            
            # Publish setpoint (this switches to heading mode)
            self.publish_yaw_setpoint(case["setpoint"])
            
            # Wait for response
            efforts = self.wait_for_efforts(duration=1.0)
            
            if efforts:
                avg_effort = sum([e['effort'] for e in efforts]) / len(efforts)
                
                # Analyze result
                if case["expected"] == "~0":
                    passed = abs(avg_effort) < 10
                elif case["expected"] == ">0":
                    passed = avg_effort > 10
                elif case["expected"] == "<0":
                    passed = avg_effort < -10
                else:
                    passed = False
                    
                results.append({
                    'case': case['description'],
                    'setpoint': case['setpoint'],
                    'current': case['current'],
                    'effort': avg_effort,
                    'passed': passed
                })
                
                status = "PASS" if passed else "FAIL"
                self.get_logger().info(f"  Result: effort={avg_effort:.2f} [{status}]")
            else:
                results.append({
                    'case': case['description'],
                    'setpoint': case['setpoint'],
                    'current': case['current'],
                    'effort': 0.0,
                    'passed': False
                })
                self.get_logger().error("  No response received!")
                
        self.test_results['heading_control'] = results
        return results
        
    def test_pid_tuning(self):
        """Test PID parameter updates"""
        self.get_logger().info("=== Testing PID Parameter Updates ===")
        
        # Test different PID configurations
        configs = [
            {"kp": 1.0, "ki": 0.0, "kd": 0.0, "description": "P-only control"},
            {"kp": 2.0, "ki": 0.1, "kd": 0.05, "description": "Standard PID"},
            {"kp": 5.0, "ki": 0.0, "kd": 0.0, "description": "High P gain"},
        ]
        
        results = []
        fixed_error = 100  # Fixed vision error for comparison
        
        for config in configs:
            self.get_logger().info(f"Testing: {config['description']}")
            
            # Update PID parameters
            self.publish_pid_config(config["kp"], config["ki"], config["kd"])
            time.sleep(0.5)  # Allow time for parameter update
            
            self.clear_effort_history()
            
            # Apply fixed error
            self.publish_vision_error(fixed_error)
            
            # Wait for response
            efforts = self.wait_for_efforts(duration=1.0)
            
            if efforts:
                avg_effort = sum([e['effort'] for e in efforts]) / len(efforts)
                results.append({
                    'config': config['description'],
                    'kp': config['kp'],
                    'ki': config['ki'],
                    'kd': config['kd'],
                    'effort': avg_effort,
                    'passed': abs(avg_effort) > 0  # Just check for response
                })
                
                self.get_logger().info(f"  Result: effort={avg_effort:.2f}")
            else:
                results.append({
                    'config': config['description'],
                    'kp': config['kp'],
                    'ki': config['ki'],
                    'kd': config['kd'],
                    'effort': 0.0,
                    'passed': False
                })
                self.get_logger().error("  No response received!")
                
        self.test_results['pid_tuning'] = results
        return results
        
    def test_killswitch(self):
        """Test killswitch functionality"""
        self.get_logger().info("=== Testing Killswitch ===")
        
        results = []
        
        # Test normal operation
        self.get_logger().info("Testing normal operation")
        self.publish_killswitch(KillSwitch.DEFAULT)
        time.sleep(0.5)
        
        self.clear_effort_history()
        self.publish_vision_error(100)  # Apply error
        efforts_normal = self.wait_for_efforts(duration=1.0)
        
        normal_effort = 0.0
        if efforts_normal:
            normal_effort = sum([e['effort'] for e in efforts_normal]) / len(efforts_normal)
            
        # Test killswitch activated
        self.get_logger().info("Testing killswitch activation")
        self.publish_killswitch(KillSwitch.KILL)
        time.sleep(0.5)
        
        self.clear_effort_history()
        self.publish_vision_error(100)  # Apply same error
        efforts_killed = self.wait_for_efforts(duration=1.0)
        
        killed_effort = 0.0
        if efforts_killed:
            killed_effort = sum([e['effort'] for e in efforts_killed]) / len(efforts_killed)
            
        # Analyze results
        normal_working = abs(normal_effort) > 10
        killswitch_working = abs(killed_effort) < 5
        
        results.append({
            'test': 'Normal operation',
            'effort': normal_effort,
            'passed': normal_working
        })
        
        results.append({
            'test': 'Killswitch active',
            'effort': killed_effort,
            'passed': killswitch_working
        })
        
        self.get_logger().info(f"Normal effort: {normal_effort:.2f} [{'PASS' if normal_working else 'FAIL'}]")
        self.get_logger().info(f"Killed effort: {killed_effort:.2f} [{'PASS' if killswitch_working else 'FAIL'}]")
        
        # Re-enable for other tests
        self.publish_killswitch(KillSwitch.DEFAULT)
        time.sleep(0.5)
        
        self.test_results['killswitch'] = results
        return results
        
    def test_step_response(self):
        """Test step response characteristics"""
        self.get_logger().info("=== Testing Step Response ===")
        
        # Apply step input and record response
        self.clear_effort_history()
        
        # Zero input first
        self.publish_vision_error(0)
        time.sleep(1.0)
        
        # Clear history and apply step
        self.clear_effort_history()
        step_input = 200  # Large step input
        self.publish_vision_error(step_input)
        
        # Record response for longer duration
        efforts = self.wait_for_efforts(duration=3.0, min_samples=50)
        
        if len(efforts) > 10:
            # Analyze response characteristics
            effort_values = [e['effort'] for e in efforts]
            times = [e['timestamp'] - efforts[0]['timestamp'] for e in efforts]
            
            max_effort = max(effort_values)
            steady_state = sum(effort_values[-10:]) / 10  # Last 10 samples
            
            # Find settling time (within 5% of steady state)
            settling_tolerance = abs(steady_state * 0.05)
            settling_time = None
            
            for i in range(len(effort_values)-1, -1, -1):
                if abs(effort_values[i] - steady_state) > settling_tolerance:
                    settling_time = times[i] if i < len(times)-1 else times[-1]
                    break
                    
            results = {
                'step_input': step_input,
                'max_effort': max_effort,
                'steady_state': steady_state,
                'settling_time': settling_time,
                'data_points': len(efforts),
                'passed': len(efforts) > 10 and abs(steady_state) > 10
            }
            
            self.get_logger().info(f"Step response analysis:")
            self.get_logger().info(f"  Max effort: {max_effort:.2f}")
            self.get_logger().info(f"  Steady state: {steady_state:.2f}")
            self.get_logger().info(f"  Settling time: {settling_time:.2f}s" if settling_time else "  Settling time: Not determined")
            
        else:
            results = {
                'step_input': step_input,
                'max_effort': 0.0,
                'steady_state': 0.0,
                'settling_time': None,
                'data_points': len(efforts),
                'passed': False
            }
            self.get_logger().error("Insufficient data for step response analysis")
            
        self.test_results['step_response'] = results
        return results
        
    def run_all_tests(self):
        """Run comprehensive test suite"""
        self.get_logger().info("Starting PID Controller Test Suite")
        self.get_logger().info("=" * 50)
        
        # Run all tests
        self.test_killswitch()
        self.test_vision_control()
        self.test_heading_control()
        self.test_pid_tuning()
        self.test_step_response()
        
        # Generate test report
        self.generate_test_report()
        
    def generate_test_report(self):
        """Generate and display test report"""
        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info("PID CONTROLLER TEST REPORT")
        self.get_logger().info("=" * 50)
        
        total_tests = 0
        passed_tests = 0
        
        for test_category, results in self.test_results.items():
            self.get_logger().info(f"\n{test_category.upper().replace('_', ' ')}:")
            
            if isinstance(results, list):
                for result in results:
                    total_tests += 1
                    if result.get('passed', False):
                        passed_tests += 1
                        status = "PASS"
                    else:
                        status = "FAIL"
                    
                    test_name = result.get('case', result.get('test', result.get('config', 'Unknown')))
                    self.get_logger().info(f"  {test_name}: {status}")
            else:
                total_tests += 1
                if results.get('passed', False):
                    passed_tests += 1
                    status = "PASS"
                else:
                    status = "FAIL"
                self.get_logger().info(f"  {test_category}: {status}")
                
        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        
        self.get_logger().info(f"\nOVERALL RESULTS:")
        self.get_logger().info(f"  Total tests: {total_tests}")
        self.get_logger().info(f"  Passed: {passed_tests}")
        self.get_logger().info(f"  Failed: {total_tests - passed_tests}")
        self.get_logger().info(f"  Success rate: {success_rate:.1f}%")
        
        if success_rate >= 80:
            self.get_logger().info("✅ PID Controller is working well!")
        elif success_rate >= 60:
            self.get_logger().warn("⚠️  PID Controller has some issues")
        else:
            self.get_logger().error("❌ PID Controller needs significant fixes")


def test_standalone_pid():
    """Test standalone PID class without ROS"""
    print("=== Testing Standalone PID Class ===")
    
    # Test basic PID functionality
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
    
    # Test step response
    setpoint = 100.0
    current_values = [0, 20, 40, 60, 80, 90, 95, 98, 99, 100]
    
    print("Step response test:")
    print("Error\tOutput")
    
    for current in current_values:
        output = pid.compute(setpoint, current)
        error = setpoint - current
        print(f"{error:.1f}\t{output:.2f}")
        time.sleep(0.02)  # Simulate 50Hz control loop
        
    # Test reset functionality
    pid.reset()
    output_after_reset = pid.compute(setpoint, 0)
    print(f"\nAfter reset with full error: {output_after_reset:.2f}")
    
    # Test parameter update
    pid.set_tunings(2.0, 0.2, 0.1)
    output_new_params = pid.compute(setpoint, 0)
    print(f"With new parameters: {output_new_params:.2f}")


def main(args=None):
    # First test standalone PID class
    test_standalone_pid()
    
    # Then test ROS integration
    try:
        rclpy.init(args=args)
        
        # Start PID controller in separate thread
        pid_controller = YawPIDController()
        
        def run_pid_controller():
            pid_controller.run()
            rclpy.spin(pid_controller)
            
        pid_thread = threading.Thread(target=run_pid_controller)
        pid_thread.daemon = True
        pid_thread.start()
        
        # Wait for PID controller to start
        time.sleep(2.0)
        
        # Run tests
        tester = PIDControllerTester()
        tester.run_all_tests()
        
        # Keep running for a bit to see results
        time.sleep(2.0)
        
    except Exception as e:
        print(f"Error in testing: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()