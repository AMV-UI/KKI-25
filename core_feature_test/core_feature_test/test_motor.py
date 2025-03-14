#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from core_feature_test.target.motor import Motor, Param, SPEED, Channel
import sys
import time

# ANSI escape codes for coloring the output
GREEN = '\033[92m'
RED = '\033[91m'
RESET = '\033[0m'

def test_motor_adjust(motor):
    print("Testing __adjust...")
    if motor._Motor__adjust(1600, 100) == 1700 and motor._Motor__adjust(1400, 100) == 1300 and motor._Motor__adjust(1500, 100) == 1500:
        print(f"{GREEN}✓ test_adjust PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_adjust FAILED{RESET}")
        return False
    

def test_motor_calcAdjustedSpeed(motor):
    print("Testing __calcAdjustedSpeed...")
    res = {Channel.MOTOR_X: 1600, Channel.MOTOR_Y: 1400}
    adjusted_res = motor._Motor__calcAdjustedSpeed(res)
    expected_res = {Channel.MOTOR_X: 1700, Channel.MOTOR_Y: 1300}
    if adjusted_res == expected_res:
        print(f"{GREEN}✓ test_calcAdjustedSpeed PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_calcAdjustedSpeed FAILED{RESET}")
        return False

def test_motor_getChannels(motor):
    print("Testing getChannels...")
    if motor.getChannels() == Channel:
        print(f"{GREEN}✓ test_getChannels PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_getChannels FAILED{RESET}")
        return False

def test_motor_calculateSpeed(motor):
    print("Testing calculateSpeed...")
    if motor.calculateSpeed(100) == 1600 and motor.calculateSpeed(-100) == 1400:
        print(f"{GREEN}✓ test_calculateSpeed PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_calculateSpeed FAILED{RESET}")
        return False

def test_motor_straight_with_conf(motor):
    print("Testing straight_with_conf...")
    result = motor.straight_with_conf(100)
    if result[Channel.MOTOR_X] == 1700 and result[Channel.MOTOR_Y] == 1700:
        print(f"{GREEN}✓ test_straight_with_conf PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_straight_with_conf FAILED{RESET}")
        return False

def test_motor_autonomous(motor):
    print("Testing autonomous...")
    motor.declare_parameter(Param.MOTOR_SPEED, 1.0)
    motor.declare_parameter(Param.X_SPEED, 1.0)
    result = motor.autonomous(50)
    if result[Channel.MOTOR_X] == 1550 and result[Channel.MOTOR_Y] == 1700:
        print(f"{GREEN}✓ test_autonomous PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_autonomous FAILED{RESET}")
        return False

def test_motor_reset_param(motor):
    print("Testing reset_param...")
    motor.declare_parameter(Param.MOTOR_SPEED, SPEED.Maximum)
    motor.declare_parameter(Param.X_SPEED, SPEED.Maximum)
    Motor.reset_param()
    if motor.get_parameter(Param.MOTOR_SPEED).value == SPEED.Maximum and motor.get_parameter(Param.X_SPEED).value == SPEED.Maximum:
        print(f"{GREEN}✓ test_reset_param PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_reset_param FAILED{RESET}")
        return False

def test_motor_half_detected(motor):
    print("Testing half_detected...")
    Motor.half_detected()
    if motor.get_parameter(Param.MOTOR_SPEED).value == SPEED.MediumFast and motor.get_parameter(Param.X_SPEED).value == SPEED.Medium:
        print(f"{GREEN}✓ test_half_detected PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_half_detected FAILED{RESET}")
        return False

def test_motor_one_is_closer_detected(motor):
    print("Testing one_is_closer_detected...")
    Motor.one_is_closer_detected()
    if motor.get_parameter(Param.MOTOR_SPEED).value == SPEED.Medium and motor.get_parameter(Param.X_SPEED).value == SPEED.Medium:
        print(f"{GREEN}✓ test_one_is_closer_detected PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_one_is_closer_detected FAILED{RESET}")
        return False

def test_motor_full_detected(motor):
    print("Testing full_detected...")
    Motor.full_detected()
    if motor.get_parameter(Param.MOTOR_SPEED).value == SPEED.Medium and motor.get_parameter(Param.X_SPEED).value == SPEED.Maximum:
        print(f"{GREEN}✓ test_full_detected PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_full_detected FAILED{RESET}")
        return False

def test_motor_not_detected(motor):
    print("Testing not_detected...")
    Motor.not_detected()
    if motor.get_parameter(Param.MOTOR_SPEED).value == SPEED.Idle and motor.get_parameter(Param.X_SPEED).value == SPEED.Slow:
        print(f"{GREEN}✓ test_not_detected PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_not_detected FAILED{RESET}")
        return False

def test_motor_searching(motor):
    print("Testing searching...")
    Motor.searching()
    if motor.get_parameter(Param.MOTOR_SPEED).value == SPEED.Medium:
        print(f"{GREEN}✓ test_searching PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_searching FAILED{RESET}")
        return False

def test_motor_forward(motor):
    print("Testing forward...")
    result = motor.forward()
    if result[Channel.MOTOR_X] == 1500 and result[Channel.MOTOR_Y] == 1500:
        print(f"{GREEN}✓ test_forward PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_forward FAILED{RESET}")
        return False

def test_motor_backward(motor):
    print("Testing backward...")
    result = motor.backward()
    if result[Channel.MOTOR_X] == 1500 and result[Channel.MOTOR_Y] == 1500:
        print(f"{GREEN}✓ test_backward PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_backward FAILED{RESET}")
        return False

def test_motor_turnLeft(motor):
    print("Testing turnLeft...")
    result = motor.turnLeft()
    if result[Channel.MOTOR_X] == 1300 and result[Channel.MOTOR_Y] == 1700:
        print(f"{GREEN}✓ test_turnLeft PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_turnLeft FAILED{RESET}")
        return False

def test_motor_turnRight(motor):
    print("Testing turnRight...")
    result = motor.turnRight()
    if result[Channel.MOTOR_X] == 1700 and result[Channel.MOTOR_Y] == 1300:
        print(f"{GREEN}✓ test_turnRight PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_turnRight FAILED{RESET}")
        return False

def test_motor_rotateLeft(motor):
    print("Testing rotateLeft...")
    result = motor.rotateLeft()
    if result[Channel.MOTOR_X] == 1700 and result[Channel.MOTOR_Y] == 1300:
        print(f"{GREEN}✓ test_rotateLeft PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_rotateLeft FAILED{RESET}")
        return False

def test_motor_rotateRight(motor):
    print("Testing rotateRight...")
    result = motor.rotateRight()
    if result[Channel.MOTOR_X] == 1300 and result[Channel.MOTOR_Y] == 1700:
        print(f"{GREEN}✓ test_rotateRight PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_rotateRight FAILED{RESET}")
        return False

def test_motor_idle(motor):
    print("Testing idle...")
    result = motor.idle()
    if result[Channel.MOTOR_X] == 1500 and result[Channel.MOTOR_Y] == 1500:
        print(f"{GREEN}✓ test_idle PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_idle FAILED{RESET}")
        return False

def run_all_tests():
    # Initialize ROS
    rclpy.init()
    node = Node('test_motor_node')
    motor = Motor()

    # Run tests
    tests = [
        test_motor_adjust,
        test_motor_calcAdjustedSpeed,
        test_motor_getChannels,
        test_motor_calculateSpeed,
        test_motor_straight_with_conf,
        test_motor_autonomous,
        test_motor_reset_param,
        test_motor_half_detected,
        test_motor_one_is_closer_detected,
        test_motor_full_detected,
        test_motor_not_detected,
        test_motor_searching,
        test_motor_forward,
        test_motor_backward,
        test_motor_turnLeft,
        test_motor_turnRight,
        test_motor_rotateLeft,
        test_motor_rotateRight,
        test_motor_idle
    ]

    results = []
    for test_func in tests:
        print(f"\n{'='*50}")
        print(f"Running {test_func.__name__}")
        print(f"{'='*50}")
        try:
            result = test_func(motor)
            results.append(result)
        except Exception as e:
            print(f"Test {test_func.__name__} failed with exception: {e}")
            results.append(False)

    # Summary
    print(f"\n{'='*50}")
    print("TEST SUMMARY")
    print(f"{'='*50}")
    for i, test_func in enumerate(tests):
        status = f"{GREEN}PASSED{RESET}" if results[i] else f"{RED}FAILED{RESET}"
        print(f"{test_func.__name__}: {status}")

    all_passed = all(results)
    print(f"\nOVERALL: {GREEN}PASSED{RESET}" if all_passed else f"\nOVERALL: {RED}FAILED{RESET}")

    # Cleanup
    motor.destroy_node()
    node.destroy_node()
    rclpy.shutdown()

    return 0 if all_passed else 1

def main():
    sys.exit(run_all_tests())

if __name__ == '__main__':
    main()