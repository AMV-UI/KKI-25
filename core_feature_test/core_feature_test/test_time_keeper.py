#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from core_feature_test.target.time_keeper import TimeKeeper
import sys
import time

GREEN = '\033[92m'
RED = '\033[91m'
RESET = '\033[0m'

def test_time_keeper_set_time(time_keeper):
    print("Testing set_time...")
    time_keeper.set_time()
    if time_keeper.is_started():
        print(f"{GREEN}✓ test_set_time PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_set_time FAILED{RESET}")
        return False

def test_time_keeper_reset(time_keeper):
    print("Testing reset...")
    time_keeper.set_time()
    time_keeper.reset()
    if not time_keeper.is_started():
        print(f"{GREEN}✓ test_reset PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_reset FAILED{RESET}")
        return False

def test_time_keeper_is_started(time_keeper):
    print("Testing is_started...")
    time_keeper.set_time()
    if time_keeper.is_started():
        print(f"{GREEN}✓ test_is_started PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_is_started FAILED{RESET}")
        return False

def test_time_keeper_get_time(time_keeper):
    print("Testing get_time...")
    time_keeper.set_time()
    if time_keeper.get_time() is not None:
        print(f"{GREEN}✓ test_get_time PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_get_time FAILED{RESET}")
        return False

def test_time_keeper_check_delay(time_keeper):
    print("Testing check_delay...")
    time_keeper.set_time()
    time.sleep(time_keeper.delay + 1)
    if time_keeper.check_delay():
        print(f"{GREEN}✓ test_check_delay PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_check_delay FAILED{RESET}")
        return False

def test_time_keeper_check_finish(time_keeper):
    print("Testing check_finish...")
    time_keeper.set_time()
    time.sleep(time_keeper.finish + 1)
    if time_keeper.check_finish():
        print(f"{GREEN}✓ test_check_finish PASSED{RESET}")
        return True
    else:
        print(f"{RED}✗ test_check_finish FAILED{RESET}")
        return False

def run_all_tests():
    # Initialize ROS
    rclpy.init()
    node = Node('test_time_keeper_node')
    time_keeper = TimeKeeper(delay=5, finish=10)

    # Run tests
    tests = [
        test_time_keeper_set_time,
        test_time_keeper_reset,
        test_time_keeper_is_started,
        test_time_keeper_get_time,
        test_time_keeper_check_delay,
        test_time_keeper_check_finish
    ]

    results = []
    for test_func in tests:
        print(f"\n{'='*50}")
        print(f"Running {test_func.__name__}")
        print(f"{'='*50}")
        try:
            result = test_func(time_keeper)
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
    time_keeper.destroy_node()
    node.destroy_node()
    rclpy.shutdown()

    return 0 if all_passed else 1

def main():
    sys.exit(run_all_tests())

if __name__ == '__main__':
    main()