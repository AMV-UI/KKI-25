#!/usr/bin/env python3

import rclpy
import threading
import pytest
import py_trees
from core_behavior_tree.mission_test import BehaviorTreeNode, HeadingSubscriber, DetectedSubscriber, PXModeSubscriber, FindMode
from core.utils.config import Topic, PxMode
import rclpy.executors
from std_msgs.msg import Float64, Bool, String
from rclpy.node import Node
import time
import random
import math


@pytest.fixture(autouse=True)
def initialise_ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def initialise_test_node():
    node = Node("test_node")
    yield node
    if node is not None:
        node.destroy_node()


@pytest.fixture
def initialise_dummy_node():
    node = Node("dummy")
    yield node
    if node is not None:
        node.destroy_node()


@pytest.fixture
def initialise_bt_thread():
    bt_node = BehaviorTreeNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bt_node)
    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )
    executor_thread.start()
    time.sleep(1)  # wait until tree setup is done before yielding
    yield bt_node
    if bt_node is not None:
        bt_node.destroy_node()
    executor.shutdown()
    executor_thread.join()


@pytest.fixture
def initialise_heading_deg_test(initialise_test_node, initialise_dummy_node):
    global_blackboard = py_trees.blackboard.Blackboard
    dummy_heading_node = initialise_dummy_node
    node = initialise_test_node
    heading_deg = random.random()
    heading_deg_behavior = HeadingSubscriber(dummy_heading_node)
    heading_deg_behavior.setup()
    heading_deg_publisher = Topic.heading_deg.createPublisher(node)
    msg = Float64()
    msg.data = heading_deg
    return global_blackboard, node, dummy_heading_node, heading_deg_behavior, \
        heading_deg_publisher, msg


@pytest.fixture
def initialise_detected_obj_test(initialise_test_node, initialise_dummy_node):
    global_blackboard = py_trees.blackboard.Blackboard
    dummy_detected_node = initialise_dummy_node
    node = initialise_test_node
    detected_obj_behavior = DetectedSubscriber(dummy_detected_node)
    detected_obj_behavior.setup()
    global_blackboard.set("detected", False)
    detected_obj_publisher = Topic.object_detected.createPublisher(node)
    msg = Bool()
    msg.data = True
    return global_blackboard, dummy_detected_node, node, detected_obj_behavior, \
        detected_obj_publisher, msg


@pytest.fixture
def initialise_pxmode_test(initialise_test_node, initialise_dummy_node):
    global_blackboard = py_trees.blackboard.Blackboard
    dummy_pxmode_node = initialise_dummy_node
    node = initialise_test_node
    pxmode_behavior = PXModeSubscriber(dummy_pxmode_node)
    pxmode_behavior.setup()
    global_blackboard.set("pxmode", PxMode.HOLD)
    global_blackboard.set("find_mode", FindMode(dummy_pxmode_node))
    global_blackboard.set("px_heading", -1)
    global_blackboard.set("buoy_visited_count", 0)
    pxmode_publisher = Topic.pxmode.createPublisher(node)
    msg = String()
    msg.data = PxMode.HOLD
    return global_blackboard, dummy_pxmode_node, node, pxmode_behavior, \
        pxmode_publisher, msg

def test_bt_subscriptions(initialise_test_node, initialise_bt_thread):
    node = initialise_test_node
    heading_deg_publisher = Topic.heading_deg.createPublisher(node)
    msg = Float64()
    msg.data = 3.4
    heading_deg_publisher.publish(msg)
    time.sleep(1)
    assert 3.4 == py_trees.blackboard.Blackboard.get("px_heading")


def test_heading_deg_behavior_subscription(initialise_heading_deg_test):
    global_blackboard, node, dummy_heading_node, heading_deg_behavior, \
        heading_deg_publisher, msg = initialise_heading_deg_test
    heading_deg_publisher.publish(msg)
    rclpy.spin_once(dummy_heading_node, timeout_sec=0.1)
    heading_deg_behavior.tick()
    assert msg.data == global_blackboard.get("px_heading")
    msg.data = 1.0
    heading_deg_publisher.publish(msg)
    rclpy.spin_once(dummy_heading_node, timeout_sec=0.1)
    heading_deg_behavior.tick()
    assert msg.data == global_blackboard.get("px_heading")


def test_detected_obj_behavior_subscription_and_timestamps(initialise_detected_obj_test):
    global_blackboard, dummy_detected_node, node, detected_obj_behavior, \
        detected_obj_publisher, msg = initialise_detected_obj_test
    global_blackboard.set("detected", False)
    detected_obj_publisher.publish(msg)
    rclpy.spin_once(dummy_detected_node, timeout_sec=0.1)
    detected_obj_behavior.tick()
    expected_time = time.time()
    assert math.isclose(expected_time, global_blackboard.get("last_detected_time"), abs_tol=0.5)
    assert global_blackboard.get("detected")

    detected_obj_publisher.publish(msg)
    rclpy.spin_once(dummy_detected_node, timeout_sec=0.1)
    detected_obj_behavior.tick()
    assert math.isclose(expected_time, global_blackboard.get("last_detected_time"), abs_tol=0.5)
    assert global_blackboard.get("detected")
    
    msg.data = False
    detected_obj_publisher.publish(msg)
    rclpy.spin_once(dummy_detected_node, timeout_sec=0.1)
    detected_obj_behavior.tick()
    assert math.isclose(expected_time, global_blackboard.get("last_detected_time"), abs_tol=0.5)
    assert not global_blackboard.get("detected")
    
    msg.data = True
    global_blackboard.set("detected", False)
    random_delay = random.uniform(3, 5)
    time.sleep(random_delay)
    expected_time += random_delay
    
    detected_obj_publisher.publish(msg)
    rclpy.spin_once(dummy_detected_node, timeout_sec=0.1)
    detected_obj_behavior.tick()
    assert math.isclose(expected_time, global_blackboard.get("last_detected_time"), abs_tol=0.5)
    assert global_blackboard.get("detected")


def test_pxmode_behavior_subscription_and_pxmode_changes(initialise_pxmode_test):
    # TODO finish pxmode behavior test case
    pass