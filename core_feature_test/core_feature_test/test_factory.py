#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import sys
import time

# Import the factory classes
from core_feature_test.target.factory import (
    TopicFactory,
    ServiceFactory,
    ParamFactory
)

def test_topic_factory(node):
    print("Testing TopicFactory...")
    topic_name = 'test_topic'
    msg_type = String
    factory = TopicFactory(topic_name, msg_type)
    
    # Track received messages
    received_msgs = []
    def callback(msg):
        received_msgs.append(msg.data)
        print(f"Received message: {msg.data}")
    
    # Create publisher and subscriber
    publisher = factory.createPublisher(node)
    subscriber = factory.createSubscriber(node, callback)
    
    # Give time for subscriber to register
    time.sleep(0.5)
    
    # Publish a message
    test_msg = String()
    test_msg.data = "Hello ROS2"
    publisher.publish(test_msg)
    
    # Process callbacks
    for _ in range(5):  # Spin multiple times to ensure message receipt
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Verify receipt
    if "Hello ROS2" in received_msgs:
        print("✓ TopicFactory publisher/subscriber test passed")
    else:
        print("✗ TopicFactory publisher/subscriber test failed - message not received")
        return False
    
    # Test latched mode
    print("Testing latched mode...")
    latched_topic = 'test_latched_topic'
    latched_factory = TopicFactory(latched_topic, msg_type, latch=True)
    latched_pub = latched_factory.createPublisher(node)
    
    # Check QoS settings
    if latched_pub.qos_profile.durability == rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL:
        print("✓ TopicFactory latched mode test passed")
    else:
        print("✗ TopicFactory latched mode test failed - incorrect durability setting")
        return False
    
    # Clean up
    node.destroy_publisher(publisher)
    node.destroy_subscription(subscriber)
    node.destroy_publisher(latched_pub)
    return True

def test_service_factory(node):
    print("Testing ServiceFactory...")
    service_name = 'test_service'
    service_type = SetParameters
    factory = ServiceFactory(service_name, service_type)
    
    # Track service calls
    call_count = [0]
    
    # Define service handler
    def handler(request, response):
        call_count[0] += 1
        print(f"Service called with {len(request.parameters)} parameters")
        # Add a dummy result
        response.results = [True]
        return response
    
    # Create service and client
    server = factory.handleService(node, handler)
    client = factory.createProxy(node)
    
    # Wait for service
    if not client.wait_for_service(timeout_sec=2.0):
        print("✗ ServiceFactory test failed - service not available")
        return False
    else:
        print("Service available")
    
    # Create request
    request = SetParameters.Request()
    param = Parameter()
    param.name = "test_param"
    param_value = ParameterValue()
    param_value.type = ParameterType.PARAMETER_BOOL
    param_value.bool_value = True
    param.value = param_value
    request.parameters = [param]
    
    # Send request
    future = client.call_async(request)
    
    # Wait for response
    timeout = 5.0  # 5 seconds timeout
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            break
    
    # Check results
    if not future.done():
        print("✗ ServiceFactory test failed - response timeout")
        return False
    
    response = future.result()
    if call_count[0] == 1 and response.results[0] == True:
        print("✓ ServiceFactory test passed")
    else:
        print(f"✗ ServiceFactory test failed - call count: {call_count[0]}, response: {response.results}")
        return False
    
    # Clean up
    node.destroy_service(server)
    node.destroy_client(client)
    return True

def test_param_factory(node):
    print("Testing ParamFactory...")
    
    # Test basic parameter creation
    param_name = 'test_param'
    param_value = 42
    factory = ParamFactory(param_name, int)
    
    factory.createParam(node, default_value=param_value)
    
    # Verify
    if not node.has_parameter(param_name):
        print("✗ ParamFactory test failed - parameter not created")
        return False
    
    retrieved_value = node.get_parameter(param_name).value
    if retrieved_value != param_value:
        print(f"✗ ParamFactory test failed - value mismatch. Expected {param_value}, got {retrieved_value}")
        return False
    else:
        print("✓ ParamFactory basic test passed")
    
    # Test different parameter types
    test_cases = [
        ('int_param', 42, int),
        ('float_param', 3.14, float),
        ('bool_param', True, bool),
        ('string_param', 'hello', str),
        ('int_list_param', [1, 2, 3], list)
    ]
    
    all_passed = True
    for name, value, param_type in test_cases:
        try:
            type_factory = ParamFactory(name, param_type)
            type_factory.createParam(node, default_value=value)
            
            # Verify
            if not node.has_parameter(name):
                print(f"✗ ParamFactory test failed for {param_type.__name__} - parameter not created")
                all_passed = False
                continue
            
            retrieved = node.get_parameter(name).value
            if isinstance(value, list):
                match = list(retrieved) == value
            else:
                match = retrieved == value
                
            if not match:
                print(f"✗ ParamFactory test failed for {param_type.__name__} - value mismatch")
                all_passed = False
            else:
                print(f"✓ ParamFactory {param_type.__name__} test passed")
        except Exception as e:
            print(f"✗ ParamFactory test failed for {param_type.__name__} with error: {e}")
            all_passed = False
    
    # Test parameter update
    update_name = 'update_param'
    initial_value = 50
    updated_value = 100
    
    update_factory = ParamFactory(update_name, int)
    update_factory.createParam(node, default_value=initial_value)
    
    # Verify initial value
    initial = node.get_parameter(update_name).value
    if initial != initial_value:
        print(f"✗ ParamFactory update test failed - initial value mismatch")
        return False
    
    # Update using parameter service
    try:
        # We'll update directly using the node API since parameter_blackboard may not exist
        param = Parameter(
            name=update_name,
            value=updated_value
        )
        node.set_parameters([param])
        
        # Verify updated value
        retrieved = node.get_parameter(update_name).value
        if retrieved != updated_value:
            print(f"✗ ParamFactory update test failed - updated value mismatch")
            return False
        else:
            print("✓ ParamFactory update test passed")
    except Exception as e:
        print(f"✗ ParamFactory update test failed with error: {e}")
        return False
    
    return all_passed

def run_all_tests():
    # Initialize ROS
    rclpy.init()
    node = Node('test_factory_node')
    
    # Run tests
    tests = [
        test_topic_factory,
        test_service_factory,
        test_param_factory
    ]
    
    results = []
    for test_func in tests:
        print(f"\n{'='*50}")
        print(f"Running {test_func.__name__}")
        print(f"{'='*50}")
        try:
            result = test_func(node)
            results.append(result)
        except Exception as e:
            print(f"Test {test_func.__name__} failed with exception: {e}")
            results.append(False)
    
    # Summary
    print(f"\n{'='*50}")
    print("TEST SUMMARY")
    print(f"{'='*50}")
    for i, test_func in enumerate(tests):
        status = "PASSED" if results[i] else "FAILED"
        print(f"{test_func.__name__}: {status}")
    
    all_passed = all(results)
    print(f"\nOVERALL: {'PASSED' if all_passed else 'FAILED'}")
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    
    return 0 if all_passed else 1

def main():
    sys.exit(run_all_tests())