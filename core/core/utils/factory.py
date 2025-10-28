#!/usr/bin/env python3
import py_trees
import rclpy
import rclpy.executors
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
import rclpy.qos
import time


class TopicFactory:
    def __init__(self, topic_name, msg_type, latch=False):
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.latch = latch
        self.qos_profile = 10

    def createPublisher(self, node):
        qos = rclpy.qos.QoSProfile(
            depth=self.qos_profile,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL if self.latch else rclpy.qos.DurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        return node.create_publisher(self.msg_type, self.topic_name, qos)
        
    
    def createSubscriber(self, node, callback):
        qos = rclpy.qos.QoSProfile(depth=self.qos_profile)
        return node.create_subscription(
            self.msg_type,
            self.topic_name,
            callback,
            qos
        )
    
    

    # TODO: FIX ON TEST_FACTORY
    def waitMessage(self, node, timeout_sec=5.0):
        """ROS2-compatible message waiting implementation"""
        future = rclpy.Future()
        received_msg = [None]
        executor = rclpy.executors.SingleThreadedExecutor()

        # Match publisher QoS for latched messages
        qos = rclpy.qos.QoSProfile(
            depth=self.qos_profile,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL if self.latch else rclpy.qos.DurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )

        def callback(msg):
            if not future.done():
                received_msg[0] = msg
                future.set_result(True)

        sub = self.createSubscriber(node, callback)
        executor.add_node(node)

        try:
            # Publish test message to trigger subscription
            test_pub = node.create_publisher(self.msg_type, self.topic_name, qos)
            test_msg = self.msg_type()
            test_msg.data = "TEST_MESSAGE"
            test_pub.publish(test_msg)

            executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
            if future.done() and received_msg[0] is not None:
                return received_msg[0]
            raise TimeoutError(f"Timeout waiting for message on {self.topic_name}")
        finally:
            executor.remove_node(node)
            node.destroy_subscription(sub)
            node.destroy_publisher(test_pub)
            executor.shutdown()



class ServiceFactory:
    def __init__(self, srv_name, srv_type):
        self.srv_name = srv_name
        self.srv_type = srv_type

    def createProxy(self, node):
        return node.create_client(self.srv_type, self.srv_name)

    def handleService(self, node, handler):
        return node.create_service(self.srv_type, self.srv_name, handler)

    def waitService(self, node, timeout_sec=1.0):
        return node.wait_for_service(timeout_sec=timeout_sec)


class ParamFactory:
    """Simplified parameter factory using native ROS 2 parameters"""
    
    def __init__(self, param_name, param_type, param_server_node='/parameter_blackboard'):
        self.param_name = param_name
        self.param_type = param_type
        self.param_server_node = param_server_node
        self.default_value = None

    def createParam(self, node, default_value=None):
        """Declare parameter on the node with correct type (native ROS 2 approach)"""
        self.default_value = default_value
        
        # Create descriptor with proper type info
        descriptor = ParameterDescriptor(
            description=f"Parameter {self.param_name}",
            read_only=False
        )
        
        # Declare with correct type
        if default_value is not None:
            # Type is inferred from default_value, ensure it matches self.param_type
            if self.param_type == float:
                default_value = float(default_value)
            elif self.param_type == int:
                default_value = int(default_value)
            elif self.param_type == bool:
                default_value = bool(default_value)
            elif self.param_type == str:
                default_value = str(default_value)
            
            node.declare_parameter(self.param_name, default_value, descriptor)
            node.get_logger().info(f"Parameter '{self.param_name}' declared as {type(default_value).__name__} with value: {default_value}")
        else:
            node.declare_parameter(self.param_name, descriptor=descriptor)
            node.get_logger().info(f"Parameter '{self.param_name}' declared")

    def setParam(self, node, value):
        """Set parameter on the node with type conversion"""
        try:
            # Ensure value matches the declared type
            if self.param_type == float:
                value = float(value)
            elif self.param_type == int:
                value = int(value)
            elif self.param_type == str:
                value = str(value)
            elif self.param_type == bool:
                value = bool(value)
            
            param = rclpy.parameter.Parameter(self.param_name, value=value)
            result = node.set_parameters([param])
            
            if result[0].successful:
                node.get_logger().debug(f"Parameter '{self.param_name}' set to {value} ({type(value).__name__})")
                return True
            else:
                node.get_logger().error(f"Failed to set parameter '{self.param_name}': {result[0].reason}")
                return False
        except Exception as e:
            node.get_logger().error(f"Error setting parameter '{self.param_name}': {e}")
            return False

    def getParam(self, node):
        """Get parameter value from the node (native ROS 2 approach)"""
        try:
            param = node.get_parameter(self.param_name)
            return param.get_parameter_value()
        except Exception as e:
            node.get_logger().warning(f"Parameter '{self.param_name}' not found: {e}")
            return None

    def getValue(self, node):
        """Get the actual parameter value (extracted from ParameterValue)"""
        try:
            param = node.get_parameter(self.param_name)
            param_value = param.get_parameter_value()
            
            if param_value.type == ParameterType.PARAMETER_BOOL:
                return param_value.bool_value
            elif param_value.type == ParameterType.PARAMETER_INTEGER:
                return param_value.integer_value
            elif param_value.type == ParameterType.PARAMETER_DOUBLE:
                return param_value.double_value
            elif param_value.type == ParameterType.PARAMETER_STRING:
                return param_value.string_value
            elif param_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
                return list(param_value.byte_array_value)
            elif param_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
                return list(param_value.bool_array_value)
            elif param_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                return list(param_value.integer_array_value)
            elif param_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                return list(param_value.double_array_value)
            elif param_value.type == ParameterType.PARAMETER_STRING_ARRAY:
                return list(param_value.string_array_value)
            
            return self.default_value
        except Exception as e:
            node.get_logger().warning(f"Could not get value for '{self.param_name}': {e}")
            return self.default_value