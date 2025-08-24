#!/usr/bin/env python3
import py_trees
import rclpy
import rclpy.executors
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
import rclpy.qos


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
            durability=self.Durability_policy,
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
    def __init__(self, param_name, param_type):
        self.param_name = param_name
        self.param_type = param_type
        self._param_client = None 
        self.default_value = None

    def createParam(self, node, default_value=None):
        self.default_value = default_value
        descriptor = ParameterDescriptor(
            name=self.param_name,
            type=self._get_parameter_type(),
            description=f'{self.param_name} parameter',
            read_only=False
        )
        node.declare_parameter(
            self.param_name, 
            default_value,
            descriptor
        )

    def getParam(self, node):
        '''
        if not self._param_client:
            self._param_client = node.create_client(GetParameters, '/parameter_blackboard/get_parameters')
            
        request = GetParameters.Request()
        request.names = [self.param_name]
        future = self._param_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        return future.result().values[0]._value
        '''
        if not node.has_parameter(self.param_name):
            self._param_client = node.declare_parameter(self.param_name)
        
        return node.get_parameter(self.param_name)

    def setParam(self, node, value):
        set_client = node.create_client(SetParameters, '/parameter_blackboard/set_parameters')
        param = Parameter()
        param.name = self.param_name
        param.value = self._create_parameter_value(value)
        
        request = SetParameters.Request()
        request.parameters = [param]
        future = set_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

    def _create_parameter_value(self, value):
        pv = ParameterValue()
        
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = value
        elif isinstance(value, str):
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = value
        elif isinstance(value, list):
            if all(isinstance(i, int) and 0 <= i <= 255 for i in value):
                pv.type = ParameterType.PARAMETER_BYTE_ARRAY
                pv.byte_array_value = bytes(value)
            elif all(isinstance(i, bool) for i in value):
                pv.type = ParameterType.PARAMETER_BOOL_ARRAY
                pv.bool_array_value = value
            elif all(isinstance(i, int) for i in value):
                pv.type = ParameterType.PARAMETER_INTEGER_ARRAY
                pv.integer_array_value = list(value)
            elif all(isinstance(i, float) for i in value):
                pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                pv.double_array_value = list(value)
            elif all(isinstance(i, str) for i in value):
                pv.type = ParameterType.PARAMETER_STRING_ARRAY
                pv.string_array_value = list(value)
            else:
                raise ValueError(f"Unsupported array type for parameter {self.param_name}")
        else:
            raise TypeError(f"Unsupported type {type(value)} for parameter {self.param_name}")
    
        return pv

    def _get_parameter_type(self):
        type_map = {
            bool: ParameterType.PARAMETER_BOOL,
            int: ParameterType.PARAMETER_INTEGER,
            float: ParameterType.PARAMETER_DOUBLE,
            str: ParameterType.PARAMETER_STRING,
            list: {
                bytes: ParameterType.PARAMETER_BYTE_ARRAY,
                bool: ParameterType.PARAMETER_BOOL_ARRAY,
                int: ParameterType.PARAMETER_INTEGER_ARRAY,
                float: ParameterType.PARAMETER_DOUBLE_ARRAY,
                str: ParameterType.PARAMETER_STRING_ARRAY
            }
        }
        
        # Handle Array Types
        if self.param_type == list:
            if self.default_value:
                element_type = type(self.default_value[0])
                return type_map[list].get(element_type, ParameterType.PARAMETER_NOT_SET)
            return ParameterType.PARAMETER_STRING_ARRAY
        
        return type_map.get(self.param_type, ParameterType.PARAMETER_NOT_SET)
