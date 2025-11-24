# #!/usr/bin/env python3
# """
# ROS2 Parameter Blackboard Node
# A centralized parameter server that allows multiple nodes to share parameters.
# This node automatically provides /parameter_blackboard/get_parameters and 
# /parameter_blackboard/set_parameters services that other nodes can call.
# """
# import rclpy
# from rclpy.node import Node
# from rcl_interfaces.msg import SetParametersResult


# class ParameterBlackboard(Node):
#     """
#     Centralized parameter server for sharing parameters across multiple nodes.
    
#     This node uses ROS2's built-in parameter system. Every ROS2 node automatically
#     provides get_parameters and set_parameters services, so we just need to:
#     1. Allow undeclared parameters (so any node can set any parameter)
#     2. Add a callback to log parameter changes
#     3. Keep the node alive
    
#     Other nodes can then access parameters via:
#     - /parameter_blackboard/get_parameters service
#     - /parameter_blackboard/set_parameters service
#     """
    
#     def __init__(self):
#         super().__init__(
#             'parameter_blackboard',
#             allow_undeclared_parameters=True,
#             automatically_declare_parameters_from_overrides=True
#         )
        
#         # Add callback to log when parameters are changed
#         self.add_on_set_parameters_callback(self.parameter_callback)
        
#         self.get_logger().info('=' * 60)
#         self.get_logger().info('Parameter Blackboard Node Started')
#         self.get_logger().info('=' * 60)
#         self.get_logger().info('Services available:')
#         self.get_logger().info('  - /parameter_blackboard/get_parameters')
#         self.get_logger().info('  - /parameter_blackboard/set_parameters')
#         self.get_logger().info('  - /parameter_blackboard/list_parameters')
#         self.get_logger().info('  - /parameter_blackboard/describe_parameters')
#         self.get_logger().info('')
#         self.get_logger().info('Allows undeclared parameters: YES')
#         self.get_logger().info('Ready to store and serve shared parameters')
#         self.get_logger().info('=' * 60)
    
#     def parameter_callback(self, params):
#         """
#         Called whenever parameters are set/modified.
#         Logs the changes for debugging purposes.
#         """
#         for param in params:
#             self.get_logger().info(
#                 f"Parameter updated: '{param.name}' = {param.value} "
#                 f"(type: {param.type_.name})"
#             )
        
#         # Always return successful to allow all parameter changes
#         return SetParametersResult(successful=True)


# def main(args=None):
#     rclpy.init(args=args)
    
#     node = ParameterBlackboard()
    
#     try:
#         node.get_logger().info('Spinning... Press Ctrl+C to exit')
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Shutting down Parameter Blackboard')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()