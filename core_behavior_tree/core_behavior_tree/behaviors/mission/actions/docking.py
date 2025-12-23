# from ..mission_behaviors import BaseExecution, BaseFallback
# from py_trees.common import Status
# from std_msgs.msg import Float64
# from core.utils.config import Topic, Param
# from core.mission.docking import Docking
# from core_msgs.msg import Pixhawk
# import time


# class DockingMission_Execution(BaseExecution):
#     """
#     Docking Mission: Navigate back to saved docking coordinates
#     - Uses GPS-based navigation with PID control
#     - Arrives when within threshold distance
#     """
#     def __init__(self, name: str = "DockingMission_Execution", node=None):
#         super().__init__(name, node=node)
#         self.node = node
#         self.docking = None
#         self.pixhawk = Pixhawk()
#         self.target_lat = 0.0
#         self.target_lon = 0.0
#         self.current_lat = 0.0
#         self.current_lon = 0.0
#         self.current_heading = 0.0
#         self.last_time = None
#         self.arrival_threshold = 2.0  # meters
        
#     def setup(self, **kwargs) -> None:
#         super().setup(**kwargs)
        
#         self.target_lat = Param.DOCKING_LAT.getValue(self.node)
#         self.target_lon = Param.DOCKING_LON.getValue(self.node)
        
#         self.node.get_logger().info(
#             f"[{self.name}] Docking target: LAT {self.target_lat}, LON {self.target_lon}"
#         )
        
#         self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
#         self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
        
#         self.pixhawk_sub = Topic.pixhawk.createSubscriber(self.node, self._pixhawk_cb)
#         self.heading_sub = Topic.heading_deg.createSubscriber(self.node, self._heading_cb)
        
#         self.last_time = time.time()
        
#     def _pixhawk_cb(self, msg: Pixhawk):
#         self.pixhawk = msg
#         self.current_lat = msg.lat
#         self.current_lon = msg.lon
        
#     def _heading_cb(self, msg: Float64):
#         self.current_heading = float(msg.data)
    
#     def execute(self) -> Status:
#         if self.docking is None:
#             if abs(self.current_lat) < 0.1 or abs(self.current_lon) < 0.1:
#                 self.node.get_logger().warn(
#                     f"[{self.name}] Waiting for valid GPS position...",
#                     throttle_duration_sec=2.0
#                 )
#                 return Status.RUNNING
                
#             self.docking = Docking(
#                 self.current_lat, self.current_lon,
#                 self.target_lat, self.target_lon
#             )
#             self.node.get_logger().info(f"[{self.name}] Docking controller initialized")
        
#         self.docking.update_position(self.current_lat, self.current_lon)
#         distance = self.docking.get_distance_to_target()
        
#         if distance <= self.arrival_threshold:
#             self.node.get_logger().info(
#                 f"[{self.name}] ARRIVED at docking station! Distance: {distance:.2f}m"
#             )
#             # Stop the vehicle
#             self.yaw_effort_pub.publish(Float64(data=1500.0))
#             self.speed_effort_pub.publish(Float64(data=1500.0))
#             return Status.SUCCESS
        
#         current_time = time.time()
#         dt = current_time - self.last_time
#         self.last_time = current_time

#         yaw_effort = self.docking.calculate_yaw_effort(self.current_heading, dt)
        
#         if distance > 10.0:
#             speed_effort = 1700.0  # Fast
#         elif distance > 5.0:
#             speed_effort = 1600.0  # Medium
#         else:
#             speed_effort = 1550.0  # Slow approach
        
#         self.yaw_effort_pub.publish(Float64(data=float(yaw_effort)))
#         self.speed_effort_pub.publish(Float64(data=speed_effort))
        
#         # Calculate bearing for logging
#         bearing = self.docking.calculate_bearing(
#             self.current_lat, self.current_lon,
#             self.target_lat, self.target_lon
#         )
        
#         self.node.get_logger().info(
#             f"[{self.name}] Distance: {distance:.2f}m | "
#             f"Bearing: {bearing:.1f}° | "
#             f"Heading: {self.current_heading:.1f}° | "
#             f"Error: {self.docking.error:.1f}° | "
#             f"Yaw: {yaw_effort}",
#             throttle_duration_sec=1.0
#         )
        
#         return Status.RUNNING


# class DockingMission_Fallback(BaseFallback):
#     """
#     Fallback for docking mission - hold position if GPS lost
#     """
#     def __init__(self, name: str = "DockingMission_Fallback", node=None):
#         super().__init__(name, node=node)
#         self.node = node
        
#     def setup(self, **kwargs) -> None:
#         super().setup(**kwargs)
#         self.yaw_effort_pub = Topic.yaw_effort.createPublisher(self.node)
#         self.speed_effort_pub = Topic.speed_effort.createPublisher(self.node)
    
#     def fallback(self) -> Status:
#         # Hold position - idle efforts
#         self.yaw_effort_pub.publish(Float64(data=1500.0))
#         self.speed_effort_pub.publish(Float64(data=1500.0))
        
#         self.node.get_logger().warn(
#             f"[{self.name}] GPS signal lost - holding position",
#             throttle_duration_sec=2.0
#         )
        
#         return Status.RUNNING