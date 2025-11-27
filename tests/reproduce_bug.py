
import sys
import os
import math
from unittest.mock import MagicMock

# Mock rclpy and messages
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['core_msgs.msg'] = MagicMock()
sys.modules['core.utils.config'] = MagicMock()

# Mock Node class
class MockNode:
    def __init__(self, name):
        self.name = name
    def get_logger(self):
        logger = MagicMock()
        logger.info = print
        logger.warn = print
        logger.error = print
        return logger
    def create_subscription(self, *args): return MagicMock()
    def create_publisher(self, *args): return MagicMock()
    def create_timer(self, *args): return MagicMock()

sys.modules['rclpy.node'].Node = MockNode

# Now import the classes
# We need to make sure the imports in movement_controller work
# We need to set up the python path
sys.path.append('d:\\playground\\KKI-25\\core_control')
sys.path.append('d:\\playground\\KKI-25\\core')

from core_control.movement_controller import MovementController, RecordingState
from core.mission.docking import DockingController

def test_playback_transition():
    print("Initializing MovementController...")
    mc = MovementController()
    
    # Setup some dummy waypoints
    # Point A: (0, 0)
    # Point B: (0.0001, 0) ~ 11m East
    # Point C: (0.0001, 0.0001) ~ 11m North of B
    
    mc.playback_lat_lon = [
        (0.0001, 0.0, 1.0),
        (0.0001, 0.0001, 1.0)
    ]
    mc.recording_state = RecordingState.PLAYING_BACK
    mc.playback_index = 0
    mc.docking_enabled = False
    
    # Initial position close to target
    # Target is (0.0001, 0.0) ~ 11m North (wait, lat is Y?)
    # Lat is Y, Lon is X usually.
    # 0.0001 lat ~ 11m North.
    # Start at 0.00008 lat ~ 8.8m North. Dist ~ 2.2m.
    mc.current_lat = 0.00008
    mc.current_lon = 0.0
    mc.current_heading = 0.0 # North
    mc.last_update_time = 0.0
    
    # Loop
    print("\nStarting Loop:")
    old_index = mc.playback_index
    for i in range(50):
        # Simulate time passing
        mc.last_update_time = i * 0.02
        
        # Calculate efforts
        # We need to mock time() inside calculate_control_efforts?
        # No, it calls time(). We can patch it or just let it use system time.
        # But system time is moving.
        # Let's just patch time.time to return controlled values
        
        import time
        time.time = MagicMock(return_value=(i+1) * 0.02)
        
        yaw, speed = mc.calculate_control_efforts()
        
        dist = mc.docking_controller.get_distance_to_target(mc.current_lat, mc.current_lon)
        print(f"Iter {i}: Pos=({mc.current_lat:.6f}, {mc.current_lon:.6f}), "
              f"TargetIdx={mc.playback_index}, Dist={dist:.2f}, "
              f"Yaw={yaw:.2f}, Speed={speed:.2f}")
        
        # Simulate movement (simple kinematic model)
        # Speed 300 ~ 1 m/s? Let's assume 1 unit = 1 m/s for simplicity
        # Actually speed_effort is PWM. Let's assume linear mapping.
        # Max speed 300.
        
        actual_speed = speed / 300.0 * 5.0 # 5 m/s max
        actual_yaw_rate = yaw / 300.0 * 1.0 # 1 rad/s max
        
        mc.current_heading += actual_yaw_rate * 0.02 * 57.29 # degrees
        mc.current_heading = (mc.current_heading + 360) % 360
        
        heading_rad = math.radians(mc.current_heading)
        mc.current_lat += (actual_speed * 0.02 * math.cos(heading_rad)) / 111320.0
        mc.current_lon += (actual_speed * 0.02 * math.sin(heading_rad)) / (111320.0 * math.cos(math.radians(mc.current_lat)))
        
        # Check if we switched waypoint
        if i > 0 and mc.playback_index > old_index:
            print(f"*** SWITCHED WAYPOINT at Iter {i} ***")
            
        old_index = mc.playback_index

if __name__ == "__main__":
    test_playback_transition()
