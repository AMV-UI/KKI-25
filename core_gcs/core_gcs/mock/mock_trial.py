import rclpy
from rclpy.node import Node
from core_msgs.msg import Pixhawk
from core.utils.config import Topic, PxMode
from std_msgs.msg import String
import random
import math

class MockPixhawkPublisher(Node):
    def __init__(self):
        super().__init__('mock_pixhawk_publisher')

        self.publisher = Topic.pixhawk.createPublisher(self)

        # Timer publishes every 1 second
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_mock_data)

        # Base southern-most latitude and western-most longitude
        # This is treated as the South-West (bottom-left) corner of the 25x25m square.
        self.base_lat = -6.2088  # southern-most
        self.base_lon = 106.8456  # western-most

        # --- Define the dimensions of the area ---
        self.square_height_meters = 25.0 # North-South dimension
        self.square_width_meters = 23.0  # East-West dimension (max 23m as requested)

        # Convert 25m (height) to degrees latitude
        self.delta_lat_total = self.square_height_meters / 111320
        
        # Convert 23m (width) to degrees longitude
        self.delta_lon_total = self.square_width_meters / (111320 * math.cos(math.radians(self.base_lat)))

        # --- Jitter parameters to make the path less straight ---
        self.jitter_meters = 1  # +/- 20cm of random noise
        self.lat_jitter_deg = self.jitter_meters / 111320
        self.lon_jitter_deg = self.jitter_meters / (111320 * math.cos(math.radians(self.base_lat)))
        # --- End of Jitter ---

        # --- Parameters for Zigzag motion ---
        # Define the corners of the 25m (H) x 23m (W) area
        self.sw_corner = (self.base_lat, self.base_lon)
        self.nw_corner = (self.base_lat + self.delta_lat_total, self.base_lon)
        self.ne_corner = (self.base_lat + self.delta_lat_total, self.base_lon + self.delta_lon_total) # Uses new 23m width
        self.se_corner = (self.base_lat, self.base_lon + self.delta_lon_total) # Uses new 23m width

        # Zigzag path parameters
        self.num_vertical_segments = 5  # Corresponds to 5 rows in the grid image (steps within a vertical pass)
        self.num_horizontal_passes = 6  # Corresponds to 6 vertical lines (A,B,O,D,E,1 in grid)

        self.vertical_step_size_lat = self.delta_lat_total / (self.num_vertical_segments) # How much to move North/South each step
        # How much to move East/West each "jog". (num_passes - 1) gives the number of horizontal gaps.
        self.horizontal_step_size_lon = self.delta_lon_total / (self.num_horizontal_passes - 1) 

        self.current_lat = self.sw_corner[0]
        self.current_lon = self.sw_corner[1]

        self.path_segment_index = 0 # Tracks which vertical segment we are on (0 to 5)
        self.segment_type = "NORTH" # "NORTH", "EAST_JOG_UP", "SOUTH", "EAST_JOG_DOWN"

        self.pxmode_subscriber = Topic.pxmode.createSubscriber(
            self,
            self.pxmode_callback
        )

        self.mode = PxMode.HOLD
        self.counter = 0
        
        # This counter is for stepping *within* a vertical segment
        self.vertical_step_counter = 0

    def pxmode_callback(self, msg: String):
        self.mode = msg.data

    def publish_mock_data(self):
        msg = Pixhawk()
        
        if self.mode == PxMode.HOLD:
            # In HOLD mode, just stay at the base coordinate (SW Corner)
            msg.lat = self.sw_corner[0]
            msg.lon = self.sw_corner[1]
            # Reset zigzag progress when going into HOLD
            self.current_lat = self.sw_corner[0]
            self.current_lon = self.sw_corner[1]
            self.segment_type = "NORTH"
            self.path_segment_index = 0
            self.vertical_step_counter = 0
        else:
            # --- Generate points along the zigzag pattern ---
            # Based on the current segment type, update lat/lon
            
            if self.segment_type == "NORTH":
                # Move one vertical step north
                self.current_lat = self.sw_corner[0] + (self.vertical_step_counter * self.vertical_step_size_lat)
                self.vertical_step_counter += 1
                
                # Check if we've completed the vertical segment
                if self.vertical_step_counter > self.num_vertical_segments:
                    self.current_lat = self.nw_corner[0] # Cap at max lat
                    self.vertical_step_counter = 0 # Reset for next segment
                    self.segment_type = "EAST_JOG_UP" # Move to next segment

            elif self.segment_type == "EAST_JOG_UP":
                # Move one horizontal step east
                self.path_segment_index += 1
                self.current_lon = self.sw_corner[1] + (self.path_segment_index * self.horizontal_step_size_lon)
                self.segment_type = "SOUTH" # Move to next segment
            
            elif self.segment_type == "SOUTH":
                # Move one vertical step south
                self.current_lat = self.nw_corner[0] - (self.vertical_step_counter * self.vertical_step_size_lat)
                self.vertical_step_counter += 1
                
                # Check if we've completed the vertical segment
                if self.vertical_step_counter > self.num_vertical_segments:
                    self.current_lat = self.sw_corner[0] # Cap at min lat
                    self.vertical_step_counter = 0 # Reset for next segment
                    
                    # Check if we are at the last segment
                    if self.path_segment_index >= (self.num_horizontal_passes - 1):
                        # Finished last south segment, reset to start
                        self.current_lat = self.sw_corner[0]
                        self.current_lon = self.sw_corner[1]
                        self.segment_type = "NORTH"
                        self.path_segment_index = 0
                    else:
                        # More zigzags are needed
                        self.segment_type = "EAST_JOG_DOWN" # Move to next segment

            elif self.segment_type == "EAST_JOG_DOWN":
                # Move one horizontal step east
                self.path_segment_index += 1
                self.current_lon = self.sw_corner[1] + (self.path_segment_index * self.horizontal_step_size_lon)
                self.segment_type = "NORTH" # Move to next segment

            # --- Add micro-jitter to the calculated path ---
            # This makes the path look more natural/less straight on a map
            final_lat = self.current_lat + random.uniform(-self.lat_jitter_deg, self.lat_jitter_deg)
            final_lon = self.current_lon + random.uniform(-self.lon_jitter_deg, self.lon_jitter_deg)

            msg.lat = final_lat
            msg.lon = final_lon
            # --- End of zigzag pattern logic ---

        # Keep altitude, speed, and heading random for mock data
        msg.alt = 5.0 + random.uniform(-0.2, 0.2)
        msg.msg_spd = random.uniform(0.5, 3.0)
        # Make heading roughly match the direction
        if self.segment_type == "NORTH":
            msg.msg_heading = 0 + random.randint(-5, 5)
        elif self.segment_type == "SOUTH":
            msg.msg_heading = 180 + random.randint(-5, 5)
        else: # East jogs
            msg.msg_heading = 90 + random.randint(-5, 5)


        self.publisher.publish(msg)
        self.get_logger().info(
            f"Publishing Pixhawk -> "
            f"lat: {msg.lat:.6f}, lon: {msg.lon:.6f}, alt: {msg.alt:.2f}, "
            f"spd: {msg.msg_spd:.2f}, heading: {msg.msg_heading}, "
            f"Segment: {self.segment_type}, H_Index: {self.path_segment_index}, V_Step: {self.vertical_step_counter}"
        )

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockPixhawkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()