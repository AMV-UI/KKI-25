#!/usr/bin/env python3

import traceback
import cv2
import rclpy
import numpy as np
import base64
import time
from datetime import datetime
from core.perception.image.inference import ObjectDetector
from core_msgs.msg import StateObject, AutoControl
from core.utils.config import NodeConfig, Topic, MissionStatus
from core.utils.device_fetching import get_webcam_device_idx
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
class CameraController(Node):
    """
    Front Camera Node for Object Detection
    
    PUBLISHES TO:
        - dsc: Float64 - Yaw control effort
        - detected: Bool - Detection status
    """

    def __init__(self):
        super().__init__("front_camera")

        self.arena = "B"

        # self.up_camera_serial_idx = get_webcam_device_idx('046d_C270_HD_WEBCAM_E0198440')
        # self.down_camera_serial_idx = get_webcam_device_idx('Generic_HD_camera_20201212000000')

        self.buoy_detector = ObjectDetector(
            "/home/apenchu/Downloads/best.pt",
            self,
            [
                "green_buoy",
                "red_buoy",
            ],
        )

        self.box_detector = ObjectDetector(
            "/home/apenchu/Downloads/box_v1.pt",
            self,
            [
                "blueBox",
                "greenBox",
            ],
        )

        # self.up_cap = cv2.VideoCapture(self.up_camera_serial_idx)
        # self.up_cap.set(1, 30)  # Set FPS
        # self.up_cap.set(3, 640)  # Set width
        # self.up_cap.set(4, 480)  # Set height

        # Configure down camera
        # self.down_cap = cv2.VideoCapture(self.down_camera_serial_idx)
        # self.down_cap.set(1, 30)  # Set FPS
        # self.down_cap.set(3, 640)  # Set width
        # self.down_cap.set(4, 480)  # Set height

        self.up_cap = None
        self.down_cap = None

        # Simulation subscriber
        self.bridge = CvBridge()
        self.sim_img = None
        self.sim_bottom_img = None
        self.first_image_received = False
        self.create_subscription(Image, '/sonobot/camera/image_color', self.sim_camera_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/blueboat/camera_bottom/image_color', self.sim_bottom_camera_cb, qos_profile_sensor_data)

        self.REC_UP_CAMERA = False
        self.REC_DOWN_CAMERA = False

        if self.REC_UP_CAMERA:
            fps = 15
            size = int(self.up_cap.get(3)), int(self.up_cap.get(4))
            file_name = "core_perception/videos/" + time.strftime("%d-%m-%Y_%H:%M:%S_up_cam.mp4", time.localtime())
            self.up_vid_writer = cv2.VideoWriter(file_name, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if self.REC_DOWN_CAMERA:
            fps = 15
            size = int(self.down_cap.get(3)), int(self.down_cap.get(4))
            file_name = "core_perception/videos/" + time.strftime("%d-%m-%Y_%H:%M:%S_down_cam.mp4", time.localtime())
            self.down_vid_writer = cv2.VideoWriter(file_name, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)

        self.result = ""
        self.dsc = 0
        self.state = [0, 0, 0, 0]
        self.img = None
        self.img_64 = ""
        self.mission_type = MissionStatus.BUOY
        self.show_result = False
        self.detected = False
        self.fps = 30

        # Setup communication
        self._setup_communication()
        
        self.get_logger().info(f"<> [{NodeConfig.camera_front}] Successfully initialized node")


    def _get_rc6_state(self, rc6_value):
        """Determine RC6 button state based on value"""
        if rc6_value < self.PWM_LOW:
            return 'LOW'
        elif self.PWM_LOW <= rc6_value < self.PWM_HIGH:
            return 'MID'
        else:
            return 'HIGH'
        
    def _take_upper_photo(self):
        if self.sim_img is not None:
            frame = self.sim_img.copy()
            timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            photo_filename = f"/home/apenchu/hobi/ros2_ws/src/KKI-25/core_perception/photos/{timestamp}_upCamera.jpg"
            cv2.imwrite(photo_filename, frame)
            self.get_logger().info(f"Photo taken and saved to {photo_filename}", throttle_duration_sec=5.0)
            self.green_box_pub.publish(self.encode_base64(frame))
        else:
            self.get_logger().error("Failed to capture image from upper camera")

    def _take_down_photo(self):
        if self.sim_bottom_img is not None:
            frame = self.sim_bottom_img.copy()
            timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            photo_filename = f"/home/apenchu/hobi/ros2_ws/src/KKI-25/core_perception/photos/{timestamp}_downCamera.jpg"
            cv2.imwrite(photo_filename, frame)
            self.get_logger().info(f"Photo taken and saved to {photo_filename}", throttle_duration_sec=5.0)
            self.blue_box_pub.publish(self.encode_base64(frame))
        else:
            self.get_logger().error("Failed to capture image from down camera")

    def _rc6_callback(self, msg):
        """Handle RC6 commands for photo capture"""
        self.rc6 = msg.data
        current_state = self._get_rc6_state(self.rc6)
        if current_state == 'MID' and self.rc6_state != 'MID':
            self._take_upper_photo()
        elif current_state == 'HIGH' and self.rc6_state != 'HIGH':
            self._take_down_photo()
        self.rc6_state = current_state

    def _setup_communication(self):
        """Initialize publishers and subscribers"""
        # Publishers
        self.dsc_pub = Topic.dsc.createPublisher(self)
        self.detected_pub = Topic.detected.createPublisher(self)
        self.camera_processed_pub = Topic.camera_processed.createPublisher(self)
        self.green_box_pub = Topic.green_box_encoded.createPublisher(self)
        self.blue_box_pub = Topic.blue_box_encoded.createPublisher(self)
        self.box_detected_pub = Topic.box_detected.createPublisher(self)
        # Subscribers (if needed)
        self.mission_type_sub = Topic.mission_type.createSubscriber(self, self.mission_callback)
        self.arena_sub = Topic.arena.createSubscriber(self, self._arena_cb)

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def mission_callback(self, msg):
        """Update current mission"""
        self.mission_type = str(msg.data)
        self.get_logger().info(f"Mission changed to: {self.mission_type}", throttle_duration_sec=3.0)

    def get_data(self):
        return self.result, self.dsc, self.state

    def visualize(self, scale=0.6):
        """Display annotated frame"""
        if self.img is None:
            return False

        display_img = cv2.resize(
            self.img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
        )
        cv2.imshow("Annotated Output", display_img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Exit requested by user (q pressed).")
            return True
        return False

    def encode_base64(self, img):
        result, encoded_image = cv2.imencode(
            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 20]
        )
        if result:
            base64_image = base64.b64encode(encoded_image).decode("utf-8")
            img_msg = String()
            img_msg.data = base64_image
            return img_msg
        else:
            self.get_logger().error("Failed to encode frame to JPG")
            return ""

    def sim_camera_cb(self, msg):
        try:
            self.sim_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if not self.first_image_received:
                self.get_logger().info("First image received from simulation!")
                self.first_image_received = True
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            
    def sim_bottom_camera_cb(self, msg):
        try:
            self.sim_bottom_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting bottom image: {e}")
 

    def process_frame(self):
        """Process a single frame - called by timer"""
        try:
            if self.sim_img is not None:
                img = self.sim_img.copy()
                success = True
            else:
                success = False
                img = None
                
            if self.sim_bottom_img is not None:
                bottom_img = self.sim_bottom_img.copy()
            else:
                bottom_img = None

            if not success:
                return

            box_detected_bool = False
            if self.mission_type == MissionStatus.BUOY or self.mission_type == MissionStatus.DOCKING:
                self.img, self.dsc, self.detected = self.buoy_detector.process_frame(self.mission_type, self.arena, img, self.up_cap, self)
                # Run box detector simultaneously to check for boxes during Buoy mission
                # We pass self.img so the box bounding boxes are drawn on the same image for GCS!
                self.img, _, box_detected_bool = self.box_detector.process_frame(MissionStatus.BOTH_BOXES, self.arena, self.img, self.up_cap, self)
            else:
                self.img, self.dsc, self.detected = self.box_detector.process_frame(self.mission_type, self.arena, img, self.up_cap, self)
                
            self.box_detected_pub.publish(Bool(data=box_detected_bool))

            if self.img is None:
                self.get_logger().warn("Failed to get frame", throttle_duration_sec=5.0)
                return

            # Visualize if enabled
            if self.show_result:
                exit_status = self.visualize()
                if exit_status:
                    rclpy.shutdown()
                    return

            dsc_msg = Float64()
            dsc_msg.data = float(self.dsc)
            self.dsc_pub.publish(dsc_msg)
            
            detected_msg = Bool()
            detected_msg.data = self.detected
            self.detected_pub.publish(detected_msg)

            self.get_logger().info(
                f"DSC: {self.dsc:.2f}, Detected: {self.detected}",
                throttle_duration_sec=2.0
            )

            # Passing image data
            top_camera = self.encode_base64(self.img)
            if bottom_img is not None:
                bot_camera = self.encode_base64(bottom_img)
            else:
                bot_camera = top_camera # fallback
            
            self.camera_processed_pub.publish(top_camera)

            # Combine both camera images to send in one String message
            combined_msg = String()
            combined_msg.data = top_camera.data + "|||" + bot_camera.data

            if(self.mission_type == MissionStatus.GREEN_BOX and self.detected):
                self.green_box_pub.publish(combined_msg)

            if(self.mission_type == MissionStatus.BLUE_BOX and self.detected):
                self.blue_box_pub.publish(combined_msg)

        except Exception as e:
            self.get_logger().error(f"Error in process_frame: {traceback.format_exc()}")


    def run(self):
        """Start the main execution loop"""
        # timer for frame processing (30 FPS = 0.033s)
        self.timer = self.create_timer(1/self.fps, self.process_frame)
        self.get_logger().info("Front camera processing started at 30 FPS")


def main():
    rclpy.init()
    front_cam = CameraController()

    try:
        front_cam.run()
        rclpy.spin(front_cam)
        
    except KeyboardInterrupt:
        front_cam.get_logger().info("Shutting down front camera node...")
    except Exception as e:
        front_cam.get_logger().error(f"Error: {traceback.format_exc()}")
    finally:
        # Cleanup
        # front_cam.up_cap.release()
        # front_cam.down_cap.release()
        if front_cam.REC_UP_CAMERA:
            front_cam.up_vid_writer.release()
        if front_cam.REC_DOWN_CAMERA:
            front_cam.down_vid_writer.release()
        cv2.destroyAllWindows()
        front_cam.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
