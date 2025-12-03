#!/usr/bin/env python3

import traceback
import cv2
import rclpy
import numpy as np
import base64
import time
from datetime import datetime
from core_msgs.msg import StateObject, AutoControl
from core.utils.config import NodeConfig, Topic, MissionStatus
from core.utils.device_fetching import *
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String

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
        self.simulation = True

        if self.simulation:
            self.up_camera_serial_idx = "/mnt/d/playground/KKI-25/test_script/lain.mp4"
            self.down_camera_serial_idx = "/mnt/d/playground/KKI-25/test_script/lain.mp4"
        else:
            from core.perception.image.inference import ObjectDetector
            self.up_camera_serial_idx = get_webcam_device_idx('046d_C270_HD_WEBCAM_E0198440')
            self.down_camera_serial_idx = get_webcam_device_idx('Generic_HD_camera_20201212000000')


            self.buoy_detector = ObjectDetector(
                "/home/amv/models/KKI-25/buoy_v1.engine",
                self,
                [
                    "green_buoy",
                    "red_buoy",
                ],
            )


        self.up_cap = cv2.VideoCapture(self.up_camera_serial_idx)
        self.up_cap.set(1, 30)  # Set FPS
        self.up_cap.set(3, 640)  # Set width
        self.up_cap.set(4, 480)  # Set height

        # Configure down camera
        self.down_cap = cv2.VideoCapture(self.down_camera_serial_idx)
        self.down_cap.set(1, 30)  # Set FPS
        self.down_cap.set(3, 640)  # Set width
        self.down_cap.set(4, 480)  # Set height

        self.result = ""
        self.dsc = 0
        self.state = [0, 0, 0, 0]
        self.img = None
        self.img_64 = ""
        self.show_result = False
        self.detected = False
        self.fps = 30

        self.rc6 = 0.0
        self.rc6_state = 'LOW'  # Possible states: LOW, MID, HIGH

        self.PWM_LOW = 1000
        self.PWM_HIGH = 1700

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
        ret, frame = self.up_cap.read()
        if ret:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            photo_filename = f"/home/amv/KKI-25/core_perception/photos/{timestamp}_upCamera.jpg"
            cv2.imwrite(photo_filename, frame)
            self.get_logger().info(f"Photo taken and saved to {photo_filename}", throttle_duration_sec=5.0)
            self.green_box_pub.publish(self.encode_base64(frame))
        else:
            self.get_logger().error("Failed to capture image from upper camera")

    def _take_down_photo(self):
        ret, frame = self.down_cap.read()
        if ret:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            photo_filename = f"/home/amv/KKI-25/core_perception/photos/{timestamp}_downCamera.jpg"
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
        self.green_box_pub = Topic.image_green_box.createPublisher(self)
        self.blue_box_pub = Topic.image_blue_box.createPublisher(self)
        # Subscribers (if needed)
        self.mission_type_sub = Topic.mission_type.createSubscriber(self, self.mission_callback)
        self.arena_sub = Topic.arena.createSubscriber(self, self._arena_cb)

        self.rc6_sub = Topic.rc6.createSubscriber(self, self._rc6_callback)

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
 

    def process_frame(self):
        """Process a single frame - called by timer"""
        try:
            success, img = self.up_cap.read()
            # print("In your mom")
            if not success:
                return

            if self.simulation:
                self.img = img
                self.dsc = 0.0
                self.detected = False
            else:
                self.img, self.dsc, self.detected = self.buoy_detector.process_frame(self.mission_type, self.arena, img, self.up_cap, self)

            if self.img is None:
                self.get_logger().warn("Failed to get frame", throttle_duration_sec=5.0)
                return

            if self.show_result:
                exit_status = self.visualize()
                if exit_status:
                    rclpy.shutdown()
                    return
            
            if not self.simulation:
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
            
            self.camera_processed_pub.publish(self.encode_base64(self.img))
            self.get_logger().info("Published processed camera frame", throttle_duration_sec=2.0)

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
        front_cam.detector.release()
        front_cam.down_cap.release()
        cv2.destroyAllWindows()
        front_cam.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
