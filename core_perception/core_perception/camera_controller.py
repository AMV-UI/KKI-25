#!/usr/bin/env python3

import traceback
import cv2
import rclpy
import numpy as np
import base64
import time
from core.perception.image.inference import ObjectDetector
from core_msgs.msg import StateObject, AutoControl
from core.utils.config import NodeConfig, Topic, MissionStatus
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

        self.detector = ObjectDetector(
            "/home/amv/models/v12/best_v12.engine",
            self,
            [
                "blueBox",
                "docking",
                "greenBox",
                "greenBuoy",
                "green_buoy",
                "redBuoy",
                "red_buoy",
            ],
            "/dev/video0",  # udev for real camera
            # "/home/amv/Videos/asv.mp4",  # path to video for sim
        )
        
        self.result = ""
        self.dsc = -9999
        self.state = [0, 0, 0, 0]
        self.img = None
        self.img_64 = ""
        self.current_state = StateObject()
        self.current_mission = MissionStatus.BUOY
        self.mission_received = AutoControl()
        self.show_result = False
        self.detected = False
        self.fps = 30

        # Setup communication
        self._setup_communication()
        
        self.get_logger().info(f"<> [{NodeConfig.camera_front}] Successfully initialized node")

    def _setup_communication(self):
        """Initialize publishers and subscribers"""
        # Publishers
        self.dsc_pub = Topic.dsc.createPublisher(self)
        self.detected_pub = Topic.detected.createPublisher(self)
        self.camera_processed_pub = Topic.camera_processed.createPublisher(self)
        # Subscribers (if needed)
        self.current_mission_sub = Topic.mission.createSubscriber(self, self.mission_callback)
        self.arena_sub = Topic.arena.createSubscriber(self, self._arena_cb)

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

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

    def process_frame(self):
        """Process a single frame - called by timer"""
        try:
            # self.get_logger().info(f"Processing frame...{self.arena}", throttle_duration_sec=2.0)
            self.img, self.dsc, self.detected = self.detector.process_frame(self.current_mission, self.arena, self)
            # self.img, self.dsc, self.detected = self.img, self.dsc, self.detected

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
            result, encoded_image = cv2.imencode(
                ".jpg", self.img, [int(cv2.IMWRITE_JPEG_QUALITY), 20]
            )
            if result:
                base64_image = base64.b64encode(encoded_image).decode("utf-8")
                img_msg = String()
                img_msg.data = base64_image
                self.camera_processed_pub.publish(img_msg)
            else:
                self.get_logger().error("Failed to encode frame to JPG")
            
        except Exception as e:
            self.get_logger().error(f"Error in process_frame: {traceback.format_exc()}")

    def mission_callback(self, msg):
        """Update current mission"""
        self.current_mission = MissionStatus(msg.data)
        self.get_logger().info(f"Mission changed to: {self.current_mission}", throttle_duration_sec=3.0)

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
        cv2.destroyAllWindows()
        front_cam.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
