#!/usr/bin/env python3

import traceback
import cv2
import rclpy
import numpy as np
import base64
import time
from core.perception.image.inference_new import ObjectDetector
from core_msgs.msg import StateObject, AutoControl
from core.utils.config import AutoState, Box, Camera, NodeConfig, Topic, ModelPath, Tower
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

class CameraController(Node):
    """
    Front Camera Node for Object Detection
    
    PUBLISHES TO:
        - dsc: Float64 - Yaw control effort
        - detected: Bool - Detection status
    """

    def __init__(self):
        super().__init__("front_camera")

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
            "/dev/topCamera",  # udev for real camera
            #"/home/amv/Videos/asv.mp4",  # path to video for sim
        )
        
        self.result = ""
        self.dsc = -9999
        self.state = [0, 0, 0, 0]
        self.img = None
        self.img_64 = ""
        self.current_state = StateObject()
        self.current_mission = 1
        self.mission_received = AutoControl()
        self.show_result = False
        self.detected = False

        # Setup communication
        self._setup_communication()
        
        self.get_logger().info(f"<> [{NodeConfig.camera_front}] Successfully initialized node")

    def _setup_communication(self):
        """Initialize publishers and subscribers"""
        # Publishers
        self.dsc_pub = Topic.dsc.createPublisher(self)
        self.detected_pub = Topic.detected.createPublisher(self)
        
        # Subscribers (if needed)
        # self.current_mission_sub = Topic.mission.createSubscriber(self, self.mission_callback)

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
            self.img, self.dsc, self.detected = self.detector.process_frame("buoy")
            
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

        except Exception as e:
            self.get_logger().error(f"Error in process_frame: {traceback.format_exc()}")

    def mission_callback(self, msg):
        """Update current mission"""
        self.current_mission = msg.data
        self.get_logger().info(f"Mission changed to: {self.current_mission}")

    def run(self):
        """Start the main execution loop"""
        # timer for frame processing (30 FPS = 0.033s)
        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info("Front camera processing started at 30 FPS")


def main():
    rclpy.init()
    front_cam = FrontCamera()

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
