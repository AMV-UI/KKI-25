#!/usr/bin/env python3

import traceback
import cv2
import rclpy
# import actionlib
import numpy as np
import base64
import time
from core.perception.image.inference_new import ObjectDetector
from core_msgs.msg import StateObject, AutoControl
from core.utils.config import AutoState, Box, Camera, NodeConfig, Topic, ModelPath, Tower
from rclpy.node import Node

class FrontCamera(Node):
    """
    SUBSCRIBES TO:
        - None
    ---
    PUBLISHES TO:
        - camera_raw: Raw image/frame from camera
        - camera_compressed: Compressed image/frame
    """

    def __init__(self):
        super().__init__("front_camera")

        self.detector = ObjectDetector(
            "/home/amv/models/v12/best.engine",
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
            #"/dev/topCamera", #udev for real camera
            "/home/amv/Videos/sim.mp4", #path to video for sim
        )
        
        self.result = ""
        self.dsc = -9999
        self.state = [0, 0, 0, 0]
        self.img = ""
        self.img_64 = ""
        self.current_state = StateObject()
        self.current_mission = 1
        self.mission_received = AutoControl()

        # self.micon = Microcontroller().request_pixhawk()
        # rate = rospy.Rate(10)

    def get_data(self):
        return self.result, self.dsc, self.state

    def init_vest(self):
        self.vest = ObjectDetector(
            "/home/amv/Videos/vest.pt",
            [
                "googles",
                "helmet",
                "no-goggles",
                "no-helmet",
                "no-vest",
                "null",
                "Person",
                "vest",
            ],
            "/dev/topCamera",  # udev
            # SIM
            # "/home/amv/Videos/videostore/footage3.mp4",
        )


    def do_impros(self):
        if self.current_mission == AutoControl.MISSION_DOCKING:
            self.init_vest()
        while rclpy.ok():
            # Read the frame
            try:

                ## TODO: transfer these mechanism to core behavior tree

                if self.current_mission < AutoControl.MISSION_POSITION_GREEN_BOX:
                    self.img, self.dsc, self.detected = self.detector.process_frame(
                        "buoy"
                    )
                elif self.current_mission < AutoControl.MISSION_POSITION_GREEN_BOX:
                    self.img, self.dsc, self.detected = self.detector.process_frame(
                        "green_box"
                    )
                elif self.current_mission < AutoControl.MISSION_POSITION_BLUE_BOX:
                    self.img_64, self.dsc, self.detected = self.detector.process_frame(
                        "blue_box"
                    )
                elif self.current_mission == AutoControl.MISSION_DOCKING:
                    self.img, self.dsc, self.detected = self.vest.process_frame(
                        "find_dock"
                    )
                elif self.current_mission == AutoControl.MISSION_DOCKING:
                    self.img, self.dsc, self.detected = self.vest.process_frame(
                        "docking"
                    )


                self.detected_pub.publish(self.detected)
                if self.img is None:
                    # rclpy.logerr("Failed to get frame")
                    break

                # Encode the processed frame to JPG format
                result, encoded_image = cv2.imencode(
                    ".jpg", self.img, [int(cv2.IMWRITE_JPEG_QUALITY), 20]
                )
                if not result:
                    # rospy.logerr("Failed to encode frame to JPG")
                    break

                # Convert to base64
                base64_image = base64.b64encode(encoded_image).decode("utf-8")
                green_box_image = base64.b64encode(self.img_64).decode("utf-8")

                # time.sleep(0.2)
                # Publish the image
                self.dsc_pub.publish(self.dsc)
                # self.dsc_flag_pub.publish(dsc_flag)

                # if self.current_mission == AutoControl.MISSION_TAKE_GREEN_BOX:
                #     self.greenBoxPub.publish(base64_image)
                if self.current_mission == AutoControl.MISSION_TAKE_BLUE_BOX:
                    self.camera_bottom_pub.publish(self.img_64)

                self.camera_processed_pub.publish(base64_image)
                # rospy.loginfo_throttle(
                #     5, f"[{Node.camera_front}] Published processed image"
                # )
                # self.fps_counter.calculate(frame)
                # self.camera_processed_pub.publish(compressImage(frame))
            except Exception as e:
                # rospy.logerr_throttle(5, f"<=> [{Node.camera_front}] Improc error,", e)
                self.get_logger().error(traceback.format_exc())

    def mission_callback(self, msg):
        self.current_mission = msg.data

    def main(self):
        ## PUBLISHERS
        # self.camera_processed_pub = Topic.camera_processed.createPublisher(self)
        # self.camera_bottom_pub = Topic.camera_bottom.createPublisher(self)
        # self.dsc_pub = Topic.dsc.createPublisher(self)
        # self.dsc_flag_pub = Topic.dsc_flag.createPublisher(self)
        # self.detected_pub = Topic.detected.createPublisher(self)

        ## SUBSCRIBERS
        # self.current_mission_sub = Topic.mission.createSubscriber(self.mission_callback)

        #
        # # self.blueBoxPub = Topic.image_blue_box.createPublisher()
        # # self.greenBoxPub = Topic.image_green_box.createPublisher()


        # # self.state_pub = Topic.state_object.createPublisher()
        # # self.state_yaw_pub = Topic.state_yaw.createPublisher()
        # # self.object_counted_pub = Topic.object_counted.createPublisher()
        #
        # Main Mission
        self.do_impros()
    


        # rospy.loginfo_once(f"<> [{Node.camera_front}] Successfully initialized Node!")



def main():
    rclpy.init()
    front_cam = FrontCamera()

    try:
        front_cam.main()
    finally:
        front_cam.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
