#!/usr/bin/env python3

import traceback
import cv2
import rospy
import actionlib
import numpy as np
import base64
import time
from utils.image.inference_new import ObjectDetector
from kki24.msg import StateObject, AutoControl
# from utils.image.localize import boxDetectionContour, imageProcessingBuoy, towerDetectionContour, init_model
# from utils.image.localize import init_model

from utils.config import AutoState, Box, Camera, Node, Topic, ModelPath, Tower
# from utils.converter import autocontrolToString
# from utils.time_keeper import TimeKeeper
# from utils.frame_counter import FrameCounter

# from utils.image.vision import preprocess
# from utils.image.format import compressImage, FPSCounter

# from kki24.msg import Option, AutoControl, SendMissionAction, SendMissionActionFeedback, SendMissionActionGoal, SendMissionActionResult, SendMissionFeedback, SendMissionGoal, SendMissionResult


# from microcontroller69 import Microcontroller


class FrontCamera:
    """
    SUBSCRIBES TO:
        - None
    ---
    PUBLISHES TO:
        - camera_raw: Raw image/frame from camera
        - camera_compressed: Compressed image/frame
    """

    def __init__(self):
        # Create capture object
        self.detector = ObjectDetector(
            # "/home/amv/main_ws/src/kki24/scripts/model/yolov8m.engine",
            "/home/amv/Videos/new.pt",
            # "/home/amv/Videos/new_test.engine",
            [
                "blueBox",
                "docking",
                "greenBox",
                "greenBuoy",
                "green_buoy",
                "redBuoy",
                "red_buoy",
            ],
            # ["blueBox", "greenBox", "greenBuoy", "redBuoy", "red_buoy", "green_buoy"],
            "/dev/topCamera",
            # SIM
            # "/home/amv/Videos/videostore/footage3.mp4",
        )
        # self.vest = ObjectDetector(
        #     "/home/amv/Videos/vest.pt",
        #     [
        #         "googles",
        #         "helmet",
        #         "no-goggles",
        #         "no-helmet",
        #         "no-vest",
        #         "null",
        #         "Person",
        #         "vest",
        #     ],
        #
        #     "/dev/topCamera", #udev
        #     # SIM
        #     # "/home/amv/Videos/videostore/footage3.mp4",
        # )

        #  self.vest_detector= ObjectDetector(
        #      "/home/amv/main_ws/src/kki24/scripts/model/vest/vest.engine",
        #      ["blueBox", "greenBox", "greenBuoy", "redBuoy"],
        # )

        # self.detector = ObjectDetector(
        #     "/home/amv/Videos/new.pt",
        #     ["blueBox", "greenBox", "greenBuoy", "redBuoy"],
        # )

        # self.vest_detector= ObjectDetector(
        #      "/home/amv/Videos/vest.engine",
        #      ["blueBox", "greenBox", "greenBuoy", "redBuoy"],
        # )
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
        while not rospy.is_shutdown():
            # Read the frame
            try:
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
                    rospy.logerr("Failed to get frame")
                    break

                # Encode the processed frame to JPG format
                result, encoded_image = cv2.imencode(
                    ".jpg", self.img, [int(cv2.IMWRITE_JPEG_QUALITY), 20]
                )
                if not result:
                    rospy.logerr("Failed to encode frame to JPG")
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
                rospy.loginfo_throttle(
                    5, f"[{Node.camera_front}] Published processed image"
                )
                # self.fps_counter.calculate(frame)
                # self.camera_processed_pub.publish(compressImage(frame))
            except Exception as e:
                rospy.logerr_throttle(5, f"<=> [{Node.camera_front}] Improc error,", e)

    def mission_callback(self, msg):
        self.current_mission = msg.data

    def main(self):
        # PUBLISHERS
        self.camera_processed_pub = Topic.camera_processed.createPublisher()

        # self.blueBoxPub = Topic.image_blue_box.createPublisher()
        # self.greenBoxPub = Topic.image_green_box.createPublisher()
        self.camera_bottom_pub = Topic.camera_bottom.createPublisher()
        self.dsc_pub = Topic.dsc.createPublisher()
        self.dsc_flag_pub = Topic.dsc_flag.createPublisher()
        self.detected_pub = Topic.detected.createPublisher()
        # self.state_pub = Topic.state_object.createPublisher()
        # self.state_yaw_pub = Topic.state_yaw.createPublisher()
        # self.object_counted_pub = Topic.object_counted.createPublisher()

        self.current_mission_sub = Topic.mission.createSubscriber(self.mission_callback)
        # Main Mission
        self.do_impros()

        rospy.loginfo_once(f"<> [{Node.camera_front}] Successfully initialized Node!")


if __name__ == "__main__":
    # try:
    # Initialize node
    rospy.init_node(Node.camera_front)

    front_cam = FrontCamera()
    front_cam.main()

# # except Exception as e:
# #     rospy.logerr(traceback.format_exc())
