#!/usr/bin/env python3

import traceback
import cv2
import rospy
import actionlib
import numpy as np
import base64
import time
from kki24.msg import StateObject, AutoControl
# from utils.image.localize import boxDetectionContour, imageProcessingBuoy, towerDetectionContour, init_model
# from utils.image.localize import init_model

from utils.config import Box, Camera, Node, Topic, ModelPath, Tower


class BottomCamera:
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
        # self.detector = ObjectDetector(
        #     # "/home/amv/main_ws/src/kki24/scripts/model/yolov8m.engine",
        #     "/home/amv/Videos/vest.pt",
        #     # "/home/amv/Videos/new_test.engine",
        #     [
        #         "blueBox",
        #         "docking",
        #         "greenBox",
        #         "greenBuoy",
        #         "green_buoy",
        #         "redBuoy",
        #         "red_buoy",
        #     ],
        #     # ["blueBox", "greenBox", "greenBuoy", "redBuoy", "red_buoy", "green_buoy"],
        # )
        #
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

        camera_name = "/dev/bottomCamera"

        # SIM
        # camera_name = "/home/amv/Videos/videostore/footage1.mp4"

        self.result = ""
        self.cap = cv2.VideoCapture(camera_name)
        # self.micon = Microcontroller().request_pixhawk()

    def do_capture(self):
        # Read the frame
        try:
            # self.img, self.dsc, self.detected = self.detector.process_frame(
            #         "vest"
            # )
            ret, self.img = self.cap.read()

            if self.img is None:
                rospy.logerr("BottomCamera : Failed to get frame!")
                return

            # Encode the processed frame to JPG format
            result, encoded_image = cv2.imencode(
                ".jpg", self.img, [int(cv2.IMWRITE_JPEG_QUALITY), 20]
            )
            if not result:
                rospy.logerr("Failed to encode frame to JPG")
                break

            # Convert to base64
            # base64_image = base64.b64encode(encoded_image).decode("utf-8")
            rospy.logerr_throttle(5, f"<=> [{Node.camera_bottom}] Captured frame")
            return encoded_image
            # self.fps_counter.calculate(frame)
            # self.camera_processed_pub.publish(compressImage(frame))
        except Exception as e:
            rospy.logerr_throttle(5, f"<=> [{Node.camera_bottom}] Improc error,", e)
