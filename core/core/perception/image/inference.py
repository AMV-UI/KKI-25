import cv2
import math
# import rospy
import base64
import datetime
import time
import asyncio

import torch
from ultralytics import YOLO
from std_msgs.msg import String
from core.utils.config import Topic
from io import BytesIO
from core_msgs.msg import ObjectCount
from core.utils.config import Param, SPEED, MissionStatus
from core.utils.motor import Motor
# from core.perception.image.camera_bottom import BottomCamera


class ObjectDetector:
    def __init__(
        self, model_path, node, class_names, width=1280, height=960, fps=30
    ):

        self.model = YOLO(model_path)
        self.class_names = class_names
        self.conf_threshold_buoy = 0.15
        self.conf_threshold_box = 0.4
        self.node = node
        self.arena = "B"
        self.max_green_box_area = 30000
        self.max_blue_box_area = 30000

        # PID
        self.pid_adjust = 200
        self.treshold = 0.25

        # Data Frame
        self.max_red = -1
        self.max_green = -1
        self.red = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
        self.green = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

        self.green_box = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
        self.blue_box = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

        # Publisher
        self.dscPub = Topic.dsc.createPublisher(self.node)
        self.arena_sub = Topic.arena.createSubscriber(self.node, self._arena_cb)
    
    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)

    def draw_detections(self, img, results):
        """Draw detection boxes and labels on frame"""
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Get confidence and class
                conf = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = self.class_names[class_id]

                # Draw box and label
                color = (0, 255, 0) if class_name == "greenBox" else (0, 0, 255)
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                label = f"{class_name} {conf:.2f}"
                cv2.putText(
                    img, label, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
                )
                return img

    def process_frame(self, mission: MissionStatus, arena, img, cap, node):
        if cap is not None:
            width = cap.get(3) // 2  # float `width`
            height = cap.get(4) // 2  # float `height`
        else:
            height, width = img.shape[:2]
            width = width // 2
            height = height // 2

        width = int(width)
        height = int(height)

        center = (width, height)
        yaw_state = center[0]

        detected = False

        with torch.no_grad():
            results = self.model(img, conf=self.conf_threshold_buoy if mission == MissionStatus.BUOY else self.conf_threshold_box, verbose=False, stream=True)

            # red / green buoy
            self.max_red = -1
            self.max_green = -1
            self.red = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
            self.green = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

            self.max_green_box = -1
            self.max_blue_box = -1 
            self.green_box = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
            self.blue_box = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

            
            # threshold boxes
            # self.minimum_blue_box_area = 200
            # self.minimum_green_box_area = 200
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = float(box.conf[0])
                    cls = int(box.cls[0])
                    if confidence < 0.1:
                        continue

                    color = (0, 0, 0)
                    area = abs((x2 - x1) * (y2 - y1))

                    # Filter out small/far away objects
                    if area < 600:
                        continue

                    if mission == MissionStatus.BUOY:
                        img, self.red, self.green = self.buoy_detected(
                            cls, img, x1, y1, x2, y2, confidence, area
                        )


                    elif mission == MissionStatus.DOCKING:
                        img, self.green_box, self.blue_box = self.dock_detected(
                            cls, img, x1, y1, x2, y2, confidence, area
                        )                    

                    elif mission == MissionStatus.GREEN_BOX:
                        img, status = self.green_box_detected(
                            cls, img, x1, y1, x2, y2, confidence
                        )

                        return img, 0, status

                    elif mission == MissionStatus.BLUE_BOX:
                        img, status = self.blue_box_detected(
                            cls, img, x1, y1, x2, y2, confidence
                        )
                        
                        return img, 0, status

            if mission == MissionStatus.BUOY: 
                if self.max_red < self.max_green * self.treshold:
                    self.max_red = -1
                elif self.max_green < self.max_red * self.treshold:
                    self.max_green = -1

                self.mid_red = (self.red["x1"] + self.red["x2"]) // 2
                self.mid_green = (self.green["x1"] + self.green["x2"]) // 2

                if self.max_red != -1 and self.max_green != -1:
                    mid_x = (self.mid_green + self.mid_red) // 2
                    dsc_x = mid_x - width
                    yaw_state = dsc_x
                elif self.max_green != -1:
                    yaw_state = self.pid_adjust * (-1 if arena == "A" else 1)
                elif self.max_red != -1:
                    yaw_state = self.pid_adjust * (1 if arena == "A" else -1)
                else:
                    yaw_state = 0
                if self.max_green != -1 or self.max_red != -1:
                    detected = True

                color = (0, 0, 0)
                cv2.rectangle(img, (width, height), (width, height), color, 3)
            elif mission == MissionStatus.DOCKING:
                if self.max_green_box < self.max_blue_box * self.treshold:
                    self.max_green_box = -1
                elif self.max_blue_box < self.max_green_box * self.treshold:
                    self.max_blue_box = -1

                self.mid_green_box = (self.green_box["x1"] + self.green_box["x2"]) // 2
                self.mid_blue_box = (self.blue_box["x1"] + self.blue_box["x2"]) // 2

                if self.max_green_box != -1 and self.max_blue_box != -1:
                    mid_x = (self.mid_green_box + self.mid_blue_box) // 2
                    dsc_x = mid_x - width
                    yaw_state = dsc_x
                elif self.max_green_box != -1:
                    yaw_state = self.pid_adjust * (-1 if arena == "B" else 1)
                elif self.max_blue_box != -1:
                    yaw_state = self.pid_adjust * (1 if arena == "B" else -1)
                else:
                    yaw_state = 0

                if self.max_blue_box != -1 or self.max_green_box != -1:
                    detected = True

                color = (0, 0, 0)
                cv2.rectangle(img, (width, height), (width, height), color, 3)
            return img, yaw_state, detected

    def buoy_detected(self, cls, img, x1, y1, x2, y2, confidence, area):
        red = self.red
        green = self.green

        if self.class_names[cls] == "redBuoy" or self.class_names[cls] == "red_buoy":
            color = (0, 0, 255)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
            cv2.putText(
                img,
                self.class_names[cls] + " " + str(confidence),
                (x1, y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                color,
                2,
            )

            if self.max_red < area:
                self.max_red = area
                red = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

        elif (
            self.class_names[cls] == "greenBuoy"
            or self.class_names[cls] == "green_buoy"
        ):
            # self.obj_count.red += 1
            color = (0, 255, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
            cv2.putText(
                img,
                self.class_names[cls] + " " + str(confidence),
                (x1, y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                color,
                2,
            )

            if self.max_green < area:
                self.max_green = area
                green = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

        return img, red, green
    
    def dock_detected(self, cls, img, x1, y1, x2, y2, confidence, area):
        green_box = self.green_box
        blue_box = self.blue_box    
        if (self.class_names[cls] == "greenBox"):
            color = (0, 0, 255)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
            cv2.putText(
                img,
                self.class_names[cls] + " " + str(confidence),
                (x1, y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                color,
                2,
            )

            if self.max_green_box < area:
                self.max_green_box = area
                green_box = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

        elif (self.class_names[cls] == "blueBox"):
            color = (0, 255, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
            cv2.putText(
                img,
                self.class_names[cls] + " " + str(confidence),
                (x1, y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                color,
                2,
            )

            if self.max_blue_box < area:
                self.max_blue_box = area
                blue_box = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

        return img, green_box, blue_box 

    def green_box_detected(self, cls, img, x1, y1, x2, y2, confidence):
        status = False
        if self.class_names[cls] == "greenBox":
            color = (0, 0, 255)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
            cv2.putText(
                img,
                self.class_names[cls] + " " + str(confidence),
                (x1, y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                color,
                2,
            )

            status = True
        return img, status

    def blue_box_detected(self, cls, img, x1, y1, x2, y2, confidence):
        status = False
        if self.class_names[cls] == "blueBox":
            color = (0, 69, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
            cv2.putText(
                img,
                self.class_names[cls] + " " + str(confidence),
                (x1, y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                color,
                2,
            )
            status = True
        return img, status

    def run(self):
        while True:
            frame = self.process_frame()
            if frame is None:
                break

            cv2.imshow("Webcam", frame)
            if cv2.waitKey(1) == ord("q"):
                break
