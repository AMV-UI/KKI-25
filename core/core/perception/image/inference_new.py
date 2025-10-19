import cv2
import math
# import rospy
import base64
import datetime
import time
import asyncio

from ultralytics import YOLO
from std_msgs.msg import String
from core.utils.config import Topic
from io import BytesIO
from core_msgs.msg import ObjectCount
from core.utils.config import Param, SPEED
from core.utils.motor import Motor
from core.perception.image.camera_bottom import BottomCamera

from rclpy.node import Node


class ObjectDetector:
    def __init__(
        self, model_path, node, class_names, camera_index, width=640, height=480, fps=30
    ):
        self.motor = Motor()
        # Camera
        self.node = node
        self.model = YOLO(model_path)
        self.class_names = class_names
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(1, fps)  # Set FPS
        self.cap.set(3, width)  # Set width
        self.cap.set(4, height)  # Set height

        self.camera_bottom = BottomCamera()
        # Track
        # self.track = rospy.get_param(Param.TRACK)

        # PID
        self.pid_adjust = 300
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
        self.cameraBottomPub = Topic.image_blue_box.createPublisher(self.node)

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

    def normalize_class_name(self, cls):
        name = self.class_names[cls]
        if name in ["greenBuoy", "green_buoy"]:
            return "greenBuoy"
        elif name in ["redBuoy", "red_buoy"]:
            return "redBuoy"
        elif name in ["greenBox"]:
            return "greenBox"
        elif name in ["blueBox"]:
            return "blueBox"
        else:
            return name


    # def process_frame(self, mission):
    #     success, img = self.cap.read()
    #     # print("In your mom")
    #     if not success:
    #         return None, None, False

    #     width = self.cap.get(3) // 2  # float `width`
    #     height = self.cap.get(4) // 2  # float `height`

    #     width = int(width)
    #     height = int(height)

    #     center = (width, height)
    #     yaw_state = center[0]

    #     detected = False

    #     # results = self.model(img, stream=True, task="detect", verbose=False)

    #     results = self.model(img, conf=0.4, verbose=False)
    #     # results = self.model(img, stream=True, task='detect')

    #     # red / green buoy
    #     self.max_red = -1
    #     self.max_green = -1
    #     self.red = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
    #     self.green = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

    #     self.green_box = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
    #     self.blue_box = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

    #     # threshold boxes
    #     # self.minimum_blue_box_area = 200
    #     # self.minimum_green_box_area = 200
    #     for r in results:
    #         # self.track = rospy.get_param(Param.TRACK)
    #         boxes = r.boxes
    #         for box in boxes:
    #             # Get bounding box coordinates
    #             x1, y1, x2, y2 = map(int, box.xyxy[0])

    #             confidence = float(box.conf[0])
    #             cls = int(box.cls[0])

    #             if confidence < 0.4:
    #                 continue

    #             label = self.normalize_class_name(cls)

    #             if label == "redBuoy":
    #                 if confidence > self.max_red:
    #                     self.max_red = confidence
    #                     self.red = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

    #             elif label == "greenBuoy":
    #                 if confidence > self.max_green:
    #                     self.max_green = confidence
    #                     self.green = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

    #             # put box in cam
    #             color = (0, 0, 0)
    #             area = abs(x1 - x2)
    #             img, self.red, self.green = self.buoy_detected(cls, img, x1, y1, x2, y2, confidence, area)

    #     color = (0, 0, 0)
    #     cv2.rectangle(img, (width, height), (width, height), color, 3)

    #     return img, yaw_state, detected

    def process_frame(self, mission):
        success, img = self.cap.read()
        if not success:
            return None, None, False

        width = int(self.cap.get(3) // 2)
        height = int(self.cap.get(4) // 2)
        center = (width, height)
        yaw_state = center[0]
        detected = False

        results = self.model(img, conf=0.2, verbose=False)

        # Reset detections
        self.max_red = -1
        self.max_green = -1
        self.red = None
        self.green = None
        self.bias = 1.15

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = float(box.conf[0])
                cls = int(box.cls[0])

                # if confidence < 0.3:
                #     continue

                label = self.normalize_class_name(cls)

                if label == "greenBuoy":
                    confidence *= self.bias
                
                if label == "redBuoy":
                    if confidence > self.max_red:
                        self.max_red = confidence
                        self.red = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                        detected = True

                elif label == "greenBuoy":
                    if confidence > self.max_green:
                        self.max_green = confidence
                        self.green = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                        detected = True

        # Draw results
        if self.red:
            cv2.rectangle(img, (self.red["x1"], self.red["y1"]), (self.red["x2"], self.red["y2"]), (0, 0, 255), 2)
            cv2.putText(img, f"Red {self.max_red:.2f}", (self.red["x1"], self.red["y1"] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        if self.green:
            cv2.rectangle(img, (self.green["x1"], self.green["y1"]), (self.green["x2"], self.green["y2"]), (0, 255, 0), 2)
            cv2.putText(img, f"Green {self.max_green:.2f}", (self.green["x1"], self.green["y1"] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw reference center
        cv2.circle(img, (width, height), 4, (255, 255, 255), -1)

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

    def green_box_detected(self, cls, img, x1, y1, x2, y2, confidence):
        status = False
        if self.class_names[cls] == "greenBox":
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

    def blue_box_detected(self, cls, img, x1, y1, x2, y2, confidence):
        status = False
        img = ""
        if self.class_names[cls] == "blueBox":
            color = (0, 0, 255)
            img = self.camera_bottom.do_capture()

            # cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
            # cv2.putText(
            #     img,
            #     self.class_names[cls] + " " + str(confidence),
            #     (x1, y1),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1,
            #     color,
            #     2,
            # )
            #
            status = True
        return img, status

    def find_dock_detected(self, cls, img, x1, y1, x2, y2, confidence, area):
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
            self.green_box = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

        if self.class_names[cls] == "blueBox":
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
            self.blue_box = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

        return img

    def docking_detected(self, cls, img, x1, y1, x2, y2, confidence):
        status = False
        if self.class_names[cls] == "docking":
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

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def run(self):
        while True:
            frame = self.process_frame()
            if frame is None:
                break

            cv2.imshow("Webcam", frame)
            if cv2.waitKey(1) == ord("q"):
                break
