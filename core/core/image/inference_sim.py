import cv2
import math
import rospy
import base64
import datetime
import time
import asyncio

from ultralytics import YOLO
from std_msgs.msg import String
from utils.config import Topic
from io import BytesIO
from kki24.msg import ObjectCount
from utils.config import Param, SPEED
from utils.motor import Motor


class ObjectDetector:
    def __init__(
        self,
        model_path,
        class_names,
        camera_index=0,
        width=640,
        height=480,
        fps=30,
    ):
        # Camera
        self.model = YOLO(model_path)
        self.class_names = class_names
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(5, fps)  # Set FPS
        self.cap.set(3, width)  # Set width
        self.cap.set(4, height)  # Set height

        # Track
        self.track = rospy.get_param(Param.TRACK)

        # PID
        self.pid_adjust = 300
        self.treshold = 0.25

        # Data Frame
        self.max_red = -1
        self.max_green = -1
        self.red = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
        self.green = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

        # Subscriber
        self.dscPub = Topic.dsc.createPublisher()

    def process_frame(self, mission):
        success, img = self.cap.read()
        # print("In your mom")
        if not success:
            return None

        width = self.cap.get(3) // 2  # float `width`
        height = self.cap.get(4) // 2  # float `height`

        width = int(width)
        height = int(height)

        center = (width, height)
        yaw_state = center[0]

        detected = False

        results = self.model(img, stream=True, task="detect", verbose=False)

        # red / green buoy
        self.max_red = -1
        self.max_green = -1
        self.red = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
        self.green = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

        # threshold boxes
        self.minimum_blue_box_area = 500
        self.minimum_green_box_area = 200
        for r in results:
            self.track = rospy.get_param(Param.TRACK)
            boxes = r.boxes
            for box in boxes:
                # Get bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])

                if confidence < 0.7:
                    continue

                # put box in cam
                color = (0, 0, 0)
                area = abs(x1 - x2)
                if mission == "buoy":
                    img, self.red, self.green = self.buoy_detected(
                        cls, img, x1, y1, x2, y2, confidence, area
                    )

                elif mission == "green_box":
                    img, green_area = self.green_box_detected(
                        cls, img, x1, y1, x2, y2, confidence
                    )
                    if green_area > self.minimum_green_box_area:
                        Motor.full_detected()
                        # yaw_state = self.pid_adjust * (1 if self.track == "A" else -1)
                        yaw_state = 0

                        detected = True
                    else:
                        Motor.not_detected()

                    return img, yaw_state, detected

                elif mission == "blue_box":
                    img, blue_area = self.blue_box_detected(
                        cls, img, x1, y1, x2, y2, confidence
                    )
                    if blue_area > self.minimum_blue_box_area:
                        Motor.full_detected()
                        # yaw_state = self.pid_adjust * (1 if self.track == "A" else -1)
                        yaw_state = 0
                        detected = True
                    else:
                        Motor.not_detected()

                    return img, yaw_state, detected

        if self.max_red < self.max_green * self.treshold:
            Motor.one_is_closer_detected()
            self.max_red = -1
        elif self.max_green < self.max_red * self.treshold:
            Motor.one_is_closer_detected()
            self.max_green = -1

        self.mid_red = (self.red["x1"] + self.red["x2"]) // 2
        self.mid_green = (self.green["x1"] + self.green["x2"]) // 2

        # Full Motor
        if self.max_red != -1 and self.max_green != -1:
            mid_x = (self.mid_green + self.mid_red) // 2

            mid_y = (
                max(self.red["y1"], self.green["y1"])
                + min(self.red["y2"], self.green["y2"])
            ) // 2

            dsc_x = mid_x - width
            # dsc_y = mid_y - height

            Motor.full_detected()
            yaw_state = dsc_x
        # Half Motor
        else:
            # Only Green Buoy
            if self.max_green != -1:
                Motor.half_detected()
                yaw_state = self.pid_adjust * (-1 if self.track == "A" else 1)

            # Only Red Buoy
            elif self.max_red != -1:
                Motor.half_detected()
                yaw_state = self.pid_adjust * (1 if self.track == "A" else -1)

            # No Buoy
            else:
                Motor.not_detected()
                yaw_state = 0

        if self.max_green != -1 or self.max_red != -1:
            detected = True

        color = (0, 0, 0)
        cv2.rectangle(img, (width, height), (width, height), color, 3)

        return img, yaw_state, detected

    def buoy_detected(self, cls, img, x1, y1, x2, y2, confidence, area):
        red = self.red
        green = self.green

        if self.class_names[cls] == "redBuoy":
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

        elif self.class_names[cls] == "greenBuoy":
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
        if self.class_names[cls] == "greenBox":
            green_area = abs(x1 - x2) * abs(y1 - y2)
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
            return img, green_area

    def blue_box_detected(self, cls, img, x1, y1, x2, y2, confidence):
        if self.class_names[cls] == "blueBox":
            blue_area = abs(x1 - x2) * abs(y1 - y2)
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
            return img, blue_area

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
