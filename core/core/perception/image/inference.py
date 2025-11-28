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
from core.utils.config import Param, SPEED, MissionStatus
from core.utils.motor import Motor
# from core.perception.image.camera_bottom import BottomCamera


class ObjectDetector:
    def __init__(
        self, model_path, node, class_names, camera_index, width=640, height=480, fps=30
    ):

        self.model = YOLO(model_path)
        self.class_names = class_names
        self.conf_threshold = 0.2
        self.node = node
        self.arena = "B"
        self.max_green_box_area = 30000
        self.max_blue_box_area = 30000

        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(1, fps)  # Set FPS
        self.cap.set(3, width)  # Set width
        self.cap.set(4, height)  # Set height

        # self.camera_bottom = BottomCamera()
        # self.motor = Motor(self.node, offset_horizontal=100, motor_adjust=0)

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

        # self.cameraBottomPub = Topic.image_blue_box.createPublisher()
    
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

    def process_frame(self, mission: MissionStatus, arena, node):
        success, img = self.cap.read()
        # print("In your mom")
        if not success:
            return None, 0, False

        width = self.cap.get(3) // 2  # float `width`
        height = self.cap.get(4) // 2  # float `height`

        width = int(width)
        height = int(height)

        center = (width, height)
        yaw_state = center[0]

        detected = False

        results = self.model(img, conf=self.conf_threshold, verbose=False)

        # red / green buoy
        self.max_red = -1
        self.max_green = -1
        self.red = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
        self.green = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

        self.green_box = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
        self.blue_box = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

        
        # threshold boxes
        # self.minimum_blue_box_area = 200
        # self.minimum_green_box_area = 200
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                

                confidence = float(box.conf[0])
                cls = int(box.cls[0])

                if confidence < 0.4:
                    continue

                # put box in cam
                color = (0, 0, 0)

                # Get area of the box experimentation with actual area instead of x only
                # area = abs(x1 - x2)
                area = abs((x2 - x1) * (y2 - y1))
                if mission == MissionStatus.BUOY:
                    img, self.red, self.green = self.buoy_detected(
                        cls, img, x1, y1, x2, y2, confidence, area
                    )

                elif mission == MissionStatus.FIND_GREEN_BOX:
                    img, status = self.green_box_detected(
                        cls, img, x1, y1, x2, y2, confidence
                    )

                    color = (0, 0, 0)
                    cv2.rectangle(img, (width, height), (width, height), color, 3)

                    node.get_logger().info(f"Area: {area}", throttle_duration_sec=1.0)

                    # Stop if area big enough
                    if area > self.max_green_box_area:
                        yaw_state = 0
                        status = False # Assume mission starts as detected, this being termination condition
                        return img, yaw_state, status
                    
                    mid = (x1 + x2) // 2
                    yaw_state = mid - width
                    return img, yaw_state, status

                elif mission == MissionStatus.FIND_BLUE_BOX:
                    img_64, status = self.blue_box_detected(
                        img, x1, y1, x2, y2, confidence
                    )
                    yaw_state = 0
                    detected = status
                    return img_64, yaw_state, detected

        if self.max_red < self.max_green * self.treshold:
            # self.motor.one_is_closer_detected()
            self.max_red = -1
        elif self.max_green < self.max_red * self.treshold:
            # self.motor.one_is_closer_detected()
            self.max_green = -1

        self.mid_red = (self.red["x1"] + self.red["x2"]) // 2
        self.mid_green = (self.green["x1"] + self.green["x2"]) // 2

        # Full Motor
        if self.max_red != -1 and self.max_green != -1:
            mid_x = (self.mid_green + self.mid_red) // 2
            dsc_x = mid_x - width
            yaw_state = dsc_x
        elif self.max_green != -1:
            # Only green detected
            yaw_state = self.pid_adjust * (-1 if arena == "A" else 1)
        elif self.max_red != -1:
            # Only red detected
            yaw_state = self.pid_adjust * (1 if arena == "A" else -1)

        if self.max_green != -1 or self.max_red != -1:
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

    def green_box_detected(self, cls, img, x1, y1, x2, y2, confidence):
        status = False
        # WARNING: Temporary use buoy, green box model still bad
        if self.class_names[cls] == "greenBuoy" or self.class_names[cls] == "green_buoy":
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
            # img = self.camera_bottom.do_capture()

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
