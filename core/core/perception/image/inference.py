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
from core.utils.config import Param, SPEED, MissionStatus, MissionParams
from core.utils.motor import Motor
# from core.perception.image.camera_bottom import BottomCamera


class ObjectDetector:
    def __init__(
        self, model_path, node, class_names, width=1280, height=960, fps=30, blue_model_path=None, blue_class_names=None
    ):

        self.model = YOLO(model_path)
        self.class_names = class_names
        
        self.blue_model = YOLO(blue_model_path) if blue_model_path else None
        self.blue_class_names = blue_class_names if blue_class_names else []
        self.conf_threshold_buoy = 0.15
        self.conf_threshold_box = 0.4
        self.node = node
        self.arena = getattr(MissionParams, 'default_arena', 'B')
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
        yaw_state = 9999.0

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

            self.max_blue_dock = -1
            self.max_red_dock = -1
            self.max_green_dock = -1
            self.blue_dock = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
            self.red_dock = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
            self.green_dock = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

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

                    # Filter out excessively large bounding boxes (e.g., > 80% of frame)
                    img_area = (width * 2) * (height * 2)
                    if area > 0.8 * img_area:
                        continue

                    # Filter out small/far away objects
                    min_buoy = getattr(MissionParams, 'min_area_buoy', 600.0)
                    
                    # DEBUG: Print all detections
                    self.node.get_logger().info(f"[DEBUG YOLO] cls:{cls} name:{self.class_names[cls] if cls < len(self.class_names) else 'Unknown'} conf:{confidence:.2f} area:{area} mission:{mission}", throttle_duration_sec=1.0)
                    
                    if area < min_buoy: #simulasi
                    # if area < 50:
                        continue

                    if mission == MissionStatus.BUOY:
                        img, self.red, self.green = self.buoy_detected(
                            cls, img, x1, y1, x2, y2, confidence, area
                        )


                    elif mission == MissionStatus.DOCKING:
                        c_name = self.class_names[cls] if cls < len(self.class_names) else 'Unknown'
                        if c_name in ["blue-buoy", "blueBuoy", "blue buoy"]:
                            color = (255, 0, 0)
                            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                            cv2.putText(img, f"blue_buoy ({area})", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                            if area > self.max_blue_dock:
                                self.max_blue_dock = area
                                self.blue_dock = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                        elif c_name in ["red-buoy", "redBuoy", "red buoy", "red_buoy"]:
                            color = (0, 0, 255)
                            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                            cv2.putText(img, f"red_buoy ({area})", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                            if area > self.max_red_dock:
                                self.max_red_dock = area
                                self.red_dock = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                        elif c_name in ["green-buoy", "greenBuoy", "green buoy", "green_buoy"]:
                            color = (0, 255, 0)
                            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                            cv2.putText(img, f"green_buoy ({area})", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                            if area > self.max_green_dock:
                                self.max_green_dock = area
                                self.green_dock = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

                    elif mission == MissionStatus.GREEN_BOX:
                        img, box_yaw_state, status = self.green_box_detected(
                            cls, img, x1, y1, x2, y2, confidence
                        )
                        if status:
                            return img, box_yaw_state, status
                        # If false, keep checking other boxes

                    elif mission == MissionStatus.BLUE_BOX:
                        img, box_yaw_state, status = self.blue_box_detected(
                            cls, img, x1, y1, x2, y2, confidence
                        )
                        if status:
                            return img, box_yaw_state, status
                        # If false, keep checking other boxes
                        
                    elif mission == MissionStatus.BOTH_BOXES:
                        if self.class_names[cls] == "greenBox":
                            if area > self.max_green_box:
                                self.max_green_box = area
                                self.green_box = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                        elif self.class_names[cls] == "blueBox":
                            if area > self.max_blue_box:
                                self.max_blue_box = area
                                self.blue_box = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

            # Run secondary blue model if available (specifically for DOCKING)
            if self.blue_model and mission == MissionStatus.DOCKING:
                blue_results = self.blue_model(img, conf=self.conf_threshold_buoy, verbose=False, stream=True)
                for r in blue_results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = float(box.conf[0])
                        if confidence < 0.1: continue
                        cls = int(box.cls[0])
                        c_name = self.blue_class_names[cls] if cls < len(self.blue_class_names) else 'Unknown'
                        area = abs((x2 - x1) * (y2 - y1))
                        img_area = (width * 2) * (height * 2)
                        if area > 0.8 * img_area: continue
                        
                        min_buoy = getattr(MissionParams, 'min_area_buoy', 600.0)
                        if area < min_buoy: continue
                        
                        self.node.get_logger().info(f"[DEBUG BLUE YOLO] cls:{cls} name:{c_name} conf:{confidence:.2f} area:{area}", throttle_duration_sec=1.0)
                        
                        if c_name in ["blue-buoy", "blueBuoy", "blue buoy", "bluebuoy"]:
                            color = (255, 0, 0)
                            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                            cv2.putText(img, f"blue_buoy ({area})", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                            if area > self.max_blue_dock:
                                self.max_blue_dock = area
                                self.blue_dock = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

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
                    yaw_state = self.pid_adjust * (1 if arena == "A" else -1)
                elif self.max_red != -1:
                    yaw_state = self.pid_adjust * (-1 if arena == "A" else 1)
                else:
                    yaw_state = 0
                if self.max_green != -1 or self.max_red != -1:
                    detected = True

                color = (0, 0, 0)
                cv2.rectangle(img, (width, height), (width, height), color, 3)
            elif mission == MissionStatus.DOCKING:
                if self.max_blue_dock != -1 and self.max_red_dock != -1:
                    # Priority 1: Blue + Red
                    mid_blue = (self.blue_dock["x1"] + self.blue_dock["x2"]) // 2
                    mid_red = (self.red_dock["x1"] + self.red_dock["x2"]) // 2
                    mid_x = (mid_blue + mid_red) // 2
                    yaw_state = mid_x - width
                    detected = True
                elif self.max_blue_dock != -1:
                    # Priority 2: Only Blue
                    # Arena A -> Left (Port), Arena B -> Right (Starboard)
                    if arena == "A":
                        yaw_state = 7777.0 # Code for hard left
                    else:
                        yaw_state = 8888.0 # Code for hard right
                    detected = True
                elif self.max_red_dock != -1 and self.max_green_dock != -1:
                    # Priority 3: Red + Green
                    mid_red = (self.red_dock["x1"] + self.red_dock["x2"]) // 2
                    mid_green = (self.green_dock["x1"] + self.green_dock["x2"]) // 2
                    mid_x = (mid_red + mid_green) // 2
                    yaw_state = mid_x - width
                    detected = True
                else:
                    yaw_state = 9999.0
                    detected = False
            
            elif mission == MissionStatus.BOTH_BOXES:
                if self.max_green_box != -1 and self.max_blue_box != -1:
                    mid_green = (self.green_box["x1"] + self.green_box["x2"]) // 2
                    mid_blue = (self.blue_box["x1"] + self.blue_box["x2"]) // 2
                    mid_x = (mid_green + mid_blue) // 2
                    yaw_state = (mid_x - width) * 0.5
                    detected = True
                    cv2.rectangle(img, (self.green_box["x1"], self.green_box["y1"]), (self.green_box["x2"], self.green_box["y2"]), (0, 255, 0), 3)
                    cv2.putText(img, "greenBox", (self.green_box["x1"], self.green_box["y1"]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.rectangle(img, (self.blue_box["x1"], self.blue_box["y1"]), (self.blue_box["x2"], self.blue_box["y2"]), (255, 0, 0), 3)
                    cv2.putText(img, "blueBox", (self.blue_box["x1"], self.blue_box["y1"]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                elif self.max_green_box != -1:
                    yaw_state = 8888.0
                    detected = True
                    cv2.rectangle(img, (self.green_box["x1"], self.green_box["y1"]), (self.green_box["x2"], self.green_box["y2"]), (0, 255, 0), 3)
                    cv2.putText(img, "greenBox", (self.green_box["x1"], self.green_box["y1"]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                elif self.max_blue_box != -1:
                    yaw_state = 7777.0
                    detected = True
                    cv2.rectangle(img, (self.blue_box["x1"], self.blue_box["y1"]), (self.blue_box["x2"], self.blue_box["y2"]), (255, 0, 0), 3)
                    cv2.putText(img, "blueBox", (self.blue_box["x1"], self.blue_box["y1"]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                else:
                    yaw_state = 9999.0
                    detected = False

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
        yaw_state = 9999.0
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

            mid_x = (x1 + x2) // 2
            width = img.shape[1] // 2
            yaw_state = (mid_x - width) * 0.5 # Scale DSC for box
            status = True
        elif self.class_names[cls] == "blueBox":
            yaw_state = 7777.0
            status = True
            
        return img, yaw_state, status

    def blue_box_detected(self, cls, img, x1, y1, x2, y2, confidence):
        status = False
        yaw_state = 9999.0
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
            
            mid_x = (x1 + x2) // 2
            width = img.shape[1] // 2
            yaw_state = (mid_x - width) * 0.5
            status = True
        elif self.class_names[cls] == "greenBox":
            yaw_state = 8888.0
            status = True
            
        return img, yaw_state, status

    def run(self):
        while True:
            frame = self.process_frame()
            if frame is None:
                break

            cv2.imshow("Webcam", frame)
            if cv2.waitKey(1) == ord("q"):
                break

    def process_unified(self, mission: MissionStatus, arena, img, cap, node):
        if img is None:
            return None, 0.0, False, False

        height, width = img.shape[:2]
        width = width // 2
        height = height // 2

        yaw_state = 0.0
        detected = False
        box_detected = False

        with torch.no_grad():
            results = self.model(img, conf=self.conf_threshold_buoy, verbose=False, stream=True)

            self.max_red = -1
            self.max_green = -1
            self.red = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
            self.green = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

            self.max_green_box = -1
            self.max_blue_box = -1 
            self.green_box = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}
            self.blue_box = {"x1": -1, "y1": -1, "x2": -1, "y2": -1}

            red_buoys_large = 0
            docking_buoys_centers = []

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = float(box.conf[0])
                    cls = int(box.cls[0])
                    
                    if confidence < 0.1:
                        continue

                    area = abs((x2 - x1) * (y2 - y1))
                    img_area = (width * 2) * (height * 2)
                    if area > 0.8 * img_area:
                        continue
                        
                    min_buoy = getattr(MissionParams, 'min_area_buoy', 600.0)
                    c_name = self.class_names[cls] if cls < len(self.class_names) else 'Unknown'
                    self.node.get_logger().info(f"[DEBUG YOLO] cls:{cls} name:{c_name} conf:{confidence:.2f} area:{area}", throttle_duration_sec=1.0)
                    
                    if area < min_buoy:
                        continue

                    # Buoys
                    if c_name in ["red buoy", "red-buoy", "redBuoy", "red_buoy"]:
                        color = (0, 0, 255)
                        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                        cv2.putText(img, f"{c_name} {confidence:.2f}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                        if self.max_red < area:
                            self.max_red = area
                            self.red = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                        if mission == MissionStatus.DOCKING:
                            docking_buoys_centers.append((x1 + x2) // 2)
                            min_dock = getattr(MissionParams, 'min_area_docking_buoy', 3000.0)
                            if area > min_dock:
                                red_buoys_large += 1
                            
                    elif c_name in ["green buoy", "green-buoy", "greenBuoy", "green_buoy"]:
                        color = (0, 255, 0)
                        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                        cv2.putText(img, f"{c_name} {confidence:.2f}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                        if self.max_green < area:
                            self.max_green = area
                            self.green = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                            
                    # Boxes
                    elif c_name == "greenBox":
                        color = (0, 255, 0)
                        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                        cv2.putText(img, f"greenBox {confidence:.2f}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                        if area > self.max_green_box:
                            self.max_green_box = area
                            self.green_box = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                            
                    elif c_name == "blueBox":
                        color = (255, 0, 0)
                        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                        cv2.putText(img, f"blueBox {confidence:.2f}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                        if area > self.max_blue_box:
                            self.max_blue_box = area
                            self.blue_box = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}

            # Buoy logic calculations
            buoy_detected_local = False
            buoy_yaw = 0.0
            
            if self.max_red < self.max_green * self.treshold:
                self.max_red = -1
            elif self.max_green < self.max_red * self.treshold:
                self.max_green = -1

            if self.max_red != -1 and self.max_green != -1:
                mid_red = (self.red["x1"] + self.red["x2"]) // 2
                mid_green = (self.green["x1"] + self.green["x2"]) // 2
                mid_x = (mid_green + mid_red) // 2
                buoy_yaw = mid_x - width
            elif self.max_green != -1:
                buoy_yaw = self.pid_adjust * (1 if arena == "A" else -1)
            elif self.max_red != -1:
                buoy_yaw = self.pid_adjust * (-1 if arena == "A" else 1)
                
            if self.max_green != -1 or self.max_red != -1:
                buoy_detected_local = True

            cv2.rectangle(img, (width, height), (width, height), (0,0,0), 3)

            # Box logic calculations
            box_detected_local = False
            box_yaw = 9999.0
            
            if self.max_green_box != -1 and self.max_blue_box != -1:
                mid_green = (self.green_box["x1"] + self.green_box["x2"]) // 2
                mid_blue = (self.blue_box["x1"] + self.blue_box["x2"]) // 2
                mid_x = (mid_green + mid_blue) // 2
                box_yaw = (mid_x - width) * 0.5
                box_detected_local = True
            elif self.max_green_box != -1:
                box_yaw = 8888.0
                box_detected_local = True
            elif self.max_blue_box != -1:
                box_yaw = 7777.0
                box_detected_local = True

            # Switch Logic Based on Mission
            if mission == MissionStatus.BUOY or mission == MissionStatus.TURN_NEXT_BUOY:
                # Priority Logic (Buoy > Box)
                if buoy_detected_local:
                    yaw_state = buoy_yaw
                    detected = True
                    box_detected = False # priority buoy
                else:
                    if box_detected_local:
                        yaw_state = box_yaw
                        detected = True 
                        box_detected = True # signal BT to switch to box mission if waiting
                    else:
                        yaw_state = 0.0
                        detected = False
                        box_detected = False
            
            elif mission == MissionStatus.BOTH_BOXES:
                yaw_state = box_yaw
                detected = box_detected_local
                
            elif mission == MissionStatus.GREEN_BOX:
                if self.max_green_box != -1:
                    mid_x = (self.green_box["x1"] + self.green_box["x2"]) // 2
                    yaw_state = (mid_x - width) * 0.5
                    detected = True
                elif self.max_blue_box != -1:
                    yaw_state = 7777.0
                    detected = True
                else:
                    yaw_state = 9999.0
                    detected = False
                    
            elif mission == MissionStatus.BLUE_BOX:
                if self.max_blue_box != -1:
                    mid_x = (self.blue_box["x1"] + self.blue_box["x2"]) // 2
                    yaw_state = (mid_x - width) * 0.5
                    detected = True
                elif self.max_green_box != -1:
                    yaw_state = 8888.0
                    detected = True
                else:
                    yaw_state = 9999.0
                    detected = False
                    
            elif mission == MissionStatus.DOCKING:
                if red_buoys_large >= 3:
                    detected = True
                else:
                    detected = False
                if len(docking_buoys_centers) > 0:
                    mid_x = sum(docking_buoys_centers) // len(docking_buoys_centers)
                    yaw_state = mid_x - width
                else:
                    yaw_state = 9999.0
                cv2.putText(img, f"Red Buoys > 3000 area: {red_buoys_large}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            return img, yaw_state, detected, box_detected
