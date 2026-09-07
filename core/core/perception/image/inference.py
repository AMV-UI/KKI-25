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

        # Tracker for depth fallback
        self.tracker = {} # {"redBuoy": {"box": (x1, y1, x2, y2), "center": (cx, cy), "depth": z, "missed": 0, "area": area}}
        self.MAX_MISSED_FRAMES = getattr(MissionParams, 'depth_missed_frames', 45)

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

    def update_depth_tracker(self, cap, img):
        if not hasattr(cap, 'depth_image') or cap.depth_image is None or not hasattr(cap, 'depth_scale'):
            return img

        import numpy as np
        depth_meters = cap.depth_image * cap.depth_scale

        # Get depth blobs
        mask = ((depth_meters > 0.3) & (depth_meters < 6.0)).astype(np.uint8) * 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w//2, y + h//2
                z = depth_meters[cy, cx]
                if z == 0:
                    roi = depth_meters[y:y+h, x:x+w]
                    valid_depths = roi[roi > 0]
                    if len(valid_depths) > 0:
                        z = np.mean(valid_depths)
                if z > 0:
                    blobs.append({'box': (x, y, x+w, y+h), 'center': (cx, cy), 'area': area, 'depth': z})

        # Process each tracked target category
        targets = {
            'redBuoy': [self.max_red, self.red],
            'greenBuoy': [self.max_green, self.green],
            'blueBox': [self.max_blue_box if hasattr(self, 'max_blue_box') else -1, self.blue_box],
            'greenBox': [self.max_green_box if hasattr(self, 'max_green_box') else -1, self.green_box],
            'blueDock': [self.max_blue_dock if hasattr(self, 'max_blue_dock') else -1, getattr(self, 'blue_dock', {'x1': -1, 'y1': -1, 'x2': -1, 'y2': -1})],
            'redDock': [self.max_red_dock if hasattr(self, 'max_red_dock') else -1, getattr(self, 'red_dock', {'x1': -1, 'y1': -1, 'x2': -1, 'y2': -1})],
            'greenDock': [self.max_green_dock if hasattr(self, 'max_green_dock') else -1, getattr(self, 'green_dock', {'x1': -1, 'y1': -1, 'x2': -1, 'y2': -1})]
        }

        for key, (max_area, box_dict) in targets.items():
            if max_area != -1:
                cx = (box_dict['x1'] + box_dict['x2']) // 2
                cy = (box_dict['y1'] + box_dict['y2']) // 2
                z = depth_meters[cy, cx]
                if z == 0:
                    roi = depth_meters[box_dict['y1']:box_dict['y2'], box_dict['x1']:box_dict['x2']]
                    valid_depths = roi[roi > 0]
                    if len(valid_depths) > 0:
                        z = np.mean(valid_depths)
                if z > 0:
                    self.tracker[key] = {'box': (box_dict['x1'], box_dict['y1'], box_dict['x2'], box_dict['y2']), 'center': (cx, cy), 'depth': z, 'missed': 0, 'area': max_area}
            else:
                if key in self.tracker:
                    tracked = self.tracker[key]
                    tracked['missed'] += 1
                    max_missed_frames = getattr(MissionParams, 'depth_missed_frames', 45)
                    if tracked['missed'] < max_missed_frames:
                        best_blob = None
                        min_dist = float('inf')
                        for blob in blobs:
                            import math
                            dist = math.hypot(blob['center'][0] - tracked['center'][0], blob['center'][1] - tracked['center'][1])
                            z_diff = abs(blob['depth'] - tracked['depth'])
                            if dist < 100 and z_diff < 0.5:
                                if dist < min_dist:
                                    min_dist = dist
                                    best_blob = blob
                        
                        if best_blob:
                            new_box = best_blob['box']
                            tracked['box'] = new_box
                            tracked['center'] = best_blob['center']
                            tracked['depth'] = best_blob['depth']
                            tracked['area'] = best_blob['area']
                            
                            box_dict['x1'], box_dict['y1'], box_dict['x2'], box_dict['y2'] = new_box
                            
                            if key == 'redBuoy': self.max_red = tracked['area']
                            elif key == 'greenBuoy': self.max_green = tracked['area']
                            elif key == 'blueBox': self.max_blue_box = tracked['area']
                            elif key == 'greenBox': self.max_green_box = tracked['area']
                            elif key == 'blueDock': self.max_blue_dock = tracked['area']
                            elif key == 'redDock': self.max_red_dock = tracked['area']
                            elif key == 'greenDock': self.max_green_dock = tracked['area']

                            cv2.rectangle(img, (new_box[0], new_box[1]), (new_box[2], new_box[3]), (0, 255, 255), 3)
                            cv2.putText(img, f"DEPTH {key}", (new_box[0], new_box[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    else:
                        del self.tracker[key]
        return img

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
            self.blue_dock_centers = []

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
                    
                    c_name = self.class_names[cls] if cls < len(self.class_names) else 'Unknown'
                    is_box = "box" in c_name.lower()
                    
                    # Fetch configuration parameters
                    if is_box:
                        min_area_limit = getattr(MissionParams, 'min_area_box', 200.0)
                        max_area_limit = getattr(MissionParams, 'max_pixel_area_box', 50000.0)
                        min_ar_limit = getattr(MissionParams, 'min_aspect_ratio_box', 0.2)
                        max_ar_limit = getattr(MissionParams, 'max_aspect_ratio_box', 3.0)
                    else:
                        min_area_limit = getattr(MissionParams, 'min_area_buoy', 200.0)
                        max_area_limit = getattr(MissionParams, 'max_pixel_area', 50000.0)
                        min_ar_limit = getattr(MissionParams, 'min_aspect_ratio', 0.2)
                        max_ar_limit = getattr(MissionParams, 'max_aspect_ratio', 3.0)

                    # 1. Pixel Area Filter
                    if area < min_area_limit or area > max_area_limit:
                        continue
                        
                    # 2. Aspect Ratio Filter (Noise/Glitches)
                    box_width = x2 - x1
                    box_height = y2 - y1
                    if box_height == 0: 
                        continue
                    aspect_ratio = float(box_width) / float(box_height)
                    
                    if aspect_ratio < min_ar_limit or aspect_ratio > max_ar_limit:
                        continue
                        
                    # DEBUG: Print all VALID detections
                    self.node.get_logger().info(f"[DEBUG YOLO] cls:{cls} name:{self.class_names[cls] if cls < len(self.class_names) else 'Unknown'} conf:{confidence:.2f} area:{area} AR:{aspect_ratio:.2f} mission:{mission}", throttle_duration_sec=1.0)

                    if mission == MissionStatus.BUOY or mission == MissionStatus.TURN_NEXT_BUOY:
                        img, self.red, self.green = self.buoy_detected(
                            cls, img, x1, y1, x2, y2, confidence, area
                        )


                    elif mission == MissionStatus.DOCKING or mission == MissionStatus.DOCKING_V2:
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
            if self.blue_model and (mission == MissionStatus.DOCKING or mission == MissionStatus.DOCKING_V2):
                blue_results = self.blue_model(img, conf=self.conf_threshold_buoy, verbose=False, stream=True)
                valid_blue_buoys = []
                
                for r in blue_results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = float(box.conf[0])
                        if confidence < 0.1: continue
                        cls = int(box.cls[0])
                        c_name = self.blue_class_names[cls] if cls < len(self.blue_class_names) else 'Unknown'
                        area = abs((x2 - x1) * (y2 - y1))
                        # Fetch configuration parameters
                        min_buoy = getattr(MissionParams, 'min_area_buoy', 200.0)
                        max_area = getattr(MissionParams, 'max_pixel_area', 50000.0)
                        min_ar = getattr(MissionParams, 'min_aspect_ratio', 0.2)
                        max_ar = getattr(MissionParams, 'max_aspect_ratio', 3.0)

                        # 1. Pixel Area Filter
                        if area < min_buoy or area > max_area:
                            continue
                            
                        # 2. Aspect Ratio Filter (Noise/Glitches)
                        box_width = x2 - x1
                        box_height = y2 - y1
                        if box_height == 0: 
                            continue
                        aspect_ratio = float(box_width) / float(box_height)
                        
                        if aspect_ratio < min_ar or aspect_ratio > max_ar:
                            continue
                        
                        if c_name in ["blue-buoy", "blueBuoy", "blue buoy", "bluebuoy"]:
                            valid_blue_buoys.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2, "area": area})

                # Filter blue buoys using threshold logic (same as red/green)
                if len(valid_blue_buoys) > 0:
                    # Cari buoy biru paling besar
                    max_blue_area = max([b["area"] for b in valid_blue_buoys])
                    
                    for b in valid_blue_buoys:
                        # Abaikan buoy yang ukurannya jauh lebih kecil dari buoy terbesar (noise)
                        if b["area"] < max_blue_area * self.treshold:
                            continue
                            
                        # Gambar buoy biru yang valid
                        color = (255, 0, 0)
                        cv2.rectangle(img, (b["x1"], b["y1"]), (b["x2"], b["y2"]), color, 3)
                        cv2.putText(img, f"blue_buoy ({b['area']})", (b["x1"], b["y1"]), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                        self.blue_dock_centers.append((b["x1"] + b["x2"]) // 2)
                        
                        if b["area"] > self.max_blue_dock:
                            self.max_blue_dock = b["area"]
                            self.blue_dock = {"x1": b["x1"], "y1": b["y1"], "x2": b["x2"], "y2": b["y2"]}

            self.blue_area = float(self.max_blue_dock)
            
            img = self.update_depth_tracker(cap, img)

            if mission == MissionStatus.BUOY or mission == MissionStatus.TURN_NEXT_BUOY: 
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
            elif mission == MissionStatus.DOCKING or mission == MissionStatus.DOCKING_V2:
                if self.max_blue_dock != -1:
                    mid_blue = (self.blue_dock["x1"] + self.blue_dock["x2"]) // 2
                    yaw_state = mid_blue - width
                    
                    z = 999.0
                    if 'blueDock' in getattr(self, 'tracker', {}):
                        z = self.tracker['blueDock']['depth']
                    
                    if z > 0 and z < 1.5:  # Trigger done if within 1.5 meters
                        detected = True
                        cv2.putText(img, f"DOCKING REACHED: {z:.2f}m", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    else:
                        detected = False
                        if z != 999.0:
                            cv2.putText(img, f"DOCKING DIST: {z:.2f}m", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
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
                    
                    c_name = self.class_names[cls] if cls < len(self.class_names) else 'Unknown'
                    is_box = "box" in c_name.lower()
                    
                    # Fetch configuration parameters
                    if is_box:
                        min_area_limit = getattr(MissionParams, 'min_area_box', 200.0)
                        max_area_limit = getattr(MissionParams, 'max_pixel_area_box', 50000.0)
                        min_ar_limit = getattr(MissionParams, 'min_aspect_ratio_box', 0.2)
                        max_ar_limit = getattr(MissionParams, 'max_aspect_ratio_box', 3.0)
                    else:
                        min_area_limit = getattr(MissionParams, 'min_area_buoy', 200.0)
                        max_area_limit = getattr(MissionParams, 'max_pixel_area', 50000.0)
                        min_ar_limit = getattr(MissionParams, 'min_aspect_ratio', 0.2)
                        max_ar_limit = getattr(MissionParams, 'max_aspect_ratio', 3.0)

                    # 1. Pixel Area Filter
                    if area < min_area_limit or area > max_area_limit:
                        continue
                        
                    # 2. Aspect Ratio Filter (Noise/Glitches)
                    box_width = x2 - x1
                    box_height = y2 - y1
                    if box_height == 0: 
                        continue
                    aspect_ratio = float(box_width) / float(box_height)
                    
                    if aspect_ratio < min_ar_limit or aspect_ratio > max_ar_limit:
                        continue
                        
                    c_name = self.class_names[cls] if cls < len(self.class_names) else 'Unknown'
                    self.node.get_logger().info(f"[DEBUG YOLO] cls:{cls} name:{c_name} conf:{confidence:.2f} area:{area} AR:{aspect_ratio:.2f}", throttle_duration_sec=1.0)

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

            img = self.update_depth_tracker(cap, img)

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
                buoy_yaw = self.pid_adjust * (-1 if arena == "A" else 1)
            elif self.max_red != -1:
                buoy_yaw = self.pid_adjust * (1 if arena == "A" else -1)
                
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
                    
            elif mission == MissionStatus.DOCKING or mission == MissionStatus.DOCKING_V2:
                if self.max_blue_dock != -1:
                    if len(self.blue_dock_centers) > 0:
                        mid_x = sum(self.blue_dock_centers) // len(self.blue_dock_centers)
                        yaw_state = mid_x - width
                    else:
                        yaw_state = 0.0

                    z = 999.0
                    if 'blueDock' in getattr(self, 'tracker', {}):
                        z = self.tracker['blueDock']['depth']
                    
                    if z > 0 and z < 1.5:  # Trigger done if within 1.5 meters
                        detected = True
                        cv2.putText(img, f"DOCKING REACHED: {z:.2f}m", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    else:
                        detected = False
                        if z != 999.0:
                            cv2.putText(img, f"DOCKING DIST: {z:.2f}m", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                else:
                    yaw_state = 9999.0
                    detected = False

            elif mission == MissionStatus.DOCKING_V2:
                if self.max_blue_dock != -1:
                    if len(self.blue_dock_centers) > 0:
                        mid_x = sum(self.blue_dock_centers) // len(self.blue_dock_centers)
                        yaw_state = mid_x - width
                    else:
                        yaw_state = 0.0

                    z = 999.0
                    if 'blueDock' in getattr(self, 'tracker', {}):
                        z = self.tracker['blueDock']['depth']
                    
                    if z > 0 and z < 1.5:  
                        detected = True
                        cv2.putText(img, f"DOCKING REACHED: {z:.2f}m", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    else:
                        detected = False
                        if z != 999.0:
                            cv2.putText(img, f"DOCKING DIST: {z:.2f}m", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                else:
                    yaw_state = 9999.0
                    detected = False

            return img, yaw_state, detected, box_detected
