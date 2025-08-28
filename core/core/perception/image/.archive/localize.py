#!/usr/bin/env python3

from tkinter.messagebox import NO
import numpy as np
import cv2
import rospy 
from utils.config import Param, SPEED, DetectedStatus
# from utils.image.detector_trt import TrtYOLO
from ultralytics import YOLO
from utils.motor import Motor

from kki24.msg import ObjectCount

# from utils.image.detector import Detector


from utils.config import Buoys, ModelPath, Tower, Box, Param

_pid_adjustment = 290
_threshold_area = 0.33 #TODO: beadin per misi
saved_frame_counter = 0

rev_mode = rospy.get_param(Param.REVERSEMODE)
buoy = Buoys()
tower = Tower()

#TODO: benerin reverse
# Hue Sat Val

threshold_data = {
    "red" : 1
}
class Threshold:
    def __init__ (self, color , low, high, bbox_color, low_2=None, high_2=None):
        self.COLOR = color
        self.LOW = np.array(low, dtype="uint8")
        self.HIGH = np.array(high, dtype="uint8")
        self.LOW_2 = np.array(low_2, dtype="uint8") if low_2 is not None else None
        self.HIGH_2 = np.array(high_2, dtype="uint8") if low_2 is not None else None
        self.BBOX_COLOR = bbox_color

    def masking(self, frame):
        mask = cv2.inRange(frame, self.LOW, self.HIGH)
        if self.LOW_2 is not None and self.HIGH_2 is not None:
            additional_mask = cv2.inRange(frame, self.LOW_2, self.HIGH_2)
            mask = mask + additional_mask
        return mask


# low=np.array([110, 56, 0]), 
# high=np.array([255, 255, 255]), 


# TODO : Tuning threshold, box tresh
red_thresh = Threshold(
    color=Tower.RED,
    low=np.array([0, 150, 20]), 
    high=np.array([9, 255, 255]), 
    low_2=np.array([118, 49, 20]), 
    high_2=np.array([179, 220, 255]), 
    bbox_color=(0, 0, 255)
    )

# green_thresh = Threshold(
#     color=Tower.GREEN,
#     low=np.array([69, 43, 0]), 
#     high=np.array([99, 255, 255]), 
#     bbox_color=(0, 255, 0)
#     )

green_thresh = Threshold(
    color=Tower.GREEN,
    low=np.array([50, 50, 20]), 
    high=np.array([179, 255, 255]), 
    bbox_color=(0, 255, 0)
    )

blue_box_thresh = Threshold(
    color=Box.BLUE,
    low=np.array([91, 53, 96]),
    high=np.array([110, 255, 255]),
    bbox_color=(255, 0, 0)
    )

green_box_thresh = Threshold(
    color=Box.GREEN,
    low=np.array([70, 55, 55]), 
    high=np.array([100, 255, 255]), 
    bbox_color=(0, 255, 0)
)

THRESH_DICT = {Box.BLUE : blue_box_thresh, Box.GREEN : green_box_thresh}
THRESH_ARR = [red_thresh, green_thresh]


# All Model

detector_buoy = None
detector_cone = None
detector_box_vest = None

def init_model():
    # global detector_buoy
    # global detector_cone
    # global detector_box_vest 
    #detector_box_vest = TrtYOLO(ModelPath.box_vest, threshold=0.8)
    ## detector_cone = TrtYOLO(ModelPath.cone)
    detector_buoy = YOLO("/home/amv/main_ws/src/kki24/scripts/model/buoy/buoy.engine")
    ## detector_cone = TrtYOLO(ModelPath.cone)

def imageProcessingBuoy(frame, finding=False, headings=None, cur_heading=None, reverse=False):
    """ Process image with YOLO detector

    Params:
     - detector: Detector
     - frame: Image
    --------------------
    Returns:
     - yaw_state: Float -> return yaw state after processing image
     - is_detected: Bool
     - frame: returns processed output frame w/ drawn bounding box
    --------------------
    Variables
    --------------------
    - indexes: [[x,y,w,h],...] -> contains bbox data of detected obj.
    - class_ids: [int,...] -> contains the index of the detected objects in indexes
        : class_ids = 0 -> red_buoy
        : class_ids = 1 -> green_buoy
    - frame_height, frame_width: int
    - is_detected: bool -> 1 if one or more object is detected, 0 otherwise
    """
    global detector_buoy
    indexes, confidence, class_ids = detector_buoy.predict(frame)
    # Default value
    rev = rospy.get_param(Param.REVERSEMODE)
    RED = int(rev) if reverse else int(not rev) # normal 1
    GREEN = int(not rev) if reverse else int(rev) # normal 0

    frame_height, frame_width, _ = frame.shape
    frame_center = (frame_width // 2, frame_height // 2)
    yaw_state = frame_center[0]
    is_detected = DetectedStatus.NOT_DETECTED
    is_reverse_buoy = False

    # obj_count = ObjectCount()
    # obj_count.red = 0
    # obj_count.green = 0
    closest_red, closest_green = 0, 0
    closest_red_mid_point = 0
    closest_green_mid_point = 0


    # TODO: bikin lebih efisien
    for i, class_id in enumerate(class_ids):
        if class_id == RED:
            # obj_count.red += 1
            # red_buoys.append(indexes[i])
            x_min, y_min, x_max, y_max = indexes[i]
            area = (x_max - x_min) * (y_max - y_min)

            if area > closest_red:
                closest_red_mid_point = getMidPoint(indexes[i])
                closest_red = area
                mid_x_red = (x_max + x_min) // 2
                mid_y_red = (y_max + y_min) // 2
                
        if class_id == GREEN:
            # obj_count.green += 1
            # green_buoys.append(indexes[i])
            x_min, y_min, x_max, y_max = indexes[i]
            area = (x_max - x_min) * (y_max - y_min)

            if area > closest_green:
                closest_green_mid_point = getMidPoint(indexes[i])
                closest_green = area
                mid_x_green = (x_max + x_min) // 2
                mid_y_green = (y_max + y_min) // 2
    

    if closest_red > 0 and closest_green > 0:
        if closest_red < closest_green * _threshold_area:
            # obj_count.red = 0
            closest_red = 0
        elif closest_green < closest_red * _threshold_area:
            # obj_count.green = 0
            closest_green = 0
    
    # Calculate yaw_state from buoy count
    if closest_red > 0 and closest_green == 0:
        is_detected = DetectedStatus.ONE_DETECTED
        yaw_state = frame_center[0] + _pid_adjustment
    elif closest_red == 0 and closest_green > 0:
        is_detected = DetectedStatus.ONE_DETECTED
        yaw_state = frame_center[0] - _pid_adjustment
    elif closest_red > 0 and closest_green > 0:
        is_detected = DetectedStatus.BOTH_DETECTED
        yaw_state = (mid_x_red + mid_x_green) // 2
        # Reverse buoy
        if finding:
            # if reverse:
            #     if closest_red_mid_point > closest_green_mid_point: # TODO: cek bener ga soalnya di config buoy nya juga bakal dibalik
            #         print(">>>>>>>>>>>>>>>>>>>>> REVERSE!!!")
            #         # rospy.set_param(Param.REVERSEMODE, False)
            #         is_reverse_buoy = True
                    
            if not reverse:
                if closest_red_mid_point > closest_green_mid_point:
                    print(">>>>>>>>>>>>>>>>>>>>> REVERSE!!!")
                    # rospy.set_param(Param.REVERSEMODE, True)
                    is_reverse_buoy = True
                    print("IS REVERSE BUOY" , is_reverse_buoy)
                    
        # if not finding and headings is not None and cur_heading is not None:
        #     min_deg, max_deg = headings
        #     if min_deg < cur_heading < max_deg:
        #         if rospy.get_param(Param.REVERSEMODE):
        #             if closest_red_mid_point < closest_green_mid_point:
        #                 print(">>>>>>>>>>>>>>>>>>>>> REVERSE!!!")
        #                 rospy.set_param(Param.REVERSEMODE, False)
        #         elif not rospy.get_param(Param.REVERSEMODE):
        #             if closest_red_mid_point > closest_green_mid_point:
        #                 print(">>>>>>>>>>>>>>>>>>>>> REVERSE!!!")
        #                 rospy.set_param(Param.REVERSEMODE, True)
                
    elif closest_red == 0 and closest_green == 0:
        yaw_state = frame_center[0]


    detector_buoy.draw(frame, indexes, confidence, class_ids)
    cv2.line(frame, (yaw_state, 0), (yaw_state,
             frame_height), (255, 255, 255), 2)
    
    # left_is_red = rospy.get_param(Param.REVERSEMODE)
    
    return yaw_state, is_detected, frame, is_reverse_buoy

# def imageProcessingBuoyTest(detector: Detector, frame):
#     """ Process image with YOLO detector

#     Params:
#      - detector: Detector
#      - frame: Image
#     --------------------
#     Returns:
#      - yaw_state: Float -> return yaw state after processing image
#      - is_detected: Bool
#      - frame: returns processed output frame w/ drawn bounding box
#     --------------------
#     Variables
#     --------------------
#     - indexes: [[x,y,w,h],...] -> contains bbox data of detected obj.
#     - class_ids: [int,...] -> contains the index of the detected objects in indexes
#         : class_ids = 0 -> red_buoy
#         : class_ids = 1 -> green_buoy
#     - frame_height, frame_width: int
#     - is_detected: bool -> 1 if one or more object is detected, 0 otherwise
#     """
#     indexes, class_ids = detector.predict(frame)
#     frame_height, frame_width, _ = frame.shape
#     frame_center = (frame_width // 2, frame_height // 2)
#     yaw_state = frame_center[0]
#     is_detected = len(indexes) > 0

#     # Declare buoys
#     red_buoy = ObjectHandler(obj_name=Buoys.RED.name,
#                              class_id=Buoys.RED, count=0)
#     green_buoy = ObjectHandler(obj_name=Buoys.GREEN.name,
#                                class_id=Buoys.GREEN, count=0)
#     buoy_map = {
#         Buoys.RED.name: red_buoy,
#         Buoys.GREEN.name: green_buoy,
#     }

#     closest_red, closest_green = None, None
#     closest_red_area, closest_green_area = None, None

#     for i, class_id in enumerate(class_ids):
#         for e in Buoys:
#             if class_id == e.value:
#                 buoy_map[e.name].add_indexes(indexes[i])

#    # Setting initial max area/closest buoy
#     if red_buoy.is_detected():
#         closest_red = red_buoy.indexes[0]
#     if green_buoy.is_detected():
#         closest_green = green_buoy.indexes[0]

#     # Find closest red buoy by finding max area of red buoys
#     for red_buoy_idx in red_buoy.indexes:
#         _, _, w, h = red_buoy_idx
#         _, _, w_closest, h_closest = closest_red
#         area = w * h
#         area_max = w_closest * h_closest

#         if area > area_max:
#             closest_red = red_buoy_idx

#     # Find closest green buoy by finding max area of green buoys
#     for green_buoy_idx in green_buoy.indexes:
#         _, _, w, h = green_buoy_idx
#         _, _, w_closest, h_closest = closest_green
#         area = w * h
#         area_max = w_closest * h_closest

#         if area > area_max:
#             closest_green = green_buoy_idx

#     # Find the area of the closest red and green buoy
#     if closest_red is not None:
#         x_red, y_red, w_red, h_red = closest_red

#         leftmost_x = x_red + (w_red // 2)
#         leftmost_y = y_red + (h_red // 2)
#         closest_red_area = w_red * h_red

#     if closest_green is not None:
#         x_green, y_green, w_green, h_green = closest_green

#         rightmost_x = x_green + (w_green // 2)
#         rightmost_y = y_green + (h_green // 2)
#         closest_green_area = w_green * h_green

#     if closest_red_area is not None and closest_green_area is not None:
#         if closest_red_area < closest_green_area * _threshold_area:
#             red_buoy.count = 0
#         elif closest_green_area < closest_red_area * _threshold_area:
#             green_buoy.count = 0

#     # Calculate yaw_state from buoy count
#     if red_buoy.count > 0 and green_buoy.count == 0:
#         yaw_state = frame_center[0] + _pid_adjustment
#     elif red_buoy.count == 0 and green_buoy.count > 0:
#         yaw_state = frame_center[0] - _pid_adjustment
#     elif red_buoy.count > 0 and green_buoy.count > 0:
#         yaw_state = (leftmost_x + rightmost_x) // 2
#     elif red_buoy.count == 0 and green_buoy.count == 0:
#         yaw_state = frame_center[0]

#     detector.draw(frame)
#     cv2.line(frame, (yaw_state, 0), (yaw_state,
#              frame_height), (255, 255, 255), 2)

#     return yaw_state, is_detected, frame

def getMidPoint(bbox):
    x_min, _, x_max, __ = bbox
    return (x_max + x_min) // 2

def getAreaFromBBox(bbox):
    x_min, y_min, x_max, y_max = bbox
    return (x_max - x_min) * (y_max - y_min)
