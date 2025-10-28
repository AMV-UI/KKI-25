#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

from utils.image.vision import enlightment


class Detector:
    def __init__(self,
                 model_path,
                 contrast=1,
                 brightness=1,
                 gamma=1,
                 conf_threshold=0.5,
                 nms_threshold=0.4):

        # Non Max Surpression property
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold

        # Image preprocessing property
        self.contrast = contrast
        self.brightness = brightness
        self.gamma = gamma

        # Init YOLOv4
        self.net = None
        self.path = os.path.abspath(model_path)
        self._init_yolo_model()

        print("cold start")
        
        whiteFrame = 255 * np.ones((1000,1000,3), np.uint8)
        # frame = np.zeros((480, 640))
        self.predict(whiteFrame)

    def _init_yolo_model(self):
        try:
            self.net = cv2.dnn.readNetFromDarknet(
                self.path + '/yolo.cfg', self.path + '/yolo.weights')
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            self.output_layers_name = self.net.getUnconnectedOutLayersNames()
            self.model = cv2.dnn_DetectionModel(self.net)
            self.model.setInputParams(
                size=(416, 416), scale=1/float(255.0), swapRB=True, crop=False)

            self.classes, self.inv_classes = self._read_class_names(
                self.path + '/classes.names')
            rospy.loginfo(
                f"[detector] Succesfully configured YOLOv4 model : {self.path} mission buoy")
        except:
            rospy.logerr("[detector] Error when configuring YOLOv4 model")

    def _read_class_names(self, path):
        names = {}
        with open(path, 'r') as data:
            for ID, name in enumerate(data):
                names[ID] = name.strip('\n')
        inv_classes = {v: k for k, v in names.items()}
        return names, inv_classes

    # YoloV4
    def predict(self, frame):
        if self.net == None:
            raise Exception(
                "Please init detector first, call Detector.init()")

        class_ids, confidences, boxes = self.model.detect(
            frame, self.conf_threshold, self.nms_threshold)

        self.res_boxes = np.asarray(boxes)
        self.res_confidences = np.asarray(confidences).flatten()
        self.res_class_ids = np.asarray(class_ids).flatten()

        return self.res_boxes, self.res_class_ids

    # YoloV4 w/ Coor
    # def predict_w_coor(self, frame):
    #     if self.net == None:
    #         raise Exception(
    #             "Please init detector first, call Detector.init()")

    #     class_ids, confidences, boxes = self.model.detect(
    #         frame, self.conf_threshold, self.nms_threshold)

    #     self.res_boxes = convert_to_bbox(boxes)
    #     self.res_confidences = np.asarray(confidences).flatten()
    #     self.res_class_ids = np.asarray(class_ids).flatten()

    #     return self.res_boxes, self.res_class_ids

    def draw(self, frame):
        cmap = plt.get_cmap('tab20b')
        colors = [cmap(i)[:3] for i in np.linspace(0, 1, 20)]

        for i in range(len(self.res_boxes)):
            x, y, w, h = [int(x) for x in self.res_boxes[i]]
            label = str(self.classes[self.res_class_ids[i]])
            confidence = str(round(self.res_confidences[i], 2))

            color = colors[self.res_class_ids[i] % len(colors)]
            color = [i * 255 for i in color]

            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            cv2.putText(frame, label + " - " + confidence,
                        (x, y-10), 0, 0.75, (255, 255, 255), 2)
