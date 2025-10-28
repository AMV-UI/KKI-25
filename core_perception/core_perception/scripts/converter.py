#!/usr/bin/env python3

from ultralytics import YOLO
model = YOLO("/home/amv/models/v12/best_v12.pt")
model.export(format="engine")
