#!/usr/bin/env python3
from ultralytics import YOLO
model = YOLO("/home/amv/models/KKI-25/box_v1.pt")
model.export(format="engine")
