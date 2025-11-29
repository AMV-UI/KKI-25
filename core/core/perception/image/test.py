from ultralytics import YOLO

model = YOLO("/home/amv/models/v12/best_v12.engine")

model.predict(source="/dev/video0", show=True)