import cv2
cap = cv2.VideoCapture(4, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'UYVY'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ret, frame = cap.read()
if ret:
    print(f"Success! Shape: {frame.shape}, Channels: {frame.shape[2]}")
else:
    print("Failed to read")
cap.release()
