import cv2
import sys
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
if not cap.isOpened():
    print("Cannot open /dev/video2 with V4L2")
    # try without V4L2
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("Cannot open /dev/video2 with auto backend")
        sys.exit(1)

print("Camera opened successfully!")
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# cap.set(cv2.CAP_PROP_FPS, 30)

ret, frame = cap.read()
print(f"Read success: {ret}")
if ret:
    print(f"Frame shape: {frame.shape}")
cap.release()
