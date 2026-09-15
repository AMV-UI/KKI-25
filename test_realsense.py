import cv2
for i in range(10):
    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
    if cap.isOpened():
        ret, frame = cap.read()
        print(f"/dev/video{i} - Opened: True, Read: {ret}, Shape: {frame.shape if ret else 'None'}")
        cap.release()
    else:
        print(f"/dev/video{i} - Opened: False")
