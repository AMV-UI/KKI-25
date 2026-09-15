import cv2
cap = cv2.VideoCapture(3, cv2.CAP_V4L2)
print("video3 isOpened:", cap.isOpened())
if cap.isOpened():
    ret, frame = cap.read()
    print("video3 read:", ret)
cap.release()
