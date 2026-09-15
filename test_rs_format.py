import cv2
for idx in [2,3,4,5]:
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"video{idx} failed to open")
        continue
    ret, frame = cap.read()
    if ret:
        is_gray = (frame[:,:,0] == frame[:,:,1]).all() and (frame[:,:,1] == frame[:,:,2]).all()
        print(f"video{idx}, Shape: {frame.shape}, Is gray? {is_gray}")
    cap.release()
