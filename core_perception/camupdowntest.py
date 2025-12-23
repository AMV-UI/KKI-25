import pyudev
import cv2



up_idx = get_webcam_device_idx("046d_C270_HD_WEBCAM_E0198440")
down_idx = get_webcam_device_idx("Generic_HD_camera_20201212000000")

# Open the default camera
up_cam = cv2.VideoCapture(up_idx)
down_cam = cv2.VideoCapture(down_idx)

while True:
    _, up_frame = up_cam.read()
    _, down_frame = down_cam.read()

    # Display the captured frame
    cv2.imshow('Up Camera', up_frame)
    cv2.imshow('Down Camera', down_frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
up_cam.release()
down_cam.release()
cv2.destroyAllWindows()
