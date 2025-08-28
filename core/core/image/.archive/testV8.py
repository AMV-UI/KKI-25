import cv2
import numpy as np
from detectorV8 import TrtYOLOv8
def main():
    # Initialize the model
    model_path = "/home/amv/main_ws/src/web-gcs/v8/buoy.engine"  # Adjust this to your model path
    yolo = TrtYOLOv8(model_path)

    # Open a connection to the webcam
    cap = cv2.VideoCapture(0)  # 0 for the default camera

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        # Capture frame-by-frame
        ret, img = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Run inference
        boxes, scores, classes = yolo.predict(img)

        # Draw the results on the frame
        yolo.draw(img, boxes, scores, classes)

        # Display the output frame
        cv2.imshow("Detections", img)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
