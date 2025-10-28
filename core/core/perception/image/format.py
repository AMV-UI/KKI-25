import rospy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2
import time


def compressImage(frame):
    """
    Compress image from cv2 to cv2Bridge
    """
    compressed_image = CompressedImage()
    compressed_image.header.stamp = rospy.Time.now()
    compressed_image.format = "jpeg"
    compressed_image.data = np.array(
        cv2.imencode(".jpg", frame)[1]).tostring()
    return compressed_image


class FPSCounter():
    def start(self):
        self.start_time = time.time()

    def calculate(self, frame):
        height, width, _= frame.shape
        font = cv2.FONT_HERSHEY_PLAIN

        time_diff = time.time() - self.start_time
        fps = 1.0/time_diff

        cv2.putText(frame, "FPS: " + str(round(fps, 2)),
                    (0, height), font, 2, (0, 255, 0), 2)


