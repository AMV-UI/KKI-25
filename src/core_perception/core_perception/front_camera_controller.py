import cv2
from pyzbar.pyzbar import decode
from core.utils.config import Topic
from core_perception.base_camera_controller import BaseCameraNode
from std_msgs.msg import String


class FrontCameraNode(BaseCameraNode):
    def __init__(self):
        super().__init__(
            node_name="front_camera",
            camera_identifier="046d_C270_HD_WEBCAM_55E22480",
            stream_url="rtsp://localhost:8554/live/frontcam",
        )
        self.qr_data_pub = Topic.qr_side.createPublisher(self)

    def process_and_publish(self, frame):
        decoded_objects = decode(frame)

        for obj in decoded_objects:
            data = obj.data.decode("utf-8")

            self.get_logger().info(
                f"QR Code Detected: {data}", throttle_duration_sec=2.0
            )

            qr_msg = String()
            qr_msg.data = data
            self.qr_data_pub.publish(qr_msg)

            points = obj.polygon
            if len(points) == 4:
                pts = [(pt.x, pt.y) for pt in points]
                for i in range(4):
                    cv2.line(frame, pts[i], pts[(i + 1) % 4], (0, 255, 0), 3)

                text_x = pts[0][0]
                text_y = max(pts[0][1] - 10, 20)

                cv2.putText(
                    frame,
                    data,
                    (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
