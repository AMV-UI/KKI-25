#!/usr/bin/env python3

import traceback
import cv2
import rclpy
import numpy as np
import base64
import time
from datetime import datetime
from core.perception.image.inference import ObjectDetector
from core_msgs.msg import StateObject, AutoControl
from core.utils.config import NodeConfig, Topic, MissionStatus
from core.utils.device_fetching import get_webcam_device_idx
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import Float64, Bool, String

class RealSenseCamera:
    """Wrapper for Intel RealSense to mimic cv2.VideoCapture interface (RGB only)"""
    def __init__(self, width=640, height=480, fps=30, json_path=None):
        import pyrealsense2 as rs
        import os
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.profile = self.pipeline.start(self.config)
        
        # Load JSON config if provided (must be applied after device is active)
        if json_path and os.path.exists(json_path):
            try:
                dev = self.profile.get_device()
                advnc_mode = rs.rs400_advanced_mode(dev)
                with open(json_path, 'r') as f:
                    json_string = f.read()
                advnc_mode.load_json(json_string)
            except Exception as e:
                print(f"[RealSense] Warning: Failed to apply JSON config: {e}")

        self.width = width
        self.height = height
        self.fps = fps

    def read(self):
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                return False, None
            color_image = np.asanyarray(color_frame.get_data())
            return True, color_image
        except Exception:
            return False, None

    def release(self):
        try:
            self.pipeline.stop()
        except:
            pass

    def get(self, propId):
        if propId == 3: # CAP_PROP_FRAME_WIDTH
            return self.width
        if propId == 4: # CAP_PROP_FRAME_HEIGHT
            return self.height
        if propId == 5: # CAP_PROP_FPS
            return self.fps
        return 0
        
    def set(self, propId, value):
        pass

class CameraController(Node):
    """
    Front Camera Node for Object Detection
    
    PUBLISHES TO:
        - dsc: Float64 - Yaw control effort
        - detected: Bool - Detection status
    """

    def __init__(self):
        super().__init__("front_camera")

        self.arena = "B"

        self.up_camera_serial_idx = get_webcam_device_idx('046d_C270_HD_WEBCAM_E0198440')
        self.down_camera_serial_idx = get_webcam_device_idx('Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001')

        self.buoy_detector = ObjectDetector(
            "/home/amv/models/KKI-25/buoy_v1.engine",
            self,
            [
                "green_buoy",
                "red_buoy",
            ],
            blue_model_path="/home/amv/models/KKI-25/bluebuoy.engine",
            blue_class_names=["bluebuoy"]
        )

        self.box_detector = ObjectDetector(
            "/home/amv/models/KKI-25/box_v1.engine",
            self,
            [
                "blueBox",
                "greenBox",
            ],
        )

        try:
            # Anda dapat memuat file JSON dari RealSense Viewer dengan mengisi path-nya di bawah ini.
            # Contoh: json_path='/home/amv/realsense_config.json'
            self.up_cap = RealSenseCamera(width=640, height=480, fps=30, json_path=None)
            self.get_logger().info("Using Intel RealSense for up_cap (RGB only)")
        except Exception as e:
            self.get_logger().info(f"Could not initialize RealSense ({e}), using standard webcam")
            self.up_cap = cv2.VideoCapture(self.up_camera_serial_idx)
            self.up_cap.set(5, 30)  # Set FPS (CAP_PROP_FPS is 5)
            self.up_cap.set(3, 640)  # Set width
            self.up_cap.set(4, 480)  # Set height

        # Configure down camera
        self.down_cap = cv2.VideoCapture(self.down_camera_serial_idx)
        self.down_cap.set(5, 30)  # Set FPS (CAP_PROP_FPS is 5)
        self.down_cap.set(3, 640)  # Set width
        self.down_cap.set(4, 480)  # Set height

        self.REC_UP_CAMERA = False
        self.REC_DOWN_CAMERA = False

        if self.REC_UP_CAMERA:
            fps = 15
            size = int(self.up_cap.get(3)), int(self.up_cap.get(4))
            file_name = "core_perception/videos/" + time.strftime("%d-%m-%Y_%H:%M:%S_up_cam.mp4", time.localtime())
            self.up_vid_writer = cv2.VideoWriter(file_name, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if self.REC_DOWN_CAMERA:
            fps = 15
            size = int(self.down_cap.get(3)), int(self.down_cap.get(4))
            file_name = "core_perception/videos/" + time.strftime("%d-%m-%Y_%H:%M:%S_down_cam.mp4", time.localtime())
            self.down_vid_writer = cv2.VideoWriter(file_name, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)

        self.result = ""
        self.dsc = 0
        self.state = [0, 0, 0, 0]
        self.img = None
        self.img_64 = ""
        self.mission_type = MissionStatus.BUOY
        self.show_result = False
        self.detected = False
        self.fps = 30

        # Setup communication
        self._setup_communication()
        
        self.get_logger().info(f"<> [{NodeConfig.camera_front}] Successfully initialized node")


    def _get_rc6_state(self, rc6_value):
        """Determine RC6 button state based on value"""
        if rc6_value < self.PWM_LOW:
            return 'LOW'
        elif self.PWM_LOW <= rc6_value < self.PWM_HIGH:
            return 'MID'
        else:
            return 'HIGH'
        
    def _take_upper_photo(self):
        ret, frame = self.up_cap.read()
        if ret:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            photo_filename = f"/home/amv/KKI-25/core_perception/photos/{timestamp}_upCamera.jpg"
            cv2.imwrite(photo_filename, frame)
            self.get_logger().info(f"Photo taken and saved to {photo_filename}", throttle_duration_sec=5.0)
            self.green_box_pub.publish(self.encode_base64(frame))
        else:
            self.get_logger().error("Failed to capture image from upper camera")

    def _take_down_photo(self):
        ret, frame = self.down_cap.read()
        if ret:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
            photo_filename = f"/home/amv/KKI-25/core_perception/photos/{timestamp}_downCamera.jpg"
            cv2.imwrite(photo_filename, frame)
            self.get_logger().info(f"Photo taken and saved to {photo_filename}", throttle_duration_sec=5.0)
            self.blue_box_pub.publish(self.encode_base64(frame))
        else:
            self.get_logger().error("Failed to capture image from down camera")

    def _rc6_callback(self, msg):
        """Handle RC6 commands for photo capture"""
        self.rc6 = msg.data
        current_state = self._get_rc6_state(self.rc6)
        if current_state == 'MID' and self.rc6_state != 'MID':
            self._take_upper_photo()
        elif current_state == 'HIGH' and self.rc6_state != 'HIGH':
            self._take_down_photo()
        self.rc6_state = current_state

    def _setup_communication(self):
        """Initialize publishers and subscribers"""
        # Publishers
        self.dsc_pub = Topic.dsc.createPublisher(self)
        self.detected_pub = Topic.detected.createPublisher(self)
        self.blue_area_pub = Topic.blue_area.createPublisher(self)
        self.camera_processed_pub = Topic.camera_processed.createPublisher(self)
        self.green_box_pub = Topic.green_box_encoded.createPublisher(self)
        self.blue_box_pub = Topic.blue_box_encoded.createPublisher(self)
        self.box_detected_pub = Topic.box_detected.createPublisher(self)
        # Subscribers (if needed)
        self.mission_type_sub = Topic.mission_type.createSubscriber(self, self.mission_callback)
        self.arena_sub = Topic.arena.createSubscriber(self, self._arena_cb)

    def _arena_cb(self, msg: String):
        self.arena = str(msg.data)
        self.get_logger().info(f"Received arena: {self.arena}")

    def mission_callback(self, msg):
        """Update current mission"""
        self.mission_type = str(msg.data)
        self.get_logger().info(f"Mission changed to: {self.mission_type}", throttle_duration_sec=3.0)

    def get_data(self):
        return self.result, self.dsc, self.state

    def visualize(self, scale=0.6):
        """Display annotated frame"""
        if self.img is None:
            return False

        display_img = cv2.resize(
            self.img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
        )
        cv2.imshow("Annotated Output", display_img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Exit requested by user (q pressed).")
            return True
        return False

    def encode_base64(self, img):
        result, encoded_image = cv2.imencode(
            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 20]
        )
        if result:
            base64_image = base64.b64encode(encoded_image).decode("utf-8")
            img_msg = String()
            img_msg.data = base64_image
            return img_msg
        else:
            self.get_logger().error("Failed to encode frame to JPG")
            return ""
 

    def process_frame(self):
        """Process a single frame - called by timer"""
        try:
            success, img = self.up_cap.read()

            if self.REC_UP_CAMERA:
                self.up_vid_writer.write(img)
            
            # Komen perekaman down camera karena sedang tidak dipakai
            # if self.REC_DOWN_CAMERA:
            #     self.down_vid_writer.write(self.down_cap.read()[1])
            
            if not success:
                return

            box_detected_bool = False
            if self.mission_type == MissionStatus.BUOY or self.mission_type == MissionStatus.DOCKING or self.mission_type == MissionStatus.DOCKING_V2:
                # Hanya model buoy yang menyala
                self.img, self.dsc, self.detected = self.buoy_detector.process_frame(self.mission_type, self.arena, img, self.up_cap, self)
            elif self.mission_type == MissionStatus.TURN_NEXT_BUOY:
                # Model buoy dan box menyala dan digambar bersamaan di gambar yang sama
                self.img, self.dsc, self.detected = self.buoy_detector.process_frame(self.mission_type, self.arena, img, self.up_cap, self)
                self.img, _, box_detected_bool = self.box_detector.process_frame(MissionStatus.BOTH_BOXES, self.arena, self.img, self.up_cap, self)
            else:
                # Hanya model box yang menyala
                self.get_logger().info(f"Masuk sini", throttle_duration_sec=1.0)
                self.img, self.dsc, self.detected = self.box_detector.process_frame(self.mission_type, self.arena, img, self.up_cap, self)
            
            self.box_detected_pub.publish(Bool(data=box_detected_bool))

            self.get_logger().info(f"Test: {self.mission_type}", throttle_duration_sec=1.0)

            if self.img is None:
                self.get_logger().warn("Failed to get frame", throttle_duration_sec=5.0)
                return

            # Visualize if enabled
            if self.show_result:
                exit_status = self.visualize()
                if exit_status:
                    rclpy.shutdown()
                    return

            dsc_msg = Float64()
            dsc_msg.data = float(self.dsc)
            self.dsc_pub.publish(dsc_msg)
            
            detected_msg = Bool()
            detected_msg.data = self.detected
            self.detected_pub.publish(detected_msg)

            if hasattr(self.buoy_detector, 'blue_area'):
                blue_area_msg = Float64()
                blue_area_msg.data = float(self.buoy_detector.blue_area)
                self.blue_area_pub.publish(blue_area_msg)

            self.get_logger().info(
                f"DSC: {self.dsc:.2f}, Detected: {self.detected}",
                throttle_duration_sec=2.0
            )

            # Passing image data
            top_camera = self.encode_base64(self.img)
            
            # Read down camera safely to avoid crash if unplugged
            # Kalau down camera tidak ada, gunakan top camera sebagai fallback
            bot_camera = None
            if self.down_cap is not None and self.down_cap.isOpened():
                ret, bot_frame = self.down_cap.read()
                if ret and bot_frame is not None:
                    bot_camera = self.encode_base64(bot_frame)
            
            if bot_camera is None:
                bot_camera = top_camera  # Fallback
            
            self.camera_processed_pub.publish(top_camera)
            
            # Combine both camera images to send in one String message
            combined_msg = String()
            combined_msg.data = top_camera.data + "|||" + bot_camera.data

            if(self.mission_type == MissionStatus.GREEN_BOX and self.detected):
                self.green_box_pub.publish(combined_msg)

            # Publish both camera images when Blue Box is detected
            if(self.mission_type == MissionStatus.BLUE_BOX and self.detected):
                self.blue_box_pub.publish(combined_msg)

        except Exception as e:
            self.get_logger().error(f"Error in process_frame: {traceback.format_exc()}")


    def run(self):
        """Start the main execution loop"""
        # timer for frame processing (30 FPS = 0.033s)
        self.timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(1/self.fps, self.process_frame, callback_group=self.timer_cb_group)
        self.get_logger().info("Front camera processing started at 30 FPS")


def main():
    rclpy.init()
    front_cam = CameraController()

    try:
        front_cam.run()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(front_cam)
        executor.spin()
        
    except KeyboardInterrupt:
        front_cam.get_logger().info("Shutting down front camera node...")
    except Exception as e:
        front_cam.get_logger().error(f"Error: {traceback.format_exc()}")
    finally:
        # Cleanup
        front_cam.up_cap.release()
        front_cam.down_cap.release()
        if front_cam.REC_UP_CAMERA:
            front_cam.up_vid_writer.release()
        if front_cam.REC_DOWN_CAMERA:
            front_cam.down_vid_writer.release()
        cv2.destroyAllWindows()
        front_cam.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
