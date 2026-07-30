import traceback
import cv2
import rclpy
from rclpy.executors import MultiThreadedExecutor
from core_perception.front_camera_controller import FrontCameraNode
from core_perception.bottom_camera_controller import BottomCameraNode


def main(args=None):
    rclpy.init(args=args)

    front_cam = FrontCameraNode()
    bottom_cam = BottomCameraNode()

    executor = MultiThreadedExecutor()
    executor.add_node(front_cam)
    executor.add_node(bottom_cam)

    try:
        executor.spin()
    except KeyboardInterrupt:
        front_cam.get_logger().info("Shutting down camera nodes...")
    except Exception:
        front_cam.get_logger().error(f"Error: {traceback.format_exc()}")
    finally:
        front_cam.cleanup()
        bottom_cam.cleanup()
        cv2.destroyAllWindows()
        front_cam.destroy_node()
        bottom_cam.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
