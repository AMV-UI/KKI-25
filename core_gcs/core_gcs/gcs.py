import rclpy
from rclpy.node import Node
import uvicorn
from fastapi import FastAPI
import threading

app = FastAPI()

# Example REST endpoint
@app.get("/status")
def read_status():
    return {"status": "Node is running", "data": "Hello from ROS 2"}

class Gcs(Node):
    def __init__(self):
        super().__init__('rest_api_node')
        self.get_logger().info("REST API Node started")

def start_api():
    # Run FastAPI server on 0.0.0.0 so it's accessible from other services
    uvicorn.run(app, host="0.0.0.0", port=8000)

def main(args=None):
    rclpy.init(args=args)
    node = Gcs()

    # Start REST API server in separate thread so it doesn't block ROS
    api_thread = threading.Thread(target=start_api, daemon=True)
    api_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

