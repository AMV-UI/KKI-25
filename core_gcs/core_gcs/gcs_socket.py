import rclpy
from rclpy.node import Node
import asyncio
import websockets
from core.utils.config import Topic


class GcsSocket(Node):
    def __init__(self):
        super().__init__('gcs_socket_node')
        self.get_logger().info("WebSocket Node started")

        self.data = ""           # last received ROS message
        self._clients = set()    # connected WebSocket clients

        # Start WebSocket server
        asyncio.ensure_future(self.start_ws_server())

        # Setup ROS subscription
        self._setup_communication()

    def _setup_communication(self):
        self.data_sub = Topic.camera_processed.createSubscriber(
            self, self.data_callback
        )
        self.get_logger().info("Subscribed to Topic.camera_processed")

    def data_callback(self, data):
        """Called whenever new ROS message arrives."""
        self.data = data.data  # extract string from std_msgs/String
        self.get_logger().info(f"Received ROS data: {self.data}")

        # Broadcast to all connected WebSocket clients
        asyncio.ensure_future(self.broadcast_latest_data())

    async def broadcast_latest_data(self):
        """Send latest ROS data to all active clients."""
        if not self._clients or not self.data:
            return

        disconnected = set()
        for client in self._clients:
            try:
                await client.send(self.data)
            except websockets.ConnectionClosed:
                disconnected.add(client)

        for client in disconnected:
            self._clients.remove(client)

    async def echo(self, websocket, path):
        """Handle a new WebSocket client connection."""
        self.get_logger().info(f"Client connected: {websocket.remote_address}")
        self._clients.add(websocket)

        try:
            # Immediately send the latest message
            if self.data:
                await websocket.send(self.data)
                self.get_logger().info(f"Sent latest data to {websocket.remote_address}")

            # Keep connection alive
            while True:
                await asyncio.sleep(0.5)

        except websockets.ConnectionClosed:
            self.get_logger().info(f"Client disconnected: {websocket.remote_address}")
        finally:
            self._clients.remove(websocket)

    async def start_ws_server(self):
        """Start the WebSocket server."""
        server = await websockets.serve(self.echo, "0.0.0.0", 8000)
        self.get_logger().info("WebSocket server running on ws://0.0.0.0:8000")
        await server.wait_closed()


async def ros_spin(node):
    """Run ROS executor inside asyncio loop."""
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        await asyncio.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = GcsSocket()

    loop = asyncio.get_event_loop()
    asyncio.ensure_future(ros_spin(node))

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
