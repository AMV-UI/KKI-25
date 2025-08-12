import rclpy
from rclpy.node import Node
import asyncio
import websockets

class WebSocketNode(Node):
    def __init__(self):
        super().__init__('websocket_node')
        self.get_logger().info("WebSocket Node started")
        # Start the WebSocket server
        asyncio.ensure_future(self.start_ws_server())

    async def echo(self, websocket, path):
        self.get_logger().info(f"Client connected: {websocket.remote_address}")
        try:
            async for message in websocket:
                self.get_logger().info(f"Received: {message}")
                await websocket.send(f"Echo: {message}")
        except websockets.ConnectionClosed:
            self.get_logger().info("Client disconnected")

    async def start_ws_server(self):
        # Bind to all interfaces so LAN can reach it
        server = await websockets.serve(self.echo, "0.0.0.0", 8000)
        self.get_logger().info("WebSocket server running on ws://0.0.0.0:8000")
        await server.wait_closed()

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketNode()
    try:
        asyncio.get_event_loop().run_until_complete(asyncio.sleep(0))  # start loop
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
