import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
from std_msgs.msg import String, UInt8
from core.utils.config import Topic, PxMode, Param
from core_msgs.msg import Pixhawk

class Gcs(Node):
    def __init__(self, loop, node = Node):
        super().__init__('Gcs')
        self.node = node
        self.loop = loop 
        self.websocket_clients = set()

        self.lon_history = []
        self.lat_history = []
        self.pxmode = PxMode.HOLD

        self.speed = 0.0
        self.yaw = 0.0
        self.comm = 0.0
        self.mission = 0

        self.arena = "B"
        self.setup()

    def setup(self):
        self.pwm_subscriber = Topic.pwm.createSubscriber(
            self,
            self.pwm_callback
        )
        self.arena_subscriber = Topic.arena.createSubscriber(
            self,
            self.arena_callback
        )
        self.image_subscriber = Topic.camera_processed.createSubscriber(
            self,
            self.image_callback
        )
        self.blue_box_subscriber = Topic.image_blue_box.createSubscriber(
            self,
            self.blue_box_callback
        )
        self.green_box_subscriber = Topic.image_green_box.createSubscriber(
            self,
            self.green_box_callback
        )
        self.pixhawk_subscriber = Topic.pixhawk.createSubscriber(
            self,
            self.pixhawk_callback
        )
        self.mission_subscriber = Topic.mission.createSubscriber(
            self,
            self.mission_callback
        )
        self.pxmode_subscriber = Topic.pxmode.createSubscriber(
            self,
            self.pxmode_callback
        )

    def pwm_callback(self, msg: String):
        self.speed = msg.channels[2]
        self.yaw = msg.channels[0]
        self.comm = msg.channels[7]

    def arena_callback(self, msg: String):
        self.arena = msg.data

    def pxmode_callback(self, msg: String):
        self.pxmode = msg.data

    def image_callback(self, msg: String):
        self._handle_incoming_data("camera_processed", msg.data)

    def blue_box_callback(self, msg: String):
        self._handle_incoming_data("show_blue", msg.data)

    def green_box_callback(self, msg: String):
        self._handle_incoming_data("show_green", msg.data)

    def mission_callback(self, msg: UInt8):
        self.mission = msg.data
        self._handle_incoming_data("mission", self.mission)

    def pixhawk_callback(self, msg: Pixhawk):
        if(msg.lat > 1):
            return

        if self.pxmode == PxMode.HOLD:
            self.lon_history = [msg.lon]
            self.lat_history = [msg.lat]
        else:
            self.lon_history.append(msg.lon)
            self.lat_history.append(msg.lat)

        data = {
            "lon": self.lon_history,
            "lat": self.lat_history,
            "alt": msg.alt,
            "msg_spd": msg.msg_spd,
            "msg_heading": msg.msg_heading,
            "track": self.arena,
            "speed": int(self.speed),
            "yaw": int(self.yaw),
            "comm": int(self.comm)
        }
        self._handle_incoming_data("pixhawk", data)

    def _handle_incoming_data(self, topic_name, data):
        message = {"topic": topic_name, "data": data}
        # self.get_logger().info(f"[{topic_name}] Received: {data}")

        asyncio.run_coroutine_threadsafe(
            self.broadcast_message(message),
            self.loop
        )
    
    async def broadcast_message(self, message):
        if not self.websocket_clients:
            return
        dead_clients = set()
        for ws in self.websocket_clients:
            try:
                await ws.send(json.dumps({"data": message}))
            except Exception:
                dead_clients.add(ws)
        self.websocket_clients -= dead_clients

async def websocket_handler(websocket, path, node):
    node.websocket_clients.add(websocket)
    node.get_logger().info("WebSocket client connected")
    try:
        # Iterate all websocket connection and keep all client listening
        async for _ in websocket:
            pass
    except websockets.ConnectionClosed:
        pass
    finally:
        node.websocket_clients.remove(websocket)
        node.get_logger().info("WebSocket client disconnected")

async def main_async():
    rclpy.init()
    loop = asyncio.get_running_loop() 
    node = rclpy.create_node('gcs_node')

    gcs = Gcs(loop, node)

    ws_server = await websockets.serve(
        lambda ws, path: websocket_handler(ws, path, gcs),
        host='0.0.0.0',
        port=8000
    )
    gcs.get_logger().info("WebSocket server started at ws://0.0.0.0:8000")

    # Vibe coding research later
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(gcs)

    loop.run_in_executor(None, executor.spin)
    try:
        await asyncio.Future() 
    finally:
        gcs.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        ws_server.close()
        await ws_server.wait_closed()

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()