import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
from std_msgs.msg import String, UInt8
from core.utils.config import Topic, PxMode, Param
from core_msgs.msg import Pixhawk



LAPTOP_URI = "ws://192.168.137.233:8000/ws"

class Gcs(Node):
    def __init__(self, loop, node = Node):
        super().__init__('Gcs')
        self.node = node
        self.loop = loop 
        self.ws_connection = None   
        self.loop.create_task(self.connect_to_server())

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

    async def connect_to_server(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Trying to Connect {LAPTOP_URI}...")
                
                # Tambahkan timeout agar tidak hang jika jaringan putus nyambung
                async with websockets.connect(LAPTOP_URI, ping_interval=None) as websocket:
                    self.ws_connection = websocket
                    self.get_logger().info("Connected to Laptop GCS via Tailscale!")
                    
                    # Block disini sampai koneksi putus
                    await websocket.wait_closed()
                    
                self.get_logger().warn("Connection closed by server.")
                
            except ConnectionRefusedError:
                self.get_logger().error(f"Connection Refused: Server at {LAPTOP_URI} is likely DOWN or BLOCKED.")
            except TimeoutError:
                self.get_logger().error(f"Connection Timed Out: Check Tailscale connection.")
            except Exception as e:
                self.get_logger().error(f"Failed to Connect: {e}")
            
            # Reset dan tunggu sebelum reconnect
            self.ws_connection = None
            self.get_logger().info("Reconnecting in 3 seconds...")
            await asyncio.sleep(3)

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
        if self.ws_connection:
            try:
                await self.ws_connection.send(json.dumps({"data": message}))
            except Exception as e:
                self.get_logger().warn(f"Failed sending data: {e}")

async def main_async():
    rclpy.init()
    loop = asyncio.get_running_loop() 
    node = rclpy.create_node('gcs_node')

    gcs = Gcs(loop, node)

    
    # Vibe coding research later
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(gcs)

    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            await asyncio.sleep(1)
    finally:
        gcs.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()
