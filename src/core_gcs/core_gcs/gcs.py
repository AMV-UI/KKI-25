import asyncio
import logging
import threading
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node

from core.utils.config import Topic

import grpc
from .server_pb2 import telemetryRequest, telemetryResponse
from .server_pb2_grpc import ServerServicer, add_ServerServicer_to_server


class RosGrpcServicer(ServerServicer):
    def __init__(self, ros_node: Node):
        self.node = ros_node

    async def getTelemetry(
        self, request: telemetryRequest, context: grpc.aio.ServicerContext
    ) -> telemetryResponse:
        self.node.get_logger().info("Client connected to stream.")

        topic_map = {
            "pixhawk": Topic.pixhawk,
            "pxmode": Topic.pxmode,
            "qr_side": Topic.qr_side,
            "control": Topic.control_state,
        }

        state_cache = {name: None for name in topic_map.keys()}

        new_data_event = asyncio.Event()
        loop = asyncio.get_running_loop()

        def make_callback(cache_key):
            def callback(msg):
                state_cache[cache_key] = msg
                loop.call_soon_threadsafe(new_data_event.set)

            return callback

        active_subs = []
        for name, factory in topic_map.items():
            cb = make_callback(name)
            sub = factory.createSubscriber(self.node, cb)
            active_subs.append(sub)

        try:
            while True:
                await new_data_event.wait()
                new_data_event.clear()

                pix_msg = state_cache["pixhawk"]
                mode_msg = state_cache["pxmode"]
                qr_side_msg = state_cache["qr_side"]
                control_msg = state_cache["control"]

                if pix_msg is None:
                    depth = 0

                    roll = 0
                    pitch = 0
                    yaw = 0
                    rollspeed = 0
                    pitchspeed = 0
                    yawspeed = 0
                else:
                    depth = pix_msg.alt

                    roll = pix_msg.roll
                    pitch = pix_msg.pitch
                    yaw = pix_msg.yaw
                    rollspeed = pix_msg.rollspeed
                    pitchspeed = pix_msg.pitchspeed
                    yawspeed = pix_msg.yawspeed

                if qr_side_msg is None:
                    qr = "NOT_FOUND"
                    qr = "A"
                else:
                    qr = qr_side_msg.data

                if mode_msg is None:
                    mode = 19
                else:
                    mode = mode_msg.data

                if control_msg is not None:
                    forward_rc = control_msg.forward
                    lateral_rc = control_msg.lateral
                    vertical_rc = control_msg.vertical
                    yaw_rc = control_msg.yaw
                    mot1 = control_msg.mot1
                    mot2 = control_msg.mot2
                    mot3 = control_msg.mot3
                    mot4 = control_msg.mot4
                    mot5 = control_msg.mot5
                    mot6 = control_msg.mot6
                else:
                    forward_rc = 1500
                    lateral_rc = 1500
                    vertical_rc = 1500
                    yaw_rc = 1500
                    mot1 = 1500
                    mot2 = 1500
                    mot3 = 1500
                    mot4 = 1500
                    mot5 = 1500
                    mot6 = 1500

                yield telemetryResponse(
                    mode=mode,
                    battery=0,
                    timestamp=datetime.fromtimestamp(0, tz=timezone.utc),
                    qr_side=qr,
                    depth=depth,
                    fc_status=True,
                    sensor_status=True,
                    yaw=yaw,
                    roll=roll,
                    pitch=pitch,
                    yawspeed=yawspeed,
                    rollspeed=rollspeed,
                    pitchspeed=pitchspeed,
                    forward_rc=forward_rc,
                    lateral_rc=lateral_rc,
                    vertical_rc=vertical_rc,
                    yaw_rc=yaw_rc,
                    mot1_eff=mot1,
                    mot2_eff=mot2,
                    mot3_eff=mot3,
                    mot4_eff=mot4,
                    mot5_eff=mot5,
                    mot6_eff=mot6,
                )

        finally:
            logging.info("Client disconnected, destroying subscriptions.")
            for sub in active_subs:
                self.node.destroy_subscription(sub)


async def serve(ros_node: Node) -> None:
    server = grpc.aio.server()
    add_ServerServicer_to_server(RosGrpcServicer(ros_node), server)

    listen_addr = "0.0.0.0:50051"
    server.add_insecure_port(listen_addr)

    ros_node.get_logger().info(f"Starting async gRPC server on {listen_addr}")

    await server.start()
    await server.wait_for_termination()


def main():
    rclpy.init()
    ros_node = rclpy.create_node("GCS")

    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    try:
        asyncio.run(serve(ros_node))
    except KeyboardInterrupt:
        logging.info("Shutting down...")
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
