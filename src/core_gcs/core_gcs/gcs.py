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

                if pix_msg is None or mode_msg is None or qr_side_msg is None:
                    continue

                yield telemetryResponse(
                    mode=mode_msg.data,
                    battery=0,
                    latitude=pix_msg.lat,
                    longitude=pix_msg.lon,
                    timestamp=datetime.fromtimestamp(pix_msg.sys_time, tz=timezone.utc),
                    qr_side=qr_side_msg.data,
                    depth=0,
                    fc_status=True,
                    sensor_status=True,
                )

        finally:
            logging.info("Client disconnected, destroying subscriptions.")
            for sub in active_subs:
                self.node.destroy_subscription(sub)


async def serve(ros_node: Node) -> None:
    server = grpc.aio.server()
    add_ServerServicer_to_server(RosGrpcServicer(ros_node), server)

    listen_addr = "[::]:50051"
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
