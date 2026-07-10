import asyncio
import logging
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import grpc
from server_pb2 import telemetryRequest, telemetryResponse
from server_pb2_grpc import ServerServicer, add_ServerServicer_to_server


class RosGrpcServicer(ServerServicer):
    def __init__(self, ros_node: Node):
        self.node = ros_node

    async def getTelemetry(
        self, request: telemetryRequest, context: grpc.aio.ServicerContext
    ) -> telemetryResponse:
        logging.info("Client connected to stream.")

        queue = asyncio.Queue()
        loop = asyncio.get_running_loop()

        # NOTE add required topics / subscribers
        def ros_callback(msg):
            loop.call_soon_threadsafe(queue.put_nowait, msg.data)

        sub = self.node.create_subscription(
            String, "my_telemetry_topic", ros_callback, 10
        )

        try:
            while context.is_active():
                ros_data = await queue.get()
                # NOTE SET DATA TO CORRECT FIELDS
                yield telemetryResponse(message=f"Live ROS Data: {ros_data}")

        finally:
            logging.info("Client disconnected, destroying subscription.")
            self.node.destroy_subscription(sub)


async def serve(ros_node: Node) -> None:
    server = grpc.aio.server()
    add_ServerServicer_to_server(RosGrpcServicer(ros_node), server)

    listen_addr = "[::]:50051"
    server.add_insecure_port(listen_addr)
    logging.info(f"Starting async gRPC server on {listen_addr}")

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
