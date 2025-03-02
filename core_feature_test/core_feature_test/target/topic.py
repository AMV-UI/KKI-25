#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class TopicFactory:
    def __init__(self, topic_name, msg_type, latch=False):
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.latch = latch
        self.qos_profile = 10

    def createPublisher(self, node):
        qos = rclpy.qos.QoSProfile(
            depth=self.qos_profile,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL if self.latch else rclpy.qos.DurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        return node.create_publisher(self.msg_type, self.topic_name, qos)

    def createSubscriber(self, node, callback):
        qos = rclpy.qos.QoSProfile(depth=self.qos_profile)
        return node.create_subscription(
            self.msg_type,
            self.topic_name,
            callback,
            qos
        )
    
    def waitMessage(self, node):
        return rclpy.wait_for_message(self.msg_type, self.topic_name, node)



class ServiceFactory:
    def __init__(self, name, srv_type):
        self.name = name
        self.srv_type = srv_type

    def createProxy(self, node):
        return node.create_client(self.srv_type, self.name)

    def handleService(self, node, handler):
        return node.create_service(self.srv_type, self.name, handler)

    def waitService(self, node, timeout_sec=1.0):
        return node.wait_for_service(timeout_sec=timeout_sec)
