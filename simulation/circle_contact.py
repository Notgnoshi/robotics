#!/usr/bin/env python3
import rclpy as ros
from rclpy.node import Node
from std_msgs.msg import Float32, ByteMultiArray
import struct


def get_hit(msg):
    hits = msg.data
    for i, hit in enumerate(hits):
        hit = struct.unpack('b', hit)[0]
        if hit:
            print(f'Contact on sensor: {i}')

ros.init()
node = Node('Circle')

left_pub = node.create_publisher(Float32, 'robot0/left_wheel')
right_pub = node.create_publisher(Float32, 'robot0/right_wheel')

contact_sub = node.create_subscription(ByteMultiArray, 'robot0/touches', get_hit)

msg = Float32()

msg.data = 5.0
left_pub.publish(msg)

msg.data = 10.0
right_pub.publish(msg)

try:
    ros.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
ros.shutdown()
