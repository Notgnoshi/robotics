#!/usr/bin/env python3
import rclpy as ros
from rclpy.node import Node
from std_msgs.msg import Float32

ros.init()
node = Node('Circle')

left_pub = node.create_publisher(Float32, 'robot0/left_wheel')
right_pub = node.create_publisher(Float32, 'robot0/right_wheel')

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
