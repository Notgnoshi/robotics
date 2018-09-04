#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

rclpy.init(args=None)

node = rclpy.create_node('publisher')
publisher = node.create_publisher(String, 'chat')
msg = String()


while True:
    msg.data = input("> ")
    if msg.data.lower() == 'exit' or msg.data.lower() == 'quit':
        break

    publisher.publish(msg)


node.destroy_node()
rclpy.shutdown()
