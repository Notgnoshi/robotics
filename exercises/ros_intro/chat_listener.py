#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

def chatter_callback(msg):
    global node
    node.get_logger().info(f'Chat: {msg.data}')


rclpy.init(args=None)
node = rclpy.create_node('chat_listener')
subscription = node.create_subscription(String, 'chat', chatter_callback)

while rclpy.ok():
    rclpy.spin_once(node)
