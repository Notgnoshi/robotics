#!/usr/bin/env python3
import time
from datetime import datetime
import rclpy
from std_msgs.msg import String

rclpy.init(args=None)

node = rclpy.create_node('publisher')
publisher = node.create_publisher(String, 'heartbeat')
msg = String()


while True:
    try:
        time.sleep(1)
        msg.data = str(datetime.now())
        print(f'publishing: {msg}')
        publisher.publish(msg)
    except KeyboardInterrupt:
        break


node.destroy_node()
rclpy.shutdown()
