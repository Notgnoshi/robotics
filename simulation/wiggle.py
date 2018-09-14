#!/usr/bin/env python3
import rclpy as ros
from rclpy.node import Node
from std_msgs.msg import Float32
from veranda.SimTimer import SimTimer

ros.init()
node = Node('Wiggle')

sim_timer = SimTimer(True, 'veranda/timestamp', node)
publeft = node.create_publisher(Float32, 'robot0/left_wheel')
pubright = node.create_publisher(Float32, 'robot0/right_wheel')

def wiggle_left():
    global timer_handle
    sim_timer.destroy_timer(timer_handle)
    msg = Float32()

    msg.data = 5.0
    pubright.publish(msg)

    msg.data = 0.0
    publeft.publish(msg)

    timer_handle = sim_timer.create_timer(1, wiggle_right)

def wiggle_right():
    global timer_handle
    sim_timer.destroy_timer(timer_handle)
    msg = Float32()

    msg.data = 0.0
    pubright.publish(msg)

    msg.data = 5.0
    publeft.publish(msg)

    timer_handle = sim_timer.create_timer(1, wiggle_left)

timer_handle = sim_timer.create_timer(0.1, wiggle_left)

try:
    ros.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
ros.shutdown()
