#!/usr/bin/env python3
import sys
sys.path.append('../..')

import rclpy

from robotics.nodes import GpsPlotter, NodeManager, BasicMotionController


def main():
    """An example of live plotting a robot path in Veranda"""
    rclpy.init()
    manager = NodeManager(policy='multi')

    controller = BasicMotionController(position_topic='robot/gps',
                                       goal_topic='goal/gps',
                                       contact_topic='robot/contact',
                                       left_control_topic='robot/left_wheel',
                                       right_control_topic='robot/right_wheel')
    plotter = GpsPlotter('GpsPlotter', 'robot/gps')

    manager.add_node(controller)
    manager.add_node(plotter)

    manager.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
