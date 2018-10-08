#!/usr/bin/env python3
import sys
sys.path.append('../..')

import rclpy

from robotics.nodes import GpsPlotter, NodeManager, BasicMotionController


def main():
    """An example of live plotting a robot path in Veranda"""
    rclpy.init()
    manager = NodeManager(policy='multi')

    controller = BasicMotionController('robot/gps', 'goal/gps', 'robot/left_wheel', 'robot/right_wheel')
    plotter = GpsPlotter('GpsPlotter', 'robot/gps')

    manager.add_node(controller)
    # manager.add_node(plotter)

    manager.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
