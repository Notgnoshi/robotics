#!/usr/bin/env python3
import sys
sys.path.append('../..')

import rclpy

from robotics.nodes import GpsPlotter, NodeManager, Wiggler


def main():
    """An example of live plotting a robot path in Veranda"""
    rclpy.init()
    manager = NodeManager(policy='single')

    wiggler = Wiggler()
    plotter = GpsPlotter('GpsPlotter', 'robot/gps')

    manager.add_node(wiggler)
    manager.add_node(plotter)

    manager.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
