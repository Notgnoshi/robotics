#!/usr/bin/env python3
import rclpy as ros
from rclpy.executors import SingleThreadedExecutor

from diff_drive import DiffDrive


def main():
    """A single main entry point for a collection of ROS nodes."""
    ros.init()

    # triangle = DiffDrive('triangle', [(0, 0), (15, 0), (5, 20), (0, 0)], loop=True)
    triangle = DiffDrive('triangle', [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)], loop=True)

    executor = SingleThreadedExecutor()

    executor.add_node(triangle.node)

    try:
        print('Running nodes...')
        executor.spin()
    except KeyboardInterrupt:
        pass

    print('Cleaning up nodes...')
    triangle.node.destroy_node()
    ros.shutdown()


if __name__ == '__main__':
    main()
