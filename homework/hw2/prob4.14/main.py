#!/usr/bin/env python3
import rclpy as ros
from rclpy.executors import SingleThreadedExecutor

from fig8 import Fig8


def main():
    """A single main entry point for a collection of ROS nodes."""
    ros.init()

    f = Fig8('robot')

    executor = SingleThreadedExecutor()

    executor.add_node(f.node)

    try:
        print('Running nodes...')
        executor.spin()
    except KeyboardInterrupt:
        pass

    print('Cleaning up nodes...')
    f.node.destroy_node()
    ros.shutdown()


if __name__ == '__main__':
    main()
