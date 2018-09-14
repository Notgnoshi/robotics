#!/usr/bin/env python3
import rclpy as ros
from rclpy.executors import SingleThreadedExecutor
from triangle import Triangle


def main():
    """A single main entry point for a collection of ROS nodes."""
    ros.init(args=None)

    triangle = Triangle()

    executor = SingleThreadedExecutor()

    executor.add_node(triangle.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    triangle.node.destroy_node()
    triangle._simulation_timer.destroy_timer(triangle.timer)
    ros.shutdown()


if __name__ == '__main__':
    main()
