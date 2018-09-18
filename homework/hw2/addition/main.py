#!/usr/bin/env python3
import rclpy as ros
from rclpy.executors import SingleThreadedExecutor

from tracer import PathTracer
from wiggle import Wiggle


def main():
    """A single main entry point for our ROS nodes."""
    # Initialize the ROS client.
    ros.init()

    # Create instances of our Wiggle and PathTracer classes.
    wiggle = Wiggle()
    tracer = PathTracer()

    # A SingleThreadedExecutor executes both nodes in the same thread. There
    # is a MultiThreadedExecutor, but it doesn't play nicely with matplotlib.
    executor = SingleThreadedExecutor()

    # Add the ROS nodes to the executor.
    executor.add_node(wiggle.node)
    executor.add_node(tracer.node)

    try:
        # Run the executor, which in turn runs the nodes.
        executor.spin()
    except KeyboardInterrupt:
        # Destroy the nodes and shutdown the ROS client when the user hits <ctrl-c>
        tracer.node.destroy_node()
        wiggle.node.destroy_node()
        ros.shutdown()


if __name__ == '__main__':
    main()
