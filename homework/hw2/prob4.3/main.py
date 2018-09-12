#!/usr/bin/env python3
import rclpy as ros
from rclpy.executors import SingleThreadedExecutor
from control import Control
from forward_k import ForwardK
from robot_plot import RobotPlot

STEP_SIZE = 0.1

def main():
    """A single main entry point for a collection of ROS nodes."""
    ros.init(args=None)

    control = Control(name='Control', step=STEP_SIZE)
    forward_k = ForwardK(name='ForwardK', step=STEP_SIZE)
    robot_plot = RobotPlot(name='RobotPlot', step=STEP_SIZE)

    # Runs all three nodes in a single thread so that there's a single entry point and I don't have
    # to topologically sort and run each python script individually.
    executor = SingleThreadedExecutor()

    executor.add_node(robot_plot.node)
    executor.add_node(forward_k.node)
    executor.add_node(control.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    control.node.destroy_node()
    forward_k.node.destroy_node()
    robot_plot.node.destroy_node()
    ros.shutdown()


if __name__ == '__main__':
    main()
