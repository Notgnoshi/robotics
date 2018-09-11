#!/usr/bin/env python3
import matplotlib.pyplot as plt
import rclpy as ros
from rclpy.executors import SingleThreadedExecutor

from nodes import ConfigGenerator, KinematicsVerifier, WorkspaceGenerator


def main():
    """A single main entry point for a collection of ROS nodes."""
    ros.init(args=None)

    kv = KinematicsVerifier(name='kinematics_verifier')
    cg = ConfigGenerator(name='config_generator')
    wg = WorkspaceGenerator(name='workspace_generator')

    # Runs all three nodes in a single thread so that there's a single entry point and I don't have
    # to topologically sort and run each python script individually.
    executor = SingleThreadedExecutor()

    executor.add_node(kv.node)
    executor.add_node(cg.node)
    executor.add_node(wg.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        # Did someone say "hacky-bandaide-bugfix"??
        print('Waiting for matplotlib figure to close...')
        plt.show(block=True)

    # TODO: There's probably some OOPy way to handle this automagically, but I'm too pissed off at
    #       matplotlib to care right now.
    kv.node.destroy_node()
    cg.node.destroy_node()
    wg.node.destroy_node()
    ros.shutdown()


if __name__ == '__main__':
    main()
