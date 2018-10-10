#!/usr/bin/env python3
"""
Description of bug:

Occasionally, when running a simulation the pieces of my robot become unstuck
and float around the Veranda workspace as if tethered by a rubber band. If left
to continue running, the oscillation gets worse and worse.

* Set up environment
    * Load turtlebot.json from this folder
    * Shift the robot to the side (seems to be worse when robot is also rotated)
    * quick save
* Steps to reproduce
    * Load workspace from a quick save
    * Run the simulation with this script
"""

import sys
import rclpy

sys.path.append('../..')
from robotics.nodes import NodeManager, Wiggler


def main():
    """An example of live plotting a robot path in Veranda"""
    rclpy.init()
    manager = NodeManager(policy='single')

    wiggler = Wiggler()

    manager.add_node(wiggler)

    manager.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
