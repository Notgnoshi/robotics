#!/usr/bin/env python3
import sys
sys.path.append('../..')

import rclpy

from robotics.nodes import GpsPlotter, NodeManager, BugOneController, DdController


def main():
    """Bug 1 motion algorithm demo."""
    rclpy.init()
    manager = NodeManager(policy='multi')

    # The Bug 1 controller sends twist messages to this controller.
    controller = DdController(twist_topic='robot/control/twist',
                              left_topic='robot/left_wheel',
                              right_topic='robot/right_wheel',
                             )

    bug_controller = BugOneController(gps_topic='robot/gps',
                                      lidar_topic='robot/lidar',
                                      twist_topic='robot/control/twist',
                                      goal_topic='goal/gps',
                                      contact_topic='robot/contact',
                                      speed=3.3,
                                      target_distance=0.5,
                                      Kp=3.0,
                                      Ki=3.5,
                                      Kd=3.0,
                                      Kth=3.1,
                                     )

    plotter = GpsPlotter('GpsPlotter', 'robot/gps', history=None)

    manager.add_node(plotter)
    manager.add_node(controller)
    manager.add_node(bug_controller)

    manager.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
