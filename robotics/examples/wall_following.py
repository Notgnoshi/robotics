#!/usr/bin/env python3
import sys
sys.path.append('../..')

import rclpy

from robotics.nodes import GpsPlotter, NodeManager, DdController, WallFollowController


def main():
    """An example of wall following in Veranda."""
    rclpy.init()
    manager = NodeManager(policy='multi')

    controller = DdController(twist_topic='robot/control/twist',
                              left_topic='robot/left_wheel',
                              right_topic='robot/right_wheel',
                             )
    plotter = GpsPlotter('GpsPlotter', 'robot/gps', history=None)
    wall_follower = WallFollowController(lidar_topic='robot/lidar',
                                         control_topic='robot/control/twist',
                                         speed=5.0,
                                         target_distance=0.5,
                                         # TODO: tune these parameters for the higher speed
                                         Kp=1.2,
                                         Kd=1.1,
                                         Kth=1.1,
                                        )

    manager.add_node(controller)
    manager.add_node(wall_follower)
    manager.add_node(plotter)

    manager.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
