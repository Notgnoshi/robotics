import numpy as np
import rclpy as ros
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class RobotPlot:
    """A ROS2 node to numerically integrate and plot the path of a differential drive robot."""

    def __init__(self, name):
        """Creates the ROS2 node.

        :param string name: The name of the ROS2 node.
        """
        self.node = ros.create_node(name)
        self.twist_sub = self.node.create_subscription(Twist, 'RobotVel', self.twist_cb)
        self.active_sub = self.node.create_subscription(Bool, 'Active', self.active_cb)

    def twist_cb(self, msg):
        """Receives the twist messages."""
        # TODO: Numerically integrate and stuff.
        pass

    def active_cb(self, msg):
        """Receives the active messages."""
        # If not active, then done generating points.
        if not msg.data:
            # TODO: Plot points when finished.
            # TODO: Find a way to cause the entire program (the executor) to terminate?
            # self.node.executor.stuff()
            pass
