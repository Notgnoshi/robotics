from collections import deque
import numpy as np
import rclpy as ros
from std_msgs.msg import Float64MultiArray, Bool


class Control:
    """A ROS2 computation node that publishes wheel velocities for a differential drive robot.

    The node publishes the velocities to `/WheelVel` at 10Hz, and publishes a 0 to `Active` to
    indicate the node is done publishing to `/WheelVel`.
    """

    def __init__(self, name, step):
        """Creates the ROS2 computation node to publish wheel velocities.

        :param name: The name of the node.
        :param step: The problem timestep.
        """
        self.node = ros.create_node(name)
        self.publisher = self.node.create_publisher(Float64MultiArray, 'WheelVel')
        self.active_pub = self.node.create_publisher(Bool, 'Active')
        # Running every 0.1 seconds is 10Hz.
        self.timer = self.node.create_timer(0.1, self.velocity_cb)
        self.active_timer = self.node.create_timer(1, self.active_cb)
        self.wheel_velocities = self.velocities(step)

    @staticmethod
    def velocities(step):
        """Returns the wheel velocities as parameterized in the problem."""
        t = np.arange(0, 10+step, step)
        phi1 = 2 + 2 * np.exp(-t)
        phi2 = 2 + np.exp(-2 * t)

        return deque(zip(phi1, phi2))

    def velocity_cb(self):
        """Publishes wheel velocities at 10Hz"""
        try:
            msg = Float64MultiArray()
            msg.data = self.wheel_velocities.popleft()
            self.publisher.publish(msg)
        except IndexError:
            pass

    def active_cb(self):
        """Indicates whether the Control node is still active"""
        msg = Bool()
        msg.data = bool(self.wheel_velocities)
        self.active_pub.publish(msg)
