from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy as ros
import seaborn as sns
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray

from kinematics import theta_dot, x_dot, y_dot


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


class ForwardK:
    """A ROS2 computation node that runs forward kinematics on the wheel velocities of a
    differential drive robot. This node publishes the forward kinematics on the topic /RobotVel as
    it receives the wheel velocities.
    """

    def __init__(self, name, step):
        """Creates the ROS2 computation node to run the forward kinematics.

        :param name: The name of the node.
        :param step: The problem timestep.
        """
        self.node = ros.create_node(name)
        self.subscriber = self.node.create_subscription(Float64MultiArray, 'WheelVel', self.callback)
        self.publisher = self.node.create_publisher(Twist, 'RobotVel')
        self.step = step

    def callback(self, msg):
        """Receives wheel velocities and runs the forward kinematics."""
        phi1, phi2 = msg.data
        dtheta = theta_dot(r=5, L=20, w1=phi1, w2=phi2)
        theta = dtheta * self.step
        dx = x_dot(r=5, w1=phi1, w2=phi2, theta=theta)
        dy = y_dot(r=5, w1=phi1, w2=phi2, theta=theta)

        msg = Twist()
        msg.linear.x = dx
        msg.linear.y = dy
        msg.angular.x = dtheta

        self.publisher.publish(msg)




class RobotPlot:
    """A ROS2 node to numerically integrate and plot the path of a differential drive robot."""

    def __init__(self, name, step):
        """Creates the ROS2 node.

        :param name: The name of the ROS2 node.
        :param step: The problem timestep.
        """
        self.node = ros.create_node(name)
        self.twist_sub = self.node.create_subscription(Twist, 'RobotVel', self.twist_cb)
        self.active_sub = self.node.create_subscription(Bool, 'Active', self.active_cb)
        # A flag to indicate whether we're finished or not.
        self.finished = False
        self.step = step
        # A list of (dx, dy, dtheta) velocities as np.arrays.
        self.velocities = []
        # A list of (x, y, theta) positions as np.arrays.
        self.positions = []

    def twist_cb(self, msg):
        """Receives the twist messages."""
        # Don't worry about spreading the numerical integration over multiple calls to the callback.
        self.velocities.append(np.array([msg.linear.x, msg.linear.y, msg.angular.x]))

    def integrate(self):
        """Performs simple numerical integration to convert a list of velocities, a known time step,
        and an initial position to a list of position points to plot.
        """
        # Start at (0, 0, 0)
        x0 = np.array([0, 0, 0])
        self.positions.append(x0)
        for v in self.velocities:
            # Update the position by multiplying the derivatives elementwise with the time step
            x0 = x0 + v * self.step
            self.positions.append(x0)

    def plot(self):
        """Plots the position points"""
        x = [p[0] for p in self.positions]
        y = [p[1] for p in self.positions]
        sns.set()
        plt.plot(x, y)
        plt.title('Differential Drive Computed Path')
        plt.xlabel('$x$')
        plt.ylabel('$y$')
        plt.show()

    def active_cb(self, msg):
        """Receives the active messages."""
        # If not active, then done generating points.
        if not msg.data and not self.finished:
            self.integrate()
            self.plot()

            # TODO: Find a way to cause the entire program (the executor) to terminate?
            # self.node.executor.stuff()

            self.finished = True
