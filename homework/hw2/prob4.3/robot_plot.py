import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import rclpy as ros
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


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
