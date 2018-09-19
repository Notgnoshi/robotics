#!/usr/bin/env python3
from collections import deque

import matplotlib.pyplot as plt
import rclpy as ros
from geometry_msgs.msg import Pose2D


class PathTracer:
    """Subscribes to Pose2D messages on /robot/gps and plots the simulated path
    traced by a robot in Veranda.
    """

    def __init__(self, max_history=1000):
        """Creates an instance of the PathTracer class.

        :param max_history: The maximum number of points to plot. The oldest
            points will be removed from the plot once the PathTracer accumulates
            this many points.
        """
        self.node = ros.create_node('PathTracer')
        self.subscriber = self.node.create_subscription(Pose2D, 'robot/gps', self.callback)
        # Create a matplotlib figure
        plt.figure()
        # Create an empty plot to continually modify. Note the comma!!
        self.plot, = plt.plot([])
        plt.title('Simulated Robot Path')
        # Turn on interactive plotting
        plt.ion()
        # Display the plot
        plt.show()

        # Memory isn't free, so save only the latest max_history points in a deque.
        # Use two deques, one for the x points, one for the y points to simplify
        # plotting.
        self.xs = deque([], maxlen=max_history)
        self.ys = deque([], maxlen=max_history)

    def callback(self, msg):
        """Receives Pose2D messages from the robot and plots them."""
        # Save the received point in a tuple, and enqueue it in self.points.
        self.xs.append(msg.x)
        self.ys.append(msg.y)

        # Now update the plot now that we have a new point.
        self.update_plot()

    def update_plot(self):
        """Updates the live plot with new points."""
        # Set the plot's data to the new x and y points.
        self.plot.set_data(self.xs, self.ys)

        # Now expand the plot's axis boundary as the path grows.
        self.plot.axes.set_xlim(min(self.xs) - 0.1, max(self.xs) + 0.1)
        self.plot.axes.set_ylim(min(self.ys) - 0.1, max(self.ys) + 0.1)

        # This is the magical bit that causes matplotlib to update the plot live.
        plt.pause(0.000001)


if __name__ == "__main__":
    ros.init()
    tracer = PathTracer()
    try:
        ros.spin(tracer.node)
    except KeyboardInterrupt:
        pass

    tracer.node.destroy_node()
    ros.shutdown()
