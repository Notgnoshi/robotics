import itertools

import matplotlib.pyplot as plt
import numpy as np
import rclpy as ros
import seaborn as sns
from std_msgs.msg import Float64MultiArray

from kinematics import forward, inverse


class WorkspaceGenerator:
    """A ROS2 computation node to generate workspace points and publish them
    to the `physData` topic as a `Float64MultiArray` message.

    The messages will be published at 5Hz.
    """

    def __init__(self, name, loop=False):
        """Create a ROS2 computation node to generator workspace points.

        :param name: The name of the computation node.
        :param loop: Whether the generator should loop to the beginning
                     if/when it finishes generating points.
        """
        self.node = ros.create_node(name)
        self.publisher = self.node.create_publisher(Float64MultiArray, 'physData')
        # Running every 0.2 seconds is 5Hz.
        self.timer = self.node.create_timer(0.2, self.callback)
        self.msg = Float64MultiArray()
        self.loop = loop
        # A generator of points to publish
        self.workspace_points = self.points()

    def points(self):
        """Returns a generator of workspace points."""
        x = np.linspace(0, 10, 100)
        y = 15 - x

        workspace_points = ((xi, yi) for xi, yi in zip(x, y))

        if self.loop:
            return itertools.cycle(workspace_points)

        return workspace_points

    def callback(self):
        """Publishes a new workspace point every time this function is called."""
        try:
            self.msg.data = next(self.workspace_points)
            self.publisher.publish(self.msg)
        except StopIteration:
            pass


class ConfigGenerator:
    """A ROS2 computation node to generate configuration space points and
    publish them to the `thetaData` topic as a `Float64MultiArray` message.

    This node subscribes to workspace points from the `physData` topic.
    """

    def __init__(self, name):
        """Create a ROS2 computation node to generator config space points.

        :param name: The name of the computation node.
        """
        self.node = ros.create_node(name)
        self.publisher = self.node.create_publisher(Float64MultiArray,
                                                    'thetaData')
        self.subscriber = self.node.create_subscription(Float64MultiArray,
                                                        'physData',
                                                        self.callback)

    def callback(self, msg):
        """Publishes a new workspace point every time this function is
        called.

        :param msg: The message this node should handle.
        """
        a1, a2 = 10, 10
        x, y = msg.data
        t1, t2 = inverse(a1, a2, x, y)
        msg.data = [t1, t2]
        self.publisher.publish(msg)


class KinematicsVerifier:
    """A ROS2 computation node to generate configuration space points and
    publish them to the `thetaData` topic as a `Float64MultiArray` message.

    This node subscribes to workspace points from the `physData` topic.
    """

    def __init__(self, name):
        """Create a ROS2 computation node to verify our inverse kinematics.

        :param name: The name of the computation node.
        """
        self.node = ros.create_node(name)
        self.workspace_subscriber = self.node.create_subscription(Float64MultiArray,
                                                                  'physData',
                                                                  self.workspace_cb)
        self.config_subscriber = self.node.create_subscription(Float64MultiArray,
                                                               'thetaData',
                                                               self.config_cb)
        # Set a nicer color scheme for matplotlib.
        sns.set()
        plt.figure()
        self.wplot, = plt.plot([], '.', color='green', label='workspace points')
        self.cplot, = plt.plot([], '.', color='blue', label='computed workspace points')
        plt.title('Verified workspace points')
        plt.legend()
        plt.ion()
        plt.show()
        # BUG: Matplotlib window becomes unresponsive (so I can't click "save") when points are
        #      finished plotting. This is because the event loop is not "spun" for lack of a better
        #      word.
        # FIX: When script is done executing, call plt.show(block=True) to continue running the MPL
        #      event loop so that the "save" and "close" buttons, etc., work.

    def update_wplot(self, xp, yp):
        """Updates the physical workspace plot.

        :param xp: A single x point to add to the plot.
        :param yp: A single y point to add to the plot.
        """
        x, y = self.wplot.get_data()
        x = np.append(x, xp)
        y = np.append(y, yp)
        self.wplot.set_data(x, y)
        self.wplot.axes.set_xlim(np.amin(x)-0.1, np.amax(x)+0.1)
        self.wplot.axes.set_ylim(np.amin(y)-0.1, np.amax(y)+0.1)
        plt.pause(0.001)

    def update_cplot(self, xp, yp):
        """Updates the verified workspace plot

        :param xp: A single x point to add to the plot.
        :param yp: A single y point to add to the plot.
        """
        x, y = self.cplot.get_data()
        x = np.append(x, xp)
        y = np.append(y, yp)
        self.cplot.set_data(x, y)
        self.cplot.axes.set_xlim(np.amin(x)-0.1, np.amax(x)+0.1)
        self.cplot.axes.set_ylim(np.amin(y)-0.1, np.amax(y)+0.1)
        plt.pause(0.001)

    def workspace_cb(self, msg):
        """Gets called for each workspace point.

        :param msg: The msg containing a workspace point.
        """
        x, y = msg.data
        self.update_wplot(x, y)

    def config_cb(self, msg):
        """Gets called for each configuration point

        :param msg: The msg containing a configuration point.
        """
        a1, a2 = 10, 10
        t1, t2 = msg.data
        x, y = forward(a1, a2, t1, t2)
        self.update_cplot(x, y)
