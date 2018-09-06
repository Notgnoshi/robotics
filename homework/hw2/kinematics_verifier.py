import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from std_msgs.msg import Float64MultiArray

from kinematics import forward
from ros_node import RosNode


class KinematicsVerifier(RosNode):
    """A ROS2 computation node to generate configuration space points and
    publish them to the `thetaData` topic as a `Float64MultiArray` message.

    This node subscribes to workspace points from the `physData` topic.
    """

    def __init__(self, name):
        """Create a ROS2 computation node to verify our inverse kinematics.

        :param name: The name of the computation node.
        """
        super().__init__(name)
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
        # TODO: Save plot when verification is finished.

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

    def shutdown(self):
        plt.savefig('prob2.svg')
        super().shutdown()
