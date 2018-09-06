import rclpy as ros
from std_msgs.msg import Float64MultiArray
from kinematics import forward
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns


class Verifier:
    """A ROS2 computation node to generate configuration space points and
    publish them to the `thetaData` topic as a `Float64MultiArray` message.

    This node subscribes to workspace points from the `physData` topic.
    """

    def __init__(self, name='verifier'):
        """Create a ROS2 computation node to verify our inverse kinematics.

            :param name: The name of the computation node.
        """
        # Set a nicer color scheme for matplotlib.
        sns.set()

        ros.init(args=None)
        self.node = ros.create_node(name)
        self.workspace_subscriber = self.node.create_subscription(Float64MultiArray,
                                                                  'physData',
                                                                  self.workspace_cb)
        self.config_subscriber = self.node.create_subscription(Float64MultiArray,
                                                               'thetaData',
                                                               self.config_cb)
        plt.figure()
        self.wplot, = plt.plot([], '.', color='green', label='workspace points')
        self.cplot, = plt.plot([], '.', color='blue', label='computed workspace points')
        plt.title('Verified workspace points')
        plt.legend()
        plt.ion()
        plt.show()

    def update_wplot(self, xp, yp):
        """Updates the physical workspace plot"""
        x, y = self.wplot.get_data()
        x = np.append(x, xp)
        y = np.append(y, yp)
        self.wplot.set_data(x, y)
        self.wplot.axes.set_xlim(np.amin(x)-0.1, np.amax(x)+0.1)
        self.wplot.axes.set_ylim(np.amin(y)-0.1, np.amax(y)+0.1)
        plt.pause(0.01)

    def update_cplot(self, xp, yp):
        """Updates the verified workspace plot"""
        x, y = self.cplot.get_data()
        x = np.append(x, xp)
        y = np.append(y, yp)
        self.cplot.set_data(x, y)
        self.cplot.axes.set_xlim(np.amin(x)-0.1, np.amax(x)+0.1)
        self.cplot.axes.set_ylim(np.amin(y)-0.1, np.amax(y)+0.1)
        plt.pause(0.01)

    def workspace_cb(self, msg):
        """Gets called for each workspace point."""
        x, y = msg.data
        self.update_wplot(x, y)

    def config_cb(self, msg):
        """Gets called for each configuration point"""
        a1, a2 = 10, 10
        t1, t2 = msg.data
        x, y = forward(a1, a2, t1, t2)
        self.update_cplot(x, y)

    def run(self):
        """Begin execution for this computation node.

        NOTE: If this function is ran in a new process, the ROS2 environment
              needs to be set up. The solution is to create this instance in
              the new process before calling this function.
        """
        try:
            while ros.ok():
                ros.spin_once(self.node)
        except KeyboardInterrupt:
            self.node.destroy_node()
            ros.shutdown()

    @classmethod
    def create_and_run(cls, name='verifier'):
        """Create and immediately run this computation node. Necessary as
        a single entry point in a new process.
        """
        instance = cls(name)
        instance.run()
