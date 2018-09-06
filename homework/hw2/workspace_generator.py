import itertools
import numpy as np
import rclpy as ros
from std_msgs.msg import Float64MultiArray


class WorkspaceGenerator:
    """A ROS2 computation node to generate workspace points and publish them
    to the `physData` topic as a `Float64MultiArray` message.

    The messages will be published at 5Hz.
    """

    def __init__(self, name='workspace_generator', loop=False):
        """Create a ROS2 computation node to generator workspace points.

            :param name: The name of the computation node.
            :param loop: Whether the generator should loop to the beginning
                         if/when it finishes generating points.
        """
        # Ensure that the ROS2 master process is initialized and running.
        ros.init(args=None)
        # Create this computation node.
        self.node = ros.create_node(name)
        # This node publishes Float64MultiArray messages to the `/physData` topic.
        self.publisher = self.node.create_publisher(Float64MultiArray, 'physData')
        # Reuse the same message object for every message this instance publishes
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
        """Publishes a new workspace point every time this function is
        called.
        """
        try:
            self.msg.data = next(self.workspace_points)
            self.publisher.publish(self.msg)
        except StopIteration:
            # TODO: Consider destroying the node at this time?
            pass

    def run(self):
        """Begin execution for this computation node.

        NOTE: If this function is ran in a new process, the ROS2 environment
              needs to be set up. The solution is to create this instance in
              the new process before calling this function.
        """
        # Running every 0.2 seconds is 5Hz.
        timer = self.node.create_timer(0.2, self.callback)

        try:
            while ros.ok():
                ros.spin_once(self.node)
        except KeyboardInterrupt:
            self.node.destroy_timer(timer)
            self.node.destroy_node()
            ros.shutdown()

    @classmethod
    def create_and_run(cls, name='workspace_generator', loop=False):
        """Create and immediately run this computation node. Necessary as
        a single entry point in a new process.
        """
        instance = cls(name, loop)
        instance.run()
