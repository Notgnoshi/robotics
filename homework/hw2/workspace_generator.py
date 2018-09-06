import itertools

import numpy as np
from std_msgs.msg import Float64MultiArray

from ros_node import RosNode


class WorkspaceGenerator(RosNode):
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
        super().__init__(name)
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
            # TODO: Consider destroying the node at this time?
            pass

    def shutdown(self):
        """Override the parent class shutdown to also destroy the timer."""
        self.node.destroy_timer(self.timer)
        super().shutdown()
