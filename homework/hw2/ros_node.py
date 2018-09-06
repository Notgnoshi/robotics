import rclpy as ros


class RosNode:
    """A Base class for a ROS2 computation node."""

    def __init__(self, name):
        """Create a ROS2 computation node.

        :param name: The name of the computation node.
        """
        ros.init(args=None)
        self.node = ros.create_node(name)

    def shutdown(self):
        """Clean up after node execution is finished."""
        self.node.destroy_node()
        ros.shutdown()

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
            self.shutdown()

    @classmethod
    def create_and_run(cls, name):
        """Create and immediately run this computation node. Necessary as
        a single entry point in a new process.

        :param name: The name of the computation node.
        """
        instance = cls(name)
        instance.run()
