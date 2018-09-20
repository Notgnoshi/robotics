from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node


class NodeManager:
    """Manages a collection of ROS2 nodes."""

    __execution_policies = [
        'single',
        'multi',
    ]

    def __init__(self, policy='single', num_threads=None):
        """Create a NodeManager to run a collection of ROS2 nodes with the
        given execution policy.

        :param policy: Specifies how to run the nodes. One of:
            'single' - Asynchronously runs the nodes in the same thread.
             'multi' - Runs each node in its own thread.
        :param num_threads: The number of threads to use if using multiple threads.
            Defaults to the number of CPUs on the machine.
        """

        if policy not in self.__execution_policies:
            raise ValueError('Invalid execution policy: {}'.format(policy))

        self.policy = policy
        self.num_threads = num_threads
        self.nodes = []

    def add_node(self, node):
        """Adds the given node to the NodeManager

        :param node: A Node node to add to the NodeManager
        """
        if not isinstance(node, Node):
            raise ValueError('Cannot add node of type: {}'.format(type(node)))

        self.nodes.append(node)

    def run(self):
        """A blocking call that runs each node managed by the NodeManager."""
        if self.policy == 'single':
            executor = SingleThreadedExecutor()
        elif self.policy == 'multi':
            executor = MultiThreadedExecutor(self.num_threads)

        for node in self.nodes:
            executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            executor.shutdown()

        for node in self.nodes:
            node.destroy_node()
