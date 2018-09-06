import rclpy as ros
from std_msgs.msg import Float64MultiArray
from kinematics import inverse


class ConfigGenerator:
    """A ROS2 computation node to generate configuration space points and
    publish them to the `thetaData` topic as a `Float64MultiArray` message.

    This node subscribes to workspace points from the `physData` topic.
    """

    def __init__(self, name='config_generator'):
        """Create a ROS2 computation node to generator config space points.

            :param name: The name of the computation node.
        """
        ros.init(args=None)
        self.node = ros.create_node(name)
        self.publisher = self.node.create_publisher(Float64MultiArray,
                                                    'thetaData')
        self.subscriber = self.node.create_subscription(Float64MultiArray,
                                                        'physData',
                                                        self.callback)

    def callback(self, msg):
        """Publishes a new workspace point every time this function is
        called.
        """
        a1, a2 = 10, 10
        x, y = msg.data
        t1, t2 = inverse(a1, a2, x, y)
        msg.data = [t1, t2]
        self.publisher.publish(msg)


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
    def create_and_run(cls, name='config_generator'):
        """Create and immediately run this computation node. Necessary as
        a single entry point in a new process.
        """
        instance = cls(name)
        instance.run()
