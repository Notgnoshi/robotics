from std_msgs.msg import Float64MultiArray

from kinematics import inverse
from ros_node import RosNode


class ConfigGenerator(RosNode):
    """A ROS2 computation node to generate configuration space points and
    publish them to the `thetaData` topic as a `Float64MultiArray` message.

    This node subscribes to workspace points from the `physData` topic.
    """

    def __init__(self, name):
        """Create a ROS2 computation node to generator config space points.

        :param name: The name of the computation node.
        """
        super().__init__(name)
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
