from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class DdController(Node):
    """A ROS node to translate Twist messages to differential drive robot
    wheel speeds.
    """

    def __init__(self, twist_topic, left_topic, right_topic, r=0.5, L=0.6):
        """Constructs a DdController node.

        :param twist_topic: The topic publishing the Twist messages.
        :param left_topic: The topic to publish the left wheel speeds to.
        :param right_topic: The topic to publish the right wheel speeds to.
        :param r: The robot wheel radius.
        :param L: The robot half-axle length.
        """
        super().__init__('DdController')

        self.r = r
        self.L = L

        self.create_subscription(Twist, twist_topic, self.callback)
        self.left = self.create_publisher(Float32, left_topic)
        self.right = self.create_publisher(Float32, right_topic)

    def callback(self, msg):
        """Converts Twist messages with x and theta velocities (in local coordinates)
        to (left, right) wheel speeds.
        """
        left, right = Float32(), Float32()
        difference = msg.angular.z * 2 * self.L / self.r

        left.data = msg.linear.x
        right.data = msg.linear.x + difference

        self.left.publish(left)
        self.right.publish(right)
