import rclpy as ros
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from kinematics import theta_dot, x_dot, y_dot


class ForwardK:
    """A ROS2 computation node that runs forward kinematics on the wheel velocities of a
    differential drive robot. This node publishes the forward kinematics on the topic /RobotVel as
    it receives the wheel velocities.
    """

    def __init__(self, name, step):
        """Creates the ROS2 computation node to run the forward kinematics.

        :param name: The name of the node.
        :param step: The problem timestep.
        """
        self.node = ros.create_node(name)
        self.subscriber = self.node.create_subscription(Float64MultiArray, 'WheelVel', self.callback)
        self.publisher = self.node.create_publisher(Twist, 'RobotVel')
        self.step = step

    def callback(self, msg):
        """Receives wheel velocities and runs the forward kinematics."""
        phi1, phi2 = msg.data
        dtheta = theta_dot(r=5, L=20, w1=phi1, w2=phi2)
        theta = dtheta * self.step
        dx = x_dot(r=5, w1=phi1, w2=phi2, theta=theta)
        dy = y_dot(r=5, w1=phi1, w2=phi2, theta=theta)

        msg = Twist()
        msg.linear.x = dx
        msg.linear.y = dy
        msg.angular.x = dtheta

        self.publisher.publish(msg)
