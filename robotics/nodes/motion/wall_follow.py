import numpy as np
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class WallFollowController(Node):
    """A ROS node for implementing wall following using a simple PD controller. C.f.
    https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_WALLFOLLOWING
    """

    def __init__(self, lidar_topic, control_topic, speed=5.0, target_distance=5,
                 Kp=1.0, Kd=1.0, Kth=1.0, side='left'):
        """Creates a wall following node.

        :param lidar_topic: The topic publishing LIDAR messages.
        :param control_topic: The topic to publish twist messages to.
        :param Kp: The proportional gain.
        :param Kd: The derivative gain.
        :param Kth: The proportional gain for the angular velocity.
        :param side: The side of the robot to follow.
        """
        super().__init__('WallFollowController')

        self.Kp = Kp
        self.Kd = Kd
        self.Kth = Kth
        # TODO: Ki?

        self.target_distance = target_distance
        self.speed = speed

        if side == 'left':
            self.side = 1
        elif side == 'right':
            self.side = -1
        else:
            raise ValueError('Can\'t follow side other than left or right?!')

        self.__sub = self.create_subscription(LaserScan, lidar_topic, self.callback)
        self.pub = self.create_publisher(Twist, control_topic)

        self.error = 0

    def callback(self, msg):
        """Receives LIDAR data at some frequency."""
        # BUG: If the ranges on the side we care about are all infinite,
        # then Veranda crashes.
        l = len(msg.ranges)
        i_0 = l * (self.side + 1) // 4
        i_n = l * (self.side + 3) // 4

        min_distance = msg.ranges[i_0]
        min_i = i_0

        # Need both the min value and the index at which the min occurs.
        for i, d in enumerate(msg.ranges[i_0:i_n + 1]):
            if d < min_distance:
                min_distance = d
                min_i = i

        min_angle = (min_i - l // 2) * msg.angle_increment
        front_distance = msg.ranges[l // 2]

        diff_error = min_distance - self.target_distance - self.error
        self.error = min_distance - self.target_distance

        self.pd_controller(min_angle, front_distance, self.error, diff_error)

    def pd_controller(self, min_angle, front_distance, error, diff_error):
        """Implements a simple PD controller to keep the robot at a set distance
        and heading (tangent to obstacle)

        :param min_angle: The angle to the minimum distance.
        :param front_distance: The distance from the front scan.
        :param error: The reference error.
        :param diff_error: The time derivative of the reference error.
        """
        msg = Twist()
        # TODO: Try adding an integral term?
        # The angular velocity of the robot.
        msg.angular.z = self.side * (self.Kp * error + self.Kd * diff_error) + \
                        self.Kth * (min_angle - np.pi * self.side / 2)

        # Here, (x, y) is in the robot local coordinate system.
        # If we're about to run into something, stop.
        if front_distance < self.target_distance:
            # TODO: This is probably too extreme.
            msg.linear.x = 0.0
        # If we're less about to run into something slow down.
        elif front_distance < self.target_distance * 2:
            msg.linear.x = self.speed / 2
        # If the angle correction is too big, slow down.
        elif abs(min_angle) > 1.75:
            msg.linear.x = self.speed / 2
        else:
            msg.linear.x = self.speed

        self.pub.publish(msg)
