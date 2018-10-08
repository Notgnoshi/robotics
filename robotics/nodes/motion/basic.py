import math
import threading as th
import time

import numpy as np
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Float32
from veranda.SimTimer import SimTimer


class BasicMotionController(Node):
    """A robot control node to implement the Basic Motion Algorithm for
    planning motion to a goal.

    Algorithm:
        set heading towards goal
        while not arrived:
            while not obstructed:
                move forward
            count = 0
            while count < N:
                while obstacle in front:
                    turn right (or left)
                move forward
                count += 1
            set heading towards goal
    """

    @staticmethod
    def _pose_equal(pose1, pose2, tol=2.0, theta=False):
        """Compare two Pose2D poses for equality

        :param pose1: The first Pose2D pose to compare.
        :param pose2: The second Pose2D pose to compare.
        :param tol: How close is close enough.
        :param theta: Whether or not to compare the pose headings.
        """
        p1 = [pose1.x, pose1.y]
        p2 = [pose2.x, pose2.y]
        if theta:
            p1.append(pose1.theta)
            p2.append(pose2.theta)

        p1 = np.array(p1)
        p2 = np.array(p2)

        distance = np.sqrt(np.sum(2 ** (p2 - p1)))

        return distance <= tol

    @staticmethod
    def remap_angle(angle):
        """Remaps the given angle to the range [0, 2pi]"""
        width = 2 * np.pi

        return angle - np.ceil(angle / width) * width

    def __init__(self, position_topic, goal_topic, left_control_topic, right_control_topic):
        """Creates a basic motion planner for controlling motion to goal.

        :param position_topic: The topic for the current position.
        :param goal_topic: The topic for the goal position.
        :param left_control_topic: The topic for controlling the left wheel speed.
        :param right_control_topic: The topic for controlling the right wheel speed.
        """
        super().__init__('BasicMotionPlanner')

        self.base_speed = 1.0

        self.position = Pose2D()
        self.goal = Pose2D()

        self.simulation = self.simulation = SimTimer(True, 'veranda/timestamp', self)

        self.left = self.create_publisher(Float32, left_control_topic)
        self.right = self.create_publisher(Float32, right_control_topic)

        self.__pos_sub = self.create_subscription(Pose2D, position_topic, self.update_position)
        self.__goal_sub = self.create_subscription(Pose2D, goal_topic, self.update_goal)

        # BUG: This thread never dies...
        self.job = th.Thread(target=self.algorithm, daemon=True)
        self.job.start()

    def update_goal(self, pose):
        """Updates the position of the goal."""
        self.goal = pose

    def update_position(self, pose):
        """Updates the current robot position."""
        # Remap the angle to [0, 2pi]
        pose.theta = self.remap_angle(pose.theta)
        self.position = pose

    def algorithm(self):
        """The top level container for the Basic Motion Algorithm."""
        while not self.arrived():
            # Didn't you know, this is the *actual* basic motion algorithm!
            self.turn(self.remap_angle(self.position.theta + np.pi/2))
            time.sleep(1)

    def turn(self, heading):
        """Turns the robot in-place to the given heading.

        :param heading: The desired heading of the robot, in radians.

        Note that this function will block until the desired heading is achieved.
        """
        print('Turning towards', heading)
        msg = Float32()
        # Stop the robot
        msg.data = 0.0
        self.left.publish(msg)
        self.right.publish(msg)

        # BUG: There's a bug here, because the heading and pose orientation are
        # both remapped. But what happens when theta is in Q1 and heading is in Q4?
        if heading - self.position.theta < 0:
            msg.data = self.base_speed
            self.left.publish(msg)
            msg.data = -self.base_speed
            self.right.publish(msg)
        else:
            msg.data = self.base_speed
            self.right.publish(msg)
            msg.data = -self.base_speed
            self.left.publish(msg)

        # Wait until the GPS heading roughly aligns with the desired heading
        while not math.isclose(self.position.theta, heading, abs_tol=0.2):
            pass

        # Stop the robot
        msg.data = 0.0
        self.left.publish(msg)
        self.right.publish(msg)

    def arrived(self):
        """Determines whether the robot has arrived at its goal"""
        # Do not require orientations to be equal, because I'm not *that* masochistic.
        return self._pose_equal(self.position, self.goal, tol=2.0, theta=False)
