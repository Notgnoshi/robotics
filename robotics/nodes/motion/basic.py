import math
import threading as th
import time

import numpy as np
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, Float32


class BasicMotionController(Node):
    """A ROS node implementing the Basic Motion Algorithm.

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
        """Compare two Pose2D poses for equality.

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

        distance = np.sqrt(np.sum((p2 - p1)**2))

        return distance <= tol

    @staticmethod
    def remap_angle(angle):
        """Remaps the given angle to the range [0, 2pi]"""
        return np.mod(angle, 2*np.pi)

    def __init__(self, position_topic, goal_topic, contact_topic, left_control_topic, right_control_topic):
        """Create a basic motion planner for controlling motion to goal.

        :param position_topic: The topic for the current position.
        :param goal_topic: The topic for the goal position.
        :param contact_topic: The topic for the contact sensor.
        :param left_control_topic: The topic for controlling the left wheel speed.
        :param right_control_topic: The topic for controlling the right wheel speed.
        """
        super().__init__('BasicMotionPlanner')

        self.base_speed = 1.0

        self.position = Pose2D()
        self.goal = Pose2D()
        # A ten sensor contact ring.
        self.contacts = [False] * 10
        # The maximum number of turns before attempting motion to goal again.
        self.max_iters = 5

        self.left = self.create_publisher(Float32, left_control_topic)
        self.right = self.create_publisher(Float32, right_control_topic)

        self.__pos_sub = self.create_subscription(Pose2D, position_topic, self.update_position)
        self.__goal_sub = self.create_subscription(Pose2D, goal_topic, self.update_goal)
        self.__contact_sub = self.create_subscription(ByteMultiArray, contact_topic, self.update_contacts)

        # BUG: This thread never dies...
        self.job = th.Thread(target=self.algorithm, daemon=True)
        self.job.start()

    def update_goal(self, pose):
        """Update the position of the goal."""
        self.goal = pose

    def update_position(self, pose):
        """Update the current robot position."""
        # Rotate so that 0rad is in pos x direction instead of pos y...
        # TODO: <rant></rant>
        pose.theta = self.remap_angle(pose.theta + np.pi/2)
        self.position = pose

    def update_contacts(self, msg):
        """Update the contact sensor array."""
        self.contacts = [bool(ord(b)) for b in msg.data]

    def algorithm(self):
        """Top level container for the Basic Motion Algorithm."""
        # TODO: Find a way to start() the thread once the node starts spinning.
        print('Waiting for node to spin.')
        while self.position == Pose2D() or self.goal == Pose2D():
            pass

        self.head_towards_goal()
        while not self.arrived():
            while not self.obstructed():
                self.move_forward()
            iters = 0
            while iters <= self.max_iters:
                while self.obstructed():
                    self.turn(self.remap_angle(self.position.theta - np.pi/4))
                self.move_forward()
                # Need to give it a chance to drive forward.
                time.sleep(0.1)
                iters += 1
            self.head_towards_goal()
        self.stop()

    def turn(self, heading):
        """Turn the robot in-place to the given heading.

        :param heading: The desired heading of the robot, in radians.

        Note that this function will block until the desired heading is achieved.
        """
        msg = Float32()
        self.stop()

        # BUG: There's a bug here, because the heading and pose orientation are
        # both remapped. But what happens when orientation is in Q1 and heading is in Q4?
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

        self.stop()

    def arrived(self):
        """Determines whether the robot has arrived at its goal"""
        # Do not require orientations to be equal, because I'm not *that* masochistic.
        return self._pose_equal(self.position, self.goal, tol=2.0, theta=False)

    def move_forward(self):
        """Drives the robot forward in a straight line until directed otherwise.

        Nonblocking.
        """
        msg = Float32()
        msg.data = self.base_speed
        self.left.publish(msg)
        self.right.publish(msg)

    def head_towards_goal(self):
        """Turn the robot to face the goal, and drives forward.

        Blocks until the robot's heading is set, then sets the wheel speeds.
        """
        print('Heading towards goal.')
        # Get the vector from the robot to the goal.
        robot = np.array([self.position.x, self.position.y])
        goal = np.array([self.goal.x, self.goal.y])
        v = goal - robot
        # Compute the direction from the robot position to the goal.
        heading = self.remap_angle(np.arctan2(v[1], v[0]))
        print('Vector:', v)
        print('Heading:', heading)
        # Turn to that heading.
        self.turn(heading)
        # Set the wheel speeds and exit.
        self.move_forward()

    def obstructed(self):
        """Determine whether the robot is obstructed by an obstacle."""
        # Sensors 1, 2, and 3 are front right, front, and front left respectively.
        return any(self.contacts[1:4])

    def stop(self):
        """Stop the robot's motion"""
        msg = Float32()
        msg.data = 0.0
        self.left.publish(msg)
        self.right.publish(msg)
