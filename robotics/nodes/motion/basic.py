import numpy as np
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Float32
from veranda.SimTimer import SimTimer


class BasicMotionPlanner(Node):
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

    # The maximum number of obstacle avoidance iterations.
    __MAX_ITERS = 20
    # Turn direction when obstacle is encountered.
    __TURN_DIR = 'right'
    #

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


    def __init__(self, position_topic, goal_topic, left_control_topic, right_control_topic):
        """Creates a basic motion planner for controlling motion to goal.

        :param position_topic: The topic for the current position.
        :param goal_topic: The topic for the goal position.
        :param left_control_topic: The topic for controlling the left wheel speed.
        :param right_control_topic: The topic for controlling the right wheel speed.
        """
        super().__init__('BasicMotionPlanner')

        self.position = Pose2D()
        self.goal = Pose2D()

        self.simulation = self.simulation = SimTimer(True, 'veranda/timestamp', self)
        self.timer = self.simulation.create_timer(0.1, self.controller)

        self.left = self.create_publisher(Float32, left_control_topic)
        self.right = self.create_publisher(Float32, right_control_topic)

        self.__pos_sub = self.create_subscription(Pose2D, position_topic, self.update_position)
        self.__goal_sub = self.create_subscription(Pose2D, goal_topic, self.update_goal)

    def update_goal(self, pose):
        """Updates the position of the goal."""
        self.goal = pose

    def update_position(self, pose):
        """Updates the current robot position."""
        self.position = pose

    def controller(self):
        """"""

    def turn(self):
        """"""
        self.simulation.destroy_timer(self.timer)
