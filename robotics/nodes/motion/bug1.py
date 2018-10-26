import math
import threading
import time
from collections import deque

import numpy as np
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ByteMultiArray


class BugOneController(Node):
    """A ROS node to Implement the Bug 1 motion algorithm.

    Algorithm:

    while True:
        repeat:
            from q_l,i-1 move to goal
        until goal is reached or obstacle encountered at q_h,i
        if goal is reached:
            exit
        repeat:
            follow boundary, recording path.
        until goal is reached or q_h,i is reencountered
        determine point q_l,i on path that is closest to goal
        go to q_l,i
        if robot cannot move towards goal:
            exit
    """

    @classmethod
    def _pose_equal(cls, pose1, pose2, tol=2.0, theta=False):
        """Compare two Pose2D poses for equality.

        :param pose1: The first Pose2D pose to compare.
        :param pose2: The second Pose2D pose to compare.
        :param tol: How close is close enough.
        :param theta: Whether or not to compare the pose headings.
        """
        return cls._pose_distance(pose1, pose2, theta) <= tol

    @staticmethod
    def _pose_distance(pose1, pose2, theta=False):
        """Find the Euclidean distance between two poses.

        :param pose1: The first pose.
        :type pose1: geometry_msgs.msg.Pose2D
        :param pose2: The second pose.
        :type pose2: geometry_msgs.msg.Pose2D
        :param theta: Whether to include the pose orientations, defaults to False.
        :type theta: bool, optional
        """
        p1 = [pose1.x, pose1.y]
        p2 = [pose2.x, pose2.y]
        if theta:
            p1.append(pose1.theta)
            p2.append(pose2.theta)

        p1 = np.array(p1)
        p2 = np.array(p2)

        return np.sqrt(np.sum((p2 - p1)**2))

    @staticmethod
    def remap_angle(angle):
        """Remap the given angle to the range [0, 2pi]."""
        return np.mod(angle, 2*np.pi)

    def __init__(self, gps_topic, lidar_topic, twist_topic, goal_topic, contact_topic, speed=2.0, target_distance=0.5, Kp=1.2, Ki=1.0, Kd=1.1, Kth=1.1):
        """Create a bug controller.

        :param gps_topic: The robot GPS sensor topic.
        :type gps_topic: str
        :param lidar_topic: The robot LIDAR sensor topic.
        :type lidar_topic: str
        :param twist_topic: The robot control topic.
        :type twist_topic: str
        :param goal_topic: The robot goal topic.
        :type goal_topic: str
        :param contact_topic: The robot touch sensor topic.
        :type contact_topic: str
        :param speed: The robot linear velocity.
        :type speed: float
        :param target_distance: The target distance to the boundary when wall-following.
        :type target_distance: float
        :param Kp: The proportional gain on the distance error term.
        :type Kp: float
        :param Ki: The integral gain on the distance error term.
        :type Ki: float
        :param Kd: The derivative gain on the distance error term.
        :type Kd: float
        :param Kth: The proportional gain on the angular error term.
        :type Kth: float
        """
        super().__init__('BugOneController')

        self.base_speed = speed
        # The angular velocity when turning in-place.
        self.turn_speed = speed / 3

        # Wall following configuration.
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kth = Kth
        self.target_distance = target_distance
        self.error = 0
        # Error calculation is at 10 Hz, so keep only a few seconds of history.
        self.errors = deque(maxlen=15)

        self.create_subscription(LaserScan, lidar_topic, self.update_lidar)
        self.create_subscription(Pose2D, gps_topic, self.update_position)
        self.create_subscription(Pose2D, goal_topic, self.update_goal)
        self.create_subscription(ByteMultiArray, contact_topic, self.update_contact)

        self.pub = self.create_publisher(Twist, twist_topic)

        # TODO: I should technically use locks, but it hasn't become an issue yet.
        # The current state of the sensors
        self.position = Pose2D()
        self.goal = Pose2D()
        self.contacts = [False] * 10

        # The q_l and q_h points
        self.hit_point = Pose2D()
        self.leave_point = Pose2D()

        self.boundary_follow = False

        self.job = threading.Thread(target=self.algorithm, daemon=True)
        self.job.start()

    def update_lidar(self, scan):
        """Receive updates from the robot LIDAR sensor.

        :param scan: The LIDAR sweep
        :type scan: sensor_msgs.msg.LaserScan
        """
        if self.boundary_follow:
            l = len(scan.ranges)
            # The start and stop indices in the sweep for the left side.
            i_0 = l // 2
            i_n = l

            min_distance = scan.ranges[i_0]
            min_i = i_0

            for i, d in enumerate(scan.ranges[i_0:i_n + 1]):
                if d < min_distance:
                    min_distance = d
                    min_i = i

            if min_distance > scan.range_max:
                print('Infinite range on left side.')
                return

            # Get the min distance by hand so that we can also get the angle it occurs at.
            min_angle = (min_i - l // 2) * scan.angle_increment
            front_distance = scan.ranges[l // 2]

            # A poor man's derivative.
            error_d = min_distance - self.target_distance - self.error
            self.error = min_distance - self.target_distance
            # Note that errors has forgetfulness.
            self.errors.append(self.error)
            # Not really a true integral, but has a similar effect.
            error_i = sum(self.errors) / len(self.errors)

            self.controller(front_distance, min_angle, self.error, error_i, error_d)

    def controller(self, front_distance, min_angle, error, error_i, error_d):
        """PD controller to keep the robot at a set distance and angle from the boundary.

        :param front_distance: The distance reading in front of the robot.
        :type front_distance: float
        :param min_angle: The angle to the minimum distance.
        :type min_angle: float
        :param error: The error to correct.
        :type error: float
        :param error_i: The integral of the error.
        :type error_i: float
        :param error_d: The time derivative of the error.
        :type error_d: float
        """
        msg = Twist()
        # A hybrid PD controller to control both the distance and the heading of the robot.
        msg.angular.z = self.Kp * error +  self.Ki * error_i + self.Kd * error_d + self.Kth * (min_angle - np.pi / 2)

        if front_distance < self.target_distance:
            msg.linear.x = 0.0
        elif front_distance < 2 * self.target_distance:
            msg.linear.x = self.base_speed / 2
        elif abs(min_angle) > 1.75:
            msg.linear.x = self.base_speed / 2
        else:
            msg.linear.x = self.base_speed

        self.pub.publish(msg)

    def update_position(self, pose):
        """Receive update from the robot GPS sensor.

        :param pose: The GPS sensor reading
        :type pose: geometry_msgs.msg.Pose2D
        """
        # Rotate so that 0rad is in pos x direction instead of pos y...
        # TODO: <rant></rant>
        pose.theta = self.remap_angle(pose.theta + np.pi/2)
        self.position = pose

        if self.boundary_follow:
            if self._pose_distance(pose, self.goal) < self._pose_distance(self.leave_point, self.goal):
                self.leave_point = pose

    def update_goal(self, pose):
        """Receive updates from the robot's goal.

        :param pose: The goal GPS sensor reading
        :type pose: geometry_msgs.msg.Pose2D
        """
        self.goal = pose

    def update_contact(self, msg):
        """Receive updates from the robot's touch sensor.

        :param msg: The touch sensor reading.
        :type msg: std_msgs.msg.ByteMultiArray
        """
        # Convert the array of bytes to a list of bools. No need for struct.unpack!
        self.contacts = [bool(ord(b)) for b in msg.data]

    def algorithm(self):
        """Run the Bug 1 algorithm.

        A blocking function that runs in a separate thread to prevent blocking
        the ROS callbacks. This way the sensor readings are still updated and we
        have access to up-to-date information.
        """
        # TODO: Find a way to start() the thread once the node starts spinning.
        print('Waiting for node to spin.')
        while self.position == Pose2D() or self.goal == Pose2D():
            pass

        # The Bug 1 algorithm
        while True:
            print('Heading towards goal.')
            self.head_towards_goal()
            while not self.arrived() and not self.obstructed():
                self.move_forward()

            if self.arrived():
                break

            # Bring the robot to a stop, and back up slightly, and stop again.
            msg = Twist()
            self.pub.publish(msg)
            msg.linear.x = -self.base_speed
            msg.angular.z = -self.turn_speed
            self.pub.publish(msg)
            time.sleep(0.4)
            msg.linear.x = 0.0
            self.pub.publish(msg)

            # The hit point is the robot's current position.
            self.hit_point = self.position
            print(f'Hit point: {self.hit_point}')

            print('Starting boundary following.')
            self.follow_boundary()

            print(f'Leave point: {self.leave_point}')

            print('Heading towards leave point.')
            self.go_to_leave()

        # Stop the robot.
        msg = Twist()
        self.pub.publish(msg)
        print('Finished. Phagocytosis.')

    def follow_boundary(self):
        """Begin following an obstacle boundary."""
        self.boundary_follow = True
        print('Boundary following: Leaving hit point.')
        # Follow the boundary until we leave the hit point.
        while self.reencountered_last():
            pass

        print('Boundary following: Searching for hit point.')
        # Follow the boundary until we reencounter the hit point.
        while not self.arrived() and not self.reencountered_last():
            pass

        print(f'Position "at" hit point: {self.position}')

        self.boundary_follow = False

    def arrived(self):
        """Determine whether the robot has arrived at its goal."""
        # Do not require orientations to be equal, because I'm not *that* masochistic.
        # Also, it's getting late and I'm nowhere near done.
        return self._pose_equal(self.position, self.goal, tol=3.0, theta=False)

    def move_forward(self):
        """Drive the robot forward until directed otherwise."""
        msg = Twist()
        msg.linear.x = self.base_speed
        self.pub.publish(msg)

    def turn(self, heading):
        """Turn the robot in-place to face the given heading.

        Note that this function blocks until the robot's heading is correctly set.

        :param heading: The desired heading in radians.
        :type heading: float
        """
        msg = Twist()

        if heading - self.position.theta < 0:
            msg.angular.z = -self.turn_speed
        else:
            msg.angular.z = self.turn_speed

        self.pub.publish(msg)

        while not math.isclose(self.position.theta, heading, abs_tol=0.1):
            pass

        msg.angular.z = 0.0
        self.pub.publish(msg)

    def head_towards_goal(self):
        """Turn the robot to face the goal, then drive forward.

        Note that this function blocks until the robot's heading is correctly set.
        """
        # Get the vector from the robot to the goal.
        robot = np.array([self.position.x, self.position.y])
        goal = np.array([self.goal.x, self.goal.y])
        v = goal - robot
        # Compute the direction from the robot position to the goal.
        heading = self.remap_angle(np.arctan2(v[1], v[0]))

        self.turn(heading)
        self.move_forward()

    def obstructed(self):
        """Determine whether the robot is obstructed by an obstacle."""
        # Sensors 1, 2, and 3 are front right, front, and front left respectively.
        return any(self.contacts)

    def reencountered_last(self):
        """Determine if the robot has reencountered the hit point."""
        return self._pose_equal(self.position, self.hit_point, tol=0.8, theta=False)

    def go_to_leave(self):
        """Follow the obstacle wall until the leave point in encountered."""
        print('Following boundary to leave point.')
        self.boundary_follow = True
        while not self._pose_equal(self.position, self.leave_point, tol=0.5, theta=False):
            pass
        print(f'Position "at" leave point: {self.position}')
        self.boundary_follow = False
