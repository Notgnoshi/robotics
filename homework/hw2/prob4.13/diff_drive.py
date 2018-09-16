import itertools

import numpy as np
import rclpy as ros
from std_msgs.msg import Float32
from veranda.SimTimer import SimTimer

from utils import pairwise, angle_between, magnitude


class DiffDrive:
    """A differential drive robot that drives point-to-point.

    The robot works by publishing left and right wheel speeds (radians-per-second) to the topics
    `<robot name>/left_wheel` and `<robot name>/right_wheel`.

    If r is the wheel radius, and L is the distance between the pivot point and the wheel, we have

        theta(t) = (r / (2 * L)) * (w1 - w2) * t + theta_0

    So, assuming fixed wheel speeds, we can solve for the time taken to turn as

        t(theta) = ((2 * L) / r) * ((theta - theta_0) / (w1 - w2))

    But note that theta - theta_0 is the angle between the two motion vectors when turning.
    """

    def __init__(self, name, points, loop=False):
        """Creates a differential drive robot with the given name.

        The name is used to determine which topics to listen and publish to.

        The initial state of the robot is assumed to be (0, 0), facing heading 0.0 radians.

        :param name: The name of the robot.
        :param points: The points through which the robot should drive.
        """
        self.node = ros.create_node(f'{name}_robot')
        # We control our robot by publishing left and right wheel speeds.
        self.left_wheel = self.node.create_publisher(Float32, f'{name}/left_wheel')
        self.right_wheel = self.node.create_publisher(Float32, f'{name}/right_wheel')

        # Use the simulation timer so we can play with the simulation speed.
        self._simulation_timer = SimTimer(True, 'veranda/timestamp', self.node)
        self.timer = self._simulation_timer.create_timer(1, self.callback)

        # Make sure each point is an np.array
        points = [np.array(p) for p in points]
        self.points = points

        configs = self.config()
        def insert_pauses(configs):
            for config in configs:
                yield config
                yield 0.0, 0.0, 1.0

        # Insert pauses between motion to make sure the robot comes to a complete stop.
        configs = insert_pauses(configs)

        self.configs = itertools.cycle(configs) if loop else configs

    def config(self):
        """A finite iterable of robot configurations.

        Makes the (big) assumption that the robot's initial state is at (0, 0),
        and faces 0 radians.

        Each configuration point is a (right speed, left speed, duration) tuple,
        where the speeds are in rad/sec.
        """
        # Fix the wheel speeds, because I'm almost out of rum.
        speed = 10.0
        left = -speed / 2
        right = speed / 2

        # Robot dimension constants
        r = 0.5
        L = 0.6

        # Convert the path points into vectors
        vectors = [p2 - p1 for p1, p2 in pairwise(self.points)]
        # Get the angles between each vector. Loop back around to close the loop.
        angles = [angle_between(v1, v2) for v1, v2 in pairwise(vectors + [vectors[0]])]
        distances = [magnitude(v) for v in vectors]

        for length, theta in zip(distances, angles):
            print(f'length = {length}')
            # Calculate how long it will take to drive the straight distance.
            drive_time = length / (2 * np.pi * r * speed)
            yield speed, speed, drive_time

            print(f'angle = {theta}')
            # Calculate how long it will take to turn the given angle with fixed wheel speeds.
            turn_time = (2 * L * theta) / (r * (right - left))
            yield right, left, turn_time

    def callback(self):
        """Publishes the wheel speeds to control the robot.

        Consumes the self.configs generator.
        """
        # Destroy the timer
        self._simulation_timer.destroy_timer(self.timer)
        try:
            # Get the next configuration point.
            right, left, time = next(self.configs)
            print(f'config = ({right}, {left}, {time})')
            msg = Float32()
            # Publish the wheel speeds, without whitespace between the lines to reduce
            # runtime, because that's totally how that works.
            msg.data = right
            self.right_wheel.publish(msg)
            msg.data = left
            self.left_wheel.publish(msg)
            # BUG: The wheels are not synchronized?
            self.timer = self._simulation_timer.create_timer(time, self.callback)
        except StopIteration:
            pass
