import itertools

import numpy as np
import rclpy as ros
from std_msgs.msg import Float32
from veranda.SimTimer import SimTimer

from utils import *


class Triangle:
    """Drives a differential drive robot along a triangular path with vertices
        (0, 0) -> (15, 0) -> (5, 20) -> (0, 0)

    We have, where $r$ is the wheel radius, and $L$ is the distance between the
    centers of the wheels,
        $$\\theta = \\frac{r}{2L}(\\omega_1 - \\omega_2)t + \\theta_0$$
    so then, for fixed and opposite wheel speeds, the time for a turn, given
    a specific angle, is given by
        $$t = \\frac{2L}{r}\\frac{\\theta - \\theta_0}{\\omega_1 - \\omega_2}$$
    """

    def __init__(self):
        """Create a ROS node to dive in a triangle."""
        self.node = ros.create_node('Triangle')

        # We control our robot by publishing left and right wheel speeds.
        self.left_wheel = self.node.create_publisher(Float32, 'triangle/left_wheel')
        self.right_wheel = self.node.create_publisher(Float32, 'triangle/right_wheel')

        # Use the simulation timer so we can play with the simulation speed.
        self._simulation_timer = SimTimer(True, 'veranda/timestamp', self.node)
        self.timer = self._simulation_timer.create_timer(1, self.callback)

        # Wheel radius
        self.r = 0.5
        # Distance from center to wheel
        self.L = 0.6

        self.vertices = [np.array((0, 0)), np.array((15, 0)), np.array((5, 20))]

        # Repeatedly loop through all of the configurations.
        self.configurations = itertools.cycle(self.config())

    def config(self):
        """A finite iterable of robot configurations.

        Assumption: The robot's initial state is at (0, 0), facing 0 radians
        (along the positive x axis).

        Each configuration is a (right speed, left speed, duration) tuple.
        """

        # The speed of the robot when it's driving in a straight line.
        speed = 10.0
        # Set the wheel speeds for turns so that it turns in place.
        right_speed = speed / 2
        left_speed = -speed / 2

        # The length of each side of the triangle.
        lengths = [distance(p1, p2) for p1, p2 in pairwise(self.vertices + [self.vertices[0]])]
        # Important: This starts at the second triangle corner.
        angles = inner_angles(*self.vertices)
        # Reorder angles to vertices 2 -> 3 -> 1.
        angles = [angles[i] for i in [1, 2, 0]]

        for length, angle in zip(lengths, angles):
            # Drive straight along a side of the triangle.
            drive_time = length / (2 * np.pi * self.r * speed)
            yield speed, speed, drive_time

            # Pause because timing isn't quite right somewhere
            yield 0.0, 0.0, 1.0

            # Turn in place to prepare for the next side of the triangle.
            turn_time = 2 * self.L * (np.pi - angle) / (self.r * (right_speed - left_speed))
            yield right_speed, left_speed, turn_time

            yield 0.0, 0.0, 1.0

    def callback(self):
        """Publishes the wheel speeds to control the robot."""
        # Destroy the current timer.
        self._simulation_timer.destroy_timer(self.timer)

        # Get the next configuration point.
        right_speed, left_speed, event_time = next(self.configurations)

        # Publish the wheel speeds
        msg = Float32()

        msg.data = left_speed
        self.left_wheel.publish(msg)

        msg.data = right_speed
        self.right_wheel.publish(msg)

        # Call the callback again, the specified number of seconds later.
        self.timer = self._simulation_timer.create_timer(event_time, self.callback)
