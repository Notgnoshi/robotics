import rclpy as ros
from std_msgs.msg import Float32
from veranda.SimTimer import SimTimer


class Fig8:
    """Drives a robot in a figure 8"""

    def __init__(self, name):
        """Assumes robot's initial pose is (10, 10), facing 3pi/4."""
        self.node = ros.create_node(name)
        self.left_wheel = self.node.create_publisher(Float32, f'{name}/left_wheel')
        self.right_wheel = self.node.create_publisher(Float32, f'{name}/right_wheel')

        r = 0.5
        L = 0.6
        R = 5.0

        self.left_speed = 3.0
        self.right_speed = self.left_speed * (R + 2 * L) / R

        circuit_time = (14 * R) / (r * (self.right_speed + self.left_speed))

        self.simulation = SimTimer(True, 'veranda/timestamp', self.node)
        self.timer = self.simulation.create_timer(circuit_time, self.callback)

    def callback(self):
        msg = Float32()

        msg.data = self.left_speed
        self.left_wheel.publish(msg)

        msg.data = self.right_speed
        self.right_wheel.publish(msg)

        # Swap the wheel speeds to reverse path direction
        self.left_speed, self.right_speed = self.right_speed, self.left_speed
