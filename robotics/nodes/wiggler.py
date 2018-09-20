from rclpy.node import Node
from std_msgs.msg import Float32
from veranda.SimTimer import SimTimer


class Wiggler(Node):
    """The wiggle code from the textbook, wrapped in a class to avoid
    contracting cancer from using global variables.
    """

    def __init__(self):
        """Creates an instance of the Wiggler class."""
        super().__init__('Wiggler')

        # Create a simulation timer so we can speed up the simulation without
        # breaking things.
        self.simulation = SimTimer(True, 'veranda/timestamp', self)
        self.timer = self.simulation.create_timer(1, self.wiggle_left)

        # Create publishers for the left and right wheels.
        self.left_pub = self.create_publisher(Float32, 'robot/left_wheel')
        self.right_pub = self.create_publisher(Float32, 'robot/right_wheel')

    def wiggle_left(self):
        """Wiggles the robot to the left"""
        self.simulation.destroy_timer(self.timer)
        msg = Float32()
        msg.data = 5.0
        self.right_pub.publish(msg)
        msg.data = 0.0
        self.left_pub.publish(msg)
        self.timer = self.simulation.create_timer(1, self.wiggle_right)

    def wiggle_right(self):
        """Wiggles the robot to the right"""
        self.simulation.destroy_timer(self.timer)
        msg = Float32()
        msg.data = 0.0
        self.right_pub.publish(msg)
        msg.data = 5.0
        self.left_pub.publish(msg)
        self.timer = self.simulation.create_timer(1, self.wiggle_left)
