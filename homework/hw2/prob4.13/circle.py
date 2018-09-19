import rclpy as ros
from std_msgs.msg import Float32


class Circle:
    """Drives a differential drive robot in a circle"""

    def __init__(self, name):
        self.node = ros.create_node(name)
        # We control our robot by publishing left and right wheel speeds.
        self.left_wheel = self.node.create_publisher(Float32, f'{name}/left_wheel')
        self.right_wheel = self.node.create_publisher(Float32, f'{name}/right_wheel')

        # Driving in a circle is easy, so assume the robot's initial position
        # is at (15, 0), facing pi/2 rad and drive with w1 = 1.08w2.
        left_speed = 10.0
        right_speed = 1.08 * left_speed
        msg = Float32()
        msg.data = right_speed
        self.right_wheel.publish(msg)
        msg.data = left_speed
        self.left_wheel.publish(msg)
