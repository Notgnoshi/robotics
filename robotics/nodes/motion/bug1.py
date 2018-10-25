from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan


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

    def __init__(self, gps_topic, lidar_topic, twist_topic):
        """Create a bug controller.

        :param gps_topic: The robot GPS sensor topic.
        :type gps_topic: str
        :param lidar_topic: The robot LIDAR sensor topic.
        :type lidar_topic: str
        :param twist_topic: The robot control topic.
        :type twist_topic: str
        """
        super().__init__('BugOneController')

        self.create_subscription(LaserScan, lidar_topic, self.update_lidar)
        self.create_subscription(Pose2D, gps_topic, self.update_position)

        self.pub = self.create_publisher(Twist, twist_topic)

    def update_lidar(self, scan):
        """Receive updates from the robot LIDAR sensor.

        :param scan: The LIDAR sweep
        :type scan: sensor_msgs.msg.LaserScan
        """
        pass

    def update_position(self, pose):
        """Receive update from the robot GPS sensor.

        :param pose: The GPS sensor reading
        :type pose: geometry_msgs.msg.Pose2D
        """
        pass
