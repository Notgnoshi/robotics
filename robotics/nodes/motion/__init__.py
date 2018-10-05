from rclpy.node import Node


class BaseMotionPlanner(Node):
    """A base class for motion planners."""

    def __init__(self, position_topic, goal_topic):
        """TODO: Define interface, but wait until a few planners have been implemented."""
        raise NotImplementedError('Base class not implemented yet')
