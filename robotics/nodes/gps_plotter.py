import multiprocessing as mp
import os
import signal
from collections import deque

import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose2D
from rclpy.node import Node


class GpsPlotter(Node):
    """Live plots robot path with matplotlib."""

    def __init__(self, name, topic, history=1000):
        """Creates a GpsPlotter instance.

        :param name: The name of the node.
        :param topic: The topic publishing Pose2D messages to subscribe to.
        :param history: The maximum number of points to plot. The most recent
            points will be kept. Set to None to disable.
        """
        super().__init__(name)
        self.subscriber = self.create_subscription(Pose2D, topic, self.callback)

        self.queue = mp.Queue()
        self.history = history

        self.job = mp.Process(target=self.plotter, daemon=True)
        self.job.start()

    def destroy_node(self):
        """Terminate the plotting job."""
        super().destroy_node()

        # HACK: Using self.job.terminate() leaves a zombie process when the plot
        #       window is closed. Using os.kill() kills the process, but raises
        #       a ProcessLookupError because the process is a zombie?
        try:
            os.kill(self.job.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def callback(self, msg):
        """Receive points from the GPS sensor to plot.

        :param msg: The Pose2D message to receive.
        """
        # Decouple the event triggering the callback and the matplotlib plotter
        # so we don't accidentally block important shit from happening.
        self.queue.put((msg.x, msg.y), block=False)

    def plotter(self):
        """Receives points from the callback in a nonblocking manner."""
        xs = deque([], maxlen=self.history)
        ys = deque([], maxlen=self.history)

        fig, ax = plt.subplots()
        plt.ion()
        plot, = plt.plot([])
        plt.title('Robot GPS Path')

        while True:
            # If the plotting window is closed, exit.
            if not plt.fignum_exists(fig.number):
                break
            x, y = self.queue.get(block=True)

            xs.append(x)
            ys.append(y)

            plot.set_data(xs, ys)

            # Fit the plot window to the data.
            ax.autoscale()
            ax.relim()

            # This should be smaller than the publishing interval.
            plt.pause(0.0001)
