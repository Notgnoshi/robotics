import numpy as np


def diff_drive_xp(r, w1, w2, theta):
    """x time derivative for diff. drive forward kinematics"""
    return (r / 2) * (w1 + w2) * np.cos(theta)

def diff_drive_yp(r, w1, w2, theta):
    """y time derivative for diff. drive forward kinematics"""
    return (r / 2) * (w1 + w2) * np.sin(theta)

def diff_drive_thetap(r, L, w1, w2):
    """theta time derivative for diff. drive forward kinematics"""
    return (r / (2 * L)) * (w1 - w2)

def diff_drive_ik(r, L, xp, yp, thetap):
    """Inverse kinematics for the differential drive robot.

    :param r: The robot wheel radius.
    :param L: The robot half-axle length.
    :param xp: The x time derivative.
    :param yp: The y time derivative.
    :param thetap: The angular theta time derivative.

    :returns: A (left, right) tuple of wheel speeds.
    """

    v = np.sqrt(xp**2 + yp**2)
    k = thetap / v

    left = (v / r) * (k * L + 1)
    right = (v / r) * (-k * L + 1)

    return left, right
