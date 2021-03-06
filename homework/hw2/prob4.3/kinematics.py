import numpy as np


def x_dot(r, w1, w2, theta):
    """x time derivative for diff. drive forward kinematics"""
    return (r / 2) * (w1 + w2) * np.cos(theta)

def y_dot(r, w1, w2, theta):
    """y time derivative for diff. drive forward kinematics"""
    return (r / 2) * (w1 + w2) * np.sin(theta)

def theta_dot(r, L, w1, w2):
    """theta time derivative for diff. drive forward kinematics"""
    return (r / (2 * L)) * (w1 - w2)
