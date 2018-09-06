"""
Runs forward and inverse kinematics for the two link manipulator
"""
import numpy as np


def forward(a1, a2, q1, q2):
    """Forward kinematics for a configuration point"""
    x = a2 * np.cos(q1 + q2) + a1 * np.cos(q1)
    y = a2 * np.sin(q1 + q2) + a1 * np.sin(q1)

    return x, y


def inverse(a1, a2, x, y):
    """Inverse kinematics for a workspace point"""
    D = (x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2)
    q2 = np.arctan2(np.sqrt(1 - D**2), D)
    q1 = np.arctan2(y, x) - np.arctan2(a2 * np.sin(q2), a1 + a2 * np.cos(q2))

    return q1, q2
