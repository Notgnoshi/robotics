import numpy as np

def angle_between(v1, v2):
    """Computes the angle between two vectors"""
    return np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))

def inner_angles(p1, p2, p3):
    """Calculate the inner angles of a triangle, given its three vertices"""
    v1, v2, v3 = p2 - p1, p3 - p2, p1 - p3
    # Have to be damn careful about signs
    return angle_between(v1, -v3), angle_between(-v1, v2), angle_between(v3, -v2)

def magnitude(v):
    """Calculates the euclidean vector norm"""
    return np.sqrt(np.sum(v**2))
