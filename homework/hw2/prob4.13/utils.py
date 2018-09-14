import itertools
import numpy as np

def angle_between(v1, v2):
    """Computes the angle between two vectors"""
    return np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))

def inner_angles(p1, p2, p3):
    """Calculate the inner angles of a triangle, given its three vertices"""
    v1, v2, v3 = p2 - p1, p3 - p2, p1 - p3
    # Have to be damn careful about signs
    return angle_between(v1, -v3), angle_between(-v1, v2), angle_between(v3, -v2)

def distance(p1, p2):
    """Calculates the euclidean distance between two points"""
    return np.sqrt(np.sum((p2 - p1)**2))

def pairwise(iterable):
    """s -> (s0, s1), (s1, s2), (s2, s3), ..."""
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)


if __name__ == '__main__':
    vertices = [np.array((0, 0)), np.array((15, 0)), np.array((5, 20))]
    print(inner_angles(*vertices))
