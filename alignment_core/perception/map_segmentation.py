import numpy as np


def segment_track(points):

    segments = []

    for i in range(1, len(points)-1):

        p1 = points[i-1]
        p2 = points[i]
        p3 = points[i+1]

        curvature = np.linalg.norm(p3 - 2*p2 + p1)

        if curvature < 0.01:
            segments.append("straight")

        elif curvature < 0.1:
            segments.append("curve")

        else:
            segments.append("hairpin")

    return segments