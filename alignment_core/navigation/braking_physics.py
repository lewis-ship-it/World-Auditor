import numpy as np

GRAVITY = 9.81


def max_safe_speed(distance, friction):

    if friction <= 0:
        return 0

    return np.sqrt(2 * friction * GRAVITY * distance)


def compute_speed_profile(distances, friction):

    speeds = []

    for d in distances:

        v = max_safe_speed(d, friction)

        speeds.append(v)

    return np.array(speeds)