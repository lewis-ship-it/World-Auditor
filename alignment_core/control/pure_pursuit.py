import math
import numpy as np

def find_lookahead_point(path, current_pos, lookahead=2.0):
    for p in path:
        dist = np.linalg.norm(np.array(p) - np.array(current_pos))
        if dist >= lookahead:
            return p
    return path[-1]


def pure_pursuit_steering(path, current_pos, heading, wheelbase, lookahead=2.0):
    if path is None or len(path) < 2:
        return 0.0

    target = find_lookahead_point(path, current_pos, lookahead)

    dx = target[0] - current_pos[0]
    dy = target[1] - current_pos[1]

    angle_to_target = math.atan2(dy, dx)

    alpha = angle_to_target - heading

    # steering angle
    steer = math.atan2(2 * wheelbase * math.sin(alpha), lookahead)

    return steer