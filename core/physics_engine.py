import math

def stopping_distance(velocity, decel, friction):

    if decel * friction == 0:
        return 999

    return (velocity ** 2) / (2 * decel * friction)


def safety_margin(distance, stop_dist):

    return distance - stop_dist