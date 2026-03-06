import numpy as np


class PhysicsEngine:

    def __init__(self, friction=0.8):
        self.g = 9.81
        self.friction = friction

    def max_corner_speed(self, radius):

        if radius <= 0:
            return 0

        return np.sqrt(self.friction * self.g * radius)

    def braking_distance(self, v, decel):

        return (v**2)/(2*decel)

    def lateral_acceleration(self, v, radius):

        if radius == 0:
            return 0

        return v**2 / radius