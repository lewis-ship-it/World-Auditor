from dataclasses import dataclass


@dataclass
class EnvironmentState:

    gravity: float = 9.81

    surface_friction: float = 0.8

    slope: float = 0

    distance_to_obstacles: float = 10