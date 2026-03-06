from dataclasses import dataclass


@dataclass
class EnvironmentState:

    gravity: float = 9.81

    surface_friction: float = 0.8

    slope: float = 0.0

    wind: float = 0.0

    temperature: float = 20.0

    distance_to_obstacles: float = 0.0