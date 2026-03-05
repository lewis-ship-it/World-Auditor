from dataclasses import dataclass

@dataclass
class EnvironmentState:

    friction: float
    slope: float

    distance_to_obstacles: float