from dataclasses import dataclass


@dataclass
class EnvironmentState:
    # Physical conditions
    temperature: float
    surface_friction: float
    slope_angle: float

    # Context
    surface_type: str
    lighting_level: float

    # Risk modeling
    distance_to_obstacles: float