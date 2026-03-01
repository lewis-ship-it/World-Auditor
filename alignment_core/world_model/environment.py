from dataclasses import dataclass
from .primitives import Vector3

@dataclass
class EnvironmentState:
    temperature: float
    air_density: float
    wind_vector: Vector3
    terrain_type: str
    friction: float
    slope: float  # degrees
    distance_to_obstacle: float