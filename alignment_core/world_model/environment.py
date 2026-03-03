from dataclasses import dataclass
from .primitives import Vector3

@dataclass
class EnvironmentState:
    # Physical conditions
    temperature: float
    air_density: float
    wind_vector: Vector3
    terrain_type: str
    friction: float  # Changed from surface_friction to friction
    slope: float

    # Context
    lighting_conditions: str

    # Risk modeling
    distance_to_obstacles: float