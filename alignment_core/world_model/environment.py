from dataclasses import dataclass
from .primitives import Vector3


@dataclass
class EnvironmentState:
    temperature: float
    air_density: float
    wind_vector: Vector3
    terrain_type: str
    surface_friction: float
    slope_vector: Vector3
    lighting_conditions: str