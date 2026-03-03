from dataclasses import dataclass
from .primitives import Vector3

@dataclass
class EnvironmentState:
    # Physical conditions
    temperature: float
    air_density: float
    wind_vector: Vector3
    terrain_type: str
    surface_friction: float
    slope: float  # Renamed from slope_angle to match usage [cite: 10]

    # Context
    lighting_conditions: str # Updated from lighting_level [cite: 68]

    # Risk modeling
    distance_to_obstacles: float