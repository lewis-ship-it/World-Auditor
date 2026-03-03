from dataclasses import dataclass
from .primitives import Vector3

@dataclass
class EnvironmentState:
    temperature: float
    air_density: float
    wind_vector: Vector3
    terrain_type: str
    friction: float              # Matches app.py logic
    slope: float                 # Matches app.py logic
    lighting_conditions: str
    distance_to_obstacles: float # Standardized to plural