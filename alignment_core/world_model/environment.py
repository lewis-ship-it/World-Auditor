from dataclasses import dataclass
from .primitives import Vector3

@dataclass
class EnvironmentState:
    temperature: float
    air_density: float        # Required by your app
    wind_vector: Vector3      # Required by your app
    terrain_type: str         # Required by your app
    friction: float           # Standardized for the Braking Constraint
    slope: float              # Standardized for the Stability Constraint
    lighting_conditions: str
    distance_to_obstacles: float