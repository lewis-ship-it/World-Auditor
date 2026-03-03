from dataclasses import dataclass
from .primitives import Vector3

@dataclass
class EnvironmentState:
    # Physical conditions
    temperature: float
    air_density: float        # Added to match app.py
    wind_vector: Vector3      # Added to match app.py
    terrain_type: str         # Added to match app.py
    surface_friction: float
    slope: float              # Renamed from slope_angle to match app.py usage

    # Context
    lighting_conditions: str  # Renamed to be more descriptive

    # Risk modeling
    distance_to_obstacles: float