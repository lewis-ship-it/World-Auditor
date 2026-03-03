from dataclasses import dataclass
from .primitives import Vector3

@dataclass
class EnvironmentState:
    # Physical conditions
    temperature: float
    air_density: float        # Required by app.py
    wind_vector: Vector3      # Required by app.py
    terrain_type: str         # Required by app.py
    
    # Surface & Slope
    surface_friction: float
    slope: float              # Standardized name for app.py
    
    # Context & Risk
    lighting_conditions: str
    distance_to_obstacles: float