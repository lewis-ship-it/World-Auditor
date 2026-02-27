from dataclasses import dataclass
from typing import List
from math import sqrt
from alignment_core.world_model.primitives import Vector3

@dataclass
class BrakingViolation:
    agent_id: str
    stopping_distance: float
    available_distance: float
    severity: str

def compute_slope_adjusted_deceleration(friction: float, gravity_z: float, slope_vector: Vector3) -> float:
    """Calculates max deceleration adjusted for terrain angle."""
    g = abs(gravity_z)
    slope_mag = sqrt(slope_vector.x**2 + slope_vector.y**2 + slope_vector.z**2)
    
    if slope_mag == 0:
        return friction * g

    # sin_theta > 0 is uphill (helps), sin_theta < 0 is downhill (fights)
    sin_theta = slope_vector.z / slope_mag
    gravity_component = g * sin_theta
    
    return (friction * g) + gravity_component

def compute_stopping_distance(speed: float, friction: float, gravity_z: float, slope_vector: Vector3 = None) -> float:
    """Core kinematic equation: d = v² / 2a."""
    if slope_vector:
        a_max = compute_slope_adjusted_deceleration(friction, gravity_z, slope_vector)
    else:
        a_max = friction * abs(gravity_z)

    # Safety: If deceleration is zero or negative, the object never stops
    if a_max <= 0.05:
        return float("inf")

    return (speed ** 2) / (2 * a_max)