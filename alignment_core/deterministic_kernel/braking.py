from dataclasses import dataclass
from typing import List
from math import sqrt

from alignment_core.world_model.world_state import WorldState
from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.primitives import Vector3

@dataclass
class BrakingViolation:
    agent_id: str
    stopping_distance: float
    available_distance: float
    severity: str

@dataclass
class BrakingResult:
    hard_violation: bool
    violations: List[BrakingViolation]

def compute_speed(velocity: Vector3) -> float:
    return sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

# --- YOUR NEW LOGIC INTEGRATED HERE ---
def compute_slope_adjusted_deceleration(
    friction: float,
    gravity_z: float,
    slope_vector: Vector3
) -> float:
    """
    Computes effective max deceleration considering slope.
    Positive slope.z = uphill (helps braking)
    Negative slope.z = downhill (fights braking)
    """
    g = abs(gravity_z)

    slope_magnitude = sqrt(
        slope_vector.x**2 +
        slope_vector.y**2 +
        slope_vector.z**2
    )

    if slope_magnitude == 0:
        return friction * g

    # sin(theta) is vertical rise over total magnitude
    sin_theta = slope_vector.z / slope_magnitude
    gravity_component = g * sin_theta

    # Effective deceleration: friction stops you, gravity either adds or subtracts
    a_effective = (friction * g) + gravity_component

    return a_effective

def compute_stopping_distance(speed: float, friction: float, gravity_z: float, slope_vector: Vector3 = None) -> float:
    """
    Using the standard d = v² / (2a) formula.
    """
    if slope_vector:
        max_deceleration = compute_slope_adjusted_deceleration(friction, gravity_z, slope_vector)
    else:
        max_deceleration = friction * abs(gravity_z)

    if max_deceleration <= 0:
        return float("inf") # Will never stop if downhill gravity > friction!

    return (speed ** 2) / (2 * max_deceleration)

def compute_available_distance(agent: AgentState, world_state: WorldState) -> float:
    # (Existing raycast logic remains the same)
    min_distance = float("inf")
    for obj in world_state.objects:
        dx = obj.position.x - agent.position.x
        if dx > 0:
            min_distance = min(min_distance, dx)
    return min_distance

def check_braking(world_state: WorldState) -> BrakingResult:
    violations = []
    for agent in world_state.agents:
        speed = compute_speed(agent.velocity)
        
        # Now passing the slope_vector into the calculation
        stopping_distance = compute_stopping_distance(
            speed,
            world_state.environment.surface_friction,
            world_state.gravity.z,
            slope_vector=agent.orientation # Assuming agent.orientation represents the slope
        )

        available_distance = compute_available_distance(agent, world_state)

        if stopping_distance > available_distance:
            ratio = stopping_distance / available_distance if available_distance > 0 else 999
            severity = "high" if ratio > 1.5 else "moderate"
            violations.append(
                BrakingViolation(
                    agent_id=agent.id,
                    stopping_distance=stopping_distance,
                    available_distance=available_distance,
                    severity=severity
                )
            )

    return BrakingResult(hard_violation=len(violations) > 0, violations=violations)