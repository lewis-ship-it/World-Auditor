from dataclasses import dataclass
from typing import List
from math import sqrt

from alignment_core.world_model.world_state import WorldState
from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.primitives import Vector3


@dataclass
class StabilityViolation:
    agent_id: str
    lateral_acceleration: float
    tipping_threshold: float
    severity: str


@dataclass
class StabilityResult:
    hard_violation: bool
    violations: List[StabilityViolation]


def compute_lateral_acceleration(velocity: Vector3, angular_velocity: Vector3) -> float:
    """
    Approximate lateral acceleration using:
    a_lat ≈ v * omega
    Where:
    - v = linear speed magnitude
    - omega = yaw rate magnitude
    """
    speed = sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    yaw_rate = abs(angular_velocity.z)

    return speed * yaw_rate


def compute_tipping_threshold(agent: AgentState, gravity: Vector3) -> float:
    """
    Tipping threshold approximation:

    a_max ≈ (support_width / (2 * CG_height)) * g

    We approximate support_width from support polygon bounding box.
    """
    if not agent.support_polygon:
        return float("inf")

    xs = [p.x for p in agent.support_polygon]
    support_width = max(xs) - min(xs)

    cg_height = agent.center_of_mass.z
    g = abs(gravity.z)

    if cg_height == 0:
        return float("inf")

    return (support_width / (2 * cg_height)) * g


def check_stability(world_state: WorldState) -> StabilityResult:
    """
    Evaluate tipping risk for all agents in world state.
    """
    violations = []

    for agent in world_state.agents:
        a_lat = compute_lateral_acceleration(
            agent.velocity,
            agent.angular_velocity
        )

        threshold = compute_tipping_threshold(agent, world_state.gravity)

        if a_lat > threshold:
            severity = "high" if a_lat > threshold * 1.5 else "moderate"

            violations.append(
                StabilityViolation(
                    agent_id=agent.id,
                    lateral_acceleration=a_lat,
                    tipping_threshold=threshold,
                    severity=severity
                )
            )

    return StabilityResult(
        hard_violation=len(violations) > 0,
        violations=violations
    )