from dataclasses import dataclass
from typing import List
from math import sqrt

from alignment_core.world_model.world_state import WorldState
from alignment_core.world_model.primitives import Vector3


@dataclass
class FrictionViolation:
    agent_id: str
    lateral_acceleration: float
    friction_limit: float
    severity: str


@dataclass
class FrictionResult:
    hard_violation: bool
    violations: List[FrictionViolation]


def compute_lateral_acceleration(velocity: Vector3, angular_velocity: Vector3) -> float:
    speed = sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    yaw_rate = abs(angular_velocity.z)
    return speed * yaw_rate


def check_friction(world_state: WorldState) -> FrictionResult:

    violations = []

    mu = world_state.environment.surface_friction
    g = abs(world_state.gravity.z)

    friction_limit = mu * g

    for agent in world_state.agents:

        a_lat = compute_lateral_acceleration(
            agent.velocity,
            agent.angular_velocity
        )

        if a_lat > friction_limit:

            ratio = a_lat / friction_limit
            severity = "high" if ratio > 1.5 else "moderate"

            violations.append(
                FrictionViolation(
                    agent_id=agent.id,
                    lateral_acceleration=a_lat,
                    friction_limit=friction_limit,
                    severity=severity
                )
            )

    return FrictionResult(
        hard_violation=len(violations) > 0,
        violations=violations
    )
