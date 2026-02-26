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


def compute_stopping_distance(speed: float, friction: float, gravity_z: float) -> float:
    """
    Using:
    a_max = μg
    d = v² / (2a)
    """
    g = abs(gravity_z)
    max_deceleration = friction * g

    if max_deceleration == 0:
        return float("inf")

    return (speed ** 2) / (2 * max_deceleration)


def compute_available_distance(agent: AgentState, world_state: WorldState) -> float:
    """
    Simple forward ray distance.
    Assumes forward is along +X axis for now.
    Replace later with real orientation transform.
    """
    min_distance = float("inf")

    for obj in world_state.objects:
        dx = obj.position.x - agent.position.x

        if dx > 0:  # object in front
            min_distance = min(min_distance, dx)

    return min_distance


def check_braking(world_state: WorldState) -> BrakingResult:
    violations = []

    for agent in world_state.agents:

        speed = compute_speed(agent.velocity)

        stopping_distance = compute_stopping_distance(
            speed,
            world_state.environment.surface_friction,
            world_state.gravity.z
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

    return BrakingResult(
        hard_violation=len(violations) > 0,
        violations=violations
    )