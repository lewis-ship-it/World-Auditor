from dataclasses import dataclass
from typing import List
from math import sqrt

from alignment_core.world_model.world_state import WorldState


@dataclass
class LoadViolation:
    agent_id: str
    adjusted_cg_height: float
    tipping_threshold: float
    severity: str


@dataclass
class LoadResult:
    hard_violation: bool
    violations: List[LoadViolation]


def check_load_stability(world_state: WorldState) -> LoadResult:

    violations = []

    for agent in world_state.agents:

        if not agent.current_load:
            continue

        load_mass = agent.current_load.mass
        load_height = agent.current_load.position.z

        total_mass = agent.mass + load_mass

        # Weighted center of mass
        adjusted_cg_height = (
            (agent.mass * agent.center_of_mass.z) +
            (load_mass * load_height)
        ) / total_mass

        if not agent.support_polygon:
            continue

        xs = [p.x for p in agent.support_polygon]
        support_width = max(xs) - min(xs)

        g = abs(world_state.gravity.z)

        tipping_threshold = (support_width / (2 * adjusted_cg_height)) * g

        # Use current lateral acceleration
        speed = sqrt(
            agent.velocity.x**2 +
            agent.velocity.y**2 +
            agent.velocity.z**2
        )

        yaw_rate = abs(agent.angular_velocity.z)
        a_lat = speed * yaw_rate

        if a_lat > tipping_threshold:

            severity = "high"

            violations.append(
                LoadViolation(
                    agent_id=agent.id,
                    adjusted_cg_height=adjusted_cg_height,
                    tipping_threshold=tipping_threshold,
                    severity=severity
                )
            )

    return LoadResult(
        hard_violation=len(violations) > 0,
        violations=violations
    )