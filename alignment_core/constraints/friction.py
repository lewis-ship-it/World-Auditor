from .base import Constraint, ConstraintResult
import math


class FrictionConstraint(Constraint):
    name = "FrictionSlipRisk"
    severity = "hard"

    def evaluate(self, world_state):
        agent = world_state.agent
        env = world_state.environment

        mu = env.friction
        slope_deg = env.slope
        g = 9.81

        slope_rad = math.radians(slope_deg)

        max_static_force = mu * agent.mass * g * math.cos(slope_rad)
        downhill_force = agent.mass * g * math.sin(slope_rad)

        violated = downhill_force > max_static_force

        return ConstraintResult(
            self.name,
            violated,
            self.severity,
            {
                "downhill_force": downhill_force,
                "max_static_force": max_static_force,
            },
        )