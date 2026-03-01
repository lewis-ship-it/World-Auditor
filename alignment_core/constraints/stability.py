from .base import Constraint, ConstraintResult
import math


class StabilityConstraint(Constraint):
    name = "TippingRisk"
    severity = "hard"

    def evaluate(self, world_state):
        agent = world_state.agents
        env = world_state.environment

        slope = env.slope
        center_height = agent.center_of_mass_height
        wheelbase = agent.wheelbase

        tipping_angle = math.degrees(math.atan((wheelbase / 2) / center_height))

        violated = slope > tipping_angle

        return ConstraintResult(
            self.name,
            violated,
            self.severity,
            {
                "slope": slope,
                "tipping_threshold": tipping_angle,
            },
        )