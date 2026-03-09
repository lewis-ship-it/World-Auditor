import math
from .base import Constraint, ConstraintResult


class StabilityConstraint(Constraint):

    name = "Tipping Risk"
    severity = "hard"

    def evaluate(self, world_state):

        slope = world_state.environment.slope

        h = world_state.agent.center_of_mass_height
        wb = world_state.agent.wheelbase

        if h == 0:
            tipping_angle = 90
        else:
            tipping_angle = math.degrees(math.atan((wb / 2) / h))

        violated = slope > tipping_angle

        msg = f"Slope={slope:.1f}° | Limit={tipping_angle:.1f}°"

        return ConstraintResult(
            self.name,
            violated,
            self.severity,
            msg
        )