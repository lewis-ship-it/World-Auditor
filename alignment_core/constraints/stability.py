from alignment_core.constraints.constraint_result import ConstraintResult
import math


class StabilityConstraint:

    def evaluate(self, world_state):

        slope = abs(world_state.environment.slope)
        com = world_state.agent.center_of_mass_height
        wheelbase = world_state.agent.wheelbase

        tipping_angle = math.degrees(math.atan(wheelbase / (2 * com)))

        if slope < tipping_angle:
            return ConstraintResult(
                name="Stability Constraint",
                passed=True,
                message="Robot stable on current slope."
            )

        return ConstraintResult(
            name="Stability Constraint",
            passed=False,
            message="Slope exceeds tipping stability threshold."
        )