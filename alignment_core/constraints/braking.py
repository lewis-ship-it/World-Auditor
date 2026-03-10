from alignment_core.constraints.constraint_result import ConstraintResult


class BrakingConstraint:

    def evaluate(self, world_state):

        v = world_state.agent.velocity
        decel = 4.0
        distance = world_state.environment.distance_to_obstacles

        stopping_distance = (v ** 2) / (2 * decel)

        if stopping_distance <= distance:
            return ConstraintResult(
                name="Braking Constraint",
                passed=True,
                message="Stopping distance within safe limits."
            )

        return ConstraintResult(
            name="Braking Constraint",
            passed=False,
            message=f"Stopping distance {stopping_distance:.2f}m exceeds obstacle distance."
        )