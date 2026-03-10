from alignment_core.constraints.constraint_result import ConstraintResult


class LoadConstraint:

    def evaluate(self, world_state):

        load = world_state.agent.load_weight
        max_load = world_state.agent.max_load

        if load <= max_load:
            return ConstraintResult(
                name="Load Constraint",
                passed=True,
                message="Load within structural limits."
            )

        return ConstraintResult(
            name="Load Constraint",
            passed=False,
            message="Payload exceeds maximum load capacity."
        )