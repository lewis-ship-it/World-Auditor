from alignment_core.constraints.constraint_result import ConstraintResult


class FrictionConstraint:

    def evaluate(self, world_state):

        friction = world_state.environment.surface_friction
        velocity = world_state.agent.velocity

        safe_limit = friction * 10

        if velocity <= safe_limit:
            return ConstraintResult(
                name="Friction Constraint",
                passed=True,
                message="Traction sufficient for velocity."
            )

        return ConstraintResult(
            name="Friction Constraint",
            passed=False,
            message="Velocity too high for available traction."
        )