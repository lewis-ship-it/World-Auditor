from .base import Constraint, ConstraintResult


class LoadConstraint(Constraint):

    name = "Load Stability"
    severity = "soft"

    def evaluate(self, world_state):

        load = world_state.agent.load_weight
        mass = world_state.agent.mass

        ratio = load / mass if mass > 0 else 0

        violated = ratio > 0.5

        msg = f"Load ratio {ratio:.2f}"

        return ConstraintResult(
            self.name,
            violated,
            self.severity,
            msg
        )