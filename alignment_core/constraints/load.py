from .base import Constraint, ConstraintResult


class LoadConstraint(Constraint):
    name = "LoadOverCapacity"
    severity = "hard"

    def evaluate(self, world_state):
        agent = world_state.agent

        violated = agent.load_weight > agent.max_load

        return ConstraintResult(
            self.name,
            violated,
            self.severity,
            {
                "load_weight": agent.load_weight,
                "max_load": agent.max_load,
            },
        )