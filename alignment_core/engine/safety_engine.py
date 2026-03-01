from alignment_core.constraints.registry import ConstraintRegistry


class SafetyEngine:
    def __init__(self):
        self.registry = ConstraintRegistry()

    def register_constraint(self, constraint):
        self.registry.register(constraint)

    def evaluate(self, world_state):
        results = []

        for constraint in self.registry.get_all():
            result = constraint.evaluate(world_state)
            results.append(result)

        return results