class SafetyEngine:

    def __init__(self):
        self.constraints = []

    def register_constraint(self, constraint):
        self.constraints.append(constraint)

    def evaluate(self, world_state):
        results = []

        for constraint in self.constraints:
            constraint_results = constraint.evaluate(world_state)
            results.extend(constraint_results)

        return results