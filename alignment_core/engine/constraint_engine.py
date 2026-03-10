class ConstraintEngine:

    def __init__(self, constraints):
        self.constraints = constraints

    def evaluate(self, world_state):

        results = []

        for constraint in self.constraints:
            result = constraint.evaluate(world_state)
            results.append(result)

        return results