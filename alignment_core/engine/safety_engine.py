class SafetyEngine:

    def __init__(self, constraints):

        self.constraints = constraints


    def evaluate(self, world_state):

        results = []

        for constraint in self.constraints:

            r = constraint.evaluate(world_state)
            results.append(r)

        return results