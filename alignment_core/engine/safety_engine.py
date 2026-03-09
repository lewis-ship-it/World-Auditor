class SafetyEngine:

    def __init__(self, constraints):
        self.constraints = constraints


    def evaluate(self, world_state):

        results = []

        for c in self.constraints:

            result = c.evaluate(world_state)

            results.append(result)

        return results