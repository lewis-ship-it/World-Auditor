class SafetyEngine:

    def __init__(self, constraints):
        self.constraints = constraints

    def evaluate(self, world_state):

        all_results = []

        for constraint in self.constraints:

            r = constraint.evaluate(world_state)

            if isinstance(r, list):
                all_results.extend(r)
            else:
                all_results.append(r)

        return all_results