class SafetyReport:

    def __init__(self, results):
        self.results = results

    def passed(self):
        return all(r.passed for r in self.results)

    def failed(self):
        return [r for r in self.results if not r.passed]


class SafetyEngine:

    def __init__(self, constraints):
        self.constraints = constraints

    def evaluate(self, world_state):

        results = []

        for constraint in self.constraints:
            result = constraint.evaluate(world_state)
            results.append(result)

        return SafetyReport(results)