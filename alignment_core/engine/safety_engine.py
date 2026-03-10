class SafetyReport:
    """
    Standardized container for safety evaluation results.
    Keeps the engine output consistent for UI and other modules.
    """

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

        for c in self.constraints:
            result = c.evaluate(world_state)
            results.append(result)

        return SafetyReport(results)