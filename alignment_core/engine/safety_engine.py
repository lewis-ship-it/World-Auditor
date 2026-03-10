from alignment_core.engine.constraint_engine import ConstraintEngine
from alignment_core.engine.risk_engine import RiskEngine


class SafetyReport:

    def __init__(self, results, score):

        self.results = results
        self.score = score


class SafetyEngine:

    def __init__(self, constraints):

        self.constraint_engine = ConstraintEngine(constraints)
        self.risk_engine = RiskEngine()

    def evaluate(self, world_state):

        results = self.constraint_engine.evaluate(world_state)

        score = self.risk_engine.compute_score(results)

        return SafetyReport(results, score)