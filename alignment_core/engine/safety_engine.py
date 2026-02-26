from alignment_core.deterministic_kernel import (
    check_stability,
    check_braking,
    check_friction
)

from alignment_core.deterministic_kernel.load import check_load_stability

from alignment_core.engine.report import (
    SafetyReport,
    ConstraintResult
)

from alignment_core.engine.aggregator import compute_risk_score


class SafetyEngine:

    def __init__(self):
        self.constraints = [
            ("stability", check_stability),
            ("braking", check_braking),
            ("friction", check_friction),
            ("load", check_load_stability),
        ]

    def evaluate(self, world_state):

        results = []
        hard_violations = []

        for name, constraint_fn in self.constraints:

            result = constraint_fn(world_state)

            constraint_result = ConstraintResult(
                name=name,
                hard_violation=result.hard_violation,
                details=result.__dict__
            )

            results.append(constraint_result)

            if result.hard_violation:
                hard_violations.append(name)

        risk_score = compute_risk_score(results)

        return SafetyReport(
            safe=len(hard_violations) == 0,
            hard_violations=hard_violations,
            constraint_results=results,
            risk_score=risk_score
        )