class ConstraintResult:

    def __init__(
        self,
        name,
        passed,
        message="",
        severity="info",
        score_impact=0
    ):
        self.name = name
        self.passed = passed
        self.message = message
        self.severity = severity
        self.score_impact = score_impact