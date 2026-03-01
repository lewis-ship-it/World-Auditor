class SafetyReport:
    def __init__(self, results):
        self.results = results

    def hard_violations(self):
        return [r for r in self.results if r.violated and r.severity == "hard"]

    def soft_violations(self):
        return [r for r in self.results if r.violated and r.severity == "soft"]

    def is_safe(self):
        return len(self.hard_violations()) == 0

    def risk_score(self):
        score = 0
        for r in self.results:
            if r.violated:
                score += 10 if r.severity == "hard" else 3
        return score

    def to_dict(self):
        return {
            "safe": self.is_safe(),
            "risk_score": self.risk_score(),
            "results": [r.to_dict() for r in self.results],
        }