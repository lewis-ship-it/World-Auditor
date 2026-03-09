class SafetyReport:

    def __init__(self, results):

        self.results = results


    def is_safe(self):

        for r in self.results:
            if r.violated and r.severity == "hard":
                return False

        return True


    def risk_score(self):

        score = 100

        for r in self.results:

            if r.violated:

                if r.severity == "hard":
                    score -= 30
                else:
                    score -= 10

        return max(score,0)