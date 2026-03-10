class RiskEngine:

    def compute_score(self, results):

        score = 100

        for r in results:

            if not r.passed:
                score -= r.score_impact

        return max(0, score)