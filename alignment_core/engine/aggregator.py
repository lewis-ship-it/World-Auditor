def compute_risk_score(results):
    score = 100.0

    for r in results:
        if r.hard_violation:
            score -= 40
        else:
            score -= 5

    return max(score, 0)