def find_failure_point(world_factory, engine, base_params):
    v = 0.5
    while v < 20.0:  # Search up to 20m/s
        test_world = world_factory(velocity=v, **base_params)
        report = engine.evaluate(test_world)
        if any(r.violated for r in report):
            return v  # This is your "Edge Case" velocity
        v += 0.5
    return None
def compute_risk_score(results):
    score = 100.0

    for r in results:
        if r.hard_violation:
            score -= 40
        else:
            score -= 5

    return max(score, 0)