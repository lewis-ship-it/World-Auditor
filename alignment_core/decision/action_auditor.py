class ActionAuditor:

    def __init__(self, safety_engine):
        self.engine = safety_engine


    def audit(self, world_state, action):

        results = self.engine.evaluate(world_state)

        violations = [r for r in results if not r["safe"]]

        if violations:

            return {
                "allowed": False,
                "reason": violations
            }

        return {
            "allowed": True,
            "reason": "Action safe"
        }