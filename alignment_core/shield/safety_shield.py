class SafetyShield:

    def __init__(self, safety_engine):
        self.engine = safety_engine


    def intercept(self, world_state, action):

        results = self.engine.evaluate(world_state)

        violations = [r for r in results if not r["safe"]]

        if violations:

            return {
                "approved": False,
                "violations": violations
            }

        return {
            "approved": True,
            "violations": []
        }