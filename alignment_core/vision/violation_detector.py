class ViolationDetector:

    def __init__(self, safety_engine):
        self.engine = safety_engine


    def analyze(self, world_state, velocities):

        violations = []

        for v in velocities:

            world_state.agent.velocity = v

            results = self.engine.evaluate(world_state)

            for r in results:

                if not r["safe"]:

                    violations.append({
                        "velocity": v,
                        "constraint": r["constraint"]
                    })

        return violations