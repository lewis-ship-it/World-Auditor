from ..engine.constraint import ConstraintResult

class LoadConstraint:

    def evaluate(self, world_state):
        results = []

        for agent in world_state.agents:

            if agent.load_weight < 0:
                results.append(
                    ConstraintResult(
                        name="Load",
                        violated=True,
                        message="Negative load detected."
                    )
                )
                continue

            violated = agent.load_weight > agent.max_load

            results.append(
                ConstraintResult(
                    name="Load",
                    violated=violated,
                    message=f"Load {agent.load_weight}kg / Max {agent.max_load}kg"
                )
            )

        return results