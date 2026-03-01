import math
from ..engine.constraint import ConstraintResult

class FrictionConstraint:

    def evaluate(self, world_state):
        results = []

        g = abs(world_state.gravity.z)

        for agent in world_state.agents:

            v = agent.velocity.x
            friction = world_state.environment.friction

            # max friction force
            max_friction_force = friction * agent.total_mass() * g

            # required centripetal force (simple slip heuristic)
            required_force = agent.total_mass() * abs(v)

            violated = required_force > max_friction_force

            results.append(
                ConstraintResult(
                    name="Friction",
                    violated=violated,
                    message=f"Required force {required_force:.2f}N / Max friction {max_friction_force:.2f}N"
                )
            )

        return results