import numpy as np


class ActionOptimizer:

    def __init__(self, safety_engine):
        self.engine = safety_engine


    def find_safe_velocity(self, world_state, max_velocity=15):

        velocities = np.linspace(0.1, max_velocity, 50)

        for v in velocities:

            world_state.agents.velocity = v

            results = self.engine.evaluate(world_state)

            violations = [r for r in results if not r["safe"]]

            if violations:
                return v - 0.2

        return max_velocity