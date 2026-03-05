import math


class BrakingConstraint:

    def evaluate(self, world_state):

        agent = world_state.agent
        env = world_state.environment

        v = agent.velocity
        mu = env.friction
        g = 9.81

        stopping_distance = (v ** 2) / (2 * mu * g)

        safe = stopping_distance < env.distance_to_obstacles

        return {
            "constraint": "Braking",
            "safe": safe,
            "stopping_distance": stopping_distance
        }