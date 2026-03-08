import math

class BrakingModel:
    def __init__(self, friction=0.9, gravity=9.81):
        self.mu = friction
        self.g = gravity

    def max_deceleration(self):
        return self.mu * self.g

    def braking_distance(self, velocity):
        a = self.max_deceleration()
        return (velocity**2) / (2*a)

    def braking_time(self, velocity):
        a = self.max_deceleration()
        return velocity / a

    def simulate_braking(self, velocity, dt=0.05):

        a = self.max_deceleration()

        v = velocity
        pos = 0

        timeline = []

        while v > 0:

            v = max(0, v - a * dt)

            pos += v * dt

            timeline.append({
                "velocity": v,
                "position": pos,
                "acceleration": -a
            })

        return timeline


class BrakingConstraint:

    name = "BrakingFeasibility"
    severity = "hard"

    def evaluate(self, world_state):

        v = world_state.agent.velocity
        dist = world_state.environment.distance_to_obstacles
        mu = world_state.environment.surface_friction

        a = mu * 9.81

        stop_dist = (v**2)/(2*a)

        violated = stop_dist > dist

        return [{
            "constraint": self.name,
            "safe": not violated,
            "required_distance": stop_dist,
            "available_distance": dist
        }]