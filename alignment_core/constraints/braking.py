from .base import Constraint, ConstraintResult


class BrakingConstraint(Constraint):

    name = "Braking Feasibility"
    severity = "hard"

    def evaluate(self, world_state):

        v = world_state.agent.velocity
        dist = world_state.environment.distance_to_obstacles
        mu = world_state.environment.surface_friction
        g = world_state.environment.gravity

        if mu == 0:
            stop_dist = float("inf")
        else:
            stop_dist = (v ** 2) / (2 * mu * g)

        violated = stop_dist > dist

        msg = f"StopDist={stop_dist:.2f}m | Available={dist:.2f}m"

        return ConstraintResult(
            self.name,
            violated,
            self.severity,
            msg
        )