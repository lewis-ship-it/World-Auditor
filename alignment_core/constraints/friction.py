from .base import Constraint, ConstraintResult
from .base import Constraint, ConstraintResult

class FrictionConstraint(Constraint):

    name = "Surface Traction"
    severity = "hard"

    def evaluate(self, world_state):

        v = world_state.agent.velocity
        mu = world_state.environment.surface_friction
        g = world_state.environment.gravity
        dist = world_state.environment.distance_to_obstacles

        if dist <= 0:
            required_mu = 1.0
        else:
            required_accel = (v ** 2) / (2 * dist)
            required_mu = required_accel / g

        violated = required_mu > mu

        msg = f"Required μ={required_mu:.2f} | Available μ={mu:.2f}"

        return ConstraintResult(
            self.name,
            violated,
            self.severity,
            msg
        )