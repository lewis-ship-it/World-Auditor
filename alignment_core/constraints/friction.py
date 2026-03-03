from .base import Constraint, ConstraintResult
import math

class FrictionConstraint(Constraint):
    name = "TractionCheck"
    severity = "medium"

    def evaluate(self, world_state):
        results = []
        env = world_state.environment
        
        # LIST-SAFE PATTERN: Iterate through agents
        for agent in world_state.agents:
            friction_coeff = env.friction
            # Simple check: if friction is dangerously low (like ice)
            violated = friction_coeff < 0.2 

            results.append(ConstraintResult(
                self.name,
                violated,
                self.severity,
                {"effective_friction": friction_coeff}
            ))
        return results