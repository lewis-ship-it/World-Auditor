from .base import Constraint, ConstraintResult

class LoadConstraint(Constraint):
    name = "LoadCapacity"
    severity = "hard"

    def evaluate(self, world_state):
        results = []
        
        # LIST-SAFE PATTERN: Iterate through agents
        for agent in world_state.agents:
            # Check current load vs max allowed load
            current = getattr(agent, "load_weight", 0.0)
            maximum = getattr(agent, "max_load", 1.0)
            
            violated = current > maximum

            results.append(ConstraintResult(
                self.name,
                violated,
                self.severity,
                {"current_load": current, "max_capacity": maximum}
            ))
        return results