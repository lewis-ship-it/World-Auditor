# alignment_core/constraints/braking.py
from .base import ConstraintResult # Ensure this import exists

def evaluate(self, world_state):
    # ... existing logic ...
    results = []
    for agent in world_state.agents: # Loop through agents like the others
        v = agent.velocity.x # Use the agent's velocity
        # ... calculation logic ...
        results.append(ConstraintResult(
            "Braking", 
            violated=(stopping_distance > env.distance_to_obstacles),
            severity="hard",
            details={"distance": stopping_distance}
        ))
    return results