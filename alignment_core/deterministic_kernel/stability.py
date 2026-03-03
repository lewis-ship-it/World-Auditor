from .base import Constraint, ConstraintResult
import math

class StabilityConstraint(Constraint):
    name = "TippingRisk"
    severity = "hard"

    def evaluate(self, world_state):
        results = []
        # FIX: world_state.agents is a list. We must iterate or select the target agent.
        for agent in world_state.agents:
            env = world_state.environment
            slope = env.slope
            
            # Access attributes from the individual agent object
            center_height = agent.center_of_mass_height
            wheelbase = agent.wheelbase

            # Safety calculation for tipping angle
            tipping_angle = math.degrees(math.atan((wheelbase / 2) / center_height))

            violated = slope > tipping_angle

            results.append(ConstraintResult(
                self.name,
                violated,
                self.severity,
                {
                    "agent_id": agent.id,
                    "slope": slope,
                    "tipping_threshold": tipping_angle,
                },
            ))
        
        # Ensure it returns a list if the engine expects extension, 
        # or a single result if that is the standard.
        return results