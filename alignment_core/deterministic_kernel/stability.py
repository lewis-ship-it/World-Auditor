from .base import Constraint, ConstraintResult
import math

class StabilityConstraint(Constraint):
    name = "TippingRisk"
    severity = "hard"

    def evaluate(self, world_state):
        results = []
        # FIX: Explicitly loop through agents to avoid 'list' attribute error
        for agent in world_state.agents:
            env = world_state.environment
            
            # Using getattr to safely handle the slope parameter
            slope = getattr(env, "slope", 0.0)
            center_height = agent.center_of_mass_height
            wheelbase = agent.wheelbase

            # Tipping physics: critical angle when CoM moves outside wheelbase
            # 
            if center_height > 0:
                tipping_angle = math.degrees(math.atan((wheelbase / 2) / center_height))
            else:
                tipping_angle = 90.0

            violated = slope > tipping_angle

            results.append(ConstraintResult(
                self.name,
                violated,
                self.severity,
                {
                    "agent_id": agent.id,
                    "slope": slope,
                    "tipping_threshold": tipping_angle,
                    "margin": tipping_angle - slope
                }
            ))
        return results