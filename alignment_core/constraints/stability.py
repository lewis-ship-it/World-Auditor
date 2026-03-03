from .base import Constraint, ConstraintResult
import math

class StabilityConstraint(Constraint):
    name = "TippingRisk"
    severity = "hard"

    def evaluate(self, world_state):
        results = []
        # FIX: Explicitly loop through the agents list
        for agent in world_state.agents:
            env = world_state.environment
            slope = env.slope
            
            # Now 'agent' is an AgentState object, so this attribute exists
            center_height = agent.center_of_mass_height
            wheelbase = agent.wheelbase

            # Tipping physics: The angle where gravity pulls the CoM outside the wheelbase
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
                }
            ))
        return results