from .base import Constraint, ConstraintResult
import math

class StabilityConstraint(Constraint):
    name = "TippingRisk"
    severity = "hard"

    def evaluate(self, world_state) -> list:
        results = []
        env = world_state.environment
        slope = env.slope

        for agent in world_state.agents:
            # Avoid division by zero if COM height is invalid 
            center_height = max(agent.center_of_mass_height, 0.01)
            wheelbase = agent.wheelbase

            # Tipping threshold calculation: tan(theta) = (wheelbase/2) / COM_height [cite: 40, 59]
            tipping_angle = math.degrees(math.atan((wheelbase / 2) / center_height))

            violated = slope > tipping_angle

            results.append(
                ConstraintResult(
                    self.name,
                    violated,
                    self.severity,
                    {
                        "agent_id": agent.id,
                        "slope": slope,
                        "tipping_threshold": tipping_angle,
                        "details": f"Slope {slope:.1f}° exceeds max safe angle {tipping_angle:.1f}°" [cite: 41, 60]
                    }
                )
            )

        return results