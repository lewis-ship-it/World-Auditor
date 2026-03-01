from ..engine.constraint import ConstraintResult
import math

class StabilityConstraint:

    def evaluate(self, world_state):
        results = []

        for agent in world_state.agents:

            slope_rad = math.radians(world_state.environment.slope)

            # tipping condition approximation
            tipping_angle = math.atan(agent.wheelbase / (2 * agent.center_of_mass_height))

            violated = slope_rad > tipping_angle

            results.append(
                ConstraintResult(
                    name="Stability",
                    violated=violated,
                    message=f"Slope {world_state.environment.slope}° / Max safe {math.degrees(tipping_angle):.2f}°"
                )
            )

        return results