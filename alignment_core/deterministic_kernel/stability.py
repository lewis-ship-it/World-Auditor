import math
from ..engine.constraint import ConstraintResult

class StabilityConstraint:

    def evaluate(self, world_state):
        results = []

        for agent in world_state.agents:

            slope_rad = math.radians(world_state.environment.slope)

            if agent.center_of_mass_height <= 0:
                results.append(
                    ConstraintResult(
                        name="Stability",
                        violated=True,
                        message="Invalid center of mass height."
                    )
                )
                continue

            tipping_angle = math.atan(
                agent.wheelbase / (2 * agent.center_of_mass_height)
            )

            violated = slope_rad > tipping_angle

            results.append(
                ConstraintResult(
                    name="Stability",
                    violated=violated,
                    message=f"Slope {world_state.environment.slope:.1f}° / Max safe {math.degrees(tipping_angle):.1f}°"
                )
            )

        return results