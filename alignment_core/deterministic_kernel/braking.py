import math
from ..engine.constraint import ConstraintResult

class BrakingConstraint:

    def evaluate(self, world_state):
        results = []

        g = abs(world_state.gravity.z)

        for agent in world_state.agents:

            v = agent.velocity.x
            slope_rad = math.radians(world_state.environment.slope)
            friction = world_state.environment.friction

            # effective deceleration
            gravity_component = g * math.sin(slope_rad)
            friction_component = friction * g * math.cos(slope_rad)

            effective_decel = friction_component - gravity_component

            if effective_decel <= 0:
                results.append(
                    ConstraintResult(
                        name="Braking",
                        violated=True,
                        message="Vehicle cannot brake on this slope."
                    )
                )
                continue

            stopping_distance = (v ** 2) / (2 * effective_decel)
            distance_available = world_state.environment.distance_to_obstacle

            violated = stopping_distance > distance_available

            results.append(
                ConstraintResult(
                    name="Braking",
                    violated=violated,
                    message=f"Stopping distance {stopping_distance:.2f}m, available {distance_available:.2f}m"
                )
            )

        return results