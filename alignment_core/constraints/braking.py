from .base import Constraint, ConstraintResult
import math

class BrakingConstraint(Constraint):
    name = "BrakingSafety"
    severity = "hard"

    def evaluate(self, world_state):
        results = []
        g = 9.81
        env = world_state.environment

        # LIST-SAFE PATTERN: Iterate through agents
        for agent in world_state.agents:
            v = agent.velocity.x
            slope_rad = math.radians(env.slope)
            friction = env.friction

            # 
            # Account for gravity pulling the robot down the slope
            eff_decel = (friction * g * math.cos(slope_rad)) - (g * math.sin(slope_rad))

            if eff_decel <= 0:
                results.append(ConstraintResult(self.name, True, "critical", {"msg": "Zero traction"}))
                continue

            stopping_dist = (v ** 2) / (2 * eff_decel)
            dist_avail = env.distance_to_obstacles

            results.append(ConstraintResult(
                self.name,
                stopping_dist > dist_avail,
                self.severity,
                {"required": stopping_dist, "available": dist_avail}
            ))
        return results