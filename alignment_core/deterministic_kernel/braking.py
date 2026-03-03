import math
from ..engine.constraint import ConstraintResult

class BrakingConstraint:
    def evaluate(self, world_state):
        results = []
        g = 9.81

        for agent in world_state.agents:
            v = agent.velocity.x
            slope_rad = math.radians(world_state.environment.slope)
            friction = world_state.environment.friction

            # Effective deceleration accounting for gravity on a slope
            # 
            eff_decel = (friction * g * math.cos(slope_rad)) - (g * math.sin(slope_rad))

            if eff_decel <= 0:
                results.append(ConstraintResult("Braking", True, "critical", {"msg": "No Grip"}))
                continue

            stopping_dist = (v ** 2) / (2 * eff_decel)
            
            # FIX: Plural name to match app.py and environment.py
            dist_avail = world_state.environment.distance_to_obstacles

            results.append(ConstraintResult(
                "Braking",
                stopping_dist > dist_avail,
                "hard",
                {"required": stopping_dist, "available": dist_avail}
            ))
        return results