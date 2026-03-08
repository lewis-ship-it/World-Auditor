import math
from .base import Constraint, ConstraintResult

class FrictionConstraint(Constraint):
    name = "Surface Traction"
    severity = "hard"

    def evaluate(self, world_state):
        results = []
        # AI Perception Mapping: Translates visual labels to physics constants
        friction_map = {
            "dry_concrete": 0.85,
            "wet_concrete": 0.45,
            "polished_tile": 0.30,
            "ice": 0.10,
            "loose_gravel": 0.35,
            "default": world_state.environment.friction
        }

        # Check if environment has an AI-detected surface, else use manual slider
        surface = getattr(world_state.environment, 'surface_type', 'default')
        mu = friction_map.get(surface, world_state.environment.friction)

        for agent in world_state.agents:
            # Calculate required friction for the current acceleration
            # Simplified: mu_required = |a| / g
            # For cornering: mu_required = v^2 / (R * g)
            v = math.sqrt(agent.velocity.x**2 + agent.velocity.y**2)
            
            # Estimate deceleration required to stop in available distance
            dist = world_state.environment.distance_to_obstacles
            required_accel = (v**2) / (2 * dist) if dist > 0 else 9.81
            mu_required = required_accel / 9.81

            violated = mu_required > mu

            results.append(ConstraintResult(
                self.name,
                violated,
                self.severity,
                {
                    "surface_detected": surface,
                    "available_friction": mu,
                    "required_friction": round(mu_required, 2),
                    "status": "SLIP RISK" if violated else "STABLE"
                }
            ))
        return results