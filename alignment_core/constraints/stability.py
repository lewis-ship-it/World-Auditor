import math
from alignment_core.constraints.base import BaseConstraint
from alignment_core.constraints.constraint_result import ConstraintResult

class StabilityConstraint(BaseConstraint):
    @property
    def name(self) -> str:
        return "Static Stability Audit"

    def evaluate(self, world_state):
        agent = world_state.agent
        env = world_state.environment
        
        # FIX: Handle cases where center_of_mass is a Vector3 or a float height
        com_h = agent.center_of_mass_height
        if hasattr(com_h, 'z'): # Check if it's a Vector3 from world_factory
            com_h = com_h.z
            
        track_width = agent.wheelbase * 0.8
        
        # Tipping angle calculation
        critical_angle = math.degrees(math.atan((track_width / 2) / com_h))
        
        # Check against environment slope
        passed = abs(env.slope) < critical_angle
        msg = f"Stable at {env.slope}°" if passed else f"Tipping risk! Slope {env.slope}° > Limit {critical_angle:.1f}°"
        
        return ConstraintResult(
            name=self.name,
            passed=passed,
            message=msg
        )