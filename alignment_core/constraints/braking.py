import numpy as np
from .base_constraint import BaseConstraint

class BrakingKernel(BaseConstraint):
    def __init__(self, max_braking_force, brake_thermal_limit=5000):
        """
        max_braking_force: The maximum Newtons the actuators can apply.
        brake_thermal_limit: Joules of energy before the brakes 'fade' (lose power).
        """
        self.max_f = max_braking_force
        self.thermal_limit = brake_thermal_limit
        self.current_heat_joules = 0
        self.g = 9.81

    def evaluate(self, velocity, mass, friction_mu, slope_angle_deg, distance_to_target):
        """
        Calculates if the robot can physically stop before the target.
        """
        slope_rad = np.radians(slope_angle_deg)
        
        # 1. BRAKE FADE LOGIC
        # Efficiency drops as heat approaches the limit
        efficiency = max(0.1, 1.0 - (self.current_heat_joules / self.thermal_limit))
        available_mechanical_f = self.max_f * efficiency

        # 2. TRACTION LIMIT (The 'Real' ceiling)
        # You cannot brake harder than the tires can grip the surface
        # F = mu * m * g * cos(theta)
        max_traction_f = friction_mu * mass * self.g * np.cos(slope_rad)

        # 3. GRAVITY VECTOR (The 'Helper' or 'Enemy')
        # F_gravity = m * g * sin(theta)
        # On a downhill (positive angle), gravity pushes the robot forward
        gravity_f_component = mass * self.g * np.sin(slope_rad)

        # 4. TOTAL STOPPING FORCE
        # We take the weaker of the two (mechanical vs traction) and subtract gravity's push
        total_stopping_f = min(available_mechanical_f, max_traction_f) - gravity_f_component

        # Runaway check
        if total_stopping_f <= 0:
            return {
                "is_legal": False,
                "reasoning": "VETO: Runaway condition. Gravity exceeds maximum braking capacity."
            }

        # 5. STOPPING DISTANCE (Kinematic Equation)
        # d = v^2 / (2 * a) where a = F/m
        max_deceleration = total_stopping_f / mass
        stopping_distance = (velocity**2) / (2 * max_deceleration + 1e-6)

        is_safe = stopping_distance < distance_to_target
        
        # 6. HEAT ACCUMULATION (Simplified)
        # Energy = 0.5 * m * v^2
        kinetic_energy = 0.5 * mass * (velocity**2)

        return {
            "is_legal": is_safe,
            "stopping_distance_m": round(stopping_distance, 2),
            "max_decel_ms2": round(max_deceleration, 2),
            "brake_fade_pct": round((1 - efficiency) * 100, 1),
            "energy_to_dissipate_j": round(kinetic_energy, 1),
            "reasoning": self._generate_report(is_safe, stopping_distance, distance_to_target)
        }

    def _generate_report(self, is_safe, stop_dist, target_dist):
        if not is_safe:
            return f"VETO: Inevitable collision. Stop distance ({stop_dist}m) exceeds available space ({target_dist}m)."
        if stop_dist > (target_dist * 0.85):
            return "WARNING: Braking buffer critically low (<15%)."
        return "NOMINAL: Safe braking envelope maintained."