# FILE: alignment_core/constraints/friction.py

import numpy as np
from .base_constraint import BaseConstraint

class FrictionKernel(BaseConstraint):
    def __init__(self, tire_model, mu_static, mu_kinetic):
        self.tire = tire_model
        self.mu_s = mu_static
        self.mu_k = mu_kinetic
        self.g = 9.81

    def evaluate(self, velocity, radius, normal_force_total, req_accel):
        """
        The 'Elastic' Friction Audit.
        """
        # 1. Calculate Required Centripetal Force
        # F_req = (m * v^2) / r
        f_lat_req = (normal_force_total / self.g) * (velocity**2 / radius) if radius != 0 else 0
        
        # 2. Calculate Required Slip Angle (Alpha)
        # alpha = F_y / Ca
        # This tells the AI how much it needs to 'drift' to make the turn
        required_alpha = f_lat_req / (self.tire.ca + 1e-6)
        
        # 3. Combined Force Check (The Friction Circle)
        f_long_req = (normal_force_total / self.g) * req_accel
        total_force_req = np.sqrt(f_lat_req**2 + f_long_req**2)
        
        # 4. Limit Check
        max_grip = normal_force_total * self.mu_s
        utilization = total_force_req / max_grip
        
        # Common Sense Check: 
        # If the required slip angle is > 12 degrees (approx 0.2 rad), 
        # the tire is usually sliding regardless of the mu check.
        is_legal = (utilization < 1.0) and (abs(required_alpha) < 0.21)

        return {
            "is_legal": is_legal,
            "slip_angle_deg": round(np.degrees(required_alpha), 2),
            "grip_utilization": round(utilization, 3),
            "reasoning": self._generate_report(utilization, required_alpha)
        }

    def _generate_report(self, util, alpha):
        if util > 1.0: return "VETO: Force request exceeds friction circle (Kinetic Slide)."
        if abs(alpha) > 0.21: return "VETO: Excessive slip angle. Tire saturation reached."
        if abs(alpha) > 0.12: return "WARNING: Heavy understeer detected. Reducing steering authority."
        return "NOMINAL: Traction and Slip within elastic limits."