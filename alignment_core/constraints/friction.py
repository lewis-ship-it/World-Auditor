import numpy as np
from .base_constraint import BaseConstraint

class FrictionKernel(BaseConstraint):
    def __init__(self, tire_model, mu_static, mu_kinetic):
        """
        tire_model: An instance of TireModel from mechanics.py
        mu_static: The grip coefficient of the current surface (Dry Asphalt ~1.0)
        mu_kinetic: The grip coefficient once sliding (usually 20-30% lower)
        """
        self.tire = tire_model
        self.mu_s = mu_static
        self.mu_k = mu_kinetic
        self.g = 9.81

    def evaluate(self, velocity, radius, normal_force_total, req_accel):
        """
        The 'Elastic' Friction Audit.
        Calculates if the tire can generate enough force via slip angle 
        before it hits the friction limit.
        """
        # 1. Required Lateral Force for the turn
        # F_lat = (m * v^2) / r. We use (Fz/g) to get mass equivalent.
        f_lat_req = (normal_force_total / self.g) * (velocity**2 / radius) if radius != 0 else 0
        
        # 2. Required Slip Angle (Alpha)
        # The AI calculates how much it needs to 'aim' into the turn
        # alpha = F_y / Cornering_Stiffness
        required_alpha = f_lat_req / (self.tire.ca + 1e-6)
        
        # 3. Required Longitudinal Force (Acceleration/Braking)
        f_long_req = (normal_force_total / self.g) * req_accel
        
        # 4. Total Resultant Force (The Friction Circle)
        total_force_req = np.sqrt(f_lat_req**2 + f_long_req**2)
        
        # 5. The Limit Check
        max_static_grip = normal_force_total * self.mu_s
        utilization = total_force_req / (max_static_grip + 1e-6)
        
        # Physics Common Sense: Tires saturate. 
        # Even if Mu is high, if the slip angle > ~12 deg (0.21 rad), 
        # the tire is effectively 'plowing' rather than 'gripping'.
        is_legal = (utilization < 1.0) and (abs(required_alpha) < 0.21)

        return {
            "is_legal": is_legal,
            "slip_angle_deg": round(np.degrees(required_alpha), 2),
            "grip_utilization": round(utilization, 3),
            "is_sliding": utilization >= 1.0,
            "reasoning": self._generate_report(utilization, required_alpha)
        }

    def _generate_report(self, util, alpha):
        if util >= 1.0: 
            return "VETO: Force request exceeds friction circle. Kinetic slide (understeer) imminent."
        if abs(alpha) > 0.21: 
            return "VETO: Tire saturation. Steering angle too steep for current velocity."
        if util > 0.85: 
            return "WARNING: Approaching traction limit. 85% grip utilized."
        return "NOMINAL: Traction and slip within elastic limits."