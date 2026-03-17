# FILE: alignment_core/constraints/friction.py
import numpy as np
from .base_constraint import BaseConstraint
# Ensure the import path matches your directory structure
from ..physics.mechanics import calculate_dynamic_normal_forces

class FrictionKernel(BaseConstraint):
    def __init__(self, robot, tire_model, terrain_manager):
        """
        robot: The RigidBody instance containing mass, wheelbase, and CoG height.
        tire_model: From mechanics.py
        terrain_manager: From world_model/terrain_manager.py
        """
        self.robot = robot  # Added to access physical constants
        self.tire = tire_model
        self.terrain = terrain_manager
        self.g = 9.81

    def evaluate(self, velocity, radius, req_accel, slope_deg=0):
        """
        Evaluates friction limits while accounting for terrain type, 
        slope angle, and dynamic weight transfer.
        """
        # 1. Fetch Surface Primitives from the World Model
        mu_s_base, mu_k_base = self.terrain.get_friction()
        
        # 2. Apply Slope Correction
        slope_rad = np.radians(slope_deg)
        mu_s = mu_s_base * np.cos(slope_rad)
        
        # 3. CALL NEW FUNCTION: Calculate Dynamic Normal Forces
        # This calculates how weight shifts between axles during acceleration/braking
        f_front, f_rear = calculate_dynamic_normal_forces(
            mass=self.robot.m,
            acceleration=req_accel,
            com_height=self.robot.cog_z,
            wheelbase=self.robot.wb
        )
        
        # We focus on the axle with the LEAST load, as it will slide first
        normal_force_min = min(f_front, f_rear)
        normal_force_total = f_front + f_rear
        
        # 4. Required Lateral Force (Centripetal)
        # F = m * v^2 / r
        f_lat_req = (normal_force_total / self.g) * (velocity**2 / radius) if radius != 0 else 0
        
        # 5. Required Slip Angle (Alpha)
        required_alpha = f_lat_req / (self.tire.ca + 1e-6)
        
        # 6. Required Longitudinal Force (Braking/Accel)
        f_long_req = (normal_force_total / self.g) * req_accel
        
        # 7. Total Resultant Force vs Friction Circle
        # We divide by 2 to check the force per axle (assuming 2 axles)
        total_force_per_axle = np.sqrt((f_lat_req/2)**2 + (f_long_req/2)**2)
        
        # Use the minimum normal force to find the true grip ceiling
        max_static_grip_per_axle = normal_force_min * mu_s
        
        utilization = total_force_per_axle / (max_static_grip_per_axle + 1e-6)
        
        # Check for sliding or tire saturation (approx 12 degrees)
        is_legal = (utilization < 1.0) and (abs(required_alpha) < 0.21)

        return {
            "is_legal": is_legal,
            "current_mu": round(mu_s, 2),
            "grip_utilization": round(utilization, 3),
            "slip_angle_deg": round(np.degrees(required_alpha), 2),
            "front_load_n": round(f_front, 1),
            "rear_load_n": round(f_rear, 1),
            "reasoning": self._generate_report(utilization, required_alpha, slope_deg)
        }

    def _generate_report(self, util, alpha, slope):
        if util >= 1.0: 
            return f"VETO: Friction limit exceeded on {slope}° slope. Rear axle unloading."
        if abs(alpha) > 0.21: 
            return "VETO: Tire saturation. Steering angle too steep."
        return "NOMINAL: Traction within safety envelope."