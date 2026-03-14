import numpy as np
from .base_constraint import BaseConstraint

class RigidBody:
    def __init__(self, mass, track_width, wheelbase, cog_z, 
                 cog_bias_x=0.0, cog_bias_y=0.0, 
                 k_suspension=25000, c_damping=1500, frontal_area=0.5):
        self.m = mass          
        self.tw = track_width  
        self.wb = wheelbase    
        self.cog_z = cog_z     
        
        # ASYMMETRIC COG: Offset from center (0.0 is perfect center)
        # y_bias: positive is right, negative is left
        # x_bias: positive is forward, negative is rear
        self.cog_y = cog_bias_y 
        self.cog_x = cog_bias_x 
        
        # SUSPENSION & AERO
        self.k = k_suspension  # N/m
        self.c = c_damping     # Ns/m
        self.area = frontal_area
        self.g = 9.81
        self.rho = 1.225       # Air density

class StabilityKernel(BaseConstraint):
    def __init__(self, robot: RigidBody):
        self.robot = robot

    def evaluate(self, velocity, radius, acceleration, slope_angle_deg=0, surface_bump_velocity=0):
        """
        The Final Audit: Validates equilibrium across all axes.
        """
        # 1. AERO DOWNFORCE
        # Cl_area is the downforce coefficient; as speed increases, stability increases
        f_downforce = 0.5 * self.robot.rho * (velocity**2) * self.robot.area * 0.3 
        effective_weight = (self.robot.m * self.robot.g) + f_downforce

        # 2. ASYMMETRIC LATERAL STABILITY (ROLL)
        slope_rad = np.radians(slope_angle_deg)
        lat_accel_ms2 = (velocity**2) / radius if radius != 0 else 0
        
        # Adjust track width for bias: if bias is -0.1 (left), 
        # the distance to the right wheel is tw/2 + 0.1
        dist_to_right = (self.robot.tw / 2) - self.robot.cog_y
        dist_to_left = (self.robot.tw / 2) + self.robot.cog_y
        
        # Choose the critical side based on turn direction
        critical_width = dist_to_right if radius > 0 else dist_to_left
        
        restoring_moment = effective_weight * critical_width * np.cos(slope_rad)
        overturning_moment = self.robot.m * lat_accel_ms2 * self.robot.cog_z
        
        # 3. DAMPING CHECK (Dynamic Disturbance)
        # If the robot just hit a bump, the suspension energy (c * v)
        # creates a transient force that reduces stability
        damping_force = self.robot.c * surface_bump_velocity
        dynamic_stability_loss = damping_force * self.robot.cog_z
        
        final_margin = (restoring_moment - overturning_moment - dynamic_stability_loss) / restoring_moment

        # 4. LONGITUDINAL AUDIT (PITCH)
        # Adjust wheelbase for bias
        dist_to_front = (self.robot.wb / 2) - self.robot.cog_x
        dynamic_shift = (self.robot.m * acceleration * self.robot.cog_z) / self.robot.wb
        f_front = (effective_weight * (dist_to_front / self.robot.wb)) - dynamic_shift
        
        is_legal = (final_margin > 0.1) and (f_front > (effective_weight * 0.05))

        return {
            "is_legal": is_legal,
            "stability_margin": round(final_margin, 3),
            "front_load_pct": round((f_front / effective_weight) * 100, 1),
            "downforce_n": round(f_downforce, 2),
            "reasoning": self._generate_report(final_margin, f_front, effective_weight)
        }

    def _generate_report(self, margin, f_front, total_w):
        if margin < 0: return "VETO: Lateral Overturn imminent (Moment Balance Failure)."
        if f_front < 0: return "VETO: Longitudinal Flip (Wheelie/Pitch-over)."
        if f_front < (total_w * 0.05): return "WARNING: Steering Authority Lost (Insufficient Front Load)."
        return "NOMINAL: Motion aligned with physical equilibrium."