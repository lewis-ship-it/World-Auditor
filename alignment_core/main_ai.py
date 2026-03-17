# FILE: alignment_core/main_ai.py
from .decision.observer import FrictionObserver

class PhysicsAI:
    def __init__(self, auditor, predictor, imu, encoders):
        self.auditor = auditor
        self.predictor = predictor
        self.imu = imu
        self.encoders = encoders
        self.observer = FrictionObserver(auditor.friction.terrain)

    def think(self, target_v, target_r, real_v, wheel_v, real_slope):
        """
        The core loop of the AI's consciousness.
        """
        # 1. Perceive Slope
        current_slope = self.imu.estimate_slope(real_slope)
        
        # 2. Perceive Traction
        slip = self.encoders.get_slip_ratio(real_v, wheel_v)
        status = self.observer.update_perception(slip) # This updates the terrain margin
        
       # 3. Audit the Intent
        audit = self.auditor.audit_intent(target_v, target_r, 0, slope=current_slope)
        
        # 4. Predict the Limit
        prediction = self.predictor.find_optimal_velocity(target_r, slope=current_slope)
        
        return {
            "authorized": audit["authorized"],
            "perception": status,
            "max_safe_speed": prediction["max_safe_velocity"], # Matches the dict above
            "limit_reason": prediction["reason"],
            "status_summary": audit["summary"]
        }