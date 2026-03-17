# FILE: alignment_core/decision/action_auditor.py

class ActionAuditor:
    def __init__(self, robot, stability, friction, braking, load, logger=None):
        self.robot = robot
        self.stability = stability
        self.friction = friction
        self.braking = braking
        self.load = load
        self.logger = logger
        self.panic_active = False

    def audit_intent(self, v_target, r_target, a_target, 
                     slope=0, dist_to_obj=100, payload=None):
        results = {}
        
        # 1. Update Physical State (Load)
        if payload:
            results['load'] = self.load.update_payload(
                payload['mass'], payload['x'], payload['y'], payload['z']
            )

        # 2. Check Equilibrium (Stability)
        results['stability'] = self.stability.evaluate(
            velocity=v_target, radius=r_target, 
            acceleration=a_target, slope_angle_deg=slope
        )

        # 3. Check Traction (Friction)
        results['friction'] = self.friction.evaluate(
            velocity=v_target, radius=r_target, 
            req_accel=a_target, slope_deg=slope
        )

        # 4. Check Safety Margin (Braking)
        mu_s, _ = self.friction.terrain.get_friction()
        results['braking'] = self.braking.evaluate(
            velocity=v_target, mass=self.robot.m, 
            friction_mu=mu_s, slope_angle_deg=slope, 
            distance_to_target=dist_to_obj
        )

        # --- PANIC MODE TRIGGER ---
        # Trigger emergency stop if utilization hits 99% of the safe limit
        if results['friction'].get('grip_utilization', 0) > 0.99:
            return self.emergency_stop(results, "Traction Loss Imminent")

        # 5. Final Decision Logic
        is_safe = all([
            results['stability']['is_legal'], 
            results['friction']['is_legal'], 
            results['braking']['is_legal']
        ])

        audit_output = {
            "authorized": is_safe,
            "kernels": results,
            "summary": self._summarize(results)
        }

        if self.logger:
            self.logger.record({"v": v_target, "r": r_target, "a": a_target}, audit_output)

        return audit_output

    def emergency_stop(self, current_res, trigger_reason):
        self.panic_active = True
        return {
            "authorized": False,
            "kernels": current_res,
            "summary": f"PANIC: {trigger_reason}. Overriding to Emergency Brake."
        }

    def _summarize(self, res):
        if not res['stability']['is_legal']: return f"VETO (Stability): {res['stability']['reasoning']}"
        if not res['friction']['is_legal']: return f"VETO (Friction): {res['friction']['reasoning']}"
        if not res['braking']['is_legal']: return f"VETO (Braking): {res['braking']['reasoning']}"
        return "Authorized"