# FILE: alignment_core/decision/action_auditor.py

class ActionAuditor:
    def __init__(self, robot, stability, friction, braking, load):
        self.robot = robot
        self.stability = stability
        self.friction = friction
        self.braking = braking
        self.load = load

    def audit_intent(self, v_target, r_target, a_target, 
                     slope=0, dist_to_obj=100, payload=None):
        """
        The Master Physics Check.
        payload: Optional dict {'mass': 10, 'x': 0, 'y': 0, 'z': 0.5}
        """
        results = {}
        
        # 1. Update Physical State (Load Kernel)
        if payload:
            load_res = self.load.apply_payload(
                payload['mass'], payload['x'], payload['y'], payload['z']
            )
            results['load'] = load_res

        # 2. Check Equilibrium (Stability Kernel)
        stab_res = self.stability.evaluate(
            velocity=v_target, 
            radius=r_target, 
            acceleration=a_target, 
            slope_angle_deg=slope
        )
        results['stability'] = stab_res

        # 3. Check Traction (Friction Kernel)
        # We derive normal force from Stability's results or mass
        f_normal = self.robot.m * self.robot.g
        fric_res = self.friction.evaluate(
            velocity=v_target, 
            radius=r_target, 
            normal_force_total=f_normal, 
            req_accel=a_target
        )
        results['friction'] = fric_res

        # 4. Check Safety Margin (Braking Kernel)
        brake_res = self.braking.evaluate(
            velocity=v_target, 
            mass=self.robot.m, 
            friction_mu=self.friction.mu_s, 
            slope_angle_deg=slope, 
            distance_to_target=dist_to_obj
        )
        results['braking'] = brake_res

        # 5. Final Decision Logic
        is_safe = all([
            stab_res['is_legal'], 
            fric_res['is_legal'], 
            brake_res['is_legal']
        ])

        return {
            "authorized": is_safe,
            "kernels": results,
            "summary": self._summarize(results)
        }

    def _summarize(self, res):
        if not res['stability']['is_legal']: return res['stability']['reasoning']
        if not res['friction']['is_legal']: return res['friction']['reasoning']
        if not res['braking']['is_legal']: return res['braking']['reasoning']
        return "Action authorized: Within physical safety envelope."