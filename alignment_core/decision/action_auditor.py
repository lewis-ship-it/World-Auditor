class ActionAuditor:
    def __init__(self, robot, stability, friction, braking, load, logger=None):
        self.robot = robot
        self.stability = stability
        self.friction = friction
        self.braking = braking
        self.load = load
        self.logger = logger
        self.panic_active = False

    def audit_intent(self, state, intent, *args, **kwargs):
        print(">>> AUDITOR IS RUNNING WITH NEW CODE <<<")
        # Fix: If brain returns a float instead of a dict, wrap it
        if isinstance(intent, (int, float)):
            intent = {"speed": float(intent), "steering": 0.0}
            
        # Fix: Safely extract slope from positional or keyword arguments
        if args:
            slope = args[0]
        else:
            slope = kwargs.get('slope', 0.0)
        
        v_val = intent.get("speed", 0)
        r_val = intent.get("steering", 0)
        a_val = intent.get("acceleration", 0)
        dist_to_obj = state.get("obstacle_distance", 100)
        
        results = {}
        
        if self.stability:
            results['stability'] = self.stability.evaluate(
                velocity=v_val, radius=r_val, acceleration=a_val, slope=slope
            )
        
        if self.friction and hasattr(self.friction, 'evaluate'):
            results['friction'] = self.friction.evaluate(
                velocity=v_val, radius=r_val, req_accel=a_val
            )
        else:
            results['friction'] = {'is_legal': True, 'grip_utilization': 0}

        if self.braking:
            results['braking'] = self.braking.evaluate(
                velocity=v_val, distance=dist_to_obj, slope=slope
            )

        # Emergency Stop if grip utilization is too high
        if results['friction'].get('grip_utilization', 0) > 0.99:
            return self.emergency_stop(results, "Traction Loss")

        is_safe = all([
            results.get('stability', {}).get('is_legal', True),
            results.get('friction', {}).get('is_legal', True),
            results.get('braking', {}).get('is_legal', True)
        ])
        
        return {
            "authorized": is_safe,
            "approved_speed": v_val if is_safe else 0.0,
            "approved_steering": r_val,
            "kernels": results
        }

    def emergency_stop(self, kernels, reason):
        if self.logger:
            self.logger.log(f"PANIC: {reason}")
        return {
            "authorized": False,
            "approved_speed": 0.0,
            "approved_steering": 0.0,
            "kernels": kernels,
            "panic": True
        }