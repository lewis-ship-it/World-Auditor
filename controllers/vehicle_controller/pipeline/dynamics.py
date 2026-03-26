import math

class VehicleDynamics:
    def __init__(self):
        self.mass = 1800
        self.g = 9.81
        self.wheelbase = 3.2
        self.mu = 0.9
        self.max_total_acc = self.mu * self.g

        # PID parameters
        self.kp = 2.0
        self.ki = 0.1
        self.kd = 0.2
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.raw_acc = 0.0

    def step(self, state, safe_action, dt):
        # FIXED: Prevent division by zero and initialize local acceleration
        dt = max(dt, 0.001)
        self.raw_acc = 0.0
        
        current_speed = state.get("speed", 0.0)
        target_speed = safe_action.get("speed", 0.0)
        steering = safe_action.get("steering", 0.0)

        # 1. Lateral Dynamics
        if abs(steering) < 1e-4:
            radius = float('inf')
            lateral_acc = 0.0
        else:
            radius = self.wheelbase / math.tan(steering)
            lateral_acc = current_speed**2 / abs(radius)

        # 2. Friction Circle Limits
        remaining_acc_sq = max(self.max_total_acc**2 - lateral_acc**2, 0.0)
        max_longitudinal_acc = math.sqrt(remaining_acc_sq)

        # 3. PID Speed Control
        error = target_speed - current_speed
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        self.raw_acc = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error

        # 4. Apply Physical Limits
        accel = max(-max_longitudinal_acc, min(max_longitudinal_acc, self.raw_acc))

        # 5. Integration
        new_speed = max(0.0, current_speed + accel * dt)

        return {
            "speed": new_speed,
            "steering": steering,
            "acceleration": accel,
            "debug": {"lat_acc": lateral_acc, "limit": max_longitudinal_acc}
        }