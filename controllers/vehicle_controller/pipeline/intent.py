import math

class IntentGenerator:
    def __init__(self):
        self.prev_error = 0.0
        self.kp = 0.5  # Proportional gain
        self.kd = 0.2  # Derivative gain (Damping) [cite: 58]

    def compute(self, state):
        # If no waypoints, stay still
        if not state["current_target"]:
            return {"target_speed": 0.0, "steering": 0.0}

        target = state["current_target"]
        dx = target[0] - state["x"]
        dz = target[1] - state["z"]
        
        # Calculate angle to target [cite: 66]
        desired_heading = math.atan2(dz, dx)
        heading_error = desired_heading - state["yaw"]
        
        # Normalize to [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # PD Control for smooth steering
        derivative = (heading_error - self.prev_error)
        steer_output = (self.kp * heading_error) + (self.kd * derivative)
        self.prev_error = heading_error

        return {
            "target_speed": 8.0, # Target cruise speed
            "steering": max(-0.6, min(0.6, steer_output))
        }