import math

class Planner:
    def __init__(self):
        self.waypoints = []
        self.index = 0

    def compute(self, state):
        pos = state["position"]
        yaw = state["yaw"]

        if not self.waypoints:
            return {"speed": 5.0, "steering": 0.0}

        target = self.waypoints[self.index]

        dx = target[0] - pos[0]
        dz = target[1] - pos[1]

        distance = math.hypot(dx, dz)

        if distance < 2.0:
            self.index = min(self.index + 1, len(self.waypoints) - 1)

        angle_to_target = math.atan2(dz, dx)
        angle_error = angle_to_target - yaw

        steering = max(-0.6, min(0.6, angle_error))

        return {
            "speed": 8.0,
            "steering": steering
        }