import math

class Planner:
    def __init__(self):
        self.waypoints = []
        self.index = 0

    def compute(self, state):
        if not self.waypoints:
            return {"speed": 6.0, "steering": 0.0}

        pos = state["position"]
        yaw = state["yaw"]

        target = self.waypoints[self.index]

        dx = target[0] - pos[0]
        dz = target[1] - pos[1]

        dist = math.hypot(dx, dz)

        if dist < 2.0:
            self.index = min(self.index + 1, len(self.waypoints) - 1)

        angle = math.atan2(dz, dx)
        error = angle - yaw

        steer = max(-0.6, min(0.6, error))

        return {"speed": 10.0, "steering": steer}