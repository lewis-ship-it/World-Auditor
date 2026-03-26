import math

class SafetySystem:
    def __init__(self, predictor):
        self.predictor = predictor

    def enforce(self, action):
        v = action["speed"]
        steer = action["steering"]

        if abs(steer) > 0.01:
            radius = 2.9 / math.tan(abs(steer))
        else:
            radius = 999.0

        safe_v = self.predictor.get_safe_speed(radius)

        v = min(v, safe_v)

        return {
            "speed": v,
            "steering": steer
        }