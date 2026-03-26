import math
from core.perception import Perception
from core.mapping import OccupancyGrid
from core.planning import Planner
from core.behavior import Behavior


class Brain:
    def __init__(self, predictor):
        self.perception = Perception()
        self.mapping = OccupancyGrid()
        self.planner = Planner()
        self.behavior = Behavior()
        self.predictor = predictor

    def step(self, sensor_data):
        state = self.perception.update(sensor_data)

        # Mapping
        self.mapping.update(
            state.get("lidar"),
            state.get("position"),
            state.get("yaw")
        )

        # Planning
        action = self.planner.compute(state)

        # Obstacle behavior
        action = self.behavior.modify(state, action)

        # Safety
        action = self.apply_safety(action)

        return action

    def apply_safety(self, action):
        v = action["speed"]
        steer = action["steering"]

        if abs(steer) > 0.01:
            radius = 2.9 / math.tan(abs(steer))
        else:
            radius = 999.0

        try:
            safe_v = self.predictor.get_safe_speed(radius)
        except Exception as e:
            print("[Safety Error]:", e)
            safe_v = 6.0

        return {
            "speed": min(v, safe_v),
            "steering": steer
        }