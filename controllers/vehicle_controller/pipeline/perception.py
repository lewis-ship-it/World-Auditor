import math

class Perception:
    def __init__(self):
        self.prev_pos = None
        self.yaw = 0.0

    def process(self, data):
        gps = data.get("gps", [0, 0, 0])
        # BMW Orientation: X is Forward, Y is Side
        current_x, current_y = gps[0], gps[1]

        if self.prev_pos:
            dx = current_x - self.prev_pos[0]
            dy = current_y - self.prev_pos[1]
            if abs(dx) > 1e-3 or abs(dy) > 1e-3:
                self.yaw = math.atan2(dy, dx)

        self.prev_pos = (current_x, current_y)

        raw_lidar = data.get("lidar_range", [])
        # Filter infinity and handle empty lists
        clean_lidar = [d if d != float('inf') else 100.0 for d in raw_lidar]
        if not clean_lidar:
            clean_lidar = [100.0] * 360

        return {
            "position": (current_x, current_y),
            "yaw": self.yaw,
            "speed": data.get("speed", 0.0),
            "lidar": clean_lidar,
            "obstacle_distance": min(clean_lidar)
        }