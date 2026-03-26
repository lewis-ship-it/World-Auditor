import math

class Perception:
    def __init__(self):
        self.prev_pos = None
        self.yaw = 0.0

    def update(self, sensors):
        gps = sensors.get("gps", [0, 0, 0])
        x, z = gps[0], gps[2]

        if self.prev_pos:
            dx = x - self.prev_pos[0]
            dz = z - self.prev_pos[1]
            if abs(dx) > 1e-4 or abs(dz) > 1e-4:
                self.yaw = math.atan2(dz, dx)

        self.prev_pos = (x, z)

        return {
            "position": (x, z),
            "yaw": self.yaw,
            "lidar": sensors.get("lidar", [])
        }