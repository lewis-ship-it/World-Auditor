import numpy as np
import math

class OccupancyGrid:
    def __init__(self, size=200, resolution=0.5):
        self.size = size
        self.res = resolution
        self.grid = np.zeros((size, size))

    def update(self, lidar, pos, yaw):
        if lidar is None:
            return

        # Current grid center based on world position
        cx = int(self.size/2 + pos[0]/self.res)
        cy = int(self.size/2 + pos[1]/self.res)

        for i, d in enumerate(lidar):
            # FIXED: Skip infinite values which cause OverflowError
            if d == float('inf') or d > 100.0:
                continue

            # Calculate point angle relative to vehicle heading
            angle = yaw + (i - len(lidar)/2) * 0.004

            # Convert polar (distance/angle) to Cartesian (x/y) grid coordinates
            x = cx + int(d * math.cos(angle) / self.res)
            y = cy + int(d * math.sin(angle) / self.res)

            # FIXED: Added boundary check to stay within the 200x200 grid
            if 0 <= x < self.size and 0 <= y < self.size:
                self.grid[x][y] = 1