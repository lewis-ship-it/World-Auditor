import numpy as np
import math

class OccupancyGrid:
    def __init__(self, size=200, resolution=0.5):
        self.size = size
        self.resolution = resolution
        # Use a float grid for probability or simple binary occupancy
        self.grid = np.zeros((size, size))

    def world_to_grid(self, x, z, current_pos):
        """Converts world coordinates to grid indices relative to the car."""
        # Offset the coordinates by the car's current position
        dx = x - current_pos[0]
        dz = z - current_pos[2]
        
        gx = int(dx / self.resolution + self.size // 2)
        gz = int(dz / self.resolution + self.size // 2)
        return gx, gz

    def update(self, lidar, position, heading):
        if lidar is None:
            return

        ranges = lidar.getRangeImage()
        fov = lidar.getFov()
        n = len(ranges)
        
        # Slowly clear old data to simulate a moving local map
        self.grid *= 0.95 

        for i, r in enumerate(ranges):
            # Webots LiDAR returns 'inf' for no hit; we ignore those or hits too close
            if math.isinf(r) or r >= lidar.getMaxRange() or r <= 0.1:
                continue

            # Calculate angle of the specific laser beam
            # Note: Webots angles usually increase counter-clockwise
            angle = (0.5 - i / n) * fov 
            global_angle = heading + angle

            # Calculate hit position in world coordinates
            hit_x = position[0] + r * math.cos(global_angle)
            hit_z = position[2] + r * math.sin(global_angle)

            gx, gz = self.world_to_grid(hit_x, hit_z, position)

            if 0 <= gx < self.size and 0 <= gz < self.size:
                self.grid[gx, gz] = 1.0