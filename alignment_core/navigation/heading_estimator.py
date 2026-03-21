import math

class HeadingEstimator:
    def __init__(self, smoothing_factor=0.1):
        """
        Initialize the estimator.
        :param smoothing_factor: 0.0 to 1.0. Lower is smoother/slower, higher is more responsive.
        """
        self.prev_pos = None
        self.heading = 0.0
        self.alpha = smoothing_factor 

    def update(self, pos):
        # Webots GPS values are typically (x, y, z) where z is the forward/backward axis
        x, z = pos[0], pos[2]

        if self.prev_pos is None:
            self.prev_pos = (x, z)
            return self.heading

        dx = x - self.prev_pos[0]
        dz = z - self.prev_pos[1]

        # Only update if the movement is significant to avoid jitter while stationary
        if math.hypot(dx, dz) > 0.005:
            new_heading = math.atan2(dz, dx)
            
            # Angle wrapping logic to ensure smoothing doesn't break at the pi/-pi boundary
            diff = new_heading - self.heading
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            
            # Apply low-pass filter: heading = (old * 0.9) + (new * 0.1)
            self.heading += self.alpha * diff
            
            # Keep final heading in [-pi, pi]
            if self.heading > math.pi: self.heading -= 2 * math.pi
            if self.heading < -math.pi: self.heading += 2 * math.pi

        self.prev_pos = (x, z)
        return self.heading