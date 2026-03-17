# FILE: alignment_core/sensors/imu.py
import numpy as np

class IMUSensor:
    def __init__(self, robot):
        self.robot = robot

    def estimate_slope(self, real_world_slope=0):
        """
        In a real robot, this reads the Accelerometer Y-axis.
        Gravity (9.81) projects onto the axis based on the tilt.
        """
        # We simulate a small amount of sensor noise (0.1 degrees)
        noise = np.random.normal(0, 0.1)
        estimated_slope = real_world_slope + noise
        
        return round(estimated_slope, 2)