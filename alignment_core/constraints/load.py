# FILE: alignment_core/constraints/load.py

import numpy as np
from .base_constraint import BaseConstraint

class LoadKernel(BaseConstraint):
    def __init__(self, robot):
        """
        robot: The RigidBody instance from mechanics.py
        """
        self.robot = robot
        # Store original "empty" state to allow for resets
        self.base_mass = robot.m
        self.base_cog_z = robot.cog_z
        self.base_cog_x = robot.cog_x
        self.base_cog_y = robot.cog_y

    def update_payload(self, payload_mass, p_x, p_y, p_z):
        """
        Recalculates the robot's physical identity based on a new load.
        p_x, p_y, p_z: Position of the payload relative to the robot center.
        """
        total_mass = self.base_mass + payload_mass
        
        # New CoG = (sum of mass * position) / total_mass
        new_x = ((self.base_mass * self.base_cog_x) + (payload_mass * p_x)) / total_mass
        new_y = ((self.base_mass * self.base_cog_y) + (payload_mass * p_y)) / total_mass
        new_z = ((self.base_mass * self.base_cog_z) + (payload_mass * p_z)) / total_mass

        # Update the RigidBody "Self-Awareness"
        self.robot.m = total_mass
        self.robot.cog_x = new_x
        self.robot.cog_y = new_y
        self.robot.cog_z = new_z

        return {
            "new_mass": round(total_mass, 2),
            "cog_shift_z": round(new_z - self.base_cog_z, 3),
            "is_lopsided": abs(new_y) > (self.robot.tw * 0.1) # Flag if CoG moved >10% of track width
        }

    def evaluate_slosh_risk(self, acceleration, fill_level=0.5):
        """
        For liquid payloads: calculates the 'Slosh' effect.
        If the liquid shifts, it creates a transient overturning moment.
        """
        # Simplified Pendulum Model for liquid shifting
        # Delta_y = L * sin(theta) where tan(theta) = accel / g
        theta = np.arctan(acceleration / self.robot.g)
        shift_magnitude = self.robot.cog_z * np.tan(theta) * fill_level
        
        return {
            "dynamic_shift_m": round(shift_magnitude, 3),
            "risk_factor": "HIGH" if abs(shift_magnitude) > 0.05 else "LOW"
        }