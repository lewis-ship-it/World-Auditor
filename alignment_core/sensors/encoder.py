# FILE: alignment_core/sensors/encoder.py

class WheelEncoder:
    def __init__(self, robot):
        self.robot = robot

    def get_slip_ratio(self, actual_velocity, wheel_velocity):
        """
        Calculates how much the wheels are slipping.
        0.0 = Perfect traction
        1.0 = Total burnout (wheels spinning, robot still)
        """
        if actual_velocity == 0:
            return 1.0 if wheel_velocity > 0 else 0.0
            
        slip = (wheel_velocity - actual_velocity) / actual_velocity
        return round(max(0, slip), 3)