class ActionState:

    def __init__(
        self,
        velocity=0,
        acceleration=0,
        steering_angle=0,
        braking_force=0
    ):
        self.velocity = velocity
        self.acceleration = acceleration
        self.steering_angle = steering_angle
        self.braking_force = braking_force