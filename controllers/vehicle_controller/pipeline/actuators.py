class ActuatorSuite:
    def __init__(self, robot):
        self.steer = []
        self.drive = []
        self.wheel_radius = 0.375 

        for i in range(robot.getNumberOfDevices()):
            dev = robot.getDeviceByIndex(i)
            name = dev.getName().lower()

            # Filter for Rotational Motors Only (Type 53)
            if dev.getNodeType() != 53:
                continue

            if "steer" in name:
                self.steer.append(dev)

            if "wheel" in name and "rear" in name:
                self.drive.append(dev)
                dev.setPosition(float('inf'))
                dev.setVelocity(0.0)
                if hasattr(dev, 'getControlP'):
                    dev.setControlP(10)  # Adjusts PID power
                dev.setAvailableTorque(250.0)

    def apply(self, control):
        steer_angle = control.get("steering", 0.0)
        for s in self.steer:
            s.setPosition(steer_angle)

        speed = control.get("speed", 0.0)
        # Convert m/s to rad/s
        angular_velocity = speed / self.wheel_radius

        for d in self.drive:
            d.setVelocity(angular_velocity)