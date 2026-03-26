from controller import Node

class WebotsAdapter:
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep

        self.steer = []
        self.drive = []
        self.gps = None
        self.lidar = None

        print("\n--- SCANNING DEVICES ---")

        for i in range(robot.getNumberOfDevices()):
            dev = robot.getDeviceByIndex(i)
            name = dev.getName()
            dtype = dev.getNodeType()

            print(name)

            # Detect motors
            if dtype in [Node.ROTATIONAL_MOTOR, Node.LINEAR_MOTOR]:

                if "steer" in name.lower():
                    self.steer.append(dev)
                    print("STEER:", name)

                elif "wheel" in name.lower():
                    self.drive.append(dev)
                    dev.setPosition(float('inf'))
                    dev.setVelocity(0.0)
                    print("DRIVE:", name)

            # Sensors
            if "gps" in name.lower():
                self.gps = dev
                self.gps.enable(timestep)

            elif "lidar" in name.lower() or "hokuyo" in name.lower():
                self.lidar = dev
                self.lidar.enable(timestep)

        print(f"\n[RESULT] steer={len(self.steer)} drive={len(self.drive)}")

    def read(self):
        return {
            "gps": self.gps.getValues() if self.gps else [0, 0, 0],
            "lidar": self.lidar.getRangeImage() if self.lidar else []
        }

    def apply(self, action):
        steer = action.get("steering", 0.0)
        speed = action.get("speed", 0.0)

        for s in self.steer:
            s.setPosition(steer)

        for d in self.drive:
            d.setVelocity(speed)