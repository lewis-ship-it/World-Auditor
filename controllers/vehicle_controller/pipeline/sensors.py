class SensorSuite:
    def __init__(self, robot, timestep):
        self.gps = None
        self.lidar = None
        self.camera = None

        for i in range(robot.getNumberOfDevices()):
            dev = robot.getDeviceByIndex(i)
            name = dev.getName().lower()

            print("[DEVICE]", name)

            if "gps" in name:
                self.gps = dev
                self.gps.enable(timestep)

            elif "lidar" in name or "hokuyo" in name:
                self.lidar = dev
                self.lidar.enable(timestep)

            elif "camera" in name:
                self.camera = dev
                self.camera.enable(timestep)

    def read(self):
        data = {}

        if self.gps:
            data["gps"] = self.gps.getValues()

        if self.lidar:
            data["lidar"] = self.lidar.getRangeImage()

        if self.camera:
            data["camera"] = self.camera.getImage()

        return data