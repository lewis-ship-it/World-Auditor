class Controller:
    def compute(self, action):
        return {
            "speed": action["speed"],
            "steering": action["steering"]
        }