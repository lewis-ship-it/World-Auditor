class Behavior:
    def modify(self, state, action):
        lidar = state.get("lidar", [])

        if not lidar:
            return action

        n = len(lidar)
        front = lidar[n//3: 2*n//3]

        if front and min(front) < 3.0:
            return {"speed": 0.0, "steering": action["steering"]}

        return action