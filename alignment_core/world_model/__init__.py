class Agent:
    def __init__(self, velocity, max_deceleration):
        self.velocity = velocity
        self.max_deceleration = max_deceleration


class Environment:
    def __init__(self, distance_to_obstacle):
        self.distance_to_obstacle = distance_to_obstacle


class WorldState:
    def __init__(self, agent, environment):
        self.agent = agent
        self.environment = environment