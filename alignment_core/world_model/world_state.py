class WorldState:

    def __init__(self, agent, environment, action=None):
        self.agent = agent
        self.environment = environment
        self.action = action