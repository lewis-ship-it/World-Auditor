# alignment_core/world_model/__init__.py

class Agent:
    def __init__(self, velocity, max_deceleration, mass, load_weight, max_load, center_of_mass_height, wheelbase):
        self.velocity = velocity
        self.max_deceleration = max_deceleration
        self.mass = mass
        self.load_weight = load_weight
        self.max_load = max_load
        self.center_of_mass_height = center_of_mass_height
        self.wheelbase = wheelbase

class Environment:
    def __init__(self, distance_to_obstacle, friction, slope):
        self.distance_to_obstacle = distance_to_obstacle
        self.friction = friction
        self.slope = slope

class WorldState:
    def __init__(self, agent, environment):
        self.agent = agent
        self.environment = environment