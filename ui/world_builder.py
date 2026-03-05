from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.environment import EnvironmentState
from alignment_core.world_model.world_state import WorldState


def build_world(
    velocity,
    mass,
    friction,
    slope,
    distance,
    load,
    com_height,
    wheelbase
):

    agent = AgentState(
        id="robot",
        type="mobile",
        mass=mass,
        velocity=velocity,
        braking_force=5.0,
        max_deceleration=5.0,
        load_weight=load,
        center_of_mass_height=com_height,
        wheelbase=wheelbase
    )

    env = EnvironmentState(
        friction=friction,
        slope=slope,
        distance_to_obstacles=distance
    )

    return WorldState(agent=agent, environment=env)