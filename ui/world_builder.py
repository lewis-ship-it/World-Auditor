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

        mass=mass,

        velocity=velocity,

        wheelbase=wheelbase,

        center_of_mass_height=com_height,

        load_weight=load
    )

    env = EnvironmentState(

        surface_friction=friction,

        slope=slope,

        distance_to_obstacles=distance

    )

    return WorldState(

        agent=agent,

        environment=env
    )