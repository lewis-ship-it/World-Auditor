def compute_total_load(world_state):
    """
    Calculates total system load on the primary robot.
    """

    agent = world_state.primary_agent()

    base_mass = agent.mass
    payload = agent.load_weight

    total_mass = base_mass + payload

    return total_mass