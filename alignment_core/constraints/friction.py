def compute_effective_friction(world_state):
    """
    Computes the effective friction coefficient for the primary agent.
    """

    agent = world_state.primary_agent()
    env = world_state.environment

    base_friction = agent.friction
    modifier = env.friction_modifier

    effective_friction = base_friction * modifier

    if env.surface == "wet":
        effective_friction *= 0.7

    if env.surface == "ice":
        effective_friction *= 0.3

    if env.surface == "sand":
        effective_friction *= 0.8

    return max(0.05, effective_friction)