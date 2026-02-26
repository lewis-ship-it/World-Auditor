from copy import deepcopy


def simulate_forward(world_state, horizon=1.0, dt=0.1):

    simulated = deepcopy(world_state)

    steps = int(horizon / dt)

    for _ in range(steps):

        for agent in simulated.agents:
            agent.position.x += agent.velocity.x * dt
            agent.position.y += agent.velocity.y * dt
            agent.position.z += agent.velocity.z * dt

    return simulated