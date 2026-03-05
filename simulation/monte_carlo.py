import random
from simulation.physics_simulator import simulate_braking

def monte_carlo_collision_test(
    runs,
    speed,
    friction,
    brake_force,
    obstacle_distance
):

    collisions = 0

    for _ in range(runs):

        f = random.uniform(friction * 0.8, friction * 1.2)
        b = random.uniform(brake_force * 0.8, brake_force * 1.2)

        sim = simulate_braking(speed, f, b)

        final_position = sim[-1]["position"]

        if final_position > obstacle_distance:
            collisions += 1

    probability = collisions / runs

    return probability