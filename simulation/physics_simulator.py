import math

def simulate_braking(speed, friction, brake_force, dt=0.1):

    g = 9.81

    velocity = speed
    position = 0

    timeline = []

    while velocity > 0:

        decel = friction * g + brake_force * 0.1

        velocity = max(0, velocity - decel * dt)

        position += velocity * dt

        timeline.append({
            "velocity": velocity,
            "position": position
        })

    return timeline