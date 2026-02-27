from math import sqrt
from alignment_core.world_model.primitives import Vector3

def check_stability(agent, gravity_z: float) -> bool:
    """
    Checks if lateral acceleration exceeds tipping threshold:
    Threshold = (Support_Width / (2 * CG_Height)) * Gravity.
    """
    if not agent.support_polygon or agent.center_of_mass.z == 0:
        return True # Cannot calculate, assume stable

    # 1. Calculate Lateral Acceleration (v * omega)
    speed = sqrt(agent.velocity.x**2 + agent.velocity.y**2)
    a_lat = speed * abs(agent.angular_velocity.z)

    # 2. Determine Support Width
    xs = [p.x for p in agent.support_polygon]
    width = max(xs) - min(xs)

    # 3. Tipping Threshold
    threshold = (width / (2 * agent.center_of_mass.z)) * abs(gravity_z)

    return a_lat < threshold