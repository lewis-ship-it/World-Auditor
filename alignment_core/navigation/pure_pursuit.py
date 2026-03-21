import math

def find_lookahead_point(path, current_pos, lookahead=5.0):
    """
    Finds the first point in the path that is at least 'lookahead' distance away.
    """
    if not path:
        return None
        
    cx, cz = current_pos
    for px, pz in path:
        dx = px - cx
        dz = pz - cz
        dist = math.hypot(dx, dz)

        if dist >= lookahead:
            return (px, pz)

    # If no point is far enough, return the last point in the path
    return path[-1]

def pure_pursuit_control(current_pos, heading, path, wheelbase, lookahead=5.0):
    """
    Calculates the steering angle required to reach a lookahead point.
    """
    target = find_lookahead_point(path, current_pos, lookahead)
    if target is None:
        return 0.0, None

    tx, tz = target
    cx, cz = current_pos

    # 1. Calculate relative displacement
    dx = tx - cx
    dz = tz - cz

    # 2. Transform into vehicle-local coordinates (Forward = +X, Right = +Z)
    # Note: In many Webots car models, 'heading' is 0 at the X-axis. 
    # We rotate the world-frame vector (dx, dz) into the car's local frame.
    local_x = math.cos(heading) * dx + math.sin(heading) * dz
    local_z = -math.sin(heading) * dx + math.cos(heading) * dz

    # 3. Geometric Pure Pursuit Formula
    # L is the lookahead distance (hypotenuse to the target)
    L = math.hypot(local_x, local_z)
    
    # Avoid division by zero if target is exactly at current_pos
    if L < 0.1:
        return 0.0, target

    # The curvature 'k' of the circular arc to the target
    # k = 2 * local_z / L^2
    curvature = (2 * local_z) / (L ** 2)

    # 4. Convert curvature to steering angle
    # delta = atan(curvature * wheelbase)
    steer_angle = math.atan(curvature * wheelbase)

    # 5. Clamp the steering angle to prevent "too low requested position" errors
    # Most Webots cars (like the BmwX5) limit steering to ~0.5 radians
    steer_angle = max(-0.5, min(0.5, steer_angle))

    return steer_angle, target