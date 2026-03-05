import math

def calculate_max_cornering_speed(radius, friction, banking_deg, gravity=9.81):
    """Calculates Vmax for a banked curve before sliding occurs."""
    theta = math.radians(banking_deg)
    num = math.tan(theta) + friction
    den = 1 - (friction * math.tan(theta))
    if den <= 0: return 100.0  # Theoretically infinite speed for high banking
    return math.sqrt(radius * gravity * (num / den))

def check_lateral_stability(velocity, radius, cog_h, track_w, banking_deg):
    """Checks for inner-wheel lift (tipping) during cornering."""
    theta = math.radians(banking_deg)
    g = 9.81
    a_centripetal = (velocity**2) / radius
    a_lat_eff = a_centripetal * math.cos(theta) - g * math.sin(theta)
    
    tipping_limit = (track_w / 2) / cog_h
    is_tipping = (a_lat_eff / g) > tipping_limit
    return is_tipping, a_lat_eff