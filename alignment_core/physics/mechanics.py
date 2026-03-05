from ..world_model.primitives import Vector3

def calculate_auto_cog(chassis_mass, chassis_h, battery_mass, battery_h, load_mass, load_h):
    """Calculates composite CoG height by summing mass moments."""
    total_mass = chassis_mass + battery_mass + load_mass
    if total_mass == 0: return 0.0, 0.0
    
    avg_height = ((chassis_mass * chassis_h) + 
                  (battery_mass * battery_h) + 
                  (load_mass * load_h)) / total_mass
    return avg_height, total_mass

def get_support_polygon(wheelbase, track_width, num_wheels=4):
    """Defines the stability boundary for 3 or 4 wheel configurations."""
    if num_wheels == 3:
        return [Vector3(wheelbase, 0, 0), Vector3(0, -track_width/2, 0), Vector3(0, track_width/2, 0)]
    return [
        Vector3(0, -track_width/2, 0), Vector3(wheelbase, -track_width/2, 0),
        Vector3(wheelbase, track_width/2, 0), Vector3(0, track_width/2, 0)
    ]