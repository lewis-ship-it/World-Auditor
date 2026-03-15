from ..world_model.primitives import Vector3
# FILE: alignment_core/physics/mechanics.py
class RigidBody:
    """
    The 'Body' - Stores the immutable physical constants of the robot.
    """
    def __init__(self, mass, track_width, wheelbase, cog_z, 
                 cog_bias_x=0.0, cog_bias_y=0.0, 
                 k_suspension=25000, c_damping=1500, 
                 frontal_area=0.5, max_braking_force=500):
        self.m = mass          # kg
        self.tw = track_width  # m
        self.wb = wheelbase    # m
        self.cog_z = cog_z     # m (Height of CoG)
        
        # ASYMMETRIC COG: Offset from center (0.0 is center)
        self.cog_y = cog_bias_y # + is right, - is left
        self.cog_x = cog_bias_x # + is forward, - is rear
        
        # SUSPENSION, AERO, & BRAKES
        self.k = k_suspension  # N/m
        self.c = c_damping     # Ns/m
        self.area = frontal_area 
        self.max_f_brake = max_braking_force
        
        self.g = 9.81
        self.rho = 1.225       # Air density

class TireModel:
    def __init__(self, cornering_stiffness=15000, relaxation_length=0.12):
        """
        cornering_stiffness (N/rad): How much lateral force the tire generates per radian of slip.
        relaxation_length (m): The distance the tire must roll to develop full lateral force.
        """
        self.ca = cornering_stiffness
        self.sigma = relaxation_length

    def calculate_lateral_force(self, slip_angle_rad, normal_force):
        """
        Computes lateral force based on slip angle and load.
        Note: Real tires saturate; we cap this at the friction limit in the kernel.
        """
        # F_y = Ca * alpha
        # We scale Ca by the normal force ratio to account for load sensitivity
        nominal_load = 500  # Newtons
        load_scaling = normal_force / nominal_load
        return self.ca * slip_angle_rad * load_scaling
def get_dynamic_forces(mass, accel, com_h, wheelbase):
    """Calculates weight shift during braking."""
    g = 9.81
    static_load = (mass * g) / 2
    transfer = (mass * abs(accel) * com_h) / wheelbase
    
    front_load = static_load + transfer
    rear_load = static_load - transfer
    
    return max(0, front_load), max(0, rear_load)
def calculate_dynamic_normal_forces(mass, acceleration, com_height, wheelbase):
    """
    Calculates how weight shifts from rear to front during braking/acceleration.
    """
    g = 9.81
    static_weight = (mass * g) / 2
    # Weight transfer formula: (mass * acceleration * com_height) / wheelbase
    transfer = (mass * abs(acceleration) * com_height) / wheelbase
    
    front_n = static_weight + transfer
    rear_n = static_weight - transfer
    
    return max(0, front_n), max(0, rear_n)

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