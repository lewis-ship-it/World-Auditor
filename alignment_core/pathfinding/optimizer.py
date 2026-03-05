import numpy as np
import cv2

def process_map_image(image_bytes):
    """Converts an uploaded image into a physics-ready grid."""
    nparr = np.frombuffer(image_bytes, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
    # Binary threshold: 255 for path, 0 for walls
    _, binary_map = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)
    return binary_map

def generate_velocity_profile(path_coords, robot_physics, curve_physics):
    """
    Calculates the maximum possible speed for every point on a path 
    based on the robot's tipping point and friction limits.
    """
    profile = []
    for i in range(1, len(path_coords) - 1):
        # Calculate local radius using three points
        p1, p2, p3 = path_coords[i-1], path_coords[i], path_coords[i+1]
        radius = calculate_radius(p1, p2, p3) 
        
        # Determine Vmax for this specific curve
        v_limit = curve_physics.calculate_max_cornering_speed(
            radius, robot_physics.friction, robot_physics.banking
        )
        profile.append(v_limit)
    return profile