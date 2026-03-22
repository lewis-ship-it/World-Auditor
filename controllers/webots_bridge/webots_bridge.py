import os
import sys
import math
from controller import Robot, Keyboard, Node

# ==========================================
# 1. RIGID PATH SETUP
# ==========================================
# This forces the script to look at the 'World_auditor' root folder
current_dir = os.path.dirname(os.path.abspath(__file__))
# We go up two levels: controllers -> webots_bridge -> World_auditor
project_root = os.path.abspath(os.path.join(current_dir, '..', '..'))

if project_root not in sys.path:
    sys.path.insert(0, project_root)
    print(f"Project Root Added: {project_root}")

# ==========================================
# 2. CORE IMPORTS
# ==========================================
try:
    from alignment_core.navigation.heading_estimator import HeadingEstimator
    from alignment_core.navigation.pure_pursuit import pure_pursuit_control
    print("Successfully imported alignment_core modules.")
except ImportError as e:
    print(f"CRITICAL ERROR: Could not find alignment_core. {e}")
    print(f"Current Sys Path: {sys.path}")
    sys.exit(1)

# ==========================================
# 3. DEVICE INITIALIZATION (BMW X5 SPECIFIC)
# ==========================================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)

steering_motors = []
drive_motors = []
gps = None
lidar = None

print("--- Starting BmwX5 Hardware Scan ---")
for i in range(robot.getNumberOfDevices()):
    dev = robot.getDeviceByIndex(i)
    name = dev.getName().lower()
    d_type = dev.getNodeType()

    # Match Steering
    if "steer" in name:
        steering_motors.append(dev)
        print(f"  [OK] Found Steering: {name}")

    # Match Drive Motors (Checks for 'wheel', 'motor', 'rear', or 'front')
    # BmwX5 Simple often uses 'left_rear_wheel_motor' or just 'left_rear_wheel'
    is_motor_node = d_type in [Node.ROTATIONAL_MOTOR, Node.LINEAR_MOTOR]
    is_drive_keyword = any(k in name for k in ["wheel", "motor", "rear", "front"])
    
    if is_motor_node and is_drive_keyword and "steer" not in name:
        drive_motors.append(dev)
        dev.setPosition(float('inf')) # Enable Velocity Control
        dev.setVelocity(0.0)
        print(f"  [OK] Found Drive: {name}")

    if "gps" in name:
        gps = dev
        gps.enable(timestep)
    if "lidar" in name:
        lidar = dev
        lidar.enable(timestep)

print(f"--- Scan Complete: {len(drive_motors)} Drive | {len(steering_motors)} Steer ---")

# ==========================================
# 4. NAVIGATION STATE
# ==========================================
heading_estimator = HeadingEstimator()
waypoints = []
wp_index = 0
autopilot = False
prev_steer = 0.0

# ==========================================
# 5. MAIN CONTROL LOOP
# ==========================================
while robot.step(timestep) != -1:
    key = keyboard.getKey()

    # Inputs
    if key == ord('A'):
        autopilot = not autopilot
        print(f"AUTOPILOT: {autopilot}")
    if key == ord('P') and gps:
        pos = gps.getValues()
        waypoints.append((pos[0], pos[2]))
        print(f"Waypoint Saved: {len(waypoints)}")

    v_target = 0.0
    steer_target = 0.0

    # Manual Override (Arrows)
    if key == Keyboard.UP:
        v_target = 10.0
    elif key == Keyboard.DOWN:
        v_target = -5.0
    
    if key == Keyboard.LEFT:
        steer_target = 0.4
    elif key == Keyboard.RIGHT:
        steer_target = -0.4

    # Autopilot Logic
    if autopilot and v_target == 0 and gps:
        pos = gps.getValues()
        yaw = heading_estimator.update(pos)
        
        if wp_index < len(waypoints):
            target = waypoints[wp_index]
            dist = math.hypot(target[0]-pos[0], target[1]-pos[2])
            
            if dist < 3.0:
                wp_index += 1
            
            if wp_index < len(waypoints):
                steer_pp, _ = pure_pursuit_control(
                    (pos[0], pos[2]), yaw, waypoints[wp_index:], 2.9, 5.0
                )
                steer_target = steer_pp
                v_target = 8.0

    # Final Smoothing and Actuation
    max_rate = 0.05
    steer_target = max(prev_steer - max_rate, min(prev_steer + max_rate, steer_target))
    steer_target = max(-0.55, min(0.55, steer_target))
    prev_steer = steer_target

    for s in steering_motors:
        s.setPosition(steer_target)
    for d in drive_motors:
        d.setVelocity(v_target)