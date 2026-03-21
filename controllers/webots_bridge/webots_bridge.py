import os
import sys
import math
import numpy as np
from controller import Robot, Keyboard, Node

# --- PATH SETUP ---
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
if project_root not in sys.path:
    sys.path.append(project_root)

# --- IMPORT CORE ---
from alignment_core.main_ai import PhysicsAI
from alignment_core.decision.action_auditor import ActionAuditor
from alignment_core.decision.predictive_kernel import PredictiveKernel
from alignment_core.constraints.stability import StabilityKernel
from alignment_core.constraints.friction import FrictionKernel
from alignment_core.constraints.braking import BrakingKernel
from alignment_core.constraints.load import LoadKernel
from alignment_core.physics.mechanics import RigidBody, TireModel
from alignment_core.world_model.terrain_manager import TerrainManager
from alignment_core.sensors.imu import IMUSensor
from alignment_core.sensors.encoder import WheelEncoder

from alignment_core.navigation.heading_estimator import HeadingEstimator
from alignment_core.navigation.pure_pursuit import pure_pursuit_control
from alignment_core.navigation.occupancy_grid import OccupancyGrid

# --- INIT ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(timestep)

# --- DEVICES ---
steering_motors = []
drive_motors = []
accel, gps, lidar = None, None, None

for i in range(robot.getNumberOfDevices()):
    dev = robot.getDeviceByIndex(i)
    name = dev.getName().lower()
    d_type = dev.getNodeType()

    if d_type in [Node.ROTATIONAL_MOTOR, Node.LINEAR_MOTOR]:
        if "steer" in name:
            steering_motors.append(dev)
        elif "wheel" in name:
            drive_motors.append(dev)
            dev.setPosition(float('inf'))
            dev.setVelocity(0.0)
    elif "accelerometer" in name:
        accel = dev
        accel.enable(timestep)
    elif "gps" in name:
        gps = dev
        gps.enable(timestep)
    elif "lidar" in name or "hokuyo" in name:
        lidar = dev
        lidar.enable(timestep)

# --- VEHICLE SETUP ---
vehicle_specs = {
    'mass': 2200.0, 'track_width': 1.6, 'wheelbase': 2.9,
    'cog_height': 0.6, 'tire_stiffness': 40000
}

body = RigidBody(
    mass=vehicle_specs['mass'],
    track_width=vehicle_specs['track_width'],
    wheelbase=vehicle_specs['wheelbase'],
    cog_z=vehicle_specs['cog_height']
)

tires = TireModel(cornering_stiffness=vehicle_specs['tire_stiffness'])
world = TerrainManager(default_surface="dry_asphalt")
auditor = ActionAuditor(
    body, 
    StabilityKernel(body), 
    FrictionKernel(body, tires, world), 
    BrakingKernel(3000), 
    LoadKernel(body)
)
predictor = PredictiveKernel(auditor)

# --- NAVIGATION SYSTEMS ---
heading_estimator = HeadingEstimator()
grid = OccupancyGrid(size=200, resolution=0.5)
waypoints = []
wp_index = 0
autopilot = False
prev_steer = 0.0

print("COMMANDS: A=Autopilot | P=Save Waypoint | V=Save Map Image")

# --- UTILITIES ---
def lidar_avoidance(lidar, threshold=5.0):
    if lidar is None: return 0.0, False
    ranges = lidar.getRangeImage()
    if not ranges: return 0.0, False
    
    n = len(ranges)
    left = ranges[:n//3]
    center = ranges[n//3:2*n//3]
    right = ranges[2*n//3:]
    
    min_c = min(center)
    # Full stop if blocked everywhere
    if min_c < threshold and min(left) < threshold and min(right) < threshold:
        return 0.0, True 
    
    # Steering avoidance
    if min_c < threshold:
        return (0.4 if min(left) > min(right) else -0.4), False
    return 0.0, False

# --- MAIN LOOP ---
while robot.step(timestep) != -1:
    key = keyboard.getKey()
    
    # --- INPUT HANDLING ---
    if key == ord('A'):
        autopilot = not autopilot
        print(f"AUTOPILOT: {'ENABLED' if autopilot else 'DISABLED'}")
    
    if key == ord('P') and gps:
        pos = gps.getValues()
        waypoints.append((pos[0], pos[2]))
        print(f"Waypoint Saved: {len(waypoints)} at ({pos[0]:.2f}, {pos[2]:.2f})")
    
   
    v_target, steer_target, r_target = 0.0, 0.0, 999.0

    if gps:
        pos = gps.getValues()
        current_yaw = heading_estimator.update(pos)
        grid.update(lidar, pos, current_yaw)

        # --- AUTOPILOT LOGIC ---
        if autopilot and wp_index < len(waypoints):
            current_pos = (pos[0], pos[2])
            target_wp = waypoints[wp_index]
            
            # Distance to current target waypoint
            dist_to_wp = math.hypot(target_wp[0] - pos[0], target_wp[1] - pos[2])
            
            # Waypoint Progression
            if dist_to_wp < 2.5:
                wp_index += 1
                print(f"Reached Waypoint! Moving to index: {wp_index}")

            if wp_index < len(waypoints):
                # Calculate Pure Pursuit Steering
                steer_pp, _ = pure_pursuit_control(
                    current_pos=current_pos,
                    heading=current_yaw,
                    path=waypoints[wp_index:], 
                    wheelbase=vehicle_specs['wheelbase'],
                    lookahead=5.0
                )
                steer_target = steer_pp

            # Obstacle Avoidance Override
            avoid_steer, should_stop = lidar_avoidance(lidar)
            if should_stop:
                v_target = 0.0
            else:
                steer_target += avoid_steer
                
                # Calculate target radius for physics-based velocity
                if abs(steer_target) > 0.01:
                    r_target = vehicle_specs['wheelbase'] / math.tan(abs(steer_target))
                else:
                    r_target = 999.0

                # Determine safe velocity based on physics constraints
                safe_v_data = predictor.find_optimal_velocity(radius=r_target)
                v_target = min(10.0, safe_v_data["max_safe_velocity"])
                
                # Maintain minimum velocity for heading estimation if not blocked
                if v_target < 0.5:
                    v_target = 1.0

        # --- MANUAL CONTROL ---
        elif not autopilot:
            if key == Keyboard.UP:
                v_target = 10.0
            elif key == Keyboard.DOWN:
                v_target = -3.0
            
            if key == Keyboard.LEFT:
                steer_target = 0.5
            elif key == Keyboard.RIGHT:
                steer_target = -0.5
            
            if abs(steer_target) > 0.01:
                r_target = vehicle_specs['wheelbase'] / math.tan(abs(steer_target))

    # --- SAFETY AUDIT ---
    audit = auditor.audit_intent(
        v_target=v_target, 
        r_target=r_target, 
        a_target=1.0, 
        slope=0.0
    )
    if not audit["authorized"]:
        v_target = 0.0

    # --- STEERING SMOOTHING & CLAMPING ---
    # Limits rate of change to avoid motor errors and jerky motion
    max_rate = 0.04
    steer_target = max(prev_steer - max_rate, min(prev_steer + max_rate, steer_target))
    
    # Final clamp to BmwX5 hardware limits (~31.5 degrees)
    steer_target = max(-0.55, min(0.55, steer_target))
    prev_steer = steer_target

    # --- ACTUATION ---
    for s in steering_motors:
        s.setPosition(steer_target)
    for d in drive_motors:
        d.setVelocity(v_target)

    # --- DEBUG CONSOLE ---
    if robot.getTime() % 1.0 < (timestep / 1000.0):
        print(f"[Status] Auto: {autopilot} | Speed: {v_target:.2f} | Steer: {steer_target:.2f} | WP: {wp_index}/{len(waypoints)}")