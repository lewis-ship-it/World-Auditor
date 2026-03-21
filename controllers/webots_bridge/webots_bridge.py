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

# --- WEBOTS INIT ---
robot = Robot()
prev_steer_error = 0.0
Kp = 0.40  # Turning strength
Kd = 0.20  # Damping (Fixes the jerky/shaking motion)
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)

# --- DEVICES ---
steering_motors = []
drive_motors = []
accel = None

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
    elif "inertial unit" in name: 
        imu = dev; imu.enable(timestep)
    elif "camera" in name: 
        camera = dev; camera.enable(timestep)
    elif "lidar" in name: 
        lidar = dev; lidar.enable(timestep)
    elif "gps" in name: 
        gps = dev; gps.enable(timestep)
    elif "distance sensor" in name:
        distance_sensor = dev; distance_sensor.enable(timestep)

# --- GPS ---
gps = robot.getDevice("gps")
if gps:
    gps.enable(timestep)

# --- DISTANCE SENSOR (OBSTACLE DETECTION) ---
distance_sensor = robot.getDevice("distance sensor")
if distance_sensor:
    distance_sensor.enable(timestep)

# --- VEHICLE ---
vehicle_specs = {
    'mass': 2200.0,
    'track_width': 1.6,
    'wheelbase': 2.9,
    'cog_height': 0.6,
    'tire_stiffness': 40000
}

body = RigidBody(mass=vehicle_specs['mass'], track_width=vehicle_specs['track_width'], 
                 wheelbase=vehicle_specs['wheelbase'], cog_z=vehicle_specs['cog_height'])
tires = TireModel(cornering_stiffness=vehicle_specs['tire_stiffness'])
world = TerrainManager(default_surface="dry_asphalt")

# --- AI ---
stability = StabilityKernel(body)
friction = FrictionKernel(body, tires, world)
braking = BrakingKernel(max_braking_force=3000)
load = LoadKernel(body)
auditor = ActionAuditor(body, stability, friction, braking, load)
predictor = PredictiveKernel(auditor)
ai = PhysicsAI(auditor=auditor, predictor=predictor, imu=IMUSensor(body), encoders=WheelEncoder(body))

# --- CONTROL STATE ---
waypoints = []
wp_index = 0
autopilot = False
last_toggle = False
last_save = False

# PD Constants for Steering Stability
prev_steer_error = 0.0
Kp = 0.45 
Kd = 0.15 

print("A = toggle autopilot | P = save waypoint | Arrows = Manual Drive")

# --- LOOP ---
while robot.step(timestep) != -1:
    key = keyboard.getKey()
    # --- ADD THIS TO START OF LOOP ---
reasoning = "Idle" # Prevents the NameError

# --- UPDATE AUTOPILOT CALCULATION ---
if autopilot and len(waypoints) > 0:
    # ... (Your GPS/Distance logic here) ...
    
   # --- REPLACE YOUR STEERING CALCULATION WITH THIS ---
    # 1. Calculate the raw error
    angle_error = math.atan2(dy, dx)
    
    # 2. Calculate the 'Derivative' (The change in error)
    # This acts as a brake to stop the wheels from snapping too fast
    error_diff = (angle_error - prev_steer_error) / (timestep / 1000.0)
    
    # 3. Apply the PD Formula
    # Kp = How hard it turns | Kd = How much it damps the shaking
    steer_target = (Kp * angle_error) + (Kd * error_diff)
    
    # 4. Limit the physical steering rack
    steer_target = max(-0.5, min(0.5, steer_target))
    
    # 5. Save error for the next frame
    prev_steer_error = angle_error
    
    reasoning = f"Tracking Waypoint {wp_index}"

    # --- TOGGLE AUTOPILOT ---
    if key == ord('A'):
        if not last_toggle:
            autopilot = not autopilot
            print("AUTOPILOT:", autopilot)
        last_toggle = True
    else:
        last_toggle = False

    # --- SAVE WAYPOINT ---
    if key == ord('P'):
        if not last_save:
            pos = gps.getValues() if gps else [0, 0, 0]
            wp = (pos[0], pos[2])
            waypoints.append(wp)
            print("Saved waypoint:", wp)
        last_save = True
    else:
        last_save = False

    raw_tilt = accel.getValues()[1] if accel else 0.0
    v_target, steer_target, r_target = 0.0, 0.0, 999.0

    # =========================
    # 1. AUTOPILOT CALCULATION
    # =========================
    if autopilot and len(waypoints) > 0:
        pos = gps.getValues() if gps else [0, 0, 0]
        x, z = pos[0], pos[2]
        target = waypoints[wp_index]
        dx, dy = target[0] - x, target[1] - z
        distance = math.hypot(dx, dy)

        if distance < 2.5 and wp_index < len(waypoints) - 1:
            wp_index += 1
            print("→ Next waypoint:", wp_index)

        # PD Control for Smooth Steering
        angle_error = math.atan2(dy, dx)
        error_diff = (angle_error - prev_steer_error) / (timestep / 1000.0)
        steer_target = (Kp * angle_error) + (Kd * error_diff)
        steer_target = max(-0.5, min(0.5, steer_target))
        prev_steer_error = angle_error

        r_target = vehicle_specs['wheelbase'] / math.tan(abs(steer_target)) if abs(steer_target) > 0.02 else 999.0
        v_target = min(8.0, predictor.find_optimal_velocity(radius=r_target)["max_safe_velocity"])

        if distance_sensor and distance_sensor.getValue() < 800:
            print("⚠️ OBSTACLE DETECTED")
            v_target = 0.0

    # =========================
    # 2. MANUAL OVERRIDE (Always Active)
    # =========================
    if key == Keyboard.UP:    v_target = 10.0
    elif key == Keyboard.DOWN:  v_target = -3.0
    
    if key == Keyboard.LEFT:  
        steer_target = 0.4
        autopilot = False # Disable auto if manual steer is detected
    elif key == Keyboard.RIGHT: 
        steer_target = -0.4
        autopilot = False

    # Re-calculate radius if manual input is used
    if not autopilot:
        r_target = vehicle_specs['wheelbase'] / math.tan(abs(steer_target)) if abs(steer_target) > 0.01 else 999.0

    # =========================
    # 3. SAFETY & ACTUATION
    # =========================
    audit = auditor.audit_intent(v_target=v_target, r_target=r_target, a_target=1.0, slope=raw_tilt)
    if not audit["authorized"]: 
        v_target = 0.0

    for s in steering_motors: 
        s.setPosition(steer_target)
    for d in drive_motors: 
        d.setVelocity(v_target)

    if robot.getTime() % 1.5 < 0.05:
        mode = "AUTO" if autopilot else "MAN"
        print(f"--- AI LOG [{robot.getTime():.1f}s] ---")
        print(f"  Decision: {reasoning}")
        print(f"  Target V: {v_target:.1f} m/s | Steer: {steer_target:.2f}")
        if not audit["authorized"]:
            print(f"  Stability Risk: {audit.get('stability_score', 'N/A')}")
        print("-" * 25)