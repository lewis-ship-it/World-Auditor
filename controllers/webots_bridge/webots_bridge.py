import os
import sys
import math
import numpy as np
from controller import Robot, Keyboard, Node

# --- PATH SETUP ---

# Fixed: changed **file** to __file__
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
timestep = int(robot.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(timestep)

# --- DEVICES ---

steering_motors = []
drive_motors = []
accel = None

# Fixed: Indentation of discovery logic
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

body = RigidBody(
    mass=vehicle_specs['mass'],
    track_width=vehicle_specs['track_width'],
    wheelbase=vehicle_specs['wheelbase'],
    cog_z=vehicle_specs['cog_height']
)

tires = TireModel(cornering_stiffness=vehicle_specs['tire_stiffness'])
world = TerrainManager(default_surface="dry_asphalt")

# --- AI ---

stability = StabilityKernel(body)
friction = FrictionKernel(body, tires, world)
braking = BrakingKernel(max_braking_force=3000)
load = LoadKernel(body)

auditor = ActionAuditor(body, stability, friction, braking, load)
predictor = PredictiveKernel(auditor)

ai = PhysicsAI(
    auditor=auditor,
    predictor=predictor,
    imu=IMUSensor(body),
    encoders=WheelEncoder(body)
)

# --- WAYPOINT SYSTEM ---

waypoints = []
wp_index = 0

# --- CONTROL STATE ---

autopilot = False
last_toggle = False
last_save = False

print("A = toggle autopilot | P = save waypoint")

# --- LOOP ---

while robot.step(timestep) != -1:
    key = keyboard.getKey()

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

    # Initialize variables to avoid NameErrors in the Safety Layer
    v_target = 0.0
    steer_target = 0.0
    r_target = 999.0

    # =========================
    # AUTOPILOT
    # =========================
    if autopilot and len(waypoints) > 0:
        pos = gps.getValues() if gps else [0, 0, 0]
        x, z = pos[0], pos[2]

        target = waypoints[wp_index]

        dx = target[0] - x
        dy = target[1] - z

        distance = math.hypot(dx, dy)

        if distance < 2.0 and wp_index < len(waypoints) - 1:
            wp_index += 1
            print("→ Next waypoint:", wp_index)

        angle = math.atan2(dy, dx)
        steer_target = max(-0.6, min(0.6, angle * 0.5))

        if abs(steer_target) > 0.01:
            r_target = vehicle_specs['wheelbase'] / math.tan(abs(steer_target))
        else:
            r_target = 999.0

        safe_v = predictor.find_optimal_velocity(radius=r_target)["max_safe_velocity"]
        v_target = min(10.0, safe_v)

        # --- OBSTACLE DETECTION ---
        if distance_sensor:
            dist = distance_sensor.getValue()
            if dist < 800:  # threshold depends on sensor
                print("⚠️ OBSTACLE DETECTED")
                v_target = 0.0

    # =========================
    # MANUAL
    # =========================
    else:
        if key == Keyboard.UP:
            v_target = 10.0
        elif key == Keyboard.DOWN:
            v_target = -3.0

        if key == Keyboard.LEFT:
            steer_target = 0.4
        elif key == Keyboard.RIGHT:
            steer_target = -0.4

        if abs(steer_target) > 0.01:
            r_target = vehicle_specs['wheelbase'] / math.tan(abs(steer_target))
        else:
            r_target = 999.0

    # =========================
    # SAFETY
    # =========================
    audit = auditor.audit_intent(
        v_target=v_target,
        r_target=r_target,
        a_target=1.0,
        slope=raw_tilt
    )

    if not audit["authorized"]:
        v_target = 0.0

    # =========================
    # ACTUATION
    # =========================
    for s in steering_motors:
        s.setPosition(steer_target)

    for d in drive_motors:
        d.setVelocity(v_target)

    # =========================
    # DEBUG
    # =========================
    if robot.getTime() % 1.0 < 0.05:
        print(f"[AUTO:{autopilot}] v={v_target:.2f} steer={steer_target:.2f} wp={wp_index}/{len(waypoints)}")