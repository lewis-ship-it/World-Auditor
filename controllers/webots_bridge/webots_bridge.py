import os
import sys
import math
from controller import Robot, Keyboard

# =========================================
# PATH SETUP
# =========================================
try:
    # Changed 'file' to '__file__' to prevent NameError
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
    if project_root not in sys.path:
        sys.path.append(project_root)
except NameError:
    project_root = os.getcwd()

# =========================================
# ALIGNMENT CORE IMPORTS
# =========================================
from alignment_core.decision.action_auditor import ActionAuditor
from alignment_core.decision.predictive_kernel import PredictiveKernel
from alignment_core.constraints.stability import StabilityKernel
from alignment_core.constraints.friction import FrictionKernel
from alignment_core.constraints.braking import BrakingKernel
from alignment_core.constraints.load import LoadKernel
from alignment_core.physics.mechanics import RigidBody, TireModel
from alignment_core.world_model.terrain_manager import TerrainManager
from alignment_core.navigation.heading_estimator import HeadingEstimator
from alignment_core.navigation.pure_pursuit import pure_pursuit_control

# =========================================
# INIT
# =========================================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)

# =========================================
# DEVICE SETUP
# =========================================
steering_motors = []
drive_motors = []
gps = None
lidar = None
camera = None

for i in range(robot.getNumberOfDevices()):
    dev = robot.getDeviceByIndex(i)
    name = dev.getName().lower()
    
    if "steer" in name:
        steering_motors.append(dev)

    if "wheel" in name:
        drive_motors.append(dev)
        dev.setPosition(float('inf'))
        dev.setVelocity(0.0)

    if "gps" in name:
        gps = dev
        gps.enable(timestep)

    if "lidar" in name or "hokuyo" in name:
        lidar = dev
        lidar.enable(timestep)

    if "camera" in name:
        camera = dev
        camera.enable(timestep)

print("Drive motors:", len(drive_motors))
print("Steering motors:", len(steering_motors))

# =========================================
# VEHICLE MODEL (ALIGNMENT CORE)
# =========================================
vehicle_specs = {
    'mass': 2200.0,
    'track_width': 1.6,
    'wheelbase': 2.9,
    'cog_height': 0.55
}

body = RigidBody(
    mass=vehicle_specs['mass'],
    track_width=vehicle_specs['track_width'],
    wheelbase=vehicle_specs['wheelbase'],
    cog_z=vehicle_specs['cog_height']
)

tires = TireModel(cornering_stiffness=40000)
world = TerrainManager(default_surface="dry_asphalt")

auditor = ActionAuditor(
    robot=body,
    stability=StabilityKernel(body),
    friction=FrictionKernel(body, tires, world),
    braking=BrakingKernel(max_braking_force=3000),
    load=LoadKernel(body)
)

predictor = PredictiveKernel(auditor)

# =========================================
# NAVIGATION
# =========================================
heading_estimator = HeadingEstimator()
waypoints = []
wp_index = 0
autopilot = False
prev_steer = 0.0

print("Controls: A=toggle auto | P=save waypoint | Arrows=manual")

# =========================================
# LIDAR AVOIDANCE
# =========================================
def lidar_avoidance(lidar_dev, threshold=5.0):
    if not lidar_dev:
        return 0.0, False
    
    ranges = lidar_dev.getRangeImage()
    if not ranges:
        return 0.0, False

    n = len(ranges)
    left = ranges[:n//3]
    center = ranges[n//3:2*n//3]
    right = ranges[2*n//3:]

    min_left = min(left) if left else 10.0
    min_center = min(center) if center else 10.0
    min_right = min(right) if right else 10.0

    # WALL / BLOCKED
    if min_center < threshold and min_left < threshold and min_right < threshold:
        return 0.0, True

    # OBSTACLE
    if min_center < threshold:
        return (0.5 if min_left > min_right else -0.5), False

    return 0.0, False

# =========================================
# MAIN LOOP
# =========================================
while robot.step(timestep) != -1:
    key = keyboard.getKey()

    if key == ord('A'):
        autopilot = not autopilot
        print("AUTOPILOT:", autopilot)

    if key == ord('P') and gps:
        pos = gps.getValues()
        waypoints.append((pos[0], pos[2]))
        print("Waypoint added:", len(waypoints))

    v_target = 0.0
    steer_target = 0.0

    if gps:
        pos = gps.getValues()
        x, z = pos[0], pos[2]
        heading = heading_estimator.update(pos)

        # =========================================
        # AUTOPILOT
        # =========================================
        if autopilot and wp_index < len(waypoints):
            target = waypoints[wp_index]
            dist = math.hypot(target[0] - x, target[1] - z)

            if dist < 2.0:
                wp_index += 1

            if wp_index < len(waypoints):
                steer_pp, _ = pure_pursuit_control(
                    (x, z),
                    heading,
                    waypoints[wp_index:],
                    vehicle_specs['wheelbase'],
                    lookahead=3.0
                )
                steer_target = steer_pp * 1.8

            avoid_steer, stop = lidar_avoidance(lidar)

            if stop:
                v_target = 0.0
            else:
                steer_target += avoid_steer

                # --- SAFE SPEED FROM ALIGNMENT CORE ---
                if abs(steer_target) > 0.01:
                    r_target = vehicle_specs['wheelbase'] / math.tan(abs(steer_target))
                else:
                    r_target = 999.0

                safe = predictor.find_optimal_velocity(radius=r_target)
                v_target = min(10.0, safe["max_safe_velocity"])

                if v_target < 1.0:
                    v_target = 1.0

        # =========================================
        # MANUAL CONTROL
        # =========================================
        else:
            if key == Keyboard.UP:
                v_target = 8.0
            elif key == Keyboard.DOWN:
                v_target = -3.0
            elif key == Keyboard.LEFT:
                steer_target = 0.5
            elif key == Keyboard.RIGHT:
                steer_target = -0.5

    # =========================================
    # STEERING SMOOTHING
    # =========================================
    max_rate = 0.05
    steer_target = max(prev_steer - max_rate, min(prev_steer + max_rate, steer_target))
    steer_target = max(-0.6, min(0.6, steer_target))
    prev_steer = steer_target

    # =========================================
    # ACTUATION
    # =========================================
    for s in steering_motors:
        s.setPosition(steer_target)

    for d in drive_motors:
        d.setVelocity(v_target)

    # =========================================
    # DEBUG
    # =========================================
    if robot.getTime() % 1 < 0.05:
        print(f"[AUTO:{autopilot}] v={v_target:.2f} steer={steer_target:.2f}")