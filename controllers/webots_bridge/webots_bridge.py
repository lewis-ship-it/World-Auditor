import os
import sys
import math
from controllers import Robot, Keyboard

# 1. PATH CONFIGURATION
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
if project_root not in sys.path:
    sys.path.append(project_root)

# 2. FULL COMPONENT IMPORTS
try:
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
    print("SUCCESS: Full alignment_core stack imported.")
except ImportError as e:
    print(f"CRITICAL: Missing core files. {e}")
    sys.exit(1)

# 3. WEBOTS HARDWARE & KEYBOARD INITIALIZATION (CAR/STEERING VERSION)
robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)

print("--- SCANNING BMW X5 FOR DEVICES ---")
steering_motors = []
drive_motors = []
accel = None

for i in range(robot.getNumberOfDevices()):
    dev = robot.getDeviceByIndex(i)
    name = dev.getName()
    
    # Identify Steering (usually contains 'steer')
    if "steer" in name.lower():
        steering_motors.append(dev)
        print(f"Found Steer: {name}")
    
    # Identify Drive Wheels (usually 'wheel' + 'front' or 'back')
    elif "wheel" in name.lower():
        drive_motors.append(dev)
        dev.setPosition(float('inf'))
        dev.setVelocity(0.0)
        print(f"Found Wheel: {name}")
    
    # Identify Accelerometer
    elif "accelerometer" in name.lower():
        accel = dev
        accel.enable(timestep)
        print(f"Found Accel: {name}")

# 4. VEHICLE PHYSICAL PROFILE (Updated for a Steered Vehicle)
vehicle_specs = {
    'mass': 1500.0,      # Heavier for a car
    'track_width': 1.6,
    'wheelbase': 2.8,    # Distance between front and rear axles
    'cog_height': 0.5,
    'tire_stiffness': 50000
}

body = RigidBody(mass=vehicle_specs['mass'], track_width=vehicle_specs['track_width'], 
                 wheelbase=vehicle_specs['wheelbase'], cog_z=vehicle_specs['cog_height'])
tires = TireModel(cornering_stiffness=vehicle_specs['tire_stiffness'])
world = TerrainManager(default_surface="dry_asphalt")

# 5. KERNEL & BRAIN ASSEMBLY
stability_monitor = StabilityKernel(body)
friction_monitor = FrictionKernel(body, tires, world)
braking_monitor = BrakingKernel(max_braking_force=2000)
load_monitor = LoadKernel(body)

auditor = ActionAuditor(body, stability_monitor, friction_monitor, braking_monitor, load_monitor)
predictor = PredictiveKernel(auditor)
ai = PhysicsAI(auditor=auditor, predictor=predictor, imu=IMUSensor(body), encoders=WheelEncoder(body))

print("--- WORLD AUDITOR: CAR SYSTEM INITIALIZED ---")
print("USE ARROW KEYS TO STEER AND DRIVE.")

# 6. MAIN SIMULATION LOOP
while robot.step(timestep) != -1:
    # A. Get Sensor Data
    raw_tilt = accel.getValues()[1] if accel else 0.0
    
    # B. Keyboard Input to Intent
    v_target = 0.0
    steering_angle = 0.0 
    
    key = keyboard.getKey()
    if key == Keyboard.UP:    v_target = 10.0   # Speed in m/s
    if key == Keyboard.DOWN:  v_target = -3.0
    if key == Keyboard.LEFT:  steering_angle = 0.4  # Radians
    if key == Keyboard.RIGHT: steering_angle = -0.4

    # C. Calculate Radius for Ackermann Steering
    # Radius = Wheelbase / tan(SteeringAngle). 
    if abs(steering_angle) > 0.01:
        r_target = vehicle_specs['wheelbase'] / math.tan(abs(steering_angle))
    else:
        r_target = 999.0 # Driving straight

    # D. THE AUDIT
    audit_results = ai.auditor.audit_intent(
        v_target=v_target,
        r_target=r_target,
        a_target=0.8,
        slope=raw_tilt
    )
    
    # E. ACTUATION LOGIC (Steering/Ackermann)
    if audit_results["authorized"]:
        # Set the steering angle
        if steering_motors:
            steering_motors.setPosition(steering_angle)
        # Apply velocity to drive motors
        for m in drive_motors:
            m.setVelocity(v_target)
    else:
        # AI Override: Safety Stop
        for m in drive_motors:
            m.setVelocity(0.0)
        if robot.getTime() % 1.0 < 0.05:
            print(f"!!! AI VETO: {audit_results['summary']}")

    # F. Telemetry
    if robot.getTime() % 2.0 < 0.05:
        status = "SAFE" if audit_results["authorized"] else "DANGER"
        print(f"[{status}] Speed: {v_target}m/s | Steer Angle: {steering_angle:.2f} | Tilt: {raw_tilt:.2f}")