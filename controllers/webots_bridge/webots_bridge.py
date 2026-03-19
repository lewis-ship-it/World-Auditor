import os
import sys
import math
from controller import Robot, Keyboard, Node

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

# 3. WEBOTS HARDWARE & KEYBOARD INITIALIZATION
robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)

print("--- SCANNING BMW X5 FOR MOTORS ---")
steering_motors = []
drive_motors = []
accel = None

# DYNAMIC DISCOVERY: We only grab MOTORS to prevent sensor-related crashes
for i in range(robot.getNumberOfDevices()):
    dev = robot.getDeviceByIndex(i)
    name = dev.getName()
    d_type = dev.getNodeType()
    
    # Check if the device is actually a motor (Rotational = 41)
    is_motor = d_type in [Node.ROTATIONAL_MOTOR, Node.LINEAR_MOTOR]

    if "steer" in name.lower() and is_motor:
        steering_motors.append(dev)
        print(f"Found Steer Motor: {name}")
    
    elif "wheel" in name.lower() and is_motor:
        drive_motors.append(dev)
        dev.setPosition(float('inf')) # Set to velocity control mode
        dev.setVelocity(0.0)
        print(f"Found Drive Motor: {name}")
    
    elif "accelerometer" in name.lower():
        accel = dev
        accel.enable(timestep)
        print(f"Found Accel: {name}")

# 4. VEHICLE PHYSICAL PROFILE (Updated for BMW X5/Ackermann)
vehicle_specs = {
    'mass': 2200.0,      # Corrected for a BMW X5
    'track_width': 1.6,
    'wheelbase': 2.9,    
    'cog_height': 0.6,
    'tire_stiffness': 40000
}

body = RigidBody(mass=vehicle_specs['mass'], track_width=vehicle_specs['track_width'], 
                 wheelbase=vehicle_specs['wheelbase'], cog_z=vehicle_specs['cog_height'])
tires = TireModel(cornering_stiffness=vehicle_specs['tire_stiffness'])
world = TerrainManager(default_surface="dry_asphalt")

# 5. KERNEL & BRAIN ASSEMBLY
stability_monitor = StabilityKernel(body)
friction_monitor = FrictionKernel(body, tires, world)
braking_monitor = BrakingKernel(max_braking_force=3000)
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
    steer_target = 0.0 
    
    key = keyboard.getKey()
    if key == Keyboard.UP:    v_target = 10.0   # Target velocity in m/s
    if key == Keyboard.DOWN:  v_target = -3.0
    if key == Keyboard.LEFT:  steer_target = 0.4  # Turn left
    if key == Keyboard.RIGHT: steer_target = -0.4 # Turn right

    # C. Calculate Radius for Ackermann Steering
    # If driving straight, radius is effectively infinite (999.0)
    if abs(steer_target) > 0.01:
        r_target = vehicle_specs['wheelbase'] / math.tan(abs(steer_target))
    else:
        r_target = 999.0

    # D. THE AUDIT
    # Runs the intent through Stability, Friction, and Braking kernels
    audit_results = ai.auditor.audit_intent(
        v_target=v_target,
        r_target=r_target,
        a_target=1.0, # Target acceleration for calculations
        slope=raw_tilt
    )
    
    # E. ACTUATION
    if audit_results["authorized"]:
        # Execute movement if safe
        for s_motor in steering_motors:
            s_motor.setPosition(steer_target)
        for d_motor in drive_motors:
            d_motor.setVelocity(v_target)
    else:
        # AI OVERRIDE: Emergency Stop if unsafe
        for d_motor in drive_motors:
            d_motor.setVelocity(0.0)
        
        # Periodic Veto Reporting
        if robot.getTime() % 1.0 < 0.05:
            print(f"!!! AI VETO: {audit_results['summary']}")

    # F. TELEMETRY (Corrected Indentation)
    if robot.getTime() % 1.5 < 0.05:
        status = "SAFE" if audit_results["authorized"] else "VETOED"
        print(f"[{status}] V:{v_target}m/s | Steer:{steer_target:.2f} | Tilt:{raw_tilt:.2f}")