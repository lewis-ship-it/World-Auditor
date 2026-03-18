import os
import sys
from controller import Robot

# 1. PATH CONFIGURATION (Ensuring the AI core is visible to Python)
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
if project_root not in sys.path:
    sys.path.append(project_root)

# 2. FULL COMPONENT IMPORTS
try:
    # Core Brain & Decision Making
    from alignment_core.main_ai import PhysicsAI
    from alignment_core.decision.action_auditor import ActionAuditor
    from alignment_core.decision.predictive_kernel import PredictiveKernel
    
    # Safety Constraint Kernels
    from alignment_core.constraints.stability import StabilityKernel
    from alignment_core.constraints.friction import FrictionKernel
    from alignment_core.constraints.braking import BrakingKernel
    from alignment_core.constraints.load import LoadKernel
    
    # Physics & Environment Models
    from alignment_core.physics.mechanics import RigidBody, TireModel
    from alignment_core.world_model.terrain_manager import TerrainManager
    
    # Virtual Hardware Interfaces
    from alignment_core.sensors.imu import IMUSensor
    from alignment_core.sensors.encoder import WheelEncoder
    
    print("SUCCESS: Full alignment_core stack imported.")
except ImportError as e:
    print(f"CRITICAL: Missing core files. {e}")
    sys.exit(1)

# 3. WEBOTS HARDWARE INITIALIZATION
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Pioneer 3-AT 4-Wheel Drive Configuration
motor_names = ['front left wheel', 'front right wheel', 'back left wheel', 'back right wheel']
motors = []
for name in motor_names:
    m = robot.getDevice(name)
    m.setPosition(float('inf')) 
    m.setVelocity(0.0)
    motors.append(m)

# Hardware Sensor Link
accel = robot.getDevice('accelerometer')
if accel:
    accel.enable(timestep)

# 4. ROBOT PHYSICAL PROFILE (Defining the "Body" for the AI)
# This section adds significant detail for the physics calculations
pioneer_specs = {
    'mass': 30.0,           # kg
    'track_width': 0.45,    # meters
    'wheelbase': 0.4,       # meters
    'cog_height': 0.25,     # Center of Gravity height
    'tire_stiffness': 15000 # N/rad
}

body = RigidBody(
    mass=pioneer_specs['mass'], 
    track_width=pioneer_specs['track_width'], 
    wheelbase=pioneer_specs['wheelbase'], 
    cog_z=pioneer_specs['cog_height']
)

tires = TireModel(cornering_stiffness=pioneer_specs['tire_stiffness'])
world = TerrainManager(default_surface="dry_asphalt")

# 5. KERNEL & BRAIN ASSEMBLY
# Initializing each specialized safety auditor
stability_monitor = StabilityKernel(body)
friction_monitor = FrictionKernel(body, tires, world)
braking_monitor = BrakingKernel(max_braking_force=500)
load_monitor = LoadKernel(body)

# Linking Sensors
imu_handler = IMUSensor(body)
encoder_handler = WheelEncoder(body)

# Connecting the Action Auditor to the Safety Kernels
auditor = ActionAuditor(
    body, 
    stability_monitor, 
    friction_monitor, 
    braking_monitor, 
    load_monitor
)

# Predictive layer for looking ahead at potential physics violations
predictor = PredictiveKernel(auditor)

# Final AI Assembly
ai = PhysicsAI(
    auditor=auditor, 
    predictor=predictor, 
    imu=imu_handler, 
    encoders=encoder_handler
)

print("--- WORLD AUDITOR: SYSTEM FULLY INITIALIZED ---")

# 6. MAIN SIMULATION LOOP
while robot.step(timestep) != -1:
    # A. Get Sensor Data
    raw_tilt = 0.0
    if accel:
        # Webots Accelerometer returns [x, y, z]. 
        # Index 1 (y) is usually the gravity vector for tilt/slope.
        raw_tilt = accel.getValues()[1] 
    
    # B. Define the "Intent" (What the robot wants to do)
    v_target = 4.0   # Linear Velocity (Speed)
    r_target = 0.0   # Turn Radius (0 = driving straight)
    a_target = 0.5   # Expected Acceleration
    
    # C. THE CRITICAL FIX: Calling 'audit_intent'
    # This matches line 14 of your action_auditor.py exactly.
    audit_results = ai.auditor.audit_intent(
        v_target=v_target,
        r_target=r_target,
        a_target=a_target,
        slope=raw_tilt
    )
    
    # D. ACTUATION LOGIC
    # Your code returns a dictionary: {"authorized": bool, "kernels": dict, "summary": str}
    if audit_results["authorized"]:
        safe_speed = v_target
    else:
        # If the AI vetos the move, we stop for safety
        safe_speed = 0.0
        if robot.getTime() % 1.0 < 0.05:
            print(f"AI VETO: {audit_results['summary']}")

    # Apply to all 4 Pioneer 3-AT motors
    for motor in motors:
        motor.setVelocity(safe_speed)

    # Telemetry
    if robot.getTime() % 2.0 < 0.05:
        print(f"STATUS: {'SAFE' if audit_results['authorized'] else 'DANGER'}")