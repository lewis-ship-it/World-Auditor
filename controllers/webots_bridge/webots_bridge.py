from controller import Robot
import sys
import os

# 1. LINK TO YOUR AI CORE
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
if project_root not in sys.path:
    sys.path.append(project_root)

# Import your components (Adjust names if they differ in your files)
try:
    from alignment_core.main_ai import PhysicsAI
    from alignment_core.decision.action_auditor import Auditor
    from alignment_core.predictor import Predictor
    # Assuming these classes exist for your IMU and Encoder logic
    # from alignment_core.sensors import IMUHandler, EncoderHandler 
except ImportError as e:
    print(f"IMPORT ERROR: {e}")
    sys.exit(1)

# 2. INITIALIZE WEBOTS
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Pioneer 3-AT 4-Wheel Drive Setup
motor_names = [
    'front left wheel', 'front right wheel', 
    'back left wheel', 'back right wheel'
]
motors = []
for name in motor_names:
    m = robot.getDevice(name)
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
    motors.append(m)

# 3. INITIALIZE SENSORS
# Note: Ensure you added an Accelerometer named "accelerometer" in the Webots Scene Tree
accel = robot.getDevice('accelerometer')
if accel:
    accel.enable(timestep)

# 4. INITIALIZE AI COMPONENTS (Solving the TypeError)
# Creating the 4 required positional arguments for PhysicsAI
my_auditor = Auditor()
my_predictor = Predictor()
my_imu_data = [0, 0, 0] # Placeholder or your IMU class instance
my_encoders = [0, 0, 0, 0] # Placeholder or your Encoder class instance

# Pass the 4 required arguments to your AI
ai = PhysicsAI(
    auditor=my_auditor, 
    predictor=my_predictor, 
    imu=my_imu_data, 
    encoders=my_encoders
)

print("--- AI BRIDGE ACTIVE AND RUNNING ---")

# 5. MAIN SIMULATION LOOP
while robot.step(timestep) != -1:
    # A. Get sensor data
    tilt = 0.0
    if accel:
        # Index 1 is usually the Y-axis (gravity vector)
        tilt = accel.getValues()[1] 
    
    # B. AI Logic
    # Calling your calculation method
    speed = ai.calculate_safe_speed(current_tilt=tilt)
    
    # C. Actuate all 4 wheels
    for m in motors:
        m.setVelocity(speed)