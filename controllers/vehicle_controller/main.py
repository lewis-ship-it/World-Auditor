from controller import Robot

from pipeline.sensors import SensorSuite
from pipeline.perception import Perception
from pipeline.intent import IntentGenerator
from pipeline.prediction import Predictor
from pipeline.safety import SafetySystem
from pipeline.control import Controller
from pipeline.actuators import ActuatorSuite
from pipeline.dynamics import VehicleDynamics

robot = Robot()
timestep = int(robot.getBasicTimeStep())

sensors = SensorSuite(robot, timestep)
perception = Perception()
intent = IntentGenerator()
predictor = Predictor()
safety = SafetySystem()
controller = Controller(robot)
actuators = ActuatorSuite(robot)

dynamics = VehicleDynamics()

while robot.step(timestep) != -1:

    sensor_data = sensors.read()

    state = perception.process(sensor_data)

    desired = intent.compute(state)

    predicted = predictor.evaluate(state, desired)

    safe = safety.enforce(state, desired, predicted)

    dt = timestep / 1000.0

    dynamic_action = dynamics.step(state, safe, dt)

    control = controller.compute(state, dynamic_action)

    actuators.apply(control)

# --- ADD TO THE BOTTOM OF main.py ---
if robot.getTime() % 1.0 < 0.05:
    print(f"--- SYSTEM TELEMETRY ---")
    print(f"Target Speed: {desired['target_speed']:.1f} | Safe Limit: {predicted['max_safe_speed']:.1f}")
    print(f"Braking Limit: {predicted['debug']['braking_limit']:.1f}")
    print(f"Friction Est: {predicted['debug']['friction']:.2f}μ")
    
    if state["obstacle_distance"]:
        print(f"Obstacle at: {state['obstacle_distance']:.2f}m")
    
    # Check if the Alignment Core Auditor is overriding
    if safe['speed'] < predicted['max_safe_speed']:
        print("⚠️ AUDITOR OVERRIDE: Stability/Load constraints active")