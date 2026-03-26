import os
import sys

# Path Setup
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_script_dir, '../../'))
if project_root not in sys.path:
    sys.path.append(project_root)

from controller import Robot, Keyboard
from pipeline.prediction import Predictor
from pipeline.perception import Perception
from alignment_core.decision.action_auditor import ActionAuditor
from core.brain import Brain
from adapters.webots_adapter import WebotsAdapter

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    kb = Keyboard()
    kb.enable(timestep)

    adapter = WebotsAdapter(robot, timestep)
    perception_unit = Perception()
    predictor = Predictor()
    
    auditor = ActionAuditor(
        robot=robot,
        stability=getattr(predictor, 'stability', None),
        friction=getattr(predictor, 'friction', None),
        braking=getattr(predictor, 'braking', None),
        load=None
    )
    
    brain = Brain(predictor)
    autopilot = True

    print("--- WORLD AUDITOR: SYSTEM ONLINE ---")

    while robot.step(timestep) != -1:
        key = kb.getKey()
        
        # Toggle Autopilot
        if key == ord('A'):
            autopilot = not autopilot
            print(f"Autopilot: {autopilot}")

        sensor_data = adapter.read()
        state = perception_unit.process(sensor_data)

        if autopilot:
            brain_output = brain.step(sensor_data)
            
            # Defensive check for float vs dict
            if isinstance(brain_output, (int, float)):
                intent = {"speed": float(brain_output), "steering": 0.0}
            else:
                intent = brain_output
            
            audit_result = auditor.audit_intent(
                state=state, 
                intent=intent, 
                **sensor_data 
            )
            
            action = {
                "speed": audit_result["approved_speed"],
                "steering": audit_result["approved_steering"]
            }
        else:
            # Manual Control Logic
            action = {"speed": 0.0, "steering": 0.0}
            if key == Keyboard.UP:    action["speed"] = 10.0
            if key == Keyboard.DOWN:  action["speed"] = -5.0
            if key == Keyboard.LEFT:  action["steering"] = 0.4
            if key == Keyboard.RIGHT: action["steering"] = -0.4

        adapter.apply(action)

if __name__ == "__main__":
    main()