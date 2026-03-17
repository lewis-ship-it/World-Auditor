# FILE: Master_Test.py
import time
from alignment_core.physics.mechanics import RigidBody, TireModel
from alignment_core.constraints.stability import StabilityKernel
from alignment_core.constraints.friction import FrictionKernel
from alignment_core.constraints.braking import BrakingKernel
from alignment_core.constraints.load import LoadKernel
from alignment_core.world_model.terrain_manager import TerrainManager
from alignment_core.decision.action_auditor import ActionAuditor
from alignment_core.decision.predictive_kernel import PredictiveKernel
from alignment_core.sensors.imu import IMUSensor
from alignment_core.sensors.encoder import WheelEncoder
from alignment_core.main_ai import PhysicsAI

def run_master_test():
    print("=== INITIALIZING PHYSICS AI MASTER TEST ===\n")

    # 1. Hardware & World Setup
    robot = RigidBody(mass=50, track_width=0.5, wheelbase=0.6, cog_z=0.2)
    tire = TireModel(cornering_stiffness=18000)
    terrain = TerrainManager(default_surface="dry_asphalt", safety_margin=0.90)
    
    # 2. Kernel Setup
    stability = StabilityKernel(robot)
    friction = FrictionKernel(robot, tire, terrain)
    braking = BrakingKernel(max_braking_force=600)
    load = LoadKernel(robot)
    
    # 3. Decision & Sensor Setup
    auditor = ActionAuditor(robot, stability, friction, braking, load)
    predictor = PredictiveKernel(auditor)
    imu = IMUSensor(robot)
    encoders = WheelEncoder(robot)
    
    # 4. Create the Brain
    ai = PhysicsAI(auditor, predictor, imu, encoders)

    # --- SIMULATION SCENARIO ---
    target_v = 12.0  
    target_r = 15.0  
    
    print(f"Scenario: Approaching {target_r}m curve at {target_v}m/s")
    print(f"Initial Safety Margin: {round(terrain.safety_margin * 100)}%\n")

    # PHASE 1: Stable Surface (Minimal Slip)
    print("--- PHASE 1: Stable Surface ---")
    res = ai.think(target_v, target_r, real_v=12.0, wheel_v=12.1, real_slope=0)
    print(f"Status: {res['perception']}")
    print(f"AI Decision: {'✅ AUTHORIZED' if res['authorized'] else '❌ VETO'}")
    print(f"Current Max Safe Speed: {res['max_safe_speed']} m/s\n")

    # PHASE 2: Hit Slipped Surface (High Slip)
    print("--- PHASE 2: Entering Slippery Patch (Simulated Slip) ---")
    res = ai.think(target_v, target_r, real_v=12.0, wheel_v=16.0, real_slope=0)
    
    print(f"Perception Alert: {res['perception']}")
    print(f"New Safety Margin: {round(terrain.safety_margin, 2)}")
    print(f"AI Decision: {'✅ AUTHORIZED' if res['authorized'] else '❌ VETO'}")
    print(f"Emergency Summary: {res['status_summary']}")
    print(f"Calculated New Max Safe Speed: {res['max_safe_speed']} m/s")

if __name__ == "__main__":
    run_master_test()