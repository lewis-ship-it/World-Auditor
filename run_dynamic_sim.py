import time
from alignment_core.physics.mechanics import RigidBody, TireModel
from alignment_core.constraints.stability import StabilityKernel
from alignment_core.constraints.friction import FrictionKernel
from alignment_core.constraints.braking import BrakingKernel
from alignment_core.constraints.load import LoadKernel
from alignment_core.world_model.terrain_manager import TerrainManager
from alignment_core.decision.action_auditor import ActionAuditor
from alignment_core.decision.predictive_kernel import PredictiveKernel

def simulate_environment_shift():
    # 1. Setup the Physical World
    robot = RigidBody(mass=50, track_width=0.5, wheelbase=0.6, cog_z=0.2)
    tire = TireModel(cornering_stiffness=18000)
    
    # 2. Setup the World Model (Terrain)
    terrain = TerrainManager(default_surface="dry_asphalt")
    
    # 3. Initialize AI Kernels
    stability = StabilityKernel(robot)
    friction = FrictionKernel(robot, tire, terrain)
    braking = BrakingKernel(max_braking_force=600)
    load = LoadKernel(robot)
    
    # 4. Initialize Brain
    auditor = ActionAuditor(robot, stability, friction, braking, load)
    predictor = PredictiveKernel(auditor)

    print(f"--- STARTING SIMULATION (Initial Surface: {terrain.surface}) ---")
    
    # Path Data: A steady 10m radius curve
    target_radius = 10.0

    # PHASE 1: Dry Asphalt
    prediction_dry = predictor.find_optimal_velocity(radius=target_radius)
    v_max_dry = prediction_dry['max_safe_velocity']
    print(f"[Asphalt] AI calculates max safe speed: {v_max_dry} m/s")

    # PHASE 2: Sudden Terrain Shift (The "Ice" Patch)
    print("\n!!! ENVIRONMENT SHIFT: Robot entering wet grass patch !!!")
    terrain.surface = "wet_grass" # The world changes
    
    # PHASE 3: Re-Audit
    prediction_wet = predictor.find_optimal_velocity(radius=target_radius)
    v_max_wet = prediction_wet['max_safe_velocity']
    
    print(f"[Wet Grass] AI calculates max safe speed: {v_max_wet} m/s")
    
    # 5. Conclusion
    diff = v_max_dry - v_max_wet
    print(f"\n[AI DECISION] Safety systems forced a speed reduction of {round(diff, 2)} m/s")
    print(f"Limiting Factor on Grass: {prediction_wet['limiting_factor']}")

if __name__ == "__main__":
    simulate_environment_shift()