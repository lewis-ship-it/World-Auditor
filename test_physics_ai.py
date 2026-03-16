from alignment_core.physics.mechanics import RigidBody, TireModel
from alignment_core.constraints.stability import StabilityKernel
from alignment_core.constraints.friction import FrictionKernel
from alignment_core.constraints.braking import BrakingKernel
from alignment_core.constraints.load import LoadKernel
from alignment_core.decision.action_auditor import ActionAuditor
from alignment_core.decision.predictive_kernel import PredictiveKernel

def run_simulation():
    # 1. Initialize the Physical Body
    # 50kg robot, 0.5m wide, 0.2m CoG height
    robot = RigidBody(mass=50, track_width=0.5, wheelbase=0.6, cog_z=0.2, max_braking_force=600)
    tire = TireModel(cornering_stiffness=18000)

    # 2. Initialize the AI Brain (Kernels)
    stability = StabilityKernel(robot)
    friction = FrictionKernel(tire, mu_static=0.9, mu_kinetic=0.7) # Dry Asphalt
    braking = BrakingKernel(max_braking_force=600)
    load = LoadKernel(robot)

    # 3. Initialize the Auditor and Strategist
    auditor = ActionAuditor(robot, stability, friction, braking, load)
    predictor = PredictiveKernel(auditor)

    print("--- PHYSICS AI INITIALIZED ---")
    
    # SCENARIO: Carrying a heavy 20kg box, placed high up (0.5m)
    payload = {'mass': 20, 'x': 0, 'y': 0, 'z': 0.5}
    
    # 4. Find the Optimal Safe Speed for a sharp 5m turn
    print("\n[AI Thinking] Calculating max safe speed for a 5m radius turn with heavy load...")
    prediction = predictor.find_optimal_velocity(radius=5.0, payload=payload)
    
    print(f"Result: Max Safe Velocity is {prediction['max_safe_velocity']} m/s")
    print(f"Limiting Factor: {prediction['limiting_factor']}")

    # 5. Audit a specific Dangerous Intent
    # User wants to go 12 m/s (approx 27 mph) into that 5m turn
    print("\n[User Command] Execute turn at 12 m/s!")
    audit = auditor.audit_intent(v_target=12.0, r_target=5.0, a_target=0, payload=payload)
    
    if not audit["authorized"]:
        print(f"Status: BLOCKED")
        print(f"Reason: {audit['summary']}")
    else:
        print("Status: AUTHORIZED")

if __name__ == "__main__":
    run_simulation()