from alignment_core.decision.predictive_kernel import PredictiveKernel
from alignment_core.decision.action_auditor import ActionAuditor
from alignment_core.constraints.stability import StabilityKernel
from alignment_core.constraints.friction import FrictionKernel
from alignment_core.constraints.braking import BrakingKernel
from alignment_core.constraints.load import LoadKernel
from alignment_core.physics.mechanics import RigidBody, TireModel
from alignment_core.world_model.terrain_manager import TerrainManager


class Predictor:
    def __init__(self):
        body = RigidBody(2200, 1.6, 2.9, 0.55)
        tires = TireModel(40000)
        terrain = TerrainManager(default_surface="dry_asphalt")

        auditor = ActionAuditor(
            robot=body,
            stability=StabilityKernel(body),
            friction=FrictionKernel(body, tires, terrain),
            braking=BrakingKernel(max_braking_force=3000),
            load=LoadKernel(body)
        )

        self.kernel = PredictiveKernel(auditor)

    def get_safe_speed(self, radius):
        try:
            result = self.kernel.find_optimal_velocity(radius)
            return result.get("max_safe_velocity", 8.0)
        except Exception as e:
            print("[Prediction Error]:", e)
            return 6.0