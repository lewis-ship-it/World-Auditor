# FILE: alignment_core/world_model/terrain_manager.py
import numpy as np

class TerrainManager:
    SURFACES = {
        "dry_asphalt": {"mu_s": 1.0,  "mu_k": 0.8},
        "wet_grass":   {"mu_s": 0.3,  "mu_k": 0.2},
        "loose_gravel": {"mu_s": 0.5,  "mu_k": 0.45},
        "ice":          {"mu_s": 0.15, "mu_k": 0.08}
    }

    def __init__(self, default_surface="dry_asphalt", safety_margin=0.90): 
        self.surface = default_surface
        # Only use a percentage of available grip to ensure a buffer
        self.safety_margin = safety_margin 

    def get_friction(self, surface_name=None):
        target = surface_name or self.surface
        props = self.SURFACES.get(target, self.SURFACES["dry_asphalt"])
        
        # Apply the safety margin to the static friction coefficient
        safe_mu_s = props["mu_s"] * self.safety_margin
        return safe_mu_s, props["mu_k"]

    def calculate_slope_adjusted_friction(self, mu_s, slope_deg):
        theta = np.radians(slope_deg)
        return mu_s * np.cos(theta)