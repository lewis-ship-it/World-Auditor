# FILE: alignment_core/decision/observer.py

class FrictionObserver:
    def __init__(self, terrain_manager):
        self.terrain = terrain_manager
        self.base_margin = 0.90 

    def update_perception(self, slip_ratio):
        # Filter out minor sensor noise
        if slip_ratio < 0.05:
            self.terrain.safety_margin = self.base_margin
            return "NORMAL"

        # Drop trust exponentially based on slip
        # new_margin = 0.90 * (1.0 - 0.20) -> 0.72
        new_margin = self.base_margin * (1.0 - slip_ratio)
        self.terrain.safety_margin = max(0.3, new_margin) 
        
        return "ADAPTING"