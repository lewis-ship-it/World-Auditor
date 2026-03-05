from dataclasses import dataclass

@dataclass
class Action:

    type: str
    target_velocity: float
    target_distance: float