from dataclasses import dataclass

@dataclass
class AgentState:

    id: str
    type: str

    mass: float
    velocity: float

    braking_force: float
    max_deceleration: float

    load_weight: float

    center_of_mass_height: float
    wheelbase: float