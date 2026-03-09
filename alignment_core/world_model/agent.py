from dataclasses import dataclass


@dataclass
class AgentState:

    id: str

    mass: float
    velocity: float

    wheelbase: float
    center_of_mass_height: float

    load_weight: float = 0