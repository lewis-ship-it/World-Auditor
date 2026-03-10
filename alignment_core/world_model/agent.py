from dataclasses import dataclass


@dataclass
class AgentState:
    """
    Physical description of a robot/vehicle.
    This structure feeds the physics constraint engine.
    """

    id: str
    type: str = "mobile"

    # Motion
    velocity: float = 0.0
    max_speed: float = 0.0

    # Physical properties
    mass: float = 0.0
    wheelbase: float = 0.0
    center_of_mass_height: float = 0.0

    # Payload
    load_weight: float = 0.0
    max_load: float = 0.0