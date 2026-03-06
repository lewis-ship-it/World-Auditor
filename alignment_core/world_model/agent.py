from dataclasses import dataclass


@dataclass
class AgentState:

    id: str
    type: str

    mass: float
    length: float
    width: float
    height: float

    wheelbase: float
    track_width: float

    center_of_mass_height: float

    velocity: float
    acceleration: float

    drive_type: str = "differential"

    contact_points: list | None = None