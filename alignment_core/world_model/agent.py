from dataclasses import dataclass
from typing import List, Optional
from .primitives import Vector3, Quaternion, ActuatorLimits

@dataclass
class AgentState:
    id: str
    type: str

    mass: float
    position: Vector3
    velocity: Vector3
    angular_velocity: Vector3
    orientation: Quaternion

    center_of_mass: Vector3
    support_polygon: List[Vector3]

    actuator_limits: ActuatorLimits

    battery_state: float
    current_load: Optional[str]
    contact_points: List[Vector3]

    # --- Added structured physical properties ---
    load_weight: float
    max_load: float
    center_of_mass_height: float
    wheelbase: float

    def total_mass(self) -> float:
        return self.mass + self.load_weight