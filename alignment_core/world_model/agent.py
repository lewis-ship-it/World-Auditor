from dataclasses import dataclass
from typing import List, Optional
from .primitives import Vector3, Quaternion, ActuatorLimits


@dataclass
class AgentState:
    id: str
    type: str

    # Kinematics
    mass: float
    position: Vector3
    velocity: Vector3
    angular_velocity: Vector3
    orientation: Quaternion

    # Geometry / Stability
    center_of_mass: Vector3
    center_of_mass_height: float
    support_polygon: List[Vector3]
    wheelbase: float

    # Load
    load_weight: float
    max_load: float

    # Systems
    actuator_limits: ActuatorLimits
    battery_state: float
    current_load: Optional[float]
    contact_points: List[Vector3]

    def total_mass(self) -> float:
        return self.mass + self.load_weight