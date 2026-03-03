from dataclasses import dataclass
from typing import List, Optional
from .primitives import Vector3, Quaternion, ActuatorLimits

@dataclass
class AgentState:
    id: str
    type: str

    # Physical properties
    mass: float

    # Kinematics
    position: Vector3
    velocity: Vector3
    angular_velocity: Vector3
    orientation: Quaternion

    # Stability properties
    center_of_mass: Vector3
    center_of_mass_height: float
    support_polygon: List[Vector3]
    wheelbase: float

    # Load properties
    load_weight: float
    max_load: float
    current_load: Optional[float]

    # Hardware constraints
    actuator_limits: ActuatorLimits
    battery_state: float

    # Contact modeling
    contact_points: List[Vector3]

    def total_mass(self) -> float:
        """Returns the combined mass of the agent and its current load."""
        return self.mass + self.load_weight