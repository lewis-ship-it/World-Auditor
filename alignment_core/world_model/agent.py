from dataclasses import dataclass
from typing import List, Optional


# --- Basic Math Primitives ---

@dataclass
class Vector3:
    x: float
    y: float
    z: float


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass
class ActuatorLimits:
    max_force: float
    max_torque: float


# --- Agent State Model ---

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