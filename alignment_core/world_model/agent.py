from dataclasses import dataclass
from typing import List, Optional
from .primitives import Vector3, Quaternion, ActuatorLimits


@dataclass
class ContactPoint:
    object_a: str
    object_b: str
    normal_force: float
    tangential_force: float
    slip_detected: bool


@dataclass
class LoadState:
    load_id: str
    mass: float
    position_relative_to_agent: Vector3
    height: float
    stability_factor: float


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
    current_load: Optional[LoadState]
    contact_points: List[ContactPoint]