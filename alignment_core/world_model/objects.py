from dataclasses import dataclass
from .primitives import Vector3, Quaternion, BoundingBox


@dataclass
class ObjectState:
    id: str
    mass: float
    material: str
    position: Vector3
    velocity: Vector3
    orientation: Quaternion
    bounding_box: BoundingBox
    friction_coefficient: float
    restitution: float
    structural_integrity: float
    is_static: bool