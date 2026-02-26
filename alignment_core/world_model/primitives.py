from dataclasses import dataclass
from typing import List


@dataclass
class Vector3:
    x: float
    y: float
    z: float


@dataclass
class Quaternion:
    w: float
    x: float
    y: float
    z: float


@dataclass
class BoundingBox:
    width: float
    height: float
    depth: float


@dataclass
class ActuatorLimits:
    max_torque: float
    max_force: float
    max_speed: float
    max_acceleration: float