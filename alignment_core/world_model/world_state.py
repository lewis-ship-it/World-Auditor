from dataclasses import dataclass
from typing import List
from .environment import EnvironmentState
from .agent import AgentState
from .objects import ObjectState
from .uncertainty import UncertaintyModel
from .primitives import Vector3


@dataclass
class WorldState:
    timestamp: float
    delta_time: float
    gravity: Vector3
    environment: EnvironmentState
    agents: List[AgentState]
    objects: List[ObjectState]
    uncertainty: UncertaintyModel