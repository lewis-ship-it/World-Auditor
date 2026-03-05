from dataclasses import dataclass, field
from typing import List, Dict, Any


@dataclass
class AgentState:
    id: str
    type: str
    mass: float
    velocity: float = 0.0
    acceleration: float = 0.0
    load_weight: float = 0.0
    friction: float = 0.6
    slope: float = 0.0
    wheelbase: float = 1.0
    center_of_mass_height: float = 0.5


@dataclass
class EnvironmentState:
    temperature: float = 20.0
    surface: str = "concrete"
    slope: float = 0.0
    friction_modifier: float = 1.0
    visibility: float = 1.0
    distance_to_obstacles: float = 10.0


@dataclass
class WorldState:
    agents: List[AgentState] = field(default_factory=list)
    environment: EnvironmentState = field(default_factory=EnvironmentState)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def primary_agent(self) -> AgentState:
        if not self.agents:
            raise ValueError("WorldState contains no agents")
        return self.agents[0]