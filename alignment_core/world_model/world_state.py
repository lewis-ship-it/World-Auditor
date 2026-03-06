from dataclasses import dataclass
from .agent import AgentState
from .environment import EnvironmentState


@dataclass
class WorldState:

    agent: AgentState

    environment: EnvironmentState