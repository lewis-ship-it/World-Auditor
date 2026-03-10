from dataclasses import dataclass
from abc import ABC, abstractmethod
from typing import Any
# FIX: Import the standard result instead of defining a conflicting one here
from alignment_core.constraints.constraint_result import ConstraintResult

class BaseConstraint(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @abstractmethod
    def evaluate(self, world_state: Any) -> ConstraintResult:
        pass