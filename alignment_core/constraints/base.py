from abc import ABC, abstractmethod


class ConstraintResult:
    def __init__(self, name, violated, severity="soft", details=None):
        self.name = name
        self.violated = violated
        self.severity = severity  # "hard" or "soft"
        self.details = details or {}

    def to_dict(self):
        return {
            "name": self.name,
            "violated": self.violated,
            "severity": self.severity,
            "details": self.details,
        }


class Constraint(ABC):
    name = "UnnamedConstraint"
    severity = "soft"

    @abstractmethod
    def evaluate(self, world_state) -> ConstraintResult:
        pass