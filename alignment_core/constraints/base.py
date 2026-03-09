from dataclasses import dataclass


class Constraint:

    name = "Constraint"
    severity = "soft"

    def evaluate(self, world_state):
        raise NotImplementedError


@dataclass
class ConstraintResult:

    name: str
    violated: bool
    severity: str
    message: str

    def to_dict(self):
        return {
            "name": self.name,
            "violated": self.violated,
            "severity": self.severity,
            "message": self.message
        }