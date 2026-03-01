from dataclasses import dataclass

@dataclass
class ConstraintResult:
    name: str
    violated: bool
    message: str