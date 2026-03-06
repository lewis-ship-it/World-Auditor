from dataclasses import dataclass


@dataclass
class ConstraintResult:

    name: str

    passed: bool

    message: str

    severity: str = "info"