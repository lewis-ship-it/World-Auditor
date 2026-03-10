from dataclasses import dataclass


@dataclass
class ConstraintResult:
    """
    Standard output for every constraint in the physics safety engine.
    """

    name: str
    passed: bool
    message: str = ""

    def to_dict(self):
        return {
            "Constraint": self.name,
            "Status": "PASS" if self.passed else "FAIL",
            "Details": self.message
        }