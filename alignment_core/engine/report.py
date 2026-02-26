from dataclasses import dataclass
from typing import List, Dict


@dataclass
class ConstraintResult:
    name: str
    hard_violation: bool
    details: Dict


@dataclass
class SafetyReport:
    safe: bool
    hard_violations: List[str]
    constraint_results: List[ConstraintResult]
    risk_score: float