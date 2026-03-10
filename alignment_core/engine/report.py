from dataclasses import dataclass
from typing import List
from alignment_core.constraints.constraint_result import ConstraintResult

@dataclass
class SafetyReport:
    results: List[ConstraintResult]

    def is_safe(self) -> bool:
        # FIX: Changed 'r.safe' to 'r.passed' to match ConstraintResult
        return all(r.passed for r in self.results)

    def risk_score(self) -> float:
        if not self.results:
            return 0.0
        # FIX: Changed 'r.safe' to 'r.passed'
        failures = sum(1 for r in self.results if not r.passed)
        return (failures / len(self.results)) * 100.0