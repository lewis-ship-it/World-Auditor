from .base import Constraint, ConstraintResult


class BrakingConstraint(Constraint):
    name = "BrakingFeasibility"
    severity = "hard"

    def evaluate(self, world_state):
        agent = world_state.agent
        environment = world_state.environment

        velocity = agent.velocity
        max_deceleration = agent.max_deceleration
        distance_to_obstacle = environment.distance_to_obstacle

        if max_deceleration <= 0:
            return ConstraintResult(
                name=self.name,
                violated=True,
                severity=self.severity,
                details={"error": "Invalid deceleration value"},
            )

        required_stop_distance = (velocity ** 2) / (2 * max_deceleration)

        violated = required_stop_distance > distance_to_obstacle

        return ConstraintResult(
            name=self.name,
            violated=violated,
            severity=self.severity,
            details={
                "velocity": velocity,
                "required_stop_distance": required_stop_distance,
                "available_distance": distance_to_obstacle,
            },
        )