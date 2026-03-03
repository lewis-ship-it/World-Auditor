# alignment_core/world_model/agent.py

from dataclasses import dataclass


@dataclass
class AgentState:
    """
    Represents a physical agent in the world model.
    Designed for deterministic safety evaluation.
    """

    id: str
    type: str  # e.g. "mobile", "drone", "vehicle"

    # Core physics
    mass: float  # kg
    velocity: float  # m/s
    position: float  # meters (1D demo position)

    # Load & structure
    load_weight: float  # kg
    center_of_mass_height: float  # meters
    wheelbase: float  # meters (distance between wheels)

    def total_mass(self) -> float:
        """
        Returns total effective mass including load.
        """
        return self.mass + self.load_weight

    def kinetic_energy(self) -> float:
        """
        Returns kinetic energy in Joules.
        KE = 1/2 m v^2
        """
        return 0.5 * self.total_mass() * (self.velocity ** 2)

    def momentum(self) -> float:
        """
        Returns linear momentum (kg·m/s)
        """
        return self.total_mass() * self.velocity

    def is_mobile(self) -> bool:
        return self.type.lower() in ["mobile", "vehicle", "robot"]

    def is_airborne(self) -> bool:
        return self.type.lower() in ["drone", "uav"]

    def to_dict(self) -> dict:
        """
        Useful for debugging or logging.
        """
        return {
            "id": self.id,
            "type": self.type,
            "mass": self.mass,
            "velocity": self.velocity,
            "position": self.position,
            "load_weight": self.load_weight,
            "center_of_mass_height": self.center_of_mass_height,
            "wheelbase": self.wheelbase,
        }