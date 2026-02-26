from dataclasses import dataclass


@dataclass
class UncertaintyModel:
    position_variance: float
    velocity_variance: float
    friction_variance: float
    sensor_noise_level: float