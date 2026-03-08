import numpy as np

class EnergyModel:

    def __init__(self,
                 vehicle_mass=1200,
                 drag_coeff=0.32,
                 frontal_area=2.2,
                 rolling_resistance=0.015,
                 air_density=1.225):

        self.mass = vehicle_mass
        self.cd = drag_coeff
        self.A = frontal_area
        self.rr = rolling_resistance
        self.rho = air_density

    def drag_force(self, v):

        return 0.5 * self.rho * self.cd * self.A * v**2

    def rolling_force(self):

        g = 9.81
        return self.mass * g * self.rr

    def power_usage(self, v):

        drag = self.drag_force(v)

        roll = self.rolling_force()

        total_force = drag + roll

        power = total_force * v

        return power

    def energy_used(self, speeds, distances):

        energy = []

        for v, d in zip(speeds, distances):

            power = self.power_usage(v)

            if v == 0:
                energy.append(0)
            else:
                energy.append(power * (d / v))

        return np.array(energy)

    def regen_energy(self, decel_energy, efficiency=0.6):

        return decel_energy * efficiency