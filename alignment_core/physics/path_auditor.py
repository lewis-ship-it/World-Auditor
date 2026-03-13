import numpy as np

class PathAuditor:
    def __init__(self, agent_state, g=9.81):
        self.agent = agent_state
        self.g = g

    def calculate_safe_velocity(self, dist_array, elevation_array, friction_array, target_stop_dist):
        """
        Calculates the maximum safe velocity at each point to ensure 
        the robot can stop before a target distance.
        """
        # 1. Calculate Slopes (Rise/Run)
        slopes = np.arctan(np.gradient(elevation_array, dist_array))
        
        # 2. Backwards Pass for Braking
        # We start at the target stop point (v=0) and work backwards to see
        # how fast we could have been going to reach that point safely.
        v_safe = np.zeros_like(dist_array)
        v_safe[dist_array >= target_stop_dist] = 0
        # Calculate how much torque is actually being used vs what is available
        # motor_force = torque / wheel_radius
        effort = (power_draw / (torque * max_rpm / 9.5488)) * 100
        efforts.append(min(effort, 100))  # Ensure it doesn't exceed 100%

        for i in range(len(dist_array) - 2, -1, -1):
            theta = slopes[i]
            mu = friction_array[i]
            ds = dist_array[i+1] - dist_array[i]
            
            # Physics: a_max = (mu * g * cos(theta)) - (g * sin(theta))
            # The 'sin' part represents gravity helping (downhill) or hurting (uphill)
            max_decel = (mu * self.g * np.cos(theta)) - (self.g * np.sin(theta))
            
            # Use kinematic equation: v_prev = sqrt(v_next^2 + 2 * a * ds)
            v_safe[i] = np.sqrt(v_safe[i+1]**2 + 2 * max_decel * ds)
            
        return np.minimum(v_safe, self.agent.max_speed)