import numpy as np
import pdb

class Acceleration:
    def __init__(self):
        # Initialize previous velocity
        self.prev_vx = None
        self.prev_vy = None
        self.prev_vz = None

        self.prev_v = None

    def compute_acceleration(self, vx, vy, vz, delta_t):
        """
        Compute the acceleration components (ax, ay, az) from the velocity components.
        
        Parameters:
        vx (float): Velocity component in the x direction (m/s).
        vy (float): Velocity component in the y direction (m/s).
        vz (float): Velocity component in the z direction (m/s).
        delta_t (float): Time step in seconds.
        
        Returns:
        tuple: A tuple containing the acceleration components (ax, ay, az) in m/s^2.
        """
        if delta_t <= 0:
            raise ValueError("Delta t must be greater than zero.")

        if self.prev_vx is None:
            # First frame: no acceleration can be calculated
            self.prev_vx, self.prev_vy, self.prev_vz = vx, vy, vz
            return 0.0, 0.0, 0.0  # No acceleration yet

        # Calculate change in velocity
        delta_vx = vx - self.prev_vx
        delta_vy = vy - self.prev_vy
        delta_vz = vz - self.prev_vz

        # Compute acceleration components
        ax = delta_vx / delta_t
        ay = delta_vy / delta_t
        az = delta_vz / delta_t

        # Update previous velocity values
        self.prev_vx, self.prev_vy, self.prev_vz = vx, vy, vz

        return ax, ay, az


    def calculate_acceleration(self, speed, delta_t):

        if delta_t <= 0:
            raise ValueError("Delta t must be greater than zero.")
        
        if self.prev_v is None:
            # First frame: no acceleration can be calculated
            self.prev_v = speed
            return 0.0  # No acceleration yet

        # Calculate change in velocity
        delta_v = speed - self.prev_v

        # Compute acceleration components
        acc = delta_v / delta_t

        # Update previous velocity values
        self.prev_v = speed

        
        return acc
