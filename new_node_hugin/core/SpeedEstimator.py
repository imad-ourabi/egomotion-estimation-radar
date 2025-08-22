import numpy as np

class SpeedEstimator:
    """
    A class for estimating the speed of a moving vehicle based on radar data.
    
    This class provides methods to:
      - Compute the average vehicle speed.
      - Compute the velocity vector components (vx, vy, vz).
    """
    
    @staticmethod
    def compute_average_vehicle_speed(doppler_value, azimuth_list, elevation_list):
        """
        Compute the average speed of a moving vehicle based on the elevation, azimuth, 
        and common Doppler (radial velocity) value.
    
        Parameters:
            doppler_value (float): The common Doppler (radial velocity) value in meters per second.
            azimuth_list (numpy.ndarray): Array of azimuth angles in radians.
            elevation_list (numpy.ndarray): Array of elevation angles in radians.
    
        Returns:
            float: The average speed of the moving vehicle in meters per second.
        """
        # Ensure inputs are numpy arrays
        azimuth_list = np.asarray(azimuth_list)
        elevation_list = np.asarray(elevation_list)
        
        # Ensure all lists are of the same length
        if azimuth_list.size != elevation_list.size:
            raise ValueError("Azimuth and elevation arrays must have the same length.")
        
        speeds = []
        
        for elevation, azimuth in zip(elevation_list, azimuth_list):
            # Calculate the denominator, which is the projection factor
            projection_factor = np.cos(elevation) * np.cos(azimuth)
            
            # Handle the case where projection_factor is zero to avoid division by zero
            if projection_factor == 0:
                continue  # Skip this iteration as speed is undefined
            
            # Compute the vehicle speed
            vehicle_speed = doppler_value / projection_factor
            speeds.append(vehicle_speed)
        
        if len(speeds) == 0:
            raise ValueError("No valid speed calculations were performed. Ensure projection factors are non-zero.")
        
        # Calculate the average speed (absolute value)
        average_speed = abs(np.mean(speeds))
        
        return average_speed

    @staticmethod
    def compute_velocity_vector(doppler_value, azimuth_list, elevation_list):
        """
        Compute the velocity vector components (vx, vy, vz) of a moving vehicle
        based on the elevation, azimuth, and common Doppler (radial velocity) value.
    
        Parameters:
            doppler_value (float): The common Doppler (radial velocity) value in meters per second.
            azimuth_list (numpy.ndarray): Array of azimuth angles in radians.
            elevation_list (numpy.ndarray): Array of elevation angles in radians.
    
        Returns:
            tuple: A tuple containing the average velocity components (vx, vy, vz) in meters per second.
        """
        # Ensure inputs are numpy arrays
        azimuth_list = np.asarray(azimuth_list)
        elevation_list = np.asarray(elevation_list)
        
        # Ensure all lists are of the same length
        if azimuth_list.size != elevation_list.size:
            raise ValueError("Azimuth and elevation arrays must have the same length.")
    
        # Use the mean values for doppler, elevation, and azimuth for calculation
        doppler_value = np.mean(doppler_value)
        elevation = np.mean(elevation_list)
        azimuth = np.mean(azimuth_list)
    
        # Compute velocity vector components using spherical to Cartesian conversion formulas
        vx = doppler_value * np.cos(elevation) * np.cos(azimuth)
        vy = doppler_value * np.cos(elevation) * np.sin(azimuth)
        vz = doppler_value * np.sin(elevation)
        
        return vx, vy, vz
