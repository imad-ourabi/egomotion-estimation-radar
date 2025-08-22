import numpy as np

class StaticPointCloudProcessor:
    """
    A class for processing static point cloud data.
    This includes finding the most common doppler value, filtering points,
    and extracting azimuth, elevation, range, and xyz values based on indices.
    """
    
    @staticmethod
    def most_common_doppler_value_and_indices(arr):
        """
        Find the most common value in a NumPy array and return all indices of this value,
        along with the value.
        
        If the array size is less than 4, return the previous output without further calculations.
        
        Parameters:
            arr (numpy.ndarray): The input array.
        
        Returns:
            tuple: A tuple containing the most common value and an array of indices where this value occurs.
        """
        arr = np.asarray(arr)
        
        # Check if the array is empty
        if arr.size == 0:
            raise ValueError("Input array is empty.")

        # Find unique values and their counts
        unique_values, counts = np.unique(arr, return_counts=True)
        
        # Determine the most common value
        most_common_index = np.argmax(counts) 
        most_common_value = unique_values[most_common_index]
        
        # Find all indices of the most common value
        indices = np.where(arr == most_common_value)[0]
        if indices.size == 0:
            raise ValueError("No occurrences found for the most common value.")
        
        return (most_common_value, indices)
    
    @staticmethod
    def filter_points_by_indices(pointsXYZ_data, indices):
        """
        Filter the point cloud based on the given indices.
        
        Parameters:
            pointsXYZ_data (numpy.ndarray): The input point cloud data.
            indices (list or numpy.ndarray): Indices to filter the data.
        
        Returns:
            numpy.ndarray: The filtered point cloud.
        """
        # Ensure indices are a valid list of integers
        indices = np.array(indices, dtype=int)
        
        # Filter the point cloud based on the given indices
        filtered_points = pointsXYZ_data[:, indices]
        
        return filtered_points
    
    @staticmethod
    def get_azimuths_of_indices(azimuth_list, indices):
        """
        Given an array of indices and an azimuth list, return the values from the azimuth list corresponding to the provided indices.
        
        Parameters:
            azimuth_list (numpy.ndarray): Array of azimuth values.
            indices (numpy.ndarray): Array of indices.
        
        Returns:
            numpy.ndarray: Array of azimuth values corresponding to the provided indices.
        """
        azimuth_list = np.asarray(azimuth_list)
        indices = np.asarray(indices)
        
        # Ensure indices are within the valid range
        if np.any(indices >= azimuth_list.size) or np.any(indices < 0):
            raise IndexError("Index out of bounds for the azimuth list.")
        
        # Return the values at the specified indices
        return azimuth_list[indices]
    
    @staticmethod
    def get_elevations_of_indices(elevation_list, indices):
        """
        Given an array of indices and an elevation list, return the values from the elevation list corresponding to the provided indices.
        
        Parameters:
            elevation_list (numpy.ndarray): Array of elevation values.
            indices (numpy.ndarray): Array of indices.
        
        Returns:
            numpy.ndarray: Array of elevation values corresponding to the provided indices.
        """
        elevation_list = np.asarray(elevation_list)
        indices = np.asarray(indices)
        
        # Ensure indices are within the valid range
        if np.any(indices >= elevation_list.size) or np.any(indices < 0):
            raise IndexError("Index out of bounds for the elevation list.")
        
        # Return the values at the specified indices
        return elevation_list[indices]
    
    @staticmethod
    def get_ranges_of_indices(range_list, indices):
        """
        Given an array of indices and a range list, return the values from the range list corresponding to the provided indices.
        
        Parameters:
            range_list (numpy.ndarray): Array of range values.
            indices (numpy.ndarray): Array of indices.
        
        Returns:
            numpy.ndarray: Array of range values corresponding to the provided indices.
        """
        range_list = np.asarray(range_list)
        indices = np.asarray(indices)
        
        # Ensure indices are within the valid range
        if np.any(indices >= range_list.size) or np.any(indices < 0):
            raise IndexError("Index out of bounds for the range list.")
        
        # Return the values at the specified indices
        return range_list[indices]
    
    @staticmethod
    def get_xyz_of_indices(xyzPoints, indices):
        """
        Given an array of indices and a xyz points array, return the values corresponding to the provided indices.
        
        Parameters:
            xyzPoints (numpy.ndarray): Array of xyz points.
            indices (numpy.ndarray): Array of indices.
        
        Returns:
            numpy.ndarray: Array of xyz points corresponding to the provided indices.
        """
        xyzPoints = np.asarray(xyzPoints)
        indices = np.asarray(indices)
        
        # Ensure indices are within the valid range
        if np.any(indices >= xyzPoints.size) or np.any(indices < 0):
            raise IndexError("Index out of bounds for the range list.")
        
        # Return the values at the specified indices
        return xyzPoints[indices]
