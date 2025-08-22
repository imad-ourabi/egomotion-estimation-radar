import numpy as np
import open3d as o3d

class PointCloudPreprocessor:
    def __init__(self):
        # Initialize default parameters
        self.crop_roi_params = {  
            'x': [-20, 20],
            'y': [0, 70],
            'z': [-3, 6]
        }
        
        self.outlier_std_ratio = 3.0
        self.outlier_nb_neighbors = 20
        self.ground_threshold = 0.1


    def crop_roi(self, points):
        """Crop to region of interest"""
        mask = (
            (points[:, 0] >= self.crop_roi_params['x'][0]) &
            (points[:, 0] <= self.crop_roi_params['x'][1]) &
            (points[:, 1] >= self.crop_roi_params['y'][0]) &
            (points[:, 1] <= self.crop_roi_params['y'][1]) &
            (points[:, 2] >= self.crop_roi_params['z'][0]) &
            (points[:, 2] <= self.crop_roi_params['z'][1])
        )
        return points[mask]

    def remove_statistical_outliers(self, points):
        """Statistical outlier removal"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        cl, ind = pcd.remove_statistical_outlier(
            nb_neighbors=self.outlier_nb_neighbors,
            std_ratio=self.outlier_std_ratio
        )
        return np.asarray(cl.points)

    def remove_ground_ransac(self, points):
        """Ground plane removal using RANSAC"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Find plane equation
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.ground_threshold,
            ransac_n=3,
            num_iterations=100
        )
        
        # Return non-ground points
        return np.asarray(pcd.select_by_index(inliers, invert=True).points)
    
    def power_doppler_filter(self, pc_points, power_list, doppler_list, most_common_doppler):
        """
        1. Remove points with weak power values
        2. Only keep the points within the window of the current
           'most common doppler'
        """
        power_threshold = np.mean(power_list) + np.std(power_list) #- 5.0 # - 5.0
        doppler = most_common_doppler
        doppler_window = 2.5 # ±2 m/s

        filtered_indices = np.where(
            (power_list > power_threshold) &
            (doppler_list >= doppler - doppler_window) & 
            (doppler_list <= doppler + doppler_window) 
        )[0]
    
        filtered_points = pc_points[filtered_indices, :]

        return filtered_points

    def apply_all(self, points, power_list, doppler_list, doppler):
        """Full preprocessing pipeline"""
        if len(points) < 200:
            return points
            
        # Apply processing steps in sequence
        points = self.power_doppler_filter(points, power_list, doppler_list, doppler)
        points = self.crop_roi(points)
        #points = self.remove_ground_ransac(points)
        #points = self.remove_statistical_outliers(points)
  
        return points