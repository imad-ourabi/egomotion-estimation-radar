import numpy as np
import open3d as o3d
import math
from collections import deque

class RotationAnglesEstimator:
    def __init__(self, voxel_size=0.01, icp_threshold=1.0): # vs = 0.2, Best recorded threshold: 3.8
        self.previous_pc = None
        self.heading = 255.0
        self.pitch = 0.0
        self.roll = 0.0
        self.vehicle_speed = 0.0
        self.voxel_size = voxel_size
        self.icp_threshold = icp_threshold
        self.last_transform = np.eye(4)  # Track transformation history
        # For Yaw Rate
        self.yaw_buffer = deque()  # Stores (yaw, timestamp) pairs
        # For Pitch Rate
        self.pitch_buffer = deque()
        # For Roll Rate
        self.roll_buffer = deque()


    def _preprocess(self, pc_np):
        """Further pre-processing with point cloud downsampling"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_np)
        new_voxel_size = self.adaptive_voxel_size(self.vehicle_speed)

        pointcloud = pcd.voxel_down_sample(self.voxel_size)

        return pointcloud
    
    def _adaptive_threshold(self, speed):
        """Dynamic threshold based on speed"""
        base_thresh = self.icp_threshold
        dynamic_threshold_factor=0.1  # Default is 0.1
        new_threshold = base_thresh * (1 + speed * dynamic_threshold_factor)

        return new_threshold
    
    def adaptive_voxel_size(self, speed, min_voxel=0.005, max_voxel=0.2, speed_threshold=2.0, decay_factor=0.4):
        """Dynamic voxel size based on speed for pre-processing"""
        # Ensure speed is a numeric value
        if not isinstance(speed, (float, int)):
            raise TypeError("speed must be a float or int")
        
        # Clamp speed to positive values
        clamped_speed = max(float(speed), 0.0)
        
        # Non-linear scaling using exponential decay
        normalized_speed = clamped_speed / speed_threshold
        scale_factor = 1 - math.exp(-decay_factor * normalized_speed)
        
        # Calculate adaptive voxel size
        voxel_size = min_voxel + scale_factor * (max_voxel - min_voxel)
        
        return min(voxel_size, max_voxel)

    def _extract_yaw_pitch_roll(self, transformation):
        """Direct yaw calculation for left-handed system"""
        # Extract rotation matrix from transformation matrix
        rotation = transformation[:3, :3]
        
        # Calculate yaw directly from rotation matrix components
        # For left-handed Z-up system: yaw = arctan2(-m10, m00)
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])  # Negative sign for left-handed
        pitch = np.arcsin(rotation[2, 0])
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        
        return yaw, pitch, roll

    def update(self, current_pc_np, vehicle_speed, current_time):

        # Pre-process with downsampling
        current_pcd = self._preprocess(current_pc_np)

        self.vehicle_speed = vehicle_speed

        # Check is previous pc is empty    
        if self.previous_pc is None:
            self.previous_pc = current_pcd
            return self.heading, self.pitch, self.roll, 0.0
        
        # Calculate adaptive threshold based on current vehicle speed
        new_threshold = self._adaptive_threshold(vehicle_speed)

        # Run ICP with continuous transform chaining
        reg = o3d.pipelines.registration.registration_icp(
            current_pcd, self.previous_pc, self.icp_threshold,
            self.last_transform,  # Use previous transform as initial guess
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=500, # Lower iterations for faster performance
                relative_fitness=1e-4,
                relative_rmse=1e-4
                ) 
        )

        # Get transformation matrix from ICP registration
        T = reg.transformation
        
        self.last_transform = T

        # Extract and convert yaw, pith and roll
        yaw_rad, pitch_rad, roll_rad = self._extract_yaw_pitch_roll(T)

        delta_deg = np.rad2deg(yaw_rad)
        pitch_deg = np.rad2deg(pitch_rad)
        roll_deg  = np.rad2deg(roll_rad)


        # Add additional stationary detection (if vehicle speed is near zero)
        stationary_threshold = 0.05 
        if vehicle_speed < stationary_threshold:
            # When completely stationary, ignore all small ICP changes
            delta_deg = 0.0
            pitch_deg = 0.0
            roll_deg = 0.0

        # Accumulate heading with yaw and keep between 0 and 360 deg
        self.heading = (self.heading + delta_deg) % 360   

        # Update previous point cloud
        self.previous_pc = current_pcd

        # Yaw Rate per second
        yaw_rate = self.yaw_to_rate(delta_deg, current_time)
        pitch_rate = self.pitch_to_rate(pitch_deg, current_time)
        roll_rate = self.roll_to_rate(roll_deg, current_time)

        return yaw_rate, pitch_rate, roll_rate, self.heading
    

    def yaw_to_rate(self, current_yaw, current_time):
        """
        Calculate the sum of yaw values over the past second.
        :param current_yaw: Current yaw in degrees.
        :param current_time: Current timestamp in seconds.
        :return: Yaw Rate in degrees per second.
        """
        # Add current yaw and timestamp to the buffer
        self.yaw_buffer.append((current_yaw, current_time))

        # Remove old entries outside the 1-second window
        while self.yaw_buffer and (current_time - self.yaw_buffer[0][1]) > 1.0:
            self.yaw_buffer.popleft()

        # Calculate average yaw
        if self.yaw_buffer:
            yaws = [yaw for yaw, _ in self.yaw_buffer]
            return sum(yaws) 
        else:
            return current_yaw  # If no data in the buffer, return current yaw
        
    def pitch_to_rate(self, current_pitch, current_time):
        """
        Calculate the sum of pitch values over the past second.
        :param current_pitch: Current yaw in degrees.
        :param current_time: Current timestamp in seconds.
        :return: Pitch Rate in degrees per second.
        """
        # Add current yaw and timestamp to the buffer
        self.pitch_buffer.append((current_pitch, current_time))

        # Remove old entries outside the 1-second window
        while self.pitch_buffer and (current_time - self.pitch_buffer[0][1]) > 1.0:
            self.pitch_buffer.popleft()

        # Calculate average yaw
        if self.pitch_buffer:
            pitchs = [pitch for pitch, _ in self.pitch_buffer]
            return sum(pitchs) 
        else:
            return current_pitch  # If no data in the buffer, return current yaw
        
    def roll_to_rate(self, current_roll, current_time):
        """
        Calculate the sum of roll values over the past second.
        :param current_roll: Current yaw in degrees.
        :param current_time: Current timestamp in seconds.
        :return: Roll Rate in degrees per second.
        """
        # Add current yaw and timestamp to the buffer
        self.roll_buffer.append((current_roll, current_time))

        # Remove old entries outside the 1-second window
        while self.roll_buffer and (current_time - self.roll_buffer[0][1]) > 1.0:
            self.roll_buffer.popleft()

        # Calculate average yaw
        if self.roll_buffer:
            rolls = [roll for roll, _ in self.roll_buffer]
            return sum(rolls) 
        else:
            return current_roll  # If no data in the buffer, return current yaw
 