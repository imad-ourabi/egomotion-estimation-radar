import numpy as np
import open3d as o3d
import math
from collections import deque

class RotationAnglesEstimator2:
    def __init__(self, voxel_size=0.2, icp_threshold=2.5):
        self.previous_pc = None
        self.heading = 185.0
        self.pitch = 0.0
        self.roll = 0.0
        self.vehicle_speed = 0.0
        self.voxel_size = voxel_size
        self.icp_threshold = icp_threshold
        self.last_transform = np.eye(4)
        self.rejection_count = 0
        self.previous_delta = 0.0

        self.delta_buffer = deque(maxlen=10)  # Track last 10 delta_deg values
        self.reset_cooldown = 0  # Prevent rapid consecutive resets
        
        # Buffers for rate calculation
        self.yaw_buffer = deque(maxlen=20)  # ~1sec at 20Hz
        self.pitch_buffer = deque(maxlen=20)
        self.roll_buffer = deque(maxlen=20)

        # Generalized ICP configuration
        self.estimation_method = o3d.pipelines.registration.TransformationEstimationForGeneralizedICP()

    def _adaptive_voxel(self, speed):
        """Speed-adaptive voxel sizing"""
        base_size = 0.05
        dynamic_factor = 0.1 * math.log1p(speed)  # Logarithmic scaling
        return max(min(base_size + dynamic_factor, 0.2), 0.05)

    def _preprocess(self, pc_np):
        """Lightweight processing"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_np)
        return pcd.voxel_down_sample(self.voxel_size)

    def _dynamic_threshold(self, speed):
        """Improved threshold adaptation"""
        base = self.icp_threshold
        return base * (1 + 0.15*speed + 0.02*speed**2)

    def _extract_angles(self, transformation):
        """Robust angle extraction"""
        rotation = transformation[:3, :3]
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        
        # Handle heading continuity
        if abs(yaw - np.radians(self.heading)) > np.pi:
            yaw += 2*np.pi * (-1 if yaw > 0 else 1)
            
        pitch = np.arcsin(-rotation[2, 0])
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])

        return yaw, pitch, roll

    def update(self, current_pc_np, vehicle_speed, current_time):
        
        # Preprocess with speed-adaptive voxelization
        current_pcd = self._preprocess(current_pc_np)
        self.vehicle_speed = vehicle_speed

        if self.previous_pc is None:
            self.previous_pc = current_pcd
            return 0.0, 0.0, 0.0, self.heading

        # Dynamic ICP configuration
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=100, # default 150
            relative_fitness=1e-5,
            relative_rmse=1e-5
        )

        # Generalized ICP execution
        reg = o3d.pipelines.registration.registration_icp(
            current_pcd, self.previous_pc,
            self.icp_threshold,
            self.last_transform,
            self.estimation_method,
            criteria
        )

        # Transform analysis
        T = reg.transformation
        self.last_transform = T
        
        # Angle extraction with dynamic compensation
        yaw_rad, pitch_rad, roll_rad = self._extract_angles(T)
        delta_deg = np.rad2deg(yaw_rad)


        delta_deg = ((delta_deg + 180) % 360) - 180 # NEW

        # Check for stuck state before updating heading
        if self._check_for_stuck_icp(delta_deg):
            # If reset, skip this frame's update
            return 0.0, 0.0, 0.0, self.heading

        if abs(delta_deg) > 3.0:
            delta_deg = self.previous_delta
        else:
            self.previous_delta = delta_deg

        print(f"Delta Degree = {delta_deg} deg")

        # Update heading with dynamic wrapping
        self.heading = (self.heading + delta_deg) % 360
        
        # Update buffers for rate calculation
        self.yaw_buffer.append(delta_deg)
        #self.pitch_buffer.append(np.rad2deg(pitch_rad))
        #self.roll_buffer.append(np.rad2deg(roll_rad))
        self.pitch = np.rad2deg(pitch_rad)
        self.roll = np.rad2deg(roll_rad)

        # Update previous cloud
        self.previous_pc = current_pcd

        # Store results and reset timing
        self.cached_delta_deg = delta_deg
        self.last_icp_time = current_time
        self.accumulated_time = 0.0
        self.cached_heading = self.heading

        return (
            self._calculate_rate(self.yaw_buffer),
            self.pitch,
            self.roll,
            self.heading
        )

    def _calculate_rate(self, buffer):
        """Efficient rate calculation from buffer"""
        return sum(buffer) * (len(buffer)/20.0)  # Normalized to 1sec
    

    def _check_for_stuck_icp(self, delta_deg):
        """Detect and handle stuck ICP states"""
        self.delta_buffer.append(delta_deg)
        
        # Check if we have enough history and cooldown is inactive
        if len(self.delta_buffer) < 10 or self.reset_cooldown > 0:
            self.reset_cooldown = max(0, self.reset_cooldown - 1)
            return False
            
        # Check if all deltas are identical (with small epsilon for float comparison)
        first = self.delta_buffer[0]
        if all(x == first for x in self.delta_buffer):
            print(f"ICP stuck detected! Performing reset... Last 10 values are all equal to {first}!")
            self._reset_icp_state()
            return True
        
    def _reset_icp_state(self):
        """Full ICP state reset"""
        self.last_transform = np.eye(4)
        self.previous_pc = None
        self.delta_buffer.clear()
