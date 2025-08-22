# This file is part of project link.developers/new-node-hugin.
# It is copyrighted by the contributors recorded in the version control history of the file,
# available from its original location git@github.com:imad-ourabi/new-node-hugin.git.
#
# SPDX-License-Identifier: MPL-2.0

import sys
import numpy as np
import time

# Link Node Basics
from draive_link2 import LINK2_SIGNAL_INTERRUPT

# Data Preperation Classes
from new_node_hugin.core.NodeHandler import EgoMotionNodeHandler
from new_node_hugin.core.RadarPCDataRetriever import GetRadarData
from new_node_hugin.core.StaticPointCloud import StaticPointCloudProcessor

# Ego Motion Estimator Classes
from new_node_hugin.core.SpeedEstimator import SpeedEstimator
from new_node_hugin.core.KalmanFilter import KalmanFilter
from new_node_hugin.core.Acceleration import Acceleration
from new_node_hugin.core.PointCloudPreprocessor import PointCloudPreprocessor
from new_node_hugin.core.RotationAngles import RotationAnglesEstimator

from new_node_hugin.core.RotationAngles2 import RotationAnglesEstimator2 # NEW

def hugin_main(args):

    node = EgoMotionNodeHandler()

    # Load node resources and announce online
    node_resources, node_discovery = node.prepare_node(args)

    # Input pin with 50 ms of Pull Timeout
    input_pin = node.initialize_input_pin(node_discovery, node_resources, 100) # 50 ms default

    # Output pin
    ego_motion_output_pin = node.initialize_egomotion_output_pin(node_discovery, node_resources)

    # Create a blocking signal handler with 0 ms timeout
    signal_handler = node.handle_signal(0) 


    # Initialize class to retrive Radar Data
    radar = GetRadarData()
    staticPC = StaticPointCloudProcessor()
    speed_class = SpeedEstimator()
    kalman = KalmanFilter()
    acceleration_class = Acceleration()
    pc_processor = PointCloudPreprocessor()
    
    rotation_estimater = RotationAnglesEstimator() # Point-to-Point ICP 

    rotation_estimater2 = RotationAnglesEstimator2() # Generalozed ICP

    # Initialize the Kalman filter state
    x, P, F, H, Q, R, spike_threshold = kalman.initialize_kalman_filter()

    # Initializing variables
    estimated_speed = 0
    measured_acceleration = 0 
    start_time = time.time()
    yaw_rate = 0.0

    # If signal is not shutdown
    while signal_handler.receiveSignal() != LINK2_SIGNAL_INTERRUPT:
        (ok, fb, meta) = input_pin.pull()
        if ok:

            #-------------- DATA PREPARATION & RECEPTION ----------#

            # Retrieve pointcloud data
            pc_data = radar.get_pointcloud_data(fb, 0)

            # Get length of Point Cloud and print it
            pc_length = radar.get_pointcloud_length(pc_data)

            # Get current Timestamp
            time_stamp = radar.get_timestamp(pc_data)

            # Get lists of azimuth, elevation, doppler, range and power as Numpy arrays
            azimuth_list, elevation_list, doppler_list, range_list, power_list = radar.get_deserialized_lists(pc_data)

            # Get xyz pointcloud coordinates using azimuth and elevation angles and distances to points
            pc_x, pc_y, pc_z, pc_points = radar.calculated_point_cloud(azimuth_list, elevation_list, range_list)

            #-------------- STATIC POINTS -------------#

            # Get most common doppler in the PC and its corresponding azimuth and elevation indices
            doppler, indices = staticPC.most_common_doppler_value_and_indices(doppler_list)

            # Get lists of other dimensions filtered by the 'most common doppler' values
            azimuths = staticPC.get_azimuths_of_indices(azimuth_list, indices)
            elevations = staticPC.get_elevations_of_indices(elevation_list, indices)
            ranges = staticPC.get_ranges_of_indices(range_list, indices)

            #-------------- SCALAR VELOCITY -----------#

            # Get speed from doppler, azimuths and elevations
            speed = speed_class.compute_average_vehicle_speed(doppler, azimuths, elevations)
            
            #-------------- VELOCITY VECTOR ------------#
            
            # Get 3D coordinates of the velocity
            vx, vy, vz = speed_class.compute_velocity_vector(doppler, azimuths, elevations)

            # Get speed calculated using the the 3D coordinates of velocity vector
            scalar_speed = np.sqrt(vx**2 + vy**2 + vz**2)

            vx = round(float(vx), 3)
            vy = round(float(vy), 3)
            vz = round(float(vz), 3)

            velocity_vector = np.array([vx, vy, vz])

            #-------------- ACCELERATION ------------#
            
            # Get current time at the current loop iteraion
            current_time = time.time()

            # Get delta_t (difference between the time of current and previous frame)
            delta_t = current_time - start_time

            # Check if delta t greater than 0.1 seconds to calculate the acceleration
            if delta_t >= 0.1:
                measured_acceleration = acceleration_class.calculate_acceleration(estimated_speed, delta_t)
                start_time = current_time  # Reset the timer

            #-------------- KALMAN FILTER FOR SPEED --------------#

            # Apply Kalman filter to the measured speed
            x, P, prediction = kalman.kalman_filter(scalar_speed, x, P, F, H, Q, R, spike_threshold)

            # Extract the filtered speed and acceleration
            estimated_speed, estimated_acceleration = x[0, 0], x[1, 0]
            estimated_speed = estimated_speed - 0.25

            #-------------- PRE-PROCESSING / FILTERING -----------#

            # Pre-process methods
            pre_process_pc = pc_processor.apply_all(pc_points, power_list, doppler_list, doppler)

            #-------------- YAW, PITCH, ROLL -----------#

            # Calculate the heading (updated between 0 and 360 degrees), yaw, pitch and roll of the current frame/loop iteration
            
            # yaw_rate, pitch, roll, heading = rotation_estimater.update(pre_process_pc, estimated_speed, current_time) # for Point-to-Point ICP (RotationAgnles.py)

            yaw_rate, pitch, roll, heading = rotation_estimater2.update(pre_process_pc, estimated_speed, current_time) # for Generalized ICP (RotationAngles2.py)

            #-------------- TERMINAL OUTPUT --------------#

            # Print the results
            print("Power & Doppler information:")
            print(f"Minimum Power = {np.min(power_list):.2f}")
            print(f"Maximum Power = {np.max(power_list):.2f}")
            print(f"Minimum doppler = {np.min(doppler_list)}")
            print(f"Maximum doppler = {np.max(doppler_list)}")
            print(f"Most common doppler = {doppler}")

            print("\nPC length information:")
            print(f"PC Length = {pc_length}")
            print(f"Pre-processed PC Length = {pre_process_pc.size}")

            print("\nVelocity and Acceleration:")
            print(f"Velocity vector : vx = {vx:.2f}, vy = {vy:.2f}, vz = {vz:.2f}")
            print(f"[from Doppler] Measured Speed = {scalar_speed:.2f} m/s (or {(scalar_speed * 3.6):.2f} km/h)")
            print(f"[from Kalman Filter] Estimated Speed = {estimated_speed:.2f} m/s (or {(estimated_speed * 3.6):.2f} km/h)")
            print(f"[from Doopler] Measured Acceleration = {measured_acceleration:.2f} m/s²")
            print(f"[from Kalman Filter] Estimated Acceleration = {estimated_acceleration:.2f} m/s²")
            print(f"Delta T (between current and previous frame): {delta_t:.4f}")

            print("\nRotation Angles:")
            print(f"Heading = {heading:.2f} degrees (accumulated yaw)")
            print(f"Yaw Rate = {yaw_rate:.2f} deg/s")
            print(f"Pitch = {pitch:.2f} degrees")
            print(f"Roll = {roll:.2f} degrees")
            print("-----------------")

            #-------- PUBLISH DATA TO  OUTPUT PINS --------#

            # Publish filtered speed to output         
            node.publish_egomotion_to_output(fb, scalar_speed, estimated_speed, measured_acceleration, estimated_acceleration, yaw_rate, pitch, roll, heading, meta, ego_motion_output_pin)
            
    return 0

def main():
    """
    try:
        return hugin_main(sys.argv)
    except Exception as e:
        print(e, file=sys.stderr)
        return 1
    """

    hugin_main(sys.argv)
    return 1

if __name__ == "__main__":
    exit(main())
