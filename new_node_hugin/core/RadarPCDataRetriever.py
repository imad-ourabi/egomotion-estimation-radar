import numpy as np
from new_node_hugin.link_dev import RadarPointCloud

class GetRadarData:
    """
    A class for processing radar point cloud data, including deserialization
    and conversion to Cartesian coordinates.
    """

    @staticmethod
    def get_pointcloud_data(buffer, offset):
        """Retrieve radar point cloud data from the given buffer and offset."""
        pointCloud = RadarPointCloud.RadarPointCloud()
        return pointCloud.GetRootAsRadarPointCloud(buffer.getBuffer(), offset)
    
    @staticmethod
    def get_pointcloud_length(pc_data):
        """Return the length of the point cloud."""
        return pc_data.PcLen()
    
    @staticmethod
    def get_timestamp(pc_data):
        """Retrieve the timestamp of the point cloud."""
        return pc_data.TimestampPc()
    
    @staticmethod
    def deserialize_points_xyz(pc_data):
        """Deserialize the XYZ coordinates from the point cloud data."""
        points_xyz = pc_data.PointsXyz()
        data_bytes = points_xyz.DataBytesAsNumpy()
        scalar_type = points_xyz.ScalarType()
        
        data_type_map = {
            1: np.float32, 2: np.float64, 3: np.int8, 4: np.int16,
            5: np.int32, 6: np.int64, 7: np.uint8, 8: np.uint16,
            9: np.uint32, 10: np.uint64
        }
        
        dtype = data_type_map.get(scalar_type)
        if dtype is not None:
            data = data_bytes.view(dtype)
            rows, cols = points_xyz.Rows(), points_xyz.Cols()
            data = data.reshape((rows, cols))
            print("Min values:", np.min(data, axis=1))
            print("Max values:", np.max(data, axis=1))
        else:
            return 0
    
    @staticmethod
    def deserialize_matrix_data(matrix):
        """Generic method to deserialize matrix data from radar point cloud."""
        matrix_bytes = matrix.DataBytesAsNumpy()
        scalar_type = matrix.ScalarType()
        
        data_type_map = {
            1: np.float32, 2: np.float64, 3: np.int8, 4: np.int16,
            5: np.int32, 6: np.int64, 7: np.uint8, 8: np.uint16,
            9: np.uint32, 10: np.uint64
        }
        
        dtype = data_type_map.get(scalar_type)
        if dtype is not None:
            data = matrix_bytes.view(dtype)
            return data.reshape(matrix.Rows(), matrix.Cols(), matrix.Depth())[0]
        else:
            return 0
    
    @staticmethod
    def deserialize_doppler_data(pc_data):
        """Extract and deserialize Doppler velocity data."""
        return GetRadarData.deserialize_matrix_data(pc_data.Doppler())
    
    @staticmethod
    def deserialize_azimuth_data(pc_data):
        """Extract and deserialize azimuth angle data."""
        return GetRadarData.deserialize_matrix_data(pc_data.Azimuth())
    
    @staticmethod
    def deserialize_elevation_data(pc_data):
        """Extract and deserialize elevation angle data."""
        return GetRadarData.deserialize_matrix_data(pc_data.Elevation())
    
    @staticmethod
    def deserialize_range_data(pc_data):
        """Extract and deserialize range (distance) data."""
        return GetRadarData.deserialize_matrix_data(pc_data.Distance())
    
    @staticmethod
    def deserialize_power_data(pc_data):
        """Extract and deserialize power data."""
        return GetRadarData.deserialize_matrix_data(pc_data.Power())
    
    @staticmethod
    def get_deserialized_lists(pc_data):
        """Retrieve all deserialized lists (azimuth, elevation, Doppler, range, power)."""
        return (
            GetRadarData.deserialize_azimuth_data(pc_data),
            GetRadarData.deserialize_elevation_data(pc_data),
            GetRadarData.deserialize_doppler_data(pc_data),
            GetRadarData.deserialize_range_data(pc_data),
            GetRadarData.deserialize_power_data(pc_data),
        )
    
    @staticmethod
    def calculated_point_cloud(azimuths, elevations, ranges):
        """Calculate Cartesian coordinates (x, y, z) from spherical coordinates."""
        azimuths, elevations, ranges = map(np.array, (azimuths, elevations, ranges))
        x = ranges * np.cos(elevations) * np.sin(azimuths)
        y = ranges * np.cos(elevations) * np.cos(azimuths)
        z = ranges * np.sin(elevations)
        return x, y, z, np.hstack((x, y, z))
