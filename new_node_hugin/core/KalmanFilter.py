import numpy as np

class KalmanFilter:
    """
    A class to encapsulate the Kalman Filter for vehicle speed and acceleration estimation.
    
    This class provides methods to:
      - Initialize the Kalman Filter parameters.
      - Perform a single iteration of the Kalman Filter update.
    """
    
    @staticmethod
    def initialize_kalman_filter():
        """
        Initializes the Kalman filter state, matrices, and parameters.
        
        Returns:
            x (numpy.ndarray): Initial state vector (2x1 array for [speed, acceleration]).
            P (numpy.ndarray): Initial state covariance matrix (2x2 array).
            F (numpy.ndarray): State transition matrix (2x2 array).
            H (numpy.ndarray): Measurement matrix (1x2 array).
            Q (numpy.ndarray): Process noise covariance matrix (2x2 array).
            R (numpy.ndarray): Measurement noise covariance matrix (1x1 array).
            spike_threshold (float): Threshold for spike detection.
        """
        # Initial state (speed, acceleration)
        x = np.array([[0], [0]], dtype=float)

        # Initial state covariance matrix
        P = np.eye(2) * 10

        # State transition matrix (constant velocity model during one time step)
        F = np.array([[1, 0.08], [0, 1]], dtype=float)

        # Measurement matrix (we only measure speed)
        H = np.array([[1, 0]], dtype=float)

        # Process noise covariance matrix
        Q = np.array([[0.000001, 0], [0, 0.007]], dtype=float)

        # Measurement noise covariance matrix
        R = np.array([[0.1]], dtype=float)

        # Spike detection threshold
        spike_threshold = 2

        return x, P, F, H, Q, R, spike_threshold

    @staticmethod
    def kalman_filter(measurement, x, P, F, H, Q, R, spike_threshold=None):
        """
        Perform one iteration of a Kalman filter update.
        
        Parameters:
            measurement (float): The current noisy measurement.
            x (numpy.ndarray): The current state estimate (2D array, shape [2, 1]).
            P (numpy.ndarray): The current state covariance matrix (2D array, shape [2, 2]).
            F (numpy.ndarray): The state transition matrix (2D array, shape [2, 2]).
            H (numpy.ndarray): The measurement matrix (2D array, shape [1, 2]).
            Q (numpy.ndarray): The process noise covariance matrix (2D array, shape [2, 2]).
            R (numpy.ndarray): The measurement noise covariance matrix (scalar or 2D array).
            spike_threshold (float, optional): Threshold to detect and ignore spikes in measurements.
        
        Returns:
            updated_x (numpy.ndarray): The updated state estimate (2D array, shape [2, 1]).
            updated_P (numpy.ndarray): The updated state covariance matrix (2D array, shape [2, 2]).
            prediction (numpy.ndarray): The predicted state before the update (2D array, shape [2, 1]).
        """
        # Prediction step
        predicted_x = np.dot(F, x)
        predicted_P = np.dot(np.dot(F, P), F.T) + Q

        # Compute the expected measurement and residual
        predicted_measurement = np.dot(H, predicted_x)
        residual = measurement - predicted_measurement

        # Spike detection: skip update if the residual is too high
        if spike_threshold is not None and abs(residual[0, 0]) > spike_threshold:
            # Uncomment the following line for debugging spikes
            # print(f"Spike detected: {measurement} (residual: {residual[0, 0]:.2f})")
            return predicted_x, predicted_P, predicted_x

        # Update Step: Compute the Kalman gain and update state estimate
        S = np.dot(np.dot(H, predicted_P), H.T) + R  # Residual covariance
        K = np.dot(np.dot(predicted_P, H.T), np.linalg.inv(S))  # Kalman gain

        updated_x = predicted_x + np.dot(K, residual)
        updated_P = predicted_P - np.dot(np.dot(K, H), predicted_P)

        return updated_x, updated_P, predicted_x
