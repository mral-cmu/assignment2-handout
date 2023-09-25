"""Estimation using Dead Reckoning, KF, and EKF for 16-362: Mobile Robot Algorithms Laboratory
"""

import numpy as np


class Estimator:
    """Estimator object containing various methods to estimate state of a differential
    drive robot.

    Attributes:
        r: (float) Radius of the wheels of the robot.
        L: (float) Distance between the two wheels.
        dt: (float) Timestep size.
    """

    def __init__(self, dt=0.04, radius=0.033, axle_length=0.16):
        self.r = radius
        self.L = axle_length
        self.dt = dt

    def dead_reckoning(self, x, u):
        """Estimate the next state using the previous state x and the control
        input u via Dead Reckoning.

        Args:
            x: (np.array for 3 floats) State at the previous timestep
            u: (np.array for 2 floats) Control input to the left (u[0]) and right (u[1]) wheels

        Returns:
            next_x: (np.array of 3 floats) State at the current timestep
        """
        # TODO: Assignment 2, Problem 2.1
        raise NotImplementedError

    def kalman_filter(self, x, P, u, y, Q, R):
        """Localization via Kalman Filtering.

        Estimate the next state and the associated uncertainty from the previous state x
        and uncertainty P, control input u, observation y, process noise covariance Q, and
        measurement noise covariance R. Note that in this case we are ignoring heading state (ψ).

        Args:
            x: (np.array for 2 floats) State at the previous timestep
            P: (np.array of floats, 2x2) Uncertainty in the state at the previous timestep
            u: (np.array for 2 floats) Control input to the left (u[0]) and right (u[1]) wheels
            y: (np.array for 2 floats) GPS observation of the position (x, y)
            Q: (np.array of floats, 2x2) Process model noise covariance matrix
            R: (np.array of floats, 2x2) Measurement model noise covariance matrix

        Returns:
            (next_x, next_P): Tuple(np.array of 2 floats, np.array of floats with shape 2x2)
                              Next state vector and covariance matrix
        """
        # TODO: Assignment 2, Problem 2.2
        raise NotImplementedError

    def extended_kalman_filter(self, x, P, u, y, Q, R):
        """Localization via Extended Kalman Filtering.

        Estimate the next state and the associated uncertainty from the previous state x
        and uncertainty P, control input u, observation y, process noise covariance Q, and
        measurement noise covariance R. Note that in this case we are not ignoring heading state (ψ).

        Args:
            x: (np.array for 3 floats) State at the previous timestep
            P: (np.array of floats, 3x3) Uncertainty in the state at the previous timestep
            u: (np.array for 2 floats) Control input to the left (u[0]) and right (u[1]) wheels
            y: (np.array for 2 floats) GPS observation of the position (x, y)
            Q: (np.array of floats, 3x3) Process model noise covariance matrix
            R: (np.array of floats, 2x2) Measurement model noise covariance matrix

        Returns:
            (next_x, next_P): Tuple(np.array of 3 floats, np.array of floats with shape 3x3)
                              Next state vector and covariance matrix
        """
        # TODO: Assignment 2, Problem 2.3
        raise NotImplementedError
