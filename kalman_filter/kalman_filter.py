#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np


class KalmanFilter(object):
    def __init__(self, T=0.1, Q=np.diag([0, 0.25]), R=1) -> None:

        # change in Time, or dT
        self.T = T

        # Array A
        self.A = np.array([[0, 1], [0, 0]])
        self.Ad = np.identity(2) + self.T * self.A

        # Array B
        self.B = np.array([[0], [1]])
        self.Bd = self.T * self.B

        # Array C, or gain term driven by model
        self.C = np.array([[1, 0]])

        # Posteriori state estimate
        # Initialize esitmate position at 0, and estimate velocity at 0
        self.x_hat = np.array([[0], [0]])

        # Posteriori estimate covariance matrix
        self.P = np.diag([10, 10])

        # Covariance of process noise
        self.Q = Q

        # Covariance of observation noise
        self.R = R / self.T

    def prediction(self, u):
        self.x_hat = np.matmul(self.Ad, self.x_hat) + self.Bd * u

        # Posterioiri estimate covariance matrix
        self.P = np.matmul(np.matmul(self.Ad, self.P), self.Ad.T) + self.Q

    def measurement(self, z):

        # Calculate Kalman Gain using R
        S = np.matmul(self.C, np.matmul(self.P, self.C.T)) + self.R
        K = np.matmul(np.matmul(self.P, self.C.T), np.linalg.inv(S))
        self.x_hat = self.x_hat + np.matmul(
            K, (z - np.matmul(self.C, self.x_hat))
        )
        self.P = np.matmul(np.identity(2) - np.matmul(K, self.C), self.P)
