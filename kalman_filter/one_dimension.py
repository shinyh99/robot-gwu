#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np


class OneDimEnv(object):
    def __init__(self, w_sig=0.5, v_sig=1, T=0.1) -> None:
        # change in Time, or dT
        self.T = T

        # Array A
        self.A = np.array([[0, 1], [0, 0]])
        self.Ad = np.identity(2) + self.T * self.A

        # Array B
        self.B = np.array([[0], [1]])
        self.Bd = self.T * self.B

        # Noise due to process error(예측 모델의 불확실성 노이즈)
        self.w_sig = w_sig
        self.Bw = np.array([[0], [1]])
        self.Bwd = self.T * self.Bw

        # Model driven gain term
        self.C = np.array([[1, 0]])

        # Initialize with position at 0, and velocity at 10
        self.x = np.array([[0], [10]])

        # Noise due to measurement error(측정 모델의 불확실성 노이즈)
        self.v_sig = v_sig

    def internal_step(self, u):
        # u is input to the model
        # x_k = A*x_k-1 + B*u_k + w_k
        # Where w_k is process noise(모델의 변동성에 의한 오류, 예시: 바람, 노면 상태 등) and represented by gaussian distribution N(0, Q)
        # Q가 생성하는 에러
        self.x = (
            np.matmul(self.Ad, self.x)
            + self.Bd * u
            + self.Bwd * self.w_sig * np.random.randn(1)
        )

    # Measure value
    def external_step(self, u):
        # u is input to the model
        # y or z = C*x_k + v_k
        # Where v_k is measurement error(측정모델의 불확실성 노이즈) and represented by gaussian distribution N(0, R)
        # R이 생성하는 에러
        self.internal_step(u)
        z = np.matmul(self.C, self.x) + self.v_sig * np.random.randn(1)
        return z
