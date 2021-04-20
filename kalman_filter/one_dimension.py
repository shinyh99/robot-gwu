#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np


class OneDimEnv(object):
    def __init__(self, w_sig=0.5, v_sig=1, T=0.1) -> None:
        self.T = T
        self.A = np.array([[0, 1], [0, 0]])
        self.B = np.array([[0], [1]])

        self.Bw = np.array([[0], [1]])

        self.Ad = np.identity(2) + self.T * self.A
        self.Bd = self.T * self.B
        self.Bwd = self.T * self.Bw

        self.C = np.array([[1, 0]])

        self.x = np.array([[0], [10]])

        self.w_sig = w_sig
        self.v_sig = v_sig

    def internal_step(self, u):
        # Q가 잡아줘야하는 에러
        self.x = (
            np.matmul(self.Ad, self.x)
            + self.Bd * u
            + self.Bwd * self.w_sig * np.random.randn(1)
        )

    def external_step(self, u):
        # V가 잡아줘야하는 에러
        self.internal_step(u)
        z = np.matmul(self.C, self.x) + self.v_sig * np.random.randn(1)
        return z
