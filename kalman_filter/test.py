# %%
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

## One dimesion test
from one_dimension import OneDimEnv
from kalman_filter import KalmanFilter

import numpy as np
import matplotlib

matplotlib.use("TkAgg")

from matplotlib import pyplot as plt


x_hat_hist = []
P_hist = []
z_hist = []
v_actual = []
env = OneDimEnv()
estimator = KalmanFilter()

# input to the system.
# in this scenario, it is acceleration
u = np.concatenate([np.ones((30,)), np.zeros((40,)), -0.5 * np.ones((30,))])
for i in range(100):
    # # Measure value
    # env steps
    z = env.external_step(u[i])

    # # State estimation
    # filter prediction
    estimator.prediction(u[i])

    # filter correction
    estimator.measurement(z)
    # record the process
    x_hat_hist.append(estimator.x_hat)
    P_hist.append(estimator.P)
    z_hist.append(z)
    v_actual.append(env.x[1, :])

# state estimation
pos_hat = np.matmul(estimator.C, np.hstack(x_hat_hist)).squeeze(0)
vel_hat = np.hstack(x_hat_hist)[1, :]


sig_p = np.sqrt(np.vstack(P_hist).reshape([-1, 2, 2])[:, 0, 0])
sig_v = np.sqrt(np.vstack(P_hist).reshape([-1, 2, 2])[:, 1, 1])
z_np = np.hstack(z_hist).squeeze()
v_np = np.hstack(v_actual).squeeze()
con = 3
plt.figure(figsize=(20, 20))
plt.subplot(211)
plt.plot(np.arange(0, 10, 0.1), pos_hat, "k-")
plt.plot(np.arange(0, 10, 0.1), pos_hat - con * sig_p, "r--")
plt.plot(np.arange(0, 10, 0.1), pos_hat + con * sig_p, "r--")
plt.plot(np.arange(0, 10, 0.1), z_np, "b-")
plt.legend(
    [
        "position_estimate",
        "position_estimate-3sigma",
        "position_estimate+3sigma",
        "measured position",
    ]
)
plt.ylabel("position")
plt.xlabel("time")

plt.subplot(212)
plt.plot(np.arange(0, 10, 0.1), vel_hat, "k-")
plt.plot(np.arange(0, 10, 0.1), vel_hat - con * sig_v, "r--")
plt.plot(np.arange(0, 10, 0.1), vel_hat + con * sig_v, "r--")
plt.plot(np.arange(0, 10, 0.1), v_np, "b-")
plt.plot(np.arange(0, 9.9, 0.1), (z_np[1:] - z_np[:-1]) / 0.1, "g-")
plt.legend(
    [
        "velocity_estimate",
        "velocity_estimate-3sigma",
        "velocity_estimate+3sigma",
        "Actual velocity",
        "Velocity derived from position",
    ]
)
plt.ylabel("velocity")
plt.xlabel("time")
plt.show()

# %%
