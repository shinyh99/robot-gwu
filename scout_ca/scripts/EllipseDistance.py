#!/usr/bin/env python3

import numpy as np


def EllipseDistance(semi_major, semi_minor, rotation, point):
    rot = -rotation
    point_rot_x = point[0] * np.cos(rot) - point[1] * np.sin(rot)
    point_rot_y = point[0] * np.sin(rot) + point[1] * np.cos(rot)
    px = abs(point_rot_x)
    py = abs(point_rot_y)

    t = 0

    a = semi_major
    b = semi_minor

    for i in range(0, 4):
        x = a * np.cos(t)
        y = b * np.sin(t)

        ex = (a ** 2 - b ** 2) * np.cos(t) ** 3 / a
        ey = (b ** 2 - a ** 2) * np.sin(t) ** 3 / b

        rx = x - ex
        ry = y - ey

        qx = px - ex
        qy = py - ey

        r = np.hypot(ry, rx)
        q = np.hypot(qy, qx)

        delta_c = r * np.arcsin((rx * qy - ry * qx) / (r * q))
        delta_t = delta_c / np.sqrt(a ** 2 + b ** 2 - x ** 2 - y ** 2)

        t += delta_t
        t = min(np.pi / 2, max(0, t))

    return [
        round(np.copysign(x, point[0]), 6),
        round(np.copysign(y, point[1]), 6),
    ]


def EllipseEndPoint(semi_major, semi_minor, rotation, point):
    rot = -rotation
    px = point[0] * np.cos(rot) - point[1] * np.sin(rot)
    py = point[0] * np.sin(rot) + point[1] * np.cos(rot)

    px2 = np.power(px, 2)
    py2 = np.power(py, 2)
    py4 = py2 * py2

    zeta = semi_minor
    zeta2 = np.power(zeta, 2)
    zeta3 = np.power(zeta, 3)
    zeta4 = zeta2 * zeta2

    delta = semi_major
    delta2 = np.power(delta, 2)
    delta3 = np.power(delta, 3)
    delta4 = delta2 * delta2
    delta5 = delta2 * delta3

    tmpAlphaTerm = zeta4 * delta4 - 4 * (
        zeta4 * delta3 * px2
        + delta4 * zeta3 * py2
        - delta4 * zeta3 * px2 * py2
        - delta5 * zeta2 * py4
    )
    alphaDen = 2 * (delta * zeta2 * px2 + delta2 * zeta * py2)

    alpha1 = ((zeta2 * delta2) + np.sqrt(tmpAlphaTerm)) / alphaDen
    beta1 = np.sqrt((zeta * delta - zeta * np.power(alpha1, 2)) / delta)
    alpha2 = ((zeta2 * delta2) - np.sqrt(tmpAlphaTerm)) / alphaDen
    beta2 = np.sqrt((zeta * delta - zeta * np.power(alpha2, 2)) / delta)

    return [[alpha1, beta1], [alpha2, beta2]]
