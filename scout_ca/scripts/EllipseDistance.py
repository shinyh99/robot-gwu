import math

def EllipseDistance(semi_major, semi_minor, point):
    px = abs(point[0])
    py = abs(point[1])

    t = 0

    a = semi_major
    b = semi_minor

    for i in range(0, 4):
        print(i)
        x = a * math.cos(t)
        y = b * math.sin(t)
        print(x,y)

        ex = (a**2 - b**2) * math.cos(t)**3 / a
        ey = (b**2 - a**2) * math.sin(t)**3 / b

        rx = x - ex
        ry = y - ey

        qx = px - ex
        qy = py - ey

        r = math.hypot(ry, rx)
        q = math.hypot(qy, qx)

        delta_c = r * math.asin((rx*qy - ry*qx)/(r*q))
        delta_t = delta_c / math.sqrt(a**2 + b**2 - x**2 - y**2)

        t += delta_t
        t = min(math.pi/2, max(0, t))
        print((x, y))

    return (round(math.copysign(x, point[0]), 6), round(math.copysign(y, point[1]), 6))
