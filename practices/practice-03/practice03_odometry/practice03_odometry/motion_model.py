import math


def integrate_pose(x, y, yaw, v, omega, dt):
    new_yaw = yaw + omega * dt

    if abs(omega) < 1e-9:
        new_x = x + v * dt * math.cos(yaw)
        new_y = y + v * dt * math.sin(yaw)
    else:
        radius = v / omega
        new_x = x + radius * (math.sin(new_yaw) - math.sin(yaw))
        new_y = y + radius * (math.cos(yaw) - math.cos(new_yaw))

    return new_x, new_y, new_yaw
