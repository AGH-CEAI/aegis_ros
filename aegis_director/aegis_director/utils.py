import numpy as np


def euler_to_quaternion(r: float, p: float, y: float) -> np.ndarray:
    # Source: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
    r /= 2.0
    p /= 2.0
    y /= 2.0
    ci = np.cos(r)
    si = np.sin(r)
    cj = np.cos(p)
    sj = np.sin(p)
    ck = np.cos(y)
    sk = np.sin(y)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q_xyzw = np.empty((4,))
    q_xyzw[0] = cj * sc - sj * cs
    q_xyzw[1] = cj * ss + sj * cc
    q_xyzw[2] = cj * cs - sj * sc
    q_xyzw[3] = cj * cc + sj * ss

    return q_xyzw


def quaternion_to_euler(q_xyzw: np.ndarray) -> np.ndarray:
    # Source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    x, y, z, w = q_xyzw
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return np.array([roll_x, pitch_y, yaw_z])
