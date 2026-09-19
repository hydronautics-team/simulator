import math
import numpy as np

D2R = math.pi / 180.0

def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

thrusters = [
    (0, (-0.890895, 0.334385, 0.528822), (0, -74.53, -53.21)),
    (1, (-0.890895, -0.334385, 0.528822), (0, -74.53, 53.21)),
    (2, (0.890895, 0.334385, 0.528822), (0, -105.47, 53.21)),
    (3, (0.890895, -0.334385, 0.528822), (0, -105.47, -53.21)),
    (4, (-0.412125, 0.505415, 0.129), (0, 0, 45)),
    (5, (-0.412125, -0.505415, 0.129), (0, 0, -45)),
    (6, (0.412125, 0.505415, 0.129), (0, 0, 135)),
    (7, (0.412125, -0.505415, 0.129), (0, 0, -135)),
]

n = len(thrusters)
T = np.zeros((6, n))
for i, (tid, pos, rpy) in enumerate(thrusters):
    R = rpy_matrix(*[a * D2R for a in rpy])
    axis = R @ np.array([1.0, 0.0, 0.0])  # local x
    p = np.array(pos)
    torque = np.cross(p, axis)
    T[:, i] = np.concatenate([axis, torque])

print("singular values:", np.linalg.svd(T, compute_uv=False))
pinv = np.linalg.pinv(T, rcond=1e-4)

for wrench in [[1500, 0, 0, 0, 0, 0], [0, 0, 1500, 0, 0, 0], [0, 0, 0, 0, 0, 1000]]:
    target = pinv @ np.array(wrench, dtype=float)
    achieved = T @ target
    print("\nwrench", wrench)
    print("target forces:", np.round(target, 2))
    print("achieved:     ", np.round(achieved, 2))
