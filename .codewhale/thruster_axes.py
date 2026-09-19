import math

D2R = math.pi / 180.0


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    # URDF: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    def mm(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mm(Rz, mm(Ry, Rx))


def axis_local(rpy, axis):
    R = rpy_matrix(*[a * D2R for a in rpy])
    idx = {'x': 0, 'y': 1, 'z': 2}[axis]
    return tuple(R[i][idx] for i in range(3))


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

print("=== x-axis (UUV joint axis convention) ===")
for tid, pos, rpy in thrusters:
    a = axis_local(rpy, 'x')
    print("t%d pos=(%6.3f,%6.3f,%5.3f) axis=(%6.3f,%6.3f,%6.3f) |z|=%.3f" % (tid, *pos, *a, abs(a[2])))

print("=== z-axis convention ===")
for tid, pos, rpy in thrusters:
    a = axis_local(rpy, 'z')
    print("t%d pos=(%6.3f,%6.3f,%5.3f) axis=(%6.3f,%6.3f,%6.3f) |z|=%.3f" % (tid, *pos, *a, abs(a[2])))
