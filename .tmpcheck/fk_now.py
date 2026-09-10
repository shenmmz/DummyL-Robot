import math

A     = [35.0, 146.0, 0.0, 0.0, 0.0, 0.0]
ALPHA = [-90.0, 0.0, 90.0, -90.0, 90.0, 0.0]
D     = [140.0, 0.0, 52.0, 115.0, 0.0, 183.0]
OFF   = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
D2R = math.pi / 180.0

def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def mats(q):
    acc = [[1.0,0,0,0],[0,1.0,0,0],[0,0,1.0,0],[0,0,0,1.0]]
    out = []
    for i in range(6):
        th = (q[i] + OFF[i]) * D2R
        al = ALPHA[i] * D2R
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(al), math.sin(al)
        t = [[ct, -st*ca,  st*sa, A[i]*ct],
             [st,  ct*ca, -ct*sa, A[i]*st],
             [0.0, sa,      ca,   D[i]],
             [0.0, 0.0,     0.0,  1.0]]
        acc = mul(acc, t)
        out.append([row[:] for row in acc])
    return out

def pos(T):
    return T[0][3], T[1][3], T[2][3]

def col(T, c):
    return [T[0][c], T[1][c], T[2][c]]

def rpy(T):
    ry = math.atan2(-T[2][0], math.sqrt(T[0][0]**2 + T[1][0]**2))
    cy = math.cos(ry)
    if abs(cy) < 1e-9:
        rz = math.atan2(T[0][1], T[1][1]); rx = 0.0
    else:
        rx = math.atan2(T[1][0]/cy, T[0][0]/cy)
        rz = math.atan2(T[2][1]/cy, T[2][2]/cy)
    return [math.degrees(v) for v in (rx, ry, rz)]

q = [-180.04, 73.51, -88.04, 6.01, 114.03, 0.0]
T = mats(q)

print("关节角 q =", q)
print()
names = ["J1基座", "J2肩", "J3肘", "J4腕1", "J5腕2", "J6法兰"]
for i, n in enumerate(names):
    x, y, z = pos(T[i])
    print("%s 原点: X=%8.2f Y=%8.2f Z=%8.2f" % (n, x, y, z))

print()
print("末端法兰(TCP): X=%.2f Y=%.2f Z=%.2f" % pos(T[5]))
print("末端水平距离 r = %.2f" % math.hypot(pos(T[5])[0], pos(T[5])[1]))
print("RPY = %.2f %.2f %.2f deg" % tuple(rpy(T[5])))
print()
print("大臂方向 x2 =", ["%.3f" % v for v in col(T[1], 0)])
print("前臂方向 z3 =", ["%.3f" % v for v in col(T[2], 2)])
print("末端轴 z6  =", ["%.3f" % v for v in col(T[5], 2)])
print()
print("大臂与竖直夹角 = %.2f deg" % math.degrees(math.acos(abs(col(T[1],0)[2]))))
print("前臂与水平夹角 = %.2f deg" % math.degrees(math.asin(abs(col(T[2],2)[2]))))
