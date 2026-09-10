import math

A     = [35.0, 146.0, 0.0, 0.0, 0.0, 0.0]
ALPHA = [-90.0, 0.0, 90.0, -90.0, 90.0, 0.0]
D     = [140.0, 0.0, 52.0, 115.0, 0.0, 183.0]
D2R = math.pi / 180.0

# 原设计 offset（现表）
OFF_DESIGN = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
# 机械零位对应的上位机角度（由当前姿态反推）
Q0 = [-180.04, 73.51, -178.04, 6.01, 114.03, 0.0]
# 吸收零点偏置后的 theta_offset
OFF = [OFF_DESIGN[i] - Q0[i] for i in range(6)]

def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def mats(q, off):
    acc = [[1.0,0,0,0],[0,1.0,0,0],[0,0,1.0,0],[0,0,0,1.0]]
    out = []
    for i in range(6):
        th = (q[i] + off[i]) * D2R
        al = ALPHA[i] * D2R
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(al), math.sin(al)
        t = [[ct, -st*ca,  st*sa, A[i]*ct],
             [st,  ct*ca, -ct*sa, A[i]*st],
             [0.0, sa,      ca,   D[i]],
             [0.0, 0.0,     0.0,  1.0]]
        acc = mul(acc, t)
        out.append([r[:] for r in acc])
    return out

def col(T, c):
    return [T[0][c], T[1][c], T[2][c]]

def report(tag, q, off):
    T = mats(q, off)
    print("== %s ==" % tag)
    print("  theta_offset =", ["%.2f" % v for v in off])
    for i, n in enumerate(["O1肩", "O2肘", "O3偏", "O4腕心", "O5", "O6法兰"]):
        p = [T[i][0][3], T[i][1][3], T[i][2][3]]
        print("  %s: X=%8.2f Y=%8.2f Z=%8.2f" % (n, p[0], p[1], p[2]))
    x2 = col(T[1], 0); z3 = col(T[2], 2); z6 = col(T[5], 2)
    print("  大臂方向 x2 =", ["%.3f" % v for v in x2], " 与竖直夹角 %.2f deg" % math.degrees(math.acos(min(1, abs(x2[2])))))
    print("  前臂方向 z3 =", ["%.3f" % v for v in z3], " 与水平夹角 %.2f deg" % math.degrees(math.asin(min(1, abs(z3[2])))))
    print("  末端轴  z6 =", ["%.3f" % v for v in z6])
    print()

q_now = [-180.04, 73.51, -88.04, 6.01, 114.03, 0.0]
report("当前上位机角（用原表）", q_now, OFF_DESIGN)
report("当前上位机角（偏置吸收进 offset）", q_now, OFF)
report("机械角 (0,0,90,0,0,0)  + 原表设计 offset", [0, 0, 90, 0, 0, 0], OFF_DESIGN)
report("机械角 (0,0,0,0,0,0) 立正  + 原表设计 offset", [0, 0, 0, 0, 0, 0], OFF_DESIGN)
print("立正姿态对应的上位机角 =", ["%.2f" % v for v in Q0])
