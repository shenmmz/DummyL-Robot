# -*- coding: utf-8 -*-
"""J2 单轴转动 → 末端（法兰）位移预测表。

用途：验证 J2 减速比时，如果手上没有量角器，可以改用【卷尺量末端位移】反推转角。
其他五轴保持在 home 角度不动（驱动器使能保持），只有 J2 转。

用法：
    python tools/j2_lift_check.py
"""
import math

# 与 src/kinematics/dh.c 的 DH_TABLE 逐位一致
#   a(mm), alpha(deg), d(mm), theta_offset(deg)
DH = [
    (35.0, -90.0, 140.0,  0.0),   # J1
    (146.0,   0.0,   0.0, -90.0),  # J2  home=-90°
    (0.0,    90.0,  52.0,  90.0),  # J3  home=90°
    (0.0,   -90.0, 115.0,   0.0),  # J4
    (0.0,    90.0,   0.0,   0.0),  # J5
    (0.0,     0.0,  91.5,   0.0),  # J6  d6 = 91.5（无工具）
]

HOME_Q = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]   # home 关节角


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def fk(q):
    """标准 DH：T_i = Rot(z,θ) · Trans(z,d) · Trans(x,a) · Rot(x,α)"""
    T = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
    for i, (a, alpha, d, off) in enumerate(DH):
        th = math.radians(q[i] + off)
        al = math.radians(alpha)
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(al), math.sin(al)
        Ti = [
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,      sa,       ca,     d],
            [0.0,     0.0,     0.0,   1.0],
        ]
        T = mul(T, Ti)
    return T[0][3], T[1][3], T[2][3]


def main():
    x0, y0, z0 = fk(HOME_Q)
    print('home 校验：q=%s -> X=%.2f Y=%.2f Z=%.2f   （程序实测 241.50 / 52.00 / 286.00）'
          % (HOME_Q, x0, y0, z0))
    print()
    print('J2 单轴转动的末端位移预测（其余五轴保持 home 角度不动）')
    print('%-8s %-26s %-14s %-14s' % ('J2转角', '末端 XYZ (mm)', '相对起点位移', '位移分量'))
    print('-' * 78)
    rows = []
    for deg in (-90, -60, -30, -15, 0, 15, 30, 45, 60, 90):
        q = list(HOME_Q)
        q[1] = deg
        x, y, z = fk(q)
        d = math.dist((x, y, z), (x0, y0, z0))
        rows.append((deg, x, y, z, d))
        print('  %+5.0f°  %7.2f %7.2f %7.2f   %8.2f mm    dX=%+.2f dY=%+.2f dZ=%+.2f'
              % (deg, x, y, z, d, x - x0, y - y0, z - z0))

    TABLE_H = 72.83   # 纸面世界高度（实测）；低于它就是撞桌
    print()
    print('⚠ 碰撞检查：Z < %.2f mm 就是撞桌面/纸面' % TABLE_H)
    for deg in (-90, -72, -60, -30, -15, 0, 15, 30, 45, 60, 90):
        q = list(HOME_Q)
        q[1] = deg
        x, y, z = fk(q)
        mark = ''
        if z < TABLE_H:
            mark = '  ✗ 撞桌面（低 %.1f mm）' % (TABLE_H - z)
        elif deg < -72 or deg > 90:
            mark = '  ✗ 越软限位'
        print('   J2=%+5.0f°  Z=%7.2f%s' % (deg, z, mark))

    print()
    print('=' * 78)
    print('★ 推荐方案：从 home 出发下发【−30°】（负方向 = 末端【上升】，撞不到东西）')
    print('=' * 78)
    print('  下发 −30°（−83333 步），用卷尺量末端【离桌面的高度】变化：')
    print('  %-22s %-14s %-14s %s' % ('真实减速比', '实际转角', '末端升高', '末端水平位移'))
    for real, red_real in ((15.0, 200), (30.0, 100), (60.0, 50)):
        q = list(HOME_Q)
        q[1] = -real
        x, y, z = fk(q)
        d = math.dist((x, y, z), (x0, y0, z0))
        print('  red=%-18d %-14s %-14s %.2f mm'
              % (red_real, '−%.0f°' % real, '**+%.2f mm**' % (z - z0), d))
    print()
    print('  判据（量【升高了多少】，卷尺 ±2mm 足够）：')
    print('     升高 ~ %.0f mm  ⇒ red=100 正确' % (fk([0, -30, 90, 0, 0, 0])[2] - z0))
    print('     升高 ~ %.0f mm  ⇒ 真实 red=50（实际转角 = 程序 2 倍）'
          % (fk([0, -60, 90, 0, 0, 0])[2] - z0))
    print('     升高 ~ %.0f mm  ⇒ 真实 red=200（实际转角 = 程序 1/2）'
          % (fk([0, -15, 90, 0, 0, 0])[2] - z0))
    print()
    print('  ⚠ 反过来【+30°】不行：末端会下降，转 30° 降 %.0f mm（还行），'
          % (z0 - fk([0, 30, 90, 0, 0, 0])[2]))
    print('     但若减速比真错了 2 倍就是转 60°，末端降到 Z=%.2f ⇒ **撞桌面**。'
          % fk([0, 60, 90, 0, 0, 0])[2])
    print('  ⇒ 所以统一用负方向，三个可能结果都是往上抬，撞不了任何东西。')
    print('  ⇒ 需要预留空间：末端会往 −X 方向移动最多 %.0f mm（转 60° 时），机械臂后方要空出来。'
          % abs(fk([0, -60, 90, 0, 0, 0])[0] - x0))


if __name__ == '__main__':
    main()
