#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 DummyL-Robot 的 `movel` 命令序列（落笔 → 走形状 → 抬笔）。

用法：
    python tools/gen_shape.py circle --cx 175 --cy 27 --r 25 --seg 24
    python tools/gen_shape.py square --x0 150 --y0 2 --size 50
    python tools/gen_shape.py circle ... | 少一个参数就走下面的默认值

输出直接可以喂给控制台：
    python tools/gen_shape.py circle | ./build/bin/dummyrobot.exe

坐标系与位姿：{X,Y,Z,Rx,Ry,Rz}，姿态默认 (-180,0,-180)（法兰朝下、笔竖直）。
笔：Z_PEN_DOWN 落笔高度，Z_PEN_UP 抬起高度（默认 114 / 116，与本机一致）。
"""
import argparse
import math
import sys

Z_DOWN = 114.0
Z_UP = 116.0
RPY = "-180,0,-180"
FEED = "70,80,90"   # speed, acc, dec


def movel(x, y, z):
    return f"movel:{x:.2f},{y:.2f},{z:.2f},{RPY},{FEED}"


def emit(points, close=True):
    """points: [(x,y), ...] 落笔后依次经过的点"""
    out = []
    x0, y0 = points[0]
    # 先抬着笔移到起点正上方，再垂直落笔。
    # 少了这一步，机械臂会从当前位置斜着插到起点并同时降 Z —— 笔一路拖过去，
    # 图形上会多出一道不应该有的划痕。
    out.append(movel(x0, y0, Z_UP))
    out.append(movel(x0, y0, Z_DOWN))          # 垂直落笔
    for (x, y) in points[1:]:
        out.append(movel(x, y, Z_DOWN))
    if close and len(points) > 2:
        out.append(movel(x0, y0, Z_DOWN))      # 闭合
    out.append(movel(x0, y0, Z_UP))            # 抬笔
    out.append("getpos")
    out.append("exit")
    return out


def circle(cx, cy, r, seg):
    pts = []
    for i in range(seg):
        a = 2.0 * math.pi * i / seg
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def square(x0, y0, size):
    return [(x0, y0), (x0, y0 + size),
            (x0 + size, y0 + size), (x0 + size, y0)]


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="shape", required=True)

    c = sub.add_parser("circle")
    c.add_argument("--cx", type=float, default=175.0)
    c.add_argument("--cy", type=float, default=27.0)
    c.add_argument("--r", type=float, default=25.0)
    c.add_argument("--seg", type=int, default=24)

    s = sub.add_parser("square")
    s.add_argument("--x0", type=float, default=150.0)
    s.add_argument("--y0", type=float, default=2.0)
    s.add_argument("--size", type=float, default=50.0)

    a = ap.parse_args()

    if a.shape == "circle":
        pts = circle(a.cx, a.cy, a.r, a.seg)
        # 多边形逼近的弓高（弦到弧的最大距离），画之前就知道误差量级
        sag = a.r * (1.0 - math.cos(math.pi / a.seg))
        print(f"# 圆：圆心({a.cx},{a.cy}) r={a.r} {a.seg} 段 "
              f"弦长 {2*a.r*math.sin(math.pi/a.seg):.2f}mm "
              f"多边形弓高 {sag:.3f}mm", file=sys.stderr)
    else:
        pts = square(a.x0, a.y0, a.size)
        print(f"# 正方形：({a.x0},{a.y0}) 边长 {a.size}", file=sys.stderr)

    for line in emit(pts):
        print(line)


if __name__ == "__main__":
    main()
