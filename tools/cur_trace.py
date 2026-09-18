#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单轴电流原始追踪（诊断用）—— 把 0x001A 在运动期间的【原始读数】打出来。

为什么需要它：CLI 的 curtest 只给统计量（最小/均值/最大），一旦某轴的电流
读数本身有问题（比如运动中恒为 0），统计量看不出"为什么"，只能看到结果异常。
本脚本不做任何过滤，逐帧打印 电流 / 位置 / 状态字，用来判断是
  (a) 电机真的没动     —— 位置计数不变
  (b) 驱动器报警/失能   —— 状态字 bit21/bit16
  (c) 电流寄存器本身异常 —— 位置在变但电流恒 0

用法：
    python tools/cur_trace.py 5            # 对 5 号站，摆 ±2°(2777 脉冲)
    python tools/cur_trace.py 5 --pulses 1000 --rpm 60

默认幅度很小（2777 脉冲 = 关节 2°，减速比 50 / 10000 细分口径），
且【必须先确认末端已抬起】。位置读数超过合理范围直接拒绝运行（安全闸门）。
"""
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial：pip install pyserial")

PORT = "COM4"
BAUD = 115200
TIMEOUT = 0.25

REG_POS = 0x0004      # INT32 实时位置（脉冲）
REG_STATUS = 0x0006   # UINT32 状态字
REG_CURRENT = 0x001A  # UINT16 实时电流 mA
REG_ALARM = 0x00A3    # 报警代码
REG_VEL = 0x00D8      # INT32 运行速度（0.01 rpm）
REG_ABS = 0x00E8      # INT32 绝对运动目标


def crc16(d: bytes) -> bytes:
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return bytes([c & 0xFF, (c >> 8) & 0xFF])


class Drv:
    def __init__(self, addr):
        self.addr = addr
        self.s = serial.Serial(PORT, baudrate=BAUD, bytesize=8, parity='N',
                               stopbits=1, timeout=TIMEOUT, write_timeout=0.5)

    def tx(self, req):
        self.s.reset_input_buffer()
        self.s.write(req)
        return self.s.read(128)

    def read(self, reg, count=1):
        body = bytes([self.addr, 0x03, (reg >> 8) & 0xFF, reg & 0xFF,
                      (count >> 8) & 0xFF, count & 0xFF])
        r = self.tx(body + crc16(body))
        if not r or len(r) < 3 or r[0] != self.addr or r[1] != 0x03:
            return None
        n = r[2]
        if len(r) < 3 + n + 2:
            return None
        return r[3:3 + n]

    def read_u16(self, reg):
        d = self.read(reg)
        return None if not d else (d[0] << 8) | d[1]

    def read_i32(self, reg):
        """低字在先：低字=(d0<<8)|d1，高字=(d2<<8)|d3"""
        d = self.read(reg, 2)
        if not d or len(d) < 4:
            return None
        lw = (d[0] << 8) | d[1]
        hw = (d[2] << 8) | d[3]
        v = (hw << 16) | lw
        return v - (1 << 32) if v >= (1 << 31) else v

    def read_u32(self, reg):
        v = self.read_i32(reg)
        return None if v is None else (v & 0xFFFFFFFF)

    def write_i32(self, reg, val):
        """写 32 位有符号整数到 reg(低字) + reg+1(高字)。
         * 【2026-09-18 事故】曾经把高字写在 reg、低字写在 reg+1，
           与 read_i32 的"首个寄存器=低字"约定相反 ⇒ 下发的目标位置变成
           3.28 亿这种荒谬值，J5 直接顶死并触发相位过流，把电源拉垮、
           六轴驱动器一起掉电重启、零点全丢。
         * 字序必须与 read_i32 严格对称：reg=低字，reg+1=高字。"""
        if abs(val) > 2_000_000_000:
            raise ValueError(f"写入值 {val} 超出合理范围，拒绝下发")
        v = val & 0xFFFFFFFF
        lo = v & 0xFFFF
        hi = (v >> 16) & 0xFFFF
        n = 2
        body = bytes([self.addr, 0x10, (reg >> 8) & 0xFF, reg & 0xFF,
                      0x00, n, n * 2,
                      (lo >> 8) & 0xFF, lo & 0xFF,
                      (hi >> 8) & 0xFF, hi & 0xFF])
        r = self.tx(body + crc16(body))
        return bool(r)

    def close(self):
        self.s.close()


def flag_text(st):
    if st is None:
        return "读失败"
    bits = []
    run = (st >> 8) & 0x03
    bits.append(["run=空闲", "run=启动", "run=停止", "run=运行"][run])
    if st & (1 << 12):
        bits.append("到位")
    if st & (1 << 16):
        bits.append("使能")
    if st & (1 << 21):
        bits.append("报警")
    if st & (1 << 10):
        bits.append("超差")
    return " ".join(bits)


def trace(d, label, seconds):
    print(f"--- {label} ---")
    print("   t(ms)   电流mA    位置脉冲      状态字 0x%08s  标志" % "")
    t0 = time.time()
    rows = []
    while time.time() - t0 < seconds:
        cur = d.read_u16(REG_CURRENT)
        pos = d.read_i32(REG_POS)
        st = d.read_u32(REG_STATUS)
        el = int((time.time() - t0) * 1000)
        rows.append((el, cur, pos, st))
        print(f"  {el:6d}  {str(cur):>7}  {str(pos):>10}  "
              f"{('0x%08X' % st) if st is not None else '    None '}  {flag_text(st)}")
    return rows


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    addr = int(sys.argv[1])
    pulses = 2777
    rpm = 60
    for i, a in enumerate(sys.argv):
        if a == "--pulses":
            pulses = int(sys.argv[i + 1])
        if a == "--rpm":
            rpm = int(sys.argv[i + 1])

    d = Drv(addr)
    print(f"=== 站号 {addr} 电流原始追踪（摆 {pulses} 脉冲，{rpm} rpm）===\n")

    pos0 = d.read_i32(REG_POS)
    if pos0 is None:
        sys.exit("读不到当前位置，退出")
    # 【安全闸门】真实位置不会是几亿这种量级（曾因字节序读错出过事故）
    if abs(pos0) > 50_000_000:
        sys.exit(f"位置读数 {pos0} 明显不合理，拒绝继续。")
    print(f"当前位置 = {pos0} pulses")
    print(f"报警 0x00A3 = {d.read_u16(REG_ALARM)}")
    print(f"状态字     = 0x{d.read_u32(REG_STATUS):08X}  {flag_text(d.read_u32(REG_STATUS))}\n")

    # 【安全闸门】目标必须离当前位置很近。
    # 没有这道闸，字序/换算一旦写错就会下发几亿脉冲的目标，轴直接顶死。
    MAX_PULSES = 5000          # ≈ 关节 3.6°（50 减速比 / 10000 细分口径）
    if abs(pulses) > MAX_PULSES:
        sys.exit(f"摆幅 {pulses} 脉冲超过安全上限 {MAX_PULSES}，拒绝运行。")
    if rpm <= 0 or rpm > 300:
        sys.exit(f"转速 {rpm} rpm 不合理，拒绝运行。")

    # 静止基线
    trace(d, "静止（使能保持）", 0.8)

    # 设速度并走 +pulses（写之前再读一次当前位置，避免用了过期读数）
    pos_now = d.read_i32(REG_POS)
    if pos_now is None:
        sys.exit("写目标前读不到当前位置，放弃")
    if abs(pos_now - pos0) > 200:
        sys.exit(f"静止期间位置自己变了（{pos0} → {pos_now}），有外力/未使能，放弃")
    print(f"\n写 0x00D8 = {rpm * 100}（{rpm} rpm）: {'OK' if d.write_i32(REG_VEL, rpm * 100) else '失败'}")
    target = pos_now + pulses
    print(f"写 0x00E8 = {target}（{pos_now}{pulses:+d}）: {'OK' if d.write_i32(REG_ABS, target) else '失败'}")
    print(f"   读回 0x00E8/0x00E9 = {d.read_i32(REG_ABS)}（必须 ≈ {target}，否则字序又错了）")
    mv = trace(d, f"运动中 → {target}", 2.0)

    # 回原位
    print(f"\n写 0x00E8 = {pos0}（回原位）: {'OK' if d.write_i32(REG_ABS, pos0) else '失败'}")
    back = trace(d, f"运动中 → {pos0}", 2.0)

    time.sleep(0.5)
    pos_end = d.read_i32(REG_POS)
    print(f"\n=== 结论 ===")
    print(f"起始位置 {pos0} → 结束位置 {pos_end}（Δ={None if pos_end is None else pos_end - pos0}）")
    moved = [p for (_, _, p, _) in mv if p is not None]
    if moved:
        print(f"去程位置范围 {min(moved)} .. {max(moved)}（跨度 {max(moved) - min(moved)} 脉冲）")
    else:
        print("去程位置全部读取失败")
    curs = [c for (_, c, _, _) in mv if c is not None]
    print(f"去程电流范围 {min(curs) if curs else '-'} .. {max(curs) if curs else '-'} mA")
    print(f"结束报警 0x00A3 = {d.read_u16(REG_ALARM)}")
    print("\n判读：")
    print("  位置跨度 ≈ 摆幅 且 电流正常 ⇒ 该轴读数是可信的")
    print("  位置跨度 ≈ 0                ⇒ 指令没被执行（失能/报警/限位），电流 0 是结果不是原因")
    print("  位置跨度正常 但 电流恒 0     ⇒ 0x001A 在该轴运动期间不可信，堵转检测在此轴失效")
    d.close()


if __name__ == "__main__":
    main()
