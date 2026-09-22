# -*- coding: utf-8 -*-
"""原始 Modbus RTU 帧发送器（可选工具，不依赖主程序）。

用途：手工验证 J2 减速比时，代替串口助手逐条敲 hex。
注意：本机用【系统 Python】跑（它装了 pyserial 3.5）。

用法：
    # 1) 发一条/多条自定义帧（每条一个引号）
    C:/Users/111/AppData/Local/Programs/Python/Python312/python.exe tools/raw_send.py \
        "02 06 00 D4 00 00 C9 C1" "02 03 00 04 00 02 85 F9"

    # 2) 跑 J2 减速比验证流程：使能→退出连续→设速度→加减速→清零→转 N 度→读回
    ...python.exe tools/raw_send.py --j2 30
    ...python.exe tools/raw_send.py --j2 90      # 仅当 --j2 30 量得 30° 才准做

参数：
    -p COM4        串口（默认 COM4）
    -b 115200      波特率（默认 115200）
    --rpm 100      电机轴转速 rpm（默认 100）
    --no-wait      发完定位帧不等到位就退出（默认会轮询到位置稳定）
"""
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial（系统 Python 3.12 已装）：pip install pyserial")

PORT = "COM4"
BAUD = 115200
TIMEOUT = 0.20
A = 2           # J2 站号
SPR = 10000     # 每转脉冲数（0x0024 实测）


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])


def hexs(b: bytes) -> str:
    return ' '.join('%02X' % x for x in b)


def parse(s: str) -> bytes:
    return bytes(int(x, 16) for x in s.replace(',', ' ').split())


def w16(reg, val):
    body = bytes([A, 0x06, (reg >> 8) & 0xFF, reg & 0xFF, (val >> 8) & 0xFF, val & 0xFF])
    return body + crc16(body)


def w32(reg, val):
    v = val & 0xFFFFFFFF
    lo, hi = v & 0xFFFF, (v >> 16) & 0xFFFF
    body = bytes([A, 0x10, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x02, 0x04,
                  (lo >> 8) & 0xFF, lo & 0xFF, (hi >> 8) & 0xFF, hi & 0xFF])
    return body + crc16(body)


def w2x16(reg, v1, v2):
    body = bytes([A, 0x10, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x02, 0x04,
                  (v1 >> 8) & 0xFF, v1 & 0xFF, (v2 >> 8) & 0xFF, v2 & 0xFF])
    return body + crc16(body)


def rd(reg, n=2):
    body = bytes([A, 0x03, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, n])
    return body + crc16(body)


class Bus:
    def __init__(self, port, baud):
        self.s = serial.Serial(port, baudrate=baud, bytesize=8, parity='N',
                               stopbits=1, timeout=TIMEOUT, write_timeout=0.5)

    def tx(self, tag, frame, wait=0.03):
        ok_crc = frame[-2:] == crc16(frame[:-2])
        self.s.reset_input_buffer()
        self.s.write(frame)
        rsp = self.s.read(64)
        rs = hexs(rsp) if rsp else '(无响应)'
        print('%-28s TX %s' % (tag, hexs(frame)))
        print('%-28s RX %s   %s' % ('', rs,
              '' if ok_crc else '⚠本帧 CRC 自检不通过，别发！'))
        time.sleep(wait)
        return rsp

    def read_pos(self):
        """读 0x0004（INT32，低字在前），返回脉冲数；失败返回 None"""
        rsp = self.tx('读位置 0x0004', rd(0x0004, 2))
        if len(rsp) < 7 or rsp[0] != A or rsp[1] != 0x03 or rsp[2] != 4:
            return None
        lo = (rsp[3] << 8) | rsp[4]
        hi = (rsp[5] << 8) | rsp[6]
        v = (hi << 16) | lo
        return v - (1 << 32) if v >= (1 << 31) else v

    def close(self):
        self.s.close()


def main():
    av = sys.argv[1:]
    port, baud, rpm, wait = PORT, BAUD, 100, True

    if '--j2' in av:
        deg = float(av[av.index('--j2') + 1])
    else:
        deg = None
    if '-p' in av:
        port = av[av.index('-p') + 1]
    if '-b' in av:
        baud = int(av[av.index('-b') + 1])
    if '--rpm' in av:
        rpm = float(av[av.index('--rpm') + 1])
    if '--no-wait' in av:
        wait = False

    frames = [x for x in av if not x.startswith('-')
              and x not in ('COM4',) and ' ' in x and all(c in '0123456789abcdefABCDEF ,' for c in x)]

    bus = Bus(port, baud)
    print('打开 %s @ %d' % (port, baud))

    if deg is None:
        if not frames:
            sys.exit(__doc__)
        for i, f in enumerate(frames, 1):
            bus.tx('帧%d' % i, parse(f))
        bus.close()
        return

    # ---- J2 减速比验证流程 ----
    red = 100.0
    steps_cmd = int(round(deg * red / 360.0 * SPR))
    print('\n目标：J2 转 %+.3f° = %d 步（按 red=100 算）' % (deg, steps_cmd))
    print('速度：电机轴 %.0f rpm → 输出轴 %.2f °/s → 预计 %.1f s\n' % (
        rpm, rpm / red * 6, abs(deg) / (rpm / red * 6)))

    bus.tx('A1 使能', w16(0x00D4, 0))
    bus.tx('A2 退出连续', w16(0x00C8, 0))
    bus.tx('A3 速度', w32(0x00D8, int(round(rpm * 100))))
    bus.tx('A4 加减速', w2x16(0x0098, 150, 200))
    bus.tx('B1 当前位置清零', w32(0x00D2, 0))
    p0 = bus.read_pos()
    print('   清零后位置 = %s 步（应为 0）\n' % p0)

    bus.tx('C 绝对定位 %+d°' % deg, w32(0x00E8, steps_cmd))
    if not wait:
        bus.close()
        return

    print('  等待到位（每 0.5 s 读一次，连续 3 次不变即认为停稳，最多 90 s）...')
    last, stable, t0 = None, 0, time.time()
    while time.time() - t0 < 90:
        time.sleep(0.5)
        p = bus.read_pos()
        if p is None:
            continue
        stable = stable + 1 if p == last else 0
        last = p
        print('    t=%5.1fs  位置=%9d 步  （按 red=100 折算 %+8.3f°）'
              % (time.time() - t0, p, p * 360.0 / (SPR * red)))
        if stable >= 3:
            print('\n  停稳。命令 %+d 步，读回 %d 步 —— 差值 %d 步。' % (steps_cmd, p, p - steps_cmd))
            print('  ⚠ 读回值【不能】用来判定减速比（编码器在电机侧，永远自洽）。')
            print('  ⇒ 请【外部量】实际转角：30°=red100 正确 / 60°=red50 / 15°=red200。')
            break
    bus.close()


if __name__ == '__main__':
    main()
