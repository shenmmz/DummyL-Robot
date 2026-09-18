#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LEESN 寄存器只读探针（独立于主程序，不下发任何运动命令）。

用法：
    python tools/reg_probe.py 1 0x0015 0x009F 0x00AA    # 读 1 号站的几个寄存器
    python tools/reg_probe.py 1 300 301 302 303         # 读编程区（地址可用十进制）

每个寄存器按 UINT16 读一次；对成对出现的 INT32 会额外按有符号 32 位再解释一遍。
只读，安全。
"""
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial：pip install pyserial")

PORT = "COM4"
BAUD = 115200
TIMEOUT = 0.20


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def read_one(s, addr, reg):
    body = bytes([addr, 0x03, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x01])
    req = body + crc16(body)
    s.reset_input_buffer()
    s.write(req)
    rsp = s.read(64)
    if len(rsp) < 5 or rsp[0] != addr or rsp[1] != 0x03:
        return None
    n = rsp[2]
    if len(rsp) < 3 + n + 2:
        return None
    val = (rsp[3] << 8) | rsp[4]
    return val


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    addr = int(sys.argv[1])
    regs = [int(x, 0) for x in sys.argv[2:]]

    s = serial.Serial(PORT, baudrate=BAUD, bytesize=8, parity='N',
                      stopbits=1, timeout=TIMEOUT, write_timeout=0.5)
    print(f"站号 {addr} @ {PORT} {BAUD}")
    print(f"{'地址':>10}  {'UINT16':>8}  {'HEX':>6}   {'INT32(与下一寄存器合并)':}")
    for r in regs:
        v = read_one(s, addr, r)
        if v is None:
            print(f"0x{r:04X}({r:4d})  {'--':>8}  {'--':>6}   超时/无响应")
            continue
        s32 = ""
        v2 = read_one(s, addr, r + 1)
        if v2 is not None:
            word_swapped = (v2 << 16) | v          # 手册：高字在后
            signed = word_swapped - (1 << 32) if word_swapped >= (1 << 31) else word_swapped
            s32 = f"{signed}"
        print(f"0x{r:04X}({r:4d})  {v:8d}  {v:04X}h   {s32}")
        time.sleep(0.01)
    s.close()


if __name__ == "__main__":
    main()
