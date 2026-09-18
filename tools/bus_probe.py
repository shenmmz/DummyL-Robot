#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RS485 总线体检探针（独立于主程序，不依赖 robot 库）。

用途：当主程序报"全部关节离线"时，用它区分三种故障：
  A. 总线上还有别的从站在说话（有裸字节流）⇒ 地址/波特率不对
  B. 某个地址在某个波特率下有回应         ⇒ 地址/波特率配置漂移
  C. 所有地址所有波特率都零字节           ⇒ 物理层：掉电 / 断线 / 适配器卡死

用法：
  python tools/bus_probe.py            # 默认 COM4，扫 5 种波特率 × 地址 1..10
  python tools/bus_probe.py COM3       # 指定串口
"""
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial：pip install pyserial")

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUDS = [9600, 19200, 38400, 57600, 115200]
ADDRS = list(range(1, 11))
REG = 0x0066          # 设备识别寄存器（只读，读到即可证明从站活着）
TIMEOUT = 0.15


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def read_holding(addr: int, reg: int, count: int = 1) -> bytes:
    body = bytes([addr, 0x03, (reg >> 8) & 0xFF, reg & 0xFF,
                  (count >> 8) & 0xFF, count & 0xFF])
    return body + crc16(body)


def probe(port: str, baud: int) -> list:
    hits = []
    try:
        s = serial.Serial(port, baudrate=baud, bytesize=8, parity='N',
                          stopbits=1, timeout=TIMEOUT, write_timeout=0.5)
    except Exception as e:
        return [("OPEN_FAIL", str(e))]

    # 1) 先纯听 1.2 s：总线上有没有别人在说话
    s.reset_input_buffer()
    idle = s.read(4096)
    time.sleep(1.2)
    idle += s.read(4096)
    if idle:
        print(f"  [监听] {baud} 空闲期间收到 {len(idle)} 字节: {idle[:32].hex(' ')}")

    # 2) 逐地址探
    for a in ADDRS:
        req = read_holding(a, REG)
        s.reset_input_buffer()
        try:
            s.write(req)
        except Exception as e:
            hits.append((a, f"WRITE_FAIL {e}"))
            continue
        rsp = s.read(64)
        if rsp:
            ok = len(rsp) >= 5 and rsp[0] == a and rsp[1] == 0x03
            hits.append((a, ("OK " if ok else "GARBAGE ") + rsp.hex(' ')))
        else:
            hits.append((a, "----"))
    s.close()
    return hits


def main() -> None:
    print(f"总线探针：{PORT}，扫 {len(BAUDS)} 种波特率 × 地址 {ADDRS[0]}..{ADDRS[-1]}")
    print(f"读取寄存器 0x{REG:04X}（只读，不会让电机动）\n")
    any_hit = False
    for baud in BAUDS:
        print(f"--- {baud} bps ---")
        for item in probe(PORT, baud):
            if item[0] == "OPEN_FAIL":
                print(f"  打开串口失败：{item[1]}")
                return
            a, msg = item
            if msg != "----":
                any_hit = True
                print(f"  地址 {a:2d}: {msg}")
        print(f"  （无输出的地址 = 超时无响应）")
    print("\n结论：")
    if any_hit:
        print("  总线上有从站回应 ⇒ 是【地址/波特率】问题，不是掉电/断线。")
    else:
        print("  所有波特率、所有地址都零字节 ⇒ 物理层断了。按序排查：")
        print("   1) 驱动器 24~80V 主电源是否还通（看驱动器电源灯/风扇）")
        print("   2) RS485 的 A/B 两根线是否松脱（端子排、航空插头）")
        print("   3) USB-RS485 适配器是否卡死 —— 拔掉重插 USB，再看 COM 口号是否变了")
        print("   4) 若换过 COM 口，改 src/config/robot_config.ini 的 [serial] port")


if __name__ == "__main__":
    main()
