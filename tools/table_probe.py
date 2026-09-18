#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LEESN 位置表（手册 §54）机制验证 —— 【不动臂】版本。

原理：往表格里写 N 个【等于当前位置】的目标。因为目标就是当前位置，
触发执行后电机不会动，但我们可以观察：
  - 表指针 0x00AB 会不会按"表指针变化常数"自动推进
  - 位置有没有真的不动（验证安全）
  - 驱动有没有报错（0x00A3）
这样就能在不让机械臂动一下的前提下，确认这套机制是不是真的可用。

用法：
    python tools/table_probe.py 1        # 对 1 号站做验证
    python tools/table_probe.py 1 --go   # 加 --go 才会真的触发执行

不加 --go 只做"写表 + 设参数 + 读回"，不触发。
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

TBL_N = 4
TBL_ADDR = 500          # 表格数据存放地址（必须 ≥300）
TBL_START = TBL_ADDR - 300   # 0x00AC 要填的是"地址 - 300"


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

    def tx(self, req, expect_reply=True):
        self.s.reset_input_buffer()
        self.s.write(req)
        if not expect_reply:
            return None
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
        """读 32 位有符号整数。
         * 03H 返回按寄存器顺序：先 reg（低字），再 reg+1（高字），
           每个寄存器 2 字节大端。
         * 【2026-09-18 教训】曾经写成 (d[0]<<24)|(d[1]<<16)|(d[2]<<8)|d[3]，
           把字节当成"整块大端"拼，结果读出 4.4 亿这种荒谬值，
           又拿它去当表格目标位置写入 ⇒ 电机真的往那个垃圾目标跑，机械臂意外移动。
           正确拼法：低字=(d[0]<<8)|d[1]，高字=(d[2]<<8)|d[3]，v=(高字<<16)|低字。"""
        d = self.read(reg, 2)
        if not d or len(d) < 4:
            return None
        lw = (d[0] << 8) | d[1]
        hw = (d[2] << 8) | d[3]
        v = (hw << 16) | lw
        return v - (1 << 32) if v >= (1 << 31) else v

    def write_single(self, reg, val):
        body = bytes([self.addr, 0x06, (reg >> 8) & 0xFF, reg & 0xFF,
                      (val >> 8) & 0xFF, val & 0xFF])
        r = self.tx(body + crc16(body))
        return bool(r)

    def write_multi(self, reg, words):
        n = len(words)
        body = bytes([self.addr, 0x10, (reg >> 8) & 0xFF, reg & 0xFF,
                      (n >> 8) & 0xFF, n & 0xFF, n * 2])
        for w in words:
            body += bytes([(w >> 8) & 0xFF, w & 0xFF])
        r = self.tx(body + crc16(body))
        return bool(r)

    def close(self):
        self.s.close()


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    addr = int(sys.argv[1])
    do_go = "--go" in sys.argv

    d = Drv(addr)
    print(f"=== 站号 {addr} 位置表机制验证（目标=当前位置，臂不会动）===\n")

    pos0 = d.read_i32(0x0004)
    if pos0 is None:
        sys.exit("读不到当前位置，退出")
    print(f"当前位置 0x0004/05 = {pos0} pulses")

    alarm = d.read_u16(0x00A3)
    print(f"报警状态 0x00A3      = {alarm}")

    # 【安全闸门】表格目标必须是"当前位置"或离它极近，否则拒绝写表。
    # 2026-09-18 就是因为缺这道闸：位置读错成 4.4 亿，写进表格后一触发，
    # J1 真的往那个垃圾目标跑，机械臂从 (200,27) 甩到 (196,48)。
    # 想做真实位移实验，必须显式传 --allow-move N 放开 N pulses 的窗口。
    allow = 0
    for a in sys.argv:
        if a.startswith("--allow-move="):
            allow = int(a.split("=", 1)[1])
    # 合理性检查：真实位置不会是几亿这种量级
    if abs(pos0) > 50_000_000:
        sys.exit(f"位置读数 {pos0} 明显不合理（多半是字节序又读错了），拒绝继续。")

    # 1) 写表：4 个等于当前位置的目标，每个占 2 个寄存器（低字在前）
    lo = pos0 & 0xFFFF
    hi = (pos0 >> 16) & 0xFFFF
    words = []
    for _ in range(TBL_N):
        words += [lo, hi]
    ok = d.write_multi(TBL_ADDR, words)
    print(f"\n1) 写表格数据 @{TBL_ADDR}..{TBL_ADDR+2*TBL_N-1}（{TBL_N}×{pos0}）: {'OK' if ok else '失败'}")
    rb = d.read(TBL_ADDR, 2)
    print(f"   读回第一条: {rb.hex(' ') if rb else '失败'}")

    # 2) 表大小 / 表指针 / 表开始地址
    print(f"\n2) 设置表参数")
    print(f"   0x00AA 表大小     = {TBL_N} : {'OK' if d.write_single(0x00AA, TBL_N) else '失败'}")
    print(f"   0x00AB 表指针     = 0     : {'OK' if d.write_single(0x00AB, 0) else '失败'}")
    print(f"   0x00AC 表开始地址 = {TBL_START} : {'OK' if d.write_single(0x00AC, TBL_START) else '失败'}")
    print(f"   读回 0x00AA={d.read_u16(0x00AA)} 0x00AB={d.read_u16(0x00AB)} 0x00AC={d.read_u16(0x00AC)}")

    if not do_go:
        print("\n（未加 --go，不触发执行。表已写好，参数已设。）")
        d.close()
        return

    # 3) 触发：绝对位置 + 指针 +1
    print(f"\n3) 触发执行 0x00DD = 0x0001（绝对位置，指针 +1）")
    for i in range(TBL_N):
        before = d.read_u16(0x00AB)
        d.write_single(0x00DD, 0x0001)
        time.sleep(0.6)
        after = d.read_u16(0x00AB)
        pos = d.read_i32(0x0004)
        al = d.read_u16(0x00A3)
        print(f"   第{i+1}次: 指针 {before} → {after} | 位置 {pos} "
              f"(Δ={pos - pos0:+d}) | 报警 {al}")

    print("\n判定：")
    print("  指针每次 +1 且位置不动 ⇒ 表格机制可用，且'目标=当前'确实是安全的空操作")
    print("  指针不动 ⇒ 该机制可能要先把数据用 0x00DC 保存到闪存才生效")
    print("  报警非 0 ⇒ 指令被拒，看手册 §52 报警码")
    d.close()


if __name__ == "__main__":
    main()
