# -*- coding: utf-8 -*-
"""生成 J2（从站 2）原始 Modbus RTU 帧 —— 绕过程序、手工验证 J2 减速比/脉冲比。

帧可以直接用任意串口助手（COM4 / 921600 / 8N1 / Hex 发送）逐条下发。

用法：
    python tools/j2_frames.py
"""
import struct

A = 2          # J2 从站地址（ROBOT_SLAVE_ADDR_TABLE = {1,2,3,4,5,6}）
SPR = 10000    # 编码器每转脉冲数（0x0024，已实测确认）


def crc16(data: bytes) -> bytes:
    """Modbus CRC16：初值 0xFFFF，多项式 0xA001，低字节先发。
    已用 3 个标准向量校验：01 03 0000 0001->840A、11 03 006B 0003->7687、
    01 03 0000 000A->C5CD。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, crc >> 8])


def hexs(b: bytes) -> str:
    return ' '.join('%02X' % x for x in b)


def w16(reg, val):
    """0x06 写单个寄存器"""
    body = bytes([A, 0x06, (reg >> 8) & 0xFF, reg & 0xFF,
                  (val >> 8) & 0xFF, val & 0xFF])
    return body + crc16(body)


def w32(reg, val):
    """0x10 写 2 个寄存器（32 位，低字在前，每字大端）"""
    v = val & 0xFFFFFFFF
    lo, hi = v & 0xFFFF, (v >> 16) & 0xFFFF
    body = bytes([A, 0x10, (reg >> 8) & 0xFF, reg & 0xFF,
                  0x00, 0x02, 0x04,
                  (lo >> 8) & 0xFF, lo & 0xFF,
                  (hi >> 8) & 0xFF, hi & 0xFF])
    return body + crc16(body)


def w2x16(reg, v1, v2):
    """0x10 写 2 个连续的 16 位寄存器"""
    body = bytes([A, 0x10, (reg >> 8) & 0xFF, reg & 0xFF,
                  0x00, 0x02, 0x04,
                  (v1 >> 8) & 0xFF, v1 & 0xFF,
                  (v2 >> 8) & 0xFF, v2 & 0xFF])
    return body + crc16(body)


def rd(reg, n=2):
    """0x03 读 n 个寄存器"""
    body = bytes([A, 0x03, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, n])
    return body + crc16(body)


def steps(deg, red):
    """程序里的 DEG2STEPS(deg, red) = deg * red / 360 * 10000"""
    return int(round(deg * red / 360.0 * SPR))


def show(tag, frame, note=''):
    print('  %-26s %s   %s' % (tag, hexs(frame), note))


print('=' * 100)
print('J2（从站地址 = 2）原始 Modbus RTU 帧   @ COM4 / 921600 / 8N1，末两字节是 CRC（低字节在前）')
print('=' * 100)

print('\n【A】准备 —— 先发这 4 条，顺序不能换')
show('A1 使能 0x00D4=0',        w16(0x00D4, 0), 'U16，0=上电使能')
show('A2 退出连续 0x00C8=0',    w16(0x00C8, 0), 'U16；不退出则 0x00E8 会被忽略')
show('A3a 速度 30rpm',          w32(0x00D8, 3000), 'INT32，单位0.01rpm → 电机轴 30rpm')
show('A3b 速度 100rpm(可选)',   w32(0x00D8, 10000), '同上，快 3.3 倍；想省时间用这条替 A3a')
show('A4 加减速 0x0098/99',     w2x16(0x0098, 150, 200), '连续WORD：加速150 减速200（与 home 一致）')

print('\n【B】在【堵转位置】把当前位置清成 0')
show('B1 清零当前位置 0x00D2=0', w32(0x00D2, 0), 'DWORD，RAM 无记忆')
show('B2 读当前位置 0x0004',     rd(0x0004, 2), '应回 02 03 04 00 00 00 00 xx xx')

print('\n【C】让 J2 转一个角度（绝对定位 0x00E8，INT32 脉冲，相对刚清的零点）')
for deg, red in ((10, 100), (30, 100), (90, 100), (-30, 100)):
    s = steps(deg, red)
    raw = struct.unpack('>i', struct.pack('>I', s & 0xFFFFFFFF))[0]
    flag = '  ★推荐先跑这条' if deg == 30 else ('  ⚠有风险' if deg == 90 else '')
    show('C %+d°  (red=100)' % deg, w32(0x00E8, raw),
         '= %d 步 = 0x%08X%s' % (raw, s & 0xFFFFFFFF, flag))

print('\n【C2】★更好的做法：相对运动 0x00DE（不用清零，不破坏零点，能原路转回来）')
print('   home 回零后直接发；-83333 转过去、+83333 转回来。若减速比真错了，')
print('   来回会转同样的倍数（去 -60° 就回 +60°），仍然正好回到原点。')
show('C2 相对 -30°', w32(0x00DE, -83333), '= -83333 步 = 0xFFFEBA7B')
show('C2 相对 +30°（转回来）', w32(0x00DE, 83333), '= +83333 步 = 0x00014585')
show('C2 相对 -10°（先探）', w32(0x00DE, -27778), '= -27778 步 = 0xFFFF9376')
show('C2 相对 +10°（转回来）', w32(0x00DE, 27778), '= +27778 步 = 0x00006C82')

print('\n【D】读回 / 复位 / 急停')
show('D1 读当前位置 0x0004',      rd(0x0004, 2), '判定无效，只能用外部量角，见下')
show('D2 读使能状态 0x00D4',      rd(0x00D4, 1))
show('D3 回到清零点 0x00E8=0',    w32(0x00E8, 0), '转回堵转点，方便复位重来')
show('D4 紧急停止 0x00C8=0x0100', w16(0x00C8, 0x0100), '立即停止并保持使能（别用 0x00D4=1，J2 带重力会掉）')

print('\n' + '=' * 100)
print('【安全余量】J2 软限位 −72 ~ +90（机械角）')
print('=' * 100)
print('  home 回零：dir = −1（往负方向顶到堵转），退让量 = mech[1]+q0[1] = 0 + 74.58 = 74.58°')
print('  ⇒ 堵转点机械角 = 0 − 74.58 = −74.58°（比软限位下限 −72 还低 2.58°，符合物理限位块）')
print('  ⇒ 从堵转点往【正】方向的可用行程 = 90 −(−74.58) = 164.58°')
print()
print('  下发     red 正确时终点    若实际转 2 倍     判定')
print('  ------   --------------    --------------   -------------------------')
for deg in (30, 90):
    p1, p2 = -74.58 + deg, -74.58 + 2 * deg
    verdict = '安全' if p2 <= 90 else '★会撞：超上限 %.1f°' % (p2 - 90)
    print('   +%-4d°   %+10.2f°     %+10.2f°     %s' % (deg, p1, p2, verdict))
print()
print('  ⇒ 先跑 +30°：就算减速比真错了 2 倍也只到 −14.58°，仍在限位内，撞不了。')
print('  ⇒ 若 +30° 量出来是 60°（说明真实减速比是 50），【禁止】再试 +90°（会到 +105°）。')

print('\n【耗时】0x00D8 是【电机轴】转速，输出轴要除以减速比')
for rpm in (30, 100):
    print('  电机 %3d rpm → 输出轴 %.2f °/s → 30° 约 %4.1f s，90° 约 %4.1f s'
          % (rpm, rpm / 100.0 * 6, 30 / (rpm / 100.0 * 6), 90 / (rpm / 100.0 * 6)))
print('  （若真实减速比是 50，实际会快一倍；是 200 则慢一倍——这个本身也是个旁证）')

print('\n' + '=' * 100)
print('【判定】必须【外部量角度】。编码器在电机侧，读回值永远自洽，不能用来判定。')
print('=' * 100)
print('  下发 +30°（83333 步）后外部量得：')
print('     30°  ⇒ 减速比 100 正确，J2 参数没问题')
print('     60°  ⇒ 真实减速比 50（实际转角 = 程序的 2 倍）')
print('     15°  ⇒ 真实减速比 200（实际转角 = 程序的 1/2）')
print('  同理 +90°（250000 步）：量得 90 / 180 / 45 分别对应 100 / 50 / 200。')
print()
print('  推导：下发步数 = deg × red/360 × 10000；实际输出轴转角 = 步数/(10000×真实减速比) 转')
print('        ⇒ 实际转角 = 下发角度 × (配置 red / 真实减速比)')
print()
print('  读回值解析（仅供核对通信）：0x0004 回 2 个寄存器，【低字在前】拼成 INT32，')
print('  度 = 值 × 360 /(10000 × red)。red=100 转 30° 读回应恒为 83333 —— 所以判定无效。')
