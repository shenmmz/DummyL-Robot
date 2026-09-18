/*
 * test_crc16 —— Modbus RTU 帧构造 + CRC 校验的离线单元测试（T8）
 *
 * 为什么需要它：CRC 错一个 bit，从站会【静默丢弃整帧】，上位机只会看到
 * "超时 / 关节离线"，极难定位。所以要在不接硬件的情况下先把这条链路钉死。
 *
 * 参考向量来自两处互相印证：
 *   1) Modbus 官方经典示例：01 03 00 00 00 01 84 0A（与协议文档一致）
 *   2) 独立 Python 实现（tools/ 里同算法）对 4 组帧的交叉计算
 *
 * 覆盖：
 *   - 03H 读 / 06H 写单 / 10H 写多 三种帧的完整字节序列（含长度）
 *   - modbus_check_crc 对正确帧必须放行
 *   - 任意翻一个 bit 必须报 ERR_CRC（不能漏检）
 *   - 帧长 < 4 必须报 ERR_LEN
 *
 * 运行：cmake --build build --target test_crc16 && build/bin/test_crc16.exe
 * 退出码 0 = 全过；非 0 = 有失败项。
 */

#include <stdio.h>
#include <string.h>
#include "comm/modbus_rtu.h"

static int g_fail = 0;

static void dump(const char *tag, const uint8_t *f, size_t n)
{
    size_t i;
    printf("  %-34s", tag);
    for (i = 0; i < n; i++) printf("%02X ", f[i]);
    printf("\n");
}

/* 逐字节比对期望帧；不同则整帧打印出来给人看 */
static void check_frame(const char *tag, const uint8_t *got, size_t got_len,
                        const uint8_t *want, size_t want_len)
{
    if (got_len != want_len || memcmp(got, want, want_len) != 0) {
        printf("  [FAIL] %s\n", tag);
        printf("    期望(%u): ", (unsigned)want_len);
        for (size_t i = 0; i < want_len; i++) printf("%02X ", want[i]);
        printf("\n    实得(%u): ", (unsigned)got_len);
        for (size_t i = 0; i < got_len; i++) printf("%02X ", got[i]);
        printf("\n");
        g_fail++;
    } else {
        dump(tag, got, got_len);
    }
}

int main(void)
{
    uint8_t f[64];
    size_t n;
    uint16_t vals[2];
    int i;

    printf("=== 1. 帧构造 vs 参考向量 ===\n");

    /* (a) 03H 读：Modbus 官方经典示例 */
    n = modbus_build_read(1, 0x0000, 1, f);
    {
        const uint8_t want[] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};
        check_frame("read(1,0x0000,1)", f, n, want, sizeof want);
    }

    /* (b) 03H 读：协议文档里 11 03 006B 0003 的示例 */
    n = modbus_build_read(0x11, 0x006B, 3, f);
    {
        const uint8_t want[] = {0x11,0x03,0x00,0x6B,0x00,0x03,0x76,0x87};
        check_frame("read(0x11,0x006B,3)", f, n, want, sizeof want);
    }

    /* (c) 06H 写单寄存器 —— bcast 广播降级路径就是用它 */
    n = modbus_build_write_single(1, 0x00D8, 1, f);
    {
        const uint8_t want[] = {0x01,0x06,0x00,0xD8,0x00,0x01,0xC8,0x31};
        check_frame("write_single(1,0x00D8,1)", f, n, want, sizeof want);
    }

    /* (d) 10H 写多寄存器 —— 日常下发位置走的就是它 */
    vals[0] = 0x2710; vals[1] = 0x0000;
    n = modbus_build_write_multi(1, 0x00D8, vals, 2, f);
    {
        const uint8_t want[] = {0x01,0x10,0x00,0xD8,0x00,0x02,0x04,
                                0x27,0x10,0x00,0x00,0xF4,0x24};
        check_frame("write_multi(1,0x00D8,[2710,0000])", f, n, want, sizeof want);
    }

    printf("\n=== 2. modbus_check_crc 必须放行正确帧 ===\n");
    {
        static const uint8_t ok[][16] = {
            {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A},
            {0x11,0x03,0x00,0x6B,0x00,0x03,0x76,0x87},
            {0x01,0x06,0x00,0xD8,0x00,0x01,0xC8,0x31},
        };
        static const size_t okn[] = {8, 8, 8};
        for (i = 0; i < 3; i++) {
            ErrCode e = modbus_check_crc(ok[i], okn[i]);
            printf("  帧%d: %s\n", i + 1, e == ERR_NONE ? "OK" : "FAIL");
            if (e != ERR_NONE) g_fail++;
        }
    }

    printf("\n=== 3. 翻一个 bit 必须被抓出来（不漏检）===\n");
    {
        uint8_t g[8] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};
        int caught = 0, total = 0;
        for (i = 0; i < 8; i++) {
            int b;
            for (b = 0; b < 8; b++) {
                uint8_t save = g[i];
                g[i] ^= (uint8_t)(1u << b);
                total++;
                if (modbus_check_crc(g, 8) != ERR_NONE) caught++;
                g[i] = save;
            }
        }
        printf("  单 bit 翻转 %d 种，抓到 %d 种\n", total, caught);
        if (caught != total) { printf("  [FAIL] 有漏检\n"); g_fail++; }
        else printf("  全部抓到 OK\n");
    }

    printf("\n=== 4. 帧长 < 4 必须报 ERR_LEN ===\n");
    {
        uint8_t s[3] = {0x01, 0x03, 0x00};
        ErrCode e = modbus_check_crc(s, 3);
        printf("  len=3 -> %s\n", e == ERR_LEN ? "ERR_LEN OK" : "FAIL");
        if (e != ERR_LEN) g_fail++;
    }

    printf("\n%s（失败项 %d）\n", g_fail ? "*** 有失败 ***" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
