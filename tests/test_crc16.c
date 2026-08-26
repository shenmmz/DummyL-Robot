/*
 * test_crc16.c —— CRC16-Modbus 校验单元测试（CTest）
 * ------------------------------------------------------------
 * 所属模块：测试（tests）
 * 对外接口：main（入口）
 * 依赖模块：comm/crc16
 */

/*
 * test_crc16: CRC16 Modbus 0xA001 向量测试
 * 使用立三（LEESN）485 通讯手册示例帧与 Modbus 标准示例帧校验。
 */

#include "comm/crc16.h"

#include <stdio.h>

static int g_fail = 0;

static void check(const uint8_t *data, size_t len, uint16_t expect, const char *name)
{
    uint16_t got = crc16_modbus(data, len);
    if (got != expect) {
        printf("失败 [%s]: 期望 0x%04X 实际 0x%04X\n", name, expect, got);
        g_fail++;
    } else {
        printf("通过 [%s]: 0x%04X\n", name, got);
    }
}

int main(void)
{
    /* Modbus 标准示例：读保持寄存器 01 03 00 00 00 01 -> CRC 0x0A84（帧尾 84 0A） */
    const uint8_t f1[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    /* 立三手册示例：读实时位置 01 03 00 04 00 02 -> CRC 0xCA85（帧尾 85 CA） */
    const uint8_t f2[] = {0x01, 0x03, 0x00, 0x04, 0x00, 0x02};
    /* 立三手册示例：读状态 01 03 00 06 00 02 -> CRC 0x0A24（帧尾 24 0A） */
    const uint8_t f3[] = {0x01, 0x03, 0x00, 0x06, 0x00, 0x02};
    /* 立三手册示例：写使能 01 06 00 D4 00 00 -> CRC 0xF2C9（帧尾 C9 F2） */
    const uint8_t f4[] = {0x01, 0x06, 0x00, 0xD4, 0x00, 0x00};
    /* 立三手册示例：写绝对位置 -8000 01 10 00 E8 00 02 04 E0 C0 FF FF -> CRC 0x0DCA（帧尾 CA 0D） */
    const uint8_t f5[] = {0x01, 0x10, 0x00, 0xE8, 0x00, 0x02, 0x04, 0xE0, 0xC0, 0xFF, 0xFF};
    /* 空数据 */
    const uint8_t empty[] = {0};

    printf("== test_crc16 ==\n");
    /* CRC16 返回值按数值表示（高字节在前），帧尾按低字节在前传输 */
    check(f1, sizeof(f1), 0x0A84, "标准读保持寄存器帧");
    check(f2, sizeof(f2), 0xCA85, "立三读位置帧");
    check(f3, sizeof(f3), 0x0A24, "立三读状态帧");
    check(f4, sizeof(f4), 0xF2C9, "立三写使能帧");
    check(f5, sizeof(f5), 0x0DCA, "立三写绝对位置帧");
    check(empty, 0, 0xFFFF, "空数据");

    if (g_fail == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("共 %d 项失败\n", g_fail);
    return 1;
}
