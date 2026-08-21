/*
 * test_crc16.c —— CRC16-Modbus 校验单元测试（CTest）
 * ------------------------------------------------------------
 * 所属模块：测试（tests）
 * 对外接口：main（入口）
 * 依赖模块：comm/crc16
 */

/*
 * test_crc16: CRC16 Modbus 0xA001 向量测试
 * 使用 Zeta 手册/Modbus 标准示例帧校验。
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
    /* Modbus 标准示例：读保持寄存器 01 03 00 00 00 01 -> 帧尾 84 0A（低字节在前） */
    const uint8_t f1[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    /* 手册示例：读步数 01 03 00 01 00 02 -> 帧尾 95 CB */
    const uint8_t f2[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x02};
    /* 手册示例：读状态与步数 01 03 00 00 00 03 -> 帧尾 05 CB */
    const uint8_t f3[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03};
    /* 空数据 */
    const uint8_t empty[] = {0};

    printf("== test_crc16 ==\n");
    /* CRC16 返回值按数值表示（高字节在前），帧尾按低字节在前传输 */
    check(f1, sizeof(f1), 0x0A84, "03H 读状态帧");
    check(f2, sizeof(f2), 0xCB95, "03H 读步数帧");
    check(f3, sizeof(f3), 0xCB05, "03H 读状态+步数帧");
    check(empty, 0, 0xFFFF, "空数据");

    if (g_fail == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("共 %d 项失败\n", g_fail);
    return 1;
}
