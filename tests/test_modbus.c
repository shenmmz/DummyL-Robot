/*
 * test_modbus.c —— Modbus RTU 帧构造与解析单元测试（CTest，不连硬件）
 * ------------------------------------------------------------
 * 所属模块：测试（tests）
 * 对外接口：main（入口）
 * 依赖模块：comm/modbus_rtu
 */

/*
 * test_modbus: Modbus RTU 03H/06H/10H 帧构造与解析测试（不连硬件）
 * 帧内容与 Zeta 手册示例对齐。
 */

#include "comm/modbus_rtu.h"

#include <stdio.h>
#include <string.h>

static int g_fail = 0;

static void check(int cond, const char *name)
{
    if (cond) {
        printf("通过 [%s]\n", name);
    } else {
        printf("失败 [%s]\n", name);
        g_fail++;
    }
}

int main(void)
{
    uint8_t frame[64];
    uint8_t rx[64];
    ModbusFrame resp;
    size_t len;
    uint16_t vals[4];

    printf("== test_modbus ==\n");

    /* 03H 读 1 个寄存器：01 03 00 00 00 01 84 0A */
    len = modbus_build_read(0x01, 0x0000, 0x0001, frame);
    check(len == 8, "03H 帧长度");
    check(frame[0] == 0x01 && frame[1] == 0x03, "03H 从站/功能码");
    check(frame[2] == 0x00 && frame[3] == 0x00 && frame[4] == 0x00 && frame[5] == 0x01,
          "03H 地址/数量");
    check(frame[6] == 0x84 && frame[7] == 0x0A, "03H CRC 低字节在前");

    /* 03H 响应解析：01 03 02 00 01 79 84（状态=运行中） */
    rx[0] = 0x01; rx[1] = 0x03; rx[2] = 0x02; rx[3] = 0x00; rx[4] = 0x01;
    rx[5] = 0x79; rx[6] = 0x84;
    check(modbus_parse_response(rx, 7, &resp) == 0, "03H 响应 CRC 通过");
    check(resp.data_len == 2 && resp.data[0] == 0x00 && resp.data[1] == 0x01,
          "03H 响应数据");

    /* 06H 写单寄存器：01 06 00 06 00 01（使能） */
    len = modbus_build_write_single(0x01, 0x0006, 0x0001, frame);
    check(len == 8, "06H 帧长度");
    check(frame[1] == 0x06 && frame[2] == 0x00 && frame[3] == 0x06 &&
          frame[4] == 0x00 && frame[5] == 0x01, "06H 内容");
    check(modbus_check_crc(frame, len) == 0, "06H CRC 自校验");

    /* 06H 响应回显解析 */
    memcpy(rx, frame, len);
    check(modbus_parse_response(rx, len, &resp) == 0, "06H 回显解析");
    check(resp.reg_addr == 0x0006 && resp.reg_count == 1, "06H 地址/数量");

    /* 10H 写多寄存器：位置模式 01 10 00 10 00 04 08 00 19 00 00 13 88 00 00 */
    vals[0] = 0x0019; vals[1] = 0x0000; vals[2] = 0x1388; vals[3] = 0x0000;
    len = modbus_build_write_multi(0x01, 0x0010, vals, 4, frame);
    check(len == 17, "10H 帧长度");
    check(frame[0] == 0x01 && frame[1] == 0x10 && frame[2] == 0x00 && frame[3] == 0x10,
          "10H 从站/功能码/地址");
    check(frame[4] == 0x00 && frame[5] == 0x04 && frame[6] == 0x08, "10H 数量/字节数");
    check(frame[7] == 0x00 && frame[8] == 0x19, "10H 数据1 高/低");
    check(modbus_check_crc(frame, len) == 0, "10H CRC 自校验");

    /* 10H 响应回显解析：01 10 00 10 00 04 */
    rx[0] = 0x01; rx[1] = 0x10; rx[2] = 0x00; rx[3] = 0x10; rx[4] = 0x00; rx[5] = 0x04;
    len = modbus_build_read(0x01, 0x0010, 0x0004, frame); /* 借用构造器算 CRC 不便，手算 */
    (void)len;
    {
        uint16_t crc = 0xFFFF;
        int i, b;
        for (i = 0; i < 6; i++) {
            crc ^= rx[i];
            for (b = 0; b < 8; b++) {
                if (crc & 1) { crc >>= 1; crc ^= 0xA001; } else { crc >>= 1; }
            }
        }
        rx[6] = (uint8_t)(crc & 0xFF);
        rx[7] = (uint8_t)(crc >> 8);
    }
    check(modbus_parse_response(rx, 8, &resp) == 0, "10H 回显解析");
    check(resp.reg_addr == 0x0010 && resp.reg_count == 4, "10H 地址/数量");

    /* 异常响应：01 83 02 错误码（功能码|0x80） */
    rx[0] = 0x01; rx[1] = 0x83; rx[2] = 0x02; rx[3] = 0xC0; rx[4] = 0xF1;
    check(modbus_parse_response(rx, 5, &resp) == MODBUS_ERR_EXCEPTION, "异常响应识别");
    check(resp.data[0] == 0x02, "异常码提取");

    /* 坏 CRC */
    rx[0] = 0x01; rx[1] = 0x03; rx[2] = 0x02; rx[3] = 0x00; rx[4] = 0x01;
    rx[5] = 0x00; rx[6] = 0x00;
    check(modbus_parse_response(rx, 7, &resp) == MODBUS_ERR_BAD_CRC, "坏 CRC 拒绝");

    if (g_fail == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("共 %d 项失败\n", g_fail);
    return 1;
}
