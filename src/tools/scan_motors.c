/*
 * scan_motors.c —— 总线电机扫描工具（命令行独立程序）
 * ------------------------------------------------------------
 * 所属模块：工具层（tools）
 * 对外接口：main（入口）
 * 依赖模块：comm/serial_win、comm/modbus_rtu、config/robot_config、utils/logger
 */

/*
 * 总线电机扫描工具
 * ------------------------------------------------------------
 * 遍历从站地址范围（默认 1..16），读取设备地址寄存器 0x00E0
 * 与状态寄存器 0x0000，输出在线电机列表。
 * 用法: scan_motors [串口名] [起始地址] [结束地址]
 */

#include "comm/serial_win.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"
#include "utils/logger.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#endif

/* main：遍历从站地址范围扫描在线电机并打印列表 */
int main(int argc, char **argv)
{
    const char *port = "COM3";
    int begin = 1, end = 16;
    SerialPort *sp;
    int addr;
    int found = 0;

#ifdef _WIN32
    SetConsoleOutputCP(65001);
#endif

    if (argc > 1) port = argv[1];
    if (argc > 2) begin = atoi(argv[2]);
    if (argc > 3) end = atoi(argv[3]);

    log_set_level(LOG_LEVEL_INFO);
    printf("扫描串口 %s，从站地址 %d..%d\n", port, begin, end);

    sp = serial_open(port, MODBUS_BAUDRATE);
    if (sp == NULL) {
        LOG_ERROR("串口打开失败");
        return 1;
    }

    for (addr = begin; addr <= end; addr++) {
        uint8_t frame[16];
        uint8_t rx[64];
        ModbusFrame resp;
        size_t len = modbus_build_read((uint8_t)addr, 0x00E0, 1, frame);
        int rx_len;

        serial_flush(sp);
        if (serial_write(sp, frame, len) != (int)len) {
            LOG_WARN("地址 %d 写入失败", addr);
            continue;
        }
        rx_len = serial_read(sp, rx, sizeof(rx), SERIAL_READ_TIMEOUT_MS);
        if (rx_len > 0 && modbus_parse_response(rx, (size_t)rx_len, &resp) == 0) {
            int dev = (int)((resp.data[0] << 8) | resp.data[1]);
            printf("地址 %d: 在线，设备地址寄存器=0x%04X (%d)\n", addr, dev, dev);
            found++;
        } else {
            printf("地址 %d: 无响应\n", addr);
        }
    }

    serial_close(sp);
    printf("扫描完成，共发现 %d 台电机。\n", found);
    return found > 0 ? 0 : 1;
}
