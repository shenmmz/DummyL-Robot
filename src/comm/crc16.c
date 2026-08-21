/*
 * crc16.c —— CRC16-Modbus（0xA001）校验值计算
 * ------------------------------------------------------------
 * 所属模块：通信层（comm）
 * 对外接口：crc16_modbus
 * 依赖模块：无
 */

#include "comm/crc16.h"

/*
 * CRC16 Modbus 0xA001 实现
 * 逐字节异或 + 右移 8 次，最低位为 1 时异或多项式 0xA001。
 */
uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t i;
    int bit;

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (bit = 0; bit < 8; bit++) {
            if (crc & 0x0001u) {
                crc >>= 1;
                crc ^= 0xA001u;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
