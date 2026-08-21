#ifndef CRC16_H
#define CRC16_H

/*
 * CRC16 Modbus (多项式 0xA001)
 * 初始值 0xFFFF，低位先传（LSB first），结果低字节在前发送。
 */

#include <stdint.h>
#include <stddef.h>

/* 计算 Modbus CRC16，返回 16 位校验值 */
uint16_t crc16_modbus(const uint8_t *data, size_t len);

#endif /* CRC16_H */
