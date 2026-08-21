#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

/*
 * Modbus RTU 主站帧构造与解析
 * 支持功能码：03H(读保持寄存器) / 06H(写单寄存器) / 10H(写多寄存器)
 * 帧格式（RTU）: [从站地址][功能码][数据...][CRC低][CRC高]
 * 本层不依赖硬件，构造/解析可离线单测。
 */

#include <stdint.h>
#include <stddef.h>

#define MODBUS_FUNC_READ_HOLDING 0x03
#define MODBUS_FUNC_WRITE_SINGLE 0x06
#define MODBUS_FUNC_WRITE_MULTI  0x10

/* 功能码错误位（从站异常响应：功能码 | 0x80） */
#define MODBUS_FUNC_ERR_BIT      0x80

#define MODBUS_ERR_NONE          0
#define MODBUS_ERR_BAD_LEN       -1
#define MODBUS_ERR_BAD_CRC       -2
#define MODBUS_ERR_BAD_FUNC      -3
#define MODBUS_ERR_BAD_SLAVE     -4
#define MODBUS_ERR_EXCEPTION     -5

/* 解析后的响应帧 */
typedef struct {
    uint8_t  slave;      /* 从站地址 */
    uint8_t  func;       /* 功能码 */
    uint16_t reg_addr;   /* 寄存器起始地址 */
    uint16_t reg_count;  /* 寄存器个数（10H 响应用） */
    uint8_t  data[256];  /* 寄存器数据（字节序已转换为大端存储） */
    size_t   data_len;   /* data 有效字节数 */
} ModbusFrame;

/* ---- 请求帧构造：返回帧长度，frame 需 >= 260 字节 ---- */

/* 03H: 读寄存器。reg_count 个寄存器 */
size_t modbus_build_read(uint8_t slave, uint16_t reg_addr,
                         uint16_t reg_count, uint8_t *frame);

/* 06H: 写单个寄存器 */
size_t modbus_build_write_single(uint8_t slave, uint16_t reg_addr,
                                 uint16_t value, uint8_t *frame);

/* 10H: 写多个寄存器。values 为大端序数值数组，count 个寄存器 */
size_t modbus_build_write_multi(uint8_t slave, uint16_t reg_addr,
                                const uint16_t *values, uint16_t count,
                                uint8_t *frame);

/* ---- 响应帧解析：成功返回 0，失败返回负错误码 ----
 * 支持 03H 数据响应、06H 回显、10H 回显、异常响应（func|0x80）。
 * rx_len 小于实际帧长时按已知最小长度校验。 */
int modbus_parse_response(const uint8_t *rx, size_t rx_len, ModbusFrame *out);

/* 校验 RTU 帧 CRC，返回 0 表示通过 */
int modbus_check_crc(const uint8_t *frame, size_t len);

#endif /* MODBUS_RTU_H */
