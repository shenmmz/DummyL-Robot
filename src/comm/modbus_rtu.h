#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H


#include <stdint.h>
#include <stddef.h>
#include "utils/err.h"
#include "comm/comm_if.h"

#define MODBUS_FUNC_READ_HOLDING 0x03
#define MODBUS_FUNC_READ_INPUT   0x04
#define MODBUS_FUNC_WRITE_SINGLE 0x06
#define MODBUS_FUNC_WRITE_MULTI  0x10

#define MODBUS_FUNC_ERR_BIT      0x80

#define MODBUS_ERR_NONE      ERR_NONE
#define MODBUS_ERR_BAD_LEN   ERR_LEN
#define MODBUS_ERR_BAD_CRC   ERR_CRC
#define MODBUS_ERR_BAD_FUNC  ERR_ARG
#define MODBUS_ERR_BAD_SLAVE ERR_ARG
#define MODBUS_ERR_EXCEPTION ERR_EXCEPTION

typedef struct ModbusFrame {
    uint8_t  slave;
    uint8_t  func;
    uint16_t reg_addr;
    uint16_t reg_count;
    uint8_t  data[256];
    size_t   data_len;
} ModbusFrame;


/* 构造 03H 读保持寄存器请求。返回帧长（8 字节）。 */
size_t modbus_build_read(uint8_t slave, uint16_t reg_addr,
                         uint16_t reg_count, uint8_t *frame);

/* 构造 04H 读输入寄存器请求。返回帧长（8 字节）。 */
size_t modbus_build_read_input(uint8_t slave, uint16_t reg_addr,
                               uint8_t *frame);

/* 构造 06H 写单寄存器请求。返回帧长（8 字节）。 */
size_t modbus_build_write_single(uint8_t slave, uint16_t reg_addr,
                                 uint16_t value, uint8_t *frame);

/* 构造 10H 写多寄存器请求。返回帧长（9 + 2*count）。 */
size_t modbus_build_write_multi(uint8_t slave, uint16_t reg_addr,
                                const uint16_t *values, uint16_t count,
                                uint8_t *frame);

/* 解析响应帧（校验从站地址/功能码/长度/CRC）。返回 ERR_NONE 或对应错误码。 */
ErrCode modbus_parse_response(const uint8_t *rx, size_t rx_len, ModbusFrame *out);

/* 校验 CRC16（Modbus 多项式 0xA001）。 */
ErrCode modbus_check_crc(const uint8_t *frame, size_t len);

/* 注入串口后端。启动时调一次。 */
void modbus_comm_set(const CommOps *ops);
/* 取当前串口后端（供需要直接收发的地方，如 looptest 回环）。 */
const CommOps *modbus_comm_get(void);
/* 发一帧 + 等响应 + 解析。 */
ErrCode modbus_transact(const uint8_t *tx, size_t len, ModbusFrame *out);

/* 清总线统计（diag 每次开始时调）。 */
void modbus_stats_reset(void);
/* 取总线统计的均值（n/flush/write/read/total/noread）。 */
void modbus_stats_get(uint32_t *n, double *flush_ms, double *write_ms,
                      double *read_ms, double *total_ms,
                      uint32_t *n_noread, double *noread_ms);

/* 发一帧但不等响应。 */
ErrCode modbus_transact_noread(const uint8_t *tx, size_t len);

#endif
