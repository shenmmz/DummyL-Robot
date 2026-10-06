#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H


#include <stdint.h>
#include <stddef.h>
#include "utils/err.h"
#include "comm/comm_if.h"

#define MODBUS_FUNC_READ_HOLDING 0x03   /* 读保持寄存器（主用） */
#define MODBUS_FUNC_READ_INPUT   0x04   /* 读输入寄存器 */
#define MODBUS_FUNC_WRITE_SINGLE 0x06   /* 写单寄存器（16 位） */
#define MODBUS_FUNC_WRITE_MULTI  0x10   /* 写多寄存器（DWORD 拆两字下发） */

#define MODBUS_FUNC_ERR_BIT      0x80   /* 响应功能码置此 bit ⇒ 异常响应 */

#define MODBUS_ERR_NONE      ERR_NONE       /* 正常 */
#define MODBUS_ERR_BAD_LEN   ERR_LEN        /* 帧长度不符 */
#define MODBUS_ERR_BAD_CRC   ERR_CRC        /* CRC16 校验失败 */
#define MODBUS_ERR_BAD_FUNC  ERR_ARG        /* 功能码不匹配 */
#define MODBUS_ERR_BAD_SLAVE ERR_ARG        /* 从站地址不匹配 */
#define MODBUS_ERR_EXCEPTION ERR_EXCEPTION  /* 驱动器回异常码 */

typedef struct ModbusFrame {
    uint8_t  slave;      /* 从站地址（1~6 对应六轴，0 = 广播） */
    uint8_t  func;       /* 功能码 03H/04H/06H/10H（异常响应含 0x80） */
    uint16_t reg_addr;   /* 起始寄存器地址 */
    uint16_t reg_count;  /* 寄存器个数 */
    uint8_t  data[256];  /* 数据区（每寄存器 2 字节，高字节在前） */
    size_t   data_len;   /* data[] 实际有效长度（字节） */
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
