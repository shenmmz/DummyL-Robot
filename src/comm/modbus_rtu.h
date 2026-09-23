#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

/*
 * Modbus RTU 主站帧构造与解析
 * 支持功能码：03H(读保持寄存器) / 04H(读单个寄存器) / 06H(写单寄存器) / 10H(写多寄存器)
 * 帧格式（RTU）: [从站地址][功能码][数据...][CRC低][CRC高]
 * 本层不依赖硬件，构造/解析可离线单测。
 */

#include <stdint.h>
#include <stddef.h>
#include "utils/err.h"
#include "comm/comm_if.h"

#define MODBUS_FUNC_READ_HOLDING 0x03 // 读保持寄存器
#define MODBUS_FUNC_READ_INPUT   0x04 // 读单个寄存器
#define MODBUS_FUNC_WRITE_SINGLE 0x06 // 写单寄存器
#define MODBUS_FUNC_WRITE_MULTI  0x10 // 写多寄存器

/* 功能码错误位（从站异常响应：功能码 | 0x80） */
#define MODBUS_FUNC_ERR_BIT      0x80 // 功能码错误位，异常响应时为 1

/* 错误码统一走 ErrCode（utils/err.h）；以下为兼容旧名的别名 */
#define MODBUS_ERR_NONE      ERR_NONE
#define MODBUS_ERR_BAD_LEN   ERR_LEN
#define MODBUS_ERR_BAD_CRC   ERR_CRC
#define MODBUS_ERR_BAD_FUNC  ERR_ARG
#define MODBUS_ERR_BAD_SLAVE ERR_ARG
#define MODBUS_ERR_EXCEPTION ERR_EXCEPTION

/* 解析后的响应帧（具名结构体，供 control 层前向声明使用） */
typedef struct ModbusFrame {
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

/* 04H: 读单个寄存器（立三手册 V126 功能码 0x04，帧格式与 03H 相同） */
size_t modbus_build_read_input(uint8_t slave, uint16_t reg_addr,
                               uint8_t *frame);

/* 06H: 写单个寄存器 */
size_t modbus_build_write_single(uint8_t slave, uint16_t reg_addr,
                                 uint16_t value, uint8_t *frame);

/* 10H: 写多个寄存器。values 为大端序数值数组，count 个寄存器 */
size_t modbus_build_write_multi(uint8_t slave, uint16_t reg_addr,
                                const uint16_t *values, uint16_t count,
                                uint8_t *frame);

/* ---- 响应帧解析：成功返回 ERR_NONE，失败返回对应 ErrCode ----
 * 支持 03H 数据响应、06H 回显、10H 回显、异常响应（func|0x80）。
 * rx_len 小于实际帧长时按已知最小长度校验。 */
ErrCode modbus_parse_response(const uint8_t *rx, size_t rx_len, ModbusFrame *out);

/* 校验 RTU 帧 CRC，通过返回 ERR_NONE */
ErrCode modbus_check_crc(const uint8_t *frame, size_t len);

/* ---- 总线收发（基于 CommOps 注入，方案一：接口抽象） ----
 * modbus_comm_set：注入 CommOps 实现（NULL 表示未注入，transact 返回 ERR_PORT）。
 * modbus_transact：flush -> write_frame(tx) -> read_frame -> parse_response。
 *   成功返回 ERR_NONE；未注入总线返回 ERR_PORT；写失败/读超时返回 ERR_PORT/ERR_TIMEOUT。 */
void modbus_comm_set(const CommOps *ops);
const CommOps *modbus_comm_get(void);
ErrCode modbus_transact(const uint8_t *tx, size_t len, ModbusFrame *out);

/* ---- 总线时延分段统计（diag 命令用）：flush / write / read 各占多少毫秒 ----
 * modbus_stats_reset：清零；modbus_stats_get：取平均值（ms）。 */
void modbus_stats_reset(void);
void modbus_stats_get(uint32_t *n, double *flush_ms, double *write_ms,
                      double *read_ms, double *total_ms,
                      uint32_t *n_noread, double *noread_ms);

/* 只 flush+write、不等从站响应的事务（对照测量 / 将来的纯写快路径） */
ErrCode modbus_transact_noread(const uint8_t *tx, size_t len);

#endif /* MODBUS_RTU_H */
