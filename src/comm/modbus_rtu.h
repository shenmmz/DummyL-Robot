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


size_t modbus_build_read(uint8_t slave, uint16_t reg_addr,
                         uint16_t reg_count, uint8_t *frame);

size_t modbus_build_read_input(uint8_t slave, uint16_t reg_addr,
                               uint8_t *frame);

size_t modbus_build_write_single(uint8_t slave, uint16_t reg_addr,
                                 uint16_t value, uint8_t *frame);

size_t modbus_build_write_multi(uint8_t slave, uint16_t reg_addr,
                                const uint16_t *values, uint16_t count,
                                uint8_t *frame);

ErrCode modbus_parse_response(const uint8_t *rx, size_t rx_len, ModbusFrame *out);

ErrCode modbus_check_crc(const uint8_t *frame, size_t len);

void modbus_comm_set(const CommOps *ops);
const CommOps *modbus_comm_get(void);
ErrCode modbus_transact(const uint8_t *tx, size_t len, ModbusFrame *out);

void modbus_stats_reset(void);
void modbus_stats_get(uint32_t *n, double *flush_ms, double *write_ms,
                      double *read_ms, double *total_ms,
                      uint32_t *n_noread, double *noread_ms);

ErrCode modbus_transact_noread(const uint8_t *tx, size_t len);

#endif
