/*
 * modbus_rtu.c —— Modbus RTU 主机 03H/06H/10H 帧构造与解析
 * ------------------------------------------------------------
 * 所属模块：通信层（comm）
 * 对外接口：modbus_build_read、modbus_build_write_single、
 *           modbus_build_write_multi、modbus_check_crc、modbus_parse_response
 * 依赖模块：comm/crc16
 */

#include "comm/modbus_rtu.h"
#include "comm/crc16.h"

/*
 * Modbus RTU 主站 03H/06H/10H 帧构造与解析
 * 帧尾 CRC 为低字节在前（Modbus 标准）。
 */

static void put_u16_be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

static uint16_t get_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void append_crc(uint8_t *frame, size_t len)
{
    uint16_t crc = crc16_modbus(frame, len);
    frame[len]     = (uint8_t)(crc & 0xFF);       /* CRC 低字节在前 */
    frame[len + 1] = (uint8_t)(crc >> 8);
}

/* modbus_build_read：构造 03H 读保持寄存器请求帧，返回帧长 */
size_t modbus_build_read(uint8_t slave, uint16_t reg_addr,
                         uint16_t reg_count, uint8_t *frame)
{
    frame[0] = slave;
    frame[1] = MODBUS_FUNC_READ_HOLDING;
    put_u16_be(&frame[2], reg_addr);
    put_u16_be(&frame[4], reg_count);
    append_crc(frame, 6);
    return 8;
}

/* modbus_build_read_input：构造 04H 读单个寄存器请求帧（立三手册 V126
 * 功能码 0x04=读单个寄存器，帧格式与 03H 相同，返回 WORD），返回帧长 */
size_t modbus_build_read_input(uint8_t slave, uint16_t reg_addr,
                               uint8_t *frame)
{
    frame[0] = slave;
    frame[1] = MODBUS_FUNC_READ_INPUT;
    put_u16_be(&frame[2], reg_addr);
    put_u16_be(&frame[4], 1);
    append_crc(frame, 6);
    return 8;
}

/* modbus_build_write_single：构造 06H 写单寄存器请求帧，返回帧长 */
size_t modbus_build_write_single(uint8_t slave, uint16_t reg_addr,
                                 uint16_t value, uint8_t *frame)
{
    frame[0] = slave;
    frame[1] = MODBUS_FUNC_WRITE_SINGLE;
    put_u16_be(&frame[2], reg_addr);
    put_u16_be(&frame[4], value);
    append_crc(frame, 6);
    return 8;
}

/* modbus_build_write_multi：构造 10H 写多寄存器请求帧，返回帧长 */
size_t modbus_build_write_multi(uint8_t slave, uint16_t reg_addr,
                                const uint16_t *values, uint16_t count,
                                uint8_t *frame)
{
    uint16_t i;
    size_t p;

    if (count == 0 || count > 124) {
        return 0;
    }
    frame[0] = slave;
    frame[1] = MODBUS_FUNC_WRITE_MULTI;
    put_u16_be(&frame[2], reg_addr);
    put_u16_be(&frame[4], count);
    frame[6] = (uint8_t)(count * 2);      /* 字节数 */
    p = 7;
    for (i = 0; i < count; i++) {
        put_u16_be(&frame[p], values[i]);
        p += 2;
    }
    append_crc(frame, p);
    return p + 2;
}

/* modbus_check_crc：校验帧 CRC，通过返回 ERR_NONE */
ErrCode modbus_check_crc(const uint8_t *frame, size_t len)
{
    uint16_t crc;
    if (len < 4) {
        return ERR_LEN;
    }
    crc = crc16_modbus(frame, len - 2);
    if ((uint8_t)(crc & 0xFF) != frame[len - 2] ||
        (uint8_t)(crc >> 8) != frame[len - 1]) {
        return ERR_CRC;
    }
    return ERR_NONE;
}

/* modbus_parse_response：解析响应帧（异常码/03H/06H/10H），成功返回 ERR_NONE */
ErrCode modbus_parse_response(const uint8_t *rx, size_t rx_len, ModbusFrame *out)
{
    uint8_t func;
    uint16_t i;
    ErrCode crc_ret;

    if (rx == NULL || out == NULL || rx_len < 5) {
        return ERR_LEN;
    }
    crc_ret = modbus_check_crc(rx, rx_len);
    if (crc_ret != ERR_NONE) {
        return crc_ret;
    }

    out->slave = rx[0];
    func = rx[1];
    out->func = func;
    out->reg_addr = 0;
    out->reg_count = 0;
    out->data_len = 0;

    /* 异常响应: func | 0x80，帧长 5 */
    if (func & MODBUS_FUNC_ERR_BIT) {
        if (rx_len != 5) {
            return ERR_LEN;
        }
        out->data[0] = rx[2];   /* 异常码 */
        out->data_len = 1;
        return ERR_EXCEPTION;
    }

    switch (func) {
    case MODBUS_FUNC_READ_HOLDING:
    case MODBUS_FUNC_READ_INPUT:
        /* [slave][03/04][byte_cnt][data...][crcL][crcH] */
        if (rx_len < 5 || rx[2] != (rx_len - 5)) {
            return ERR_LEN;
        }
        out->data_len = rx[2];
        for (i = 0; i < out->data_len; i++) {
            out->data[i] = rx[3 + i];
        }
        break;

    case MODBUS_FUNC_WRITE_SINGLE:
        /* [slave][06][addrH][addrL][valH][valL][crcL][crcH] 回显 */
        if (rx_len != 8) {
            return ERR_LEN;
        }
        out->reg_addr = get_u16_be(&rx[2]);
        out->reg_count = 1;
        out->data[0] = rx[4];
        out->data[1] = rx[5];
        out->data_len = 2;
        break;

    case MODBUS_FUNC_WRITE_MULTI:
        /* [slave][10][addrH][addrL][cntH][cntL][crcL][crcH] 回显 */
        if (rx_len != 8) {
            return ERR_LEN;
        }
        out->reg_addr = get_u16_be(&rx[2]);
        out->reg_count = get_u16_be(&rx[4]);
        out->data_len = 0;
        break;

    default:
        return ERR_ARG;
    }
    return ERR_NONE;
}

/* ================= CommOps 注入与总线收发（方案一） ================= */

/* 全局 CommOps 指针：由调用方在初始化时注入（串口实现或内存假串口） */
static const CommOps *g_comm_ops = NULL;

void modbus_comm_set(const CommOps *ops)
{
    g_comm_ops = ops;
}

const CommOps *modbus_comm_get(void)
{
    return g_comm_ops;
}

/* modbus_transact：flush -> write -> read -> parse 完整一次主从交互 */
ErrCode modbus_transact(const uint8_t *tx, size_t len, ModbusFrame *out)
{
    uint8_t rx[300];
    int got;
    const CommOps *ops = g_comm_ops;

    if (ops == NULL || tx == NULL || len == 0 || out == NULL) {
        return ERR_PORT;
    }
    if (ops->flush != NULL) {
        ops->flush();
    }
    if (ops->write_frame == NULL || ops->write_frame(tx, (int)len) != (int)len) {
        return ERR_PORT;
    }
    if (ops->read_frame == NULL) {
        return ERR_PORT;
    }
    got = ops->read_frame(rx, (int)sizeof(rx), 200);
    if (got <= 0) {
        return ERR_TIMEOUT;
    }
    return modbus_parse_response(rx, (size_t)got, out);
}
