
#include "comm/modbus_rtu.h"
#include <windows.h>


/* Modbus CRC16（多项式 0xA001）。 */
static uint16_t crc16_modbus(const uint8_t *data, size_t len)
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


/* 写大端 u16。 */
static void put_u16_be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

/* 读大端 u16。 */
static uint16_t get_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* 在帧尾追加 CRC16（低字节在前）。 */
static void append_crc(uint8_t *frame, size_t len)
{
    uint16_t crc = crc16_modbus(frame, len);
    frame[len]     = (uint8_t)(crc & 0xFF);
    frame[len + 1] = (uint8_t)(crc >> 8);
}

/* 组 03（读保持寄存器）请求帧：从站 + 0x03 + 起始地址 + 寄存器数 + CRC，共 8 字节。 */
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

/* 组 04（读输入寄存器）请求帧：只读一个寄存器，共 8 字节。驱动器用它读电流等只读量。 */
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

/* 组 06（写单个寄存器）请求帧，共 8 字节。 */
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

/* 组 10H（写多个寄存器）请求帧，返回总长度。 */
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
    frame[6] = (uint8_t)(count * 2);
    p = 7;
    for (i = 0; i < count; i++) {
        put_u16_be(&frame[p], values[i]);
        p += 2;
    }
    append_crc(frame, p);
    return p + 2;
}

/* 校验整帧 CRC16。 */
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

/* 把应答帧解析成 ModbusFrame（区分正常/异常应答，校验站号与功能码）。 */
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

    if (func & MODBUS_FUNC_ERR_BIT) {
        if (rx_len != 5) {
            return ERR_LEN;
        }
        out->data[0] = rx[2];
        out->data_len = 1;
        return ERR_EXCEPTION;
    }

    switch (func) {
    case MODBUS_FUNC_READ_HOLDING:
    case MODBUS_FUNC_READ_INPUT:
        if (rx_len < 5 || rx[2] != (rx_len - 5)) {
            return ERR_LEN;
        }
        out->data_len = rx[2];
        for (i = 0; i < out->data_len; i++) {
            out->data[i] = rx[3 + i];
        }
        break;

    case MODBUS_FUNC_WRITE_SINGLE:
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


static const CommOps *g_comm_ops = NULL;

/* 注入底层通信实现（由 serial_win.c 提供的 ops）。 */
void modbus_comm_set(const CommOps *ops)
{
    g_comm_ops = ops;
}

/* 取当前底层通信实现。 */
const CommOps *modbus_comm_get(void)
{
    return g_comm_ops;
}

static struct {
    uint32_t n;
    double   flush_ms, write_ms, read_ms, total_ms;
    uint32_t n_noread;
    double   noread_ms;
} g_bus_stat;

/* 取当前毫秒时基（QueryPerformanceCounter）。 */
static double bus_ms_now(void)
{
    static LARGE_INTEGER freq;
    LARGE_INTEGER c;
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart * 1000.0 / (double)freq.QuadPart;
}

/* 清零事务统计（flush/write/read/合计 + noread 的笔数与耗时）。 */
void modbus_stats_reset(void)
{
    memset(&g_bus_stat, 0, sizeof(g_bus_stat));
}

/* 取总线统计的均值。 */
void modbus_stats_get(uint32_t *n, double *flush_ms, double *write_ms,
                      double *read_ms, double *total_ms,
                      uint32_t *n_noread, double *noread_ms)
{
    if (n)         *n         = g_bus_stat.n;
    if (flush_ms)  *flush_ms  = (g_bus_stat.n > 0) ? g_bus_stat.flush_ms / g_bus_stat.n : 0.0;
    if (write_ms)  *write_ms  = (g_bus_stat.n > 0) ? g_bus_stat.write_ms / g_bus_stat.n : 0.0;
    if (read_ms)   *read_ms   = (g_bus_stat.n > 0) ? g_bus_stat.read_ms  / g_bus_stat.n : 0.0;
    if (total_ms)  *total_ms  = (g_bus_stat.n > 0) ? g_bus_stat.total_ms / g_bus_stat.n : 0.0;
    if (n_noread)  *n_noread  = g_bus_stat.n_noread;
    if (noread_ms) *noread_ms = (g_bus_stat.n_noread > 0) ? g_bus_stat.noread_ms / g_bus_stat.n_noread : 0.0;
}

/* 按应答的实际长度分三段精确读。 */
static int modbus_read_reply(const CommOps *ops, uint8_t *rx, int cap, int timeout_ms)
{
    int got, total = 0, want;

    if (ops == NULL || ops->read_frame == NULL || rx == NULL || cap < 5) {
        return -1;
    }
    got = ops->read_frame(rx, 2, timeout_ms);
    if (got <= 0) return got;
    total = got;
    if (total < 2) return total;

    if (rx[1] & 0x80) {
        want = 3;
    } else {
        switch (rx[1]) {
        case 0x01: case 0x02: case 0x03: case 0x04:
            got = ops->read_frame(rx + total, 1, timeout_ms);
            if (got <= 0) return got;
            total += got;
            want = (int)rx[2] + 2;
            break;
        default:
            want = 6;
            break;
        }
    }
    if (total + want > cap) return total;
    got = ops->read_frame(rx + total, want, timeout_ms);
    if (got > 0) total += got;
    return total;
}

/* 一次完整事务：flush + write + read（等响应）+ 解析。 */
ErrCode modbus_transact(const uint8_t *tx, size_t len, ModbusFrame *out)
{
    uint8_t rx[300];
    int got;
    const CommOps *ops = g_comm_ops;
    double t0, t1, t2;

    if (ops == NULL || tx == NULL || len == 0 || out == NULL) {
        return ERR_PORT;
    }
    t0 = bus_ms_now();

    if (ops->flush != NULL) {
        ops->flush();
    }
    double tf = bus_ms_now();
    if (ops->write_frame == NULL || ops->write_frame(tx, (int)len) != (int)len) {
        return ERR_PORT;
    }
    t1 = bus_ms_now();

    if (ops->read_frame == NULL) {
        return ERR_PORT;
    }
    got = modbus_read_reply(ops, rx, (int)sizeof(rx), 50);
    t2 = bus_ms_now();

    g_bus_stat.n++;
    g_bus_stat.flush_ms += (tf - t0);
    g_bus_stat.write_ms += (t1 - tf);
    g_bus_stat.read_ms  += (t2 - t1);
    g_bus_stat.total_ms += (t2 - t0);

    if (got <= 0) {
        return ERR_TIMEOUT;
    }
    return modbus_parse_response(rx, (size_t)got, out);
}

/* 只写请求、不等响应（noread）。 */
ErrCode modbus_transact_noread(const uint8_t *tx, size_t len)
{
    const CommOps *ops = g_comm_ops;
    double t0, t1;

    if (ops == NULL || tx == NULL || len == 0) {
        return ERR_PORT;
    }
    t0 = bus_ms_now();
    if (ops->flush != NULL) {
        ops->flush();
    }
    if (ops->write_frame == NULL || ops->write_frame(tx, (int)len) != (int)len) {
        return ERR_PORT;
    }
    t1 = bus_ms_now();

    g_bus_stat.n_noread++;
    g_bus_stat.noread_ms += (t1 - t0);
    return ERR_NONE;
}
