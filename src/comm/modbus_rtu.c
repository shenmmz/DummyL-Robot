
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

/* 组 10H（写多个寄存器）请求帧，返回总长度。
 * count 上限 124（Modbus 单帧 256 字节的限制）；count==0 或超限返回 0。
 * ⚠️ 本机驱动器对 10H 广播写 0x00D8 会六轴一起执行（bcast 命令靠它）。 */
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

/* 取总线统计的【均值】（diag / busrate 打印的就是这里）。
 * 输出：事务笔数 n、flush/write/read/total 各自平均耗时、noread 笔数与其均值。
 * 实测（921600，读法修复后）：read ≈1.63ms、total ≈1.70ms、write 0.08ms。
 * 修复前：total 15.4ms（其中 13.6ms 是 cap=300 白等的驱动节拍）。 */
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

/* ★ 按应答的【实际长度】分三段精确读，是整套系统最大的一次性能修正。
 *
 * 背景：原来是一次性 read_frame(rx, 300, 50)。
 * 【ReadFile 请求的字节数 > 实际会到达的字节数 ⇒ 它不会因"读满"而完成】，
 * 只能干等 ReadIntervalTimeout；而 CH341SER 把它挂在 ~15.6ms 节拍上
 * （= Windows 默认 64Hz 系统定时器）⇒ 每笔白等一个节拍。与波特率/USB/驱动器全无关。
 * 判据实测（同一笔读，只改请求长度 cap）：
 *   cap=300 → 15.37ms ｜ cap=9（正好一帧）→ **1.70ms** ｜ cap=10 → 15.17ms ｜ cap=64 → 15.44ms
 * 修法（本函数）：读类 2 → 1(字节数) → n+2；写类 2 → 6；异常应答 2 → 1 → 2。
 * 收益：单事务 15.4 → 1.70ms（9 倍）、六轴一轮 93.6 → 10.2ms、刷新率 10.7 → 97.9Hz。
 * ⚠️ 这条修好之前，所有"USB 延迟 10~11ms""换 FTDI/CP210x"的结论都是假的。 */
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

/* 一次完整事务：flush + write + read(等响应) + 解析，并累计统计。
 * 实测单事务 **1.70ms** = 线上 0.23 + 驱动器周转 1.47 + USB 栈 <=0.08。
 * ⇒ 86% 的时间在等驱动器应答，波特率/转换器都不是瓶颈。 */
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

/* 只写请求、不等响应（noread）。
 * ⚠️ 实测【零收益】：从站照样要花 1.47ms 周转、还要占线把响应发出来，
 * 所以它和等响应一样慢，却少了确认 ⇒ 现在只剩 nrtest / busrate 探针在用。
 * ⚠️ 它会留下没人读走的回帧，被下一个读当成应答 ⇒ CRC 失败。
 * 症状：同一条命令行 diag 后紧跟 busrate，busrate 第一笔读报"关节1 不在线"。
 * ⇒ 所以 diag / busrate / nrtest 首尾都要调 bus_drain()。 */
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
