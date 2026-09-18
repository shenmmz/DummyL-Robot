/*
 * modbus_rtu.c —— Modbus RTU 主机 03H/06H/10H 帧构造与解析
 * ------------------------------------------------------------
 * 所属模块：通信层（comm）
 * 对外接口：modbus_build_read、modbus_build_write_single、
 *           modbus_build_write_multi、modbus_check_crc、modbus_parse_response
 * 包含 CRC16-Modbus（0xA001）校验计算
 */

#include "comm/modbus_rtu.h"
#include <windows.h>

/* ================= CRC16-Modbus（0xA001） ================= */

/* 逐字节异或 + 右移 8 次，最低位为 1 时异或多项式 0xA001 */
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

/* ================= 总线时延分段统计（供 diag 命令定位瓶颈） =================
 * 一个事务 = flush + write + read(等从站响应) 三段。单事务实测 26ms 到底花在哪一段
 * 必须实测，不能猜：本项目用的是 CH340(WCH CH341SER 驱动)，注册表 Device Parameters
 * 里【没有】FTDI 那种 LatencyTimer 项，"改延迟计时器"这条路对它不成立，
 * 真正的耗时点得靠分段计时找出来。 */
static struct {
    uint32_t n;
    double   flush_ms, write_ms, read_ms, total_ms;
    uint32_t n_noread;
    double   noread_ms;      /* 只写不读：flush+write 总耗时 */
} g_bus_stat;

static double bus_ms_now(void)
{
    static LARGE_INTEGER freq;
    LARGE_INTEGER c;
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart * 1000.0 / (double)freq.QuadPart;
}

void modbus_stats_reset(void)
{
    memset(&g_bus_stat, 0, sizeof(g_bus_stat));
}

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

/* modbus_transact：flush -> write -> read响应 -> parse */
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

    /* 帧间延时：Modbus RTU 3.5字符间隔（115200bps下约0.3ms）。
     * 前次读响应的耗时远超3.5字符时间，flush清空缓冲后总线已空闲，
     * 无需额外Sleep——每事务省2ms，6轴12事务=24ms，短行程MoveL显著提速。 */
    if (ops->flush != NULL) {
        ops->flush();
    }
    double tf = bus_ms_now();               /* flush(PurgeComm) 单独计时：USB 串口上它不便宜 */
    if (ops->write_frame == NULL || ops->write_frame(tx, (int)len) != (int)len) {
        return ERR_PORT;
    }
    t1 = bus_ms_now();

    /* RTS_CONTROL_TOGGLE 模式下，Windows 自动管理 RS485 方向：
     * 发送时 RTS 高（发送模式），发送完成后 RTS 低（接收模式）。
     * 不需要手动处理回声。 */
    if (ops->read_frame == NULL) {
        return ERR_PORT;
    }
    got = ops->read_frame(rx, (int)sizeof(rx), 50);
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

/* modbus_transact_noread：只 flush + write，【不等从站响应】。
 * 用途有二：
 *   1) 给 diag 命令做对照测量——"不读响应"能省掉多少毫秒；
 *   2) 将来若确认为主要瓶颈，可让纯写类指令（设速度/发绝对位置）走这条路。
 * 注意：RS485 半双工下，从站仍会回一帧。同一总线上紧接着发下一帧前，
 * 必须留出"从站响应发完"的时间，否则两帧在总线上撞车。本函数不负责该间隔。 */
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
